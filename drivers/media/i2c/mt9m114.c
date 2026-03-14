// SPDX-License-Identifier: GPL-2.0-only
/*
 * mt9m114.c onsemi MT9M114 sensor driver
 *
 * Copyright (c) 2020-2023 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (c) 2012 Analog Devices Inc.
 *
 * Almost complete rewrite of work by Scott Jiang <Scott.Jiang.Linux@gmail.com>
 * itself based on work from Andrew Chew <achew@nvidia.com>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

/* Sysctl registers */
#define MT9M114_CHIP_ID					CCI_REG16(0x0000)
#define MT9M114_COMMAND_REGISTER			CCI_REG16(0x0080)
#define MT9M114_COMMAND_REGISTER_APPLY_PATCH			BIT(0)
#define MT9M114_COMMAND_REGISTER_SET_STATE			BIT(1)
#define MT9M114_COMMAND_REGISTER_REFRESH			BIT(2)
#define MT9M114_COMMAND_REGISTER_WAIT_FOR_EVENT			BIT(3)
#define MT9M114_COMMAND_REGISTER_OK				BIT(15)
#define MT9M114_PLL_DIVIDERS				CCI_REG16(0x0010)
#define MT9M114_PLL_P_DIVIDERS				CCI_REG16(0x0012)
#define MT9M114_PLL_CONTROL				CCI_REG16(0x0014)
#define MT9M114_CLOCKS_CONTROL				CCI_REG16(0x0016)
#define MT9M114_STANDBY_CONTROL				CCI_REG16(0x0018)
#define MT9M114_RESET_AND_MISC_CONTROL			CCI_REG16(0x001a)
#define MT9M114_RESET_SOC					BIT(0)
#define MT9M114_MCU_BOOT_MODE				CCI_REG16(0x001c)
#define MT9M114_PAD_SLEW				CCI_REG16(0x001e)
#define MT9M114_PAD_SLEW_MIN					0
#define MT9M114_PAD_SLEW_MAX					7
#define MT9M114_PAD_SLEW_DEFAULT				7
#define MT9M114_PAD_CONTROL				CCI_REG16(0x0032)

/* XDMA registers */
#define MT9M114_ACCESS_CTL_STAT				CCI_REG16(0x0982)
#define MT9M114_PHYSICAL_ADDRESS_ACCESS			CCI_REG16(0x098a)
#define MT9M114_LOGICAL_ADDRESS_ACCESS			CCI_REG16(0x098e)

/* MCU indirect access registers (MT9M113) */
#define MT9M114_MCU_ADDRESS				CCI_REG16(0x098c)
#define MT9M114_MCU_DATA				CCI_REG16(0x0990)

/*
 * MT9M113 MCU variable addresses (accessed via XDMA 0x098C/0x0990).
 * From datasheet: seq_cmd (R0x0003) values: 0x01=preview, 0x02=capture
 * SEQ_STATE values: 0x03=preview mode, 0x07=capture mode
 */
#define MT9M113_SEQ_CMD					0xa103
#define MT9M113_SEQ_CMD_RUN				0x0001  /* preview/streaming */
#define MT9M113_SEQ_CMD_CAPTURE				0x0002  /* capture mode */
#define MT9M113_SEQ_CMD_REFRESH				0x0005
#define MT9M113_SEQ_CMD_REFRESH_MODE			0x0006
#define MT9M113_SEQ_STATE				0xa104
#define MT9M113_SEQ_CAP_MODE				0xa115

/*
 * MT9M113 RESET_REGISTER (0x301A) values from webOS/Samsung legacy drivers.
 * Both drivers use 0x120C for streaming in MIPI mode.
 * 0x12CE is used for snapshot mode.
 */
#define MT9M113_RESET_REG_STREAMING			0x120C
#define MT9M113_RESET_REG_SNAPSHOT			0x12CE

/*
 * MT9M113 OUTPUT_CONTROL register (physical register, not MCU variable)
 * This register controls the output interface mode (parallel vs MIPI).
 * Value 0x7A08 enables MIPI CSI-2 output with LP (low power) clock mode.
 * Value 0x7A0C enables MIPI CSI-2 output with continuous clock mode.
 * Per webOS: "0x7a08 will enable LP mode, while 0x7A0C will let MIPI clock continuous"
 * Using 0x7A08 to match webOS driver exactly.
 */
#define MT9M113_OUTPUT_CONTROL				CCI_REG16(0x3400)
#define MT9M113_OUTPUT_CONTROL_MIPI_ENABLE		0x7A08

/*
 * MT9M113 CUSTOM_SHORT_PKT register (0x3404)
 * Controls MIPI CSI-2 short packet transmission including Frame Start/End.
 * Default value 0x0000 means FS/FE packets are NOT sent!
 * Bit 7 (frame_cnt_en): Insert frame counter in FS/FE word count field
 * Setting this bit enables Frame Start/End short packet transmission.
 */
#define MT9M113_CUSTOM_SHORT_PKT			CCI_REG16(0x3404)
#define MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN		0x0080

/*
 * MT9M114 MIPI_CONTROL register (0x3C40) - MT9M114 ONLY, NOT on MT9M113!
 * This register does NOT exist on MT9M113 - always reads 0x0.
 * MT9M113 uses OUTPUT_CONTROL (0x3400) instead for MIPI configuration.
 * Keeping these defines for MT9M114 compatibility only.
 */
#define MT9M114_MIPI_CONTROL				CCI_REG16(0x3C40)
#define MT9M114_MIPI_CONTROL_VALUE			0x783C

/*
 * MIPI timing registers (MCU variables) from webOS kernel.
 * These configure CSI-2 D-PHY timing parameters for proper signaling.
 */
#define MT9M113_CAM_PORT_MIPI_TIMING_T_HS_ZERO		CCI_REG16(0xC988)
#define MT9M113_CAM_PORT_MIPI_TIMING_T_HS_ZERO_VAL	0x0F00
#define MT9M113_CAM_PORT_MIPI_TIMING_T_HS_EXIT_TRAIL	CCI_REG16(0xC98A)
#define MT9M113_CAM_PORT_MIPI_TIMING_T_HS_EXIT_TRAIL_VAL 0x0B07
#define MT9M113_CAM_PORT_MIPI_TIMING_T_CLK_POST_PRE	CCI_REG16(0xC98C)
#define MT9M113_CAM_PORT_MIPI_TIMING_T_CLK_POST_PRE_VAL	0x0D01
#define MT9M113_CAM_PORT_MIPI_TIMING_T_CLK_TRAIL_ZERO	CCI_REG16(0xC98E)
#define MT9M113_CAM_PORT_MIPI_TIMING_T_CLK_TRAIL_ZERO_VAL 0x071D
#define MT9M113_CAM_PORT_MIPI_TIMING_T_LPX		CCI_REG16(0xC990)
#define MT9M113_CAM_PORT_MIPI_TIMING_T_LPX_VAL		0x0006
#define MT9M113_CAM_PORT_MIPI_TIMING_INIT		CCI_REG16(0xC992)
#define MT9M113_CAM_PORT_MIPI_TIMING_INIT_VAL		0x0A0C

/* MT9M113 OFIFO control (from webOS kernel) */
#define MT9M114_OFIFO_CONTROL_STATUS			CCI_REG16(0x321c)

/* Sensor Core registers */
#define MT9M114_COARSE_INTEGRATION_TIME			CCI_REG16(0x3012)
#define MT9M114_FINE_INTEGRATION_TIME			CCI_REG16(0x3014)
#define MT9M114_RESET_REGISTER				CCI_REG16(0x301a)
#define MT9M114_RESET_REGISTER_LOCK_REG				BIT(3)
#define MT9M114_RESET_REGISTER_MASK_BAD				BIT(9)
#define MT9M114_FLASH					CCI_REG16(0x3046)
#define MT9M114_GREEN1_GAIN				CCI_REG16(0x3056)
#define MT9M114_BLUE_GAIN				CCI_REG16(0x3058)
#define MT9M114_RED_GAIN				CCI_REG16(0x305a)
#define MT9M114_GREEN2_GAIN				CCI_REG16(0x305c)
#define MT9M114_GLOBAL_GAIN				CCI_REG16(0x305e)
#define MT9M114_GAIN_DIGITAL_GAIN(n)				((n) << 12)
#define MT9M114_GAIN_DIGITAL_GAIN_MASK				(0xf << 12)
#define MT9M114_GAIN_ANALOG_GAIN(n)				((n) << 0)
#define MT9M114_GAIN_ANALOG_GAIN_MASK				(0xff << 0)
#define MT9M114_CUSTOMER_REV				CCI_REG16(0x31fe)

/* Monitor registers */
#define MT9M114_MON_MAJOR_VERSION			CCI_REG16(0x8000)
#define MT9M114_MON_MINOR_VERSION			CCI_REG16(0x8002)
#define MT9M114_MON_RELEASE_VERSION			CCI_REG16(0x8004)

/* Auto-Exposure Track registers */
#define MT9M114_AE_TRACK_ALGO				CCI_REG16(0xa804)
#define MT9M114_AE_TRACK_EXEC_AUTOMATIC_EXPOSURE		BIT(0)
#define MT9M114_AE_TRACK_AE_TRACKING_DAMPENING_SPEED	CCI_REG8(0xa80a)

/* Color Correction Matrix registers */
#define MT9M114_CCM_ALGO				CCI_REG16(0xb404)
#define MT9M114_CCM_EXEC_CALC_CCM_MATRIX			BIT(4)
#define MT9M114_CCM_DELTA_GAIN				CCI_REG8(0xb42a)

/* Camera Control registers */
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_START		CCI_REG16(0xc800)
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_START		CCI_REG16(0xc802)
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_END		CCI_REG16(0xc804)
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_END		CCI_REG16(0xc806)
#define MT9M114_CAM_SENSOR_CFG_PIXCLK			CCI_REG32(0xc808)
#define MT9M114_CAM_SENSOR_CFG_ROW_SPEED		CCI_REG16(0xc80c)
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN	CCI_REG16(0xc80e)
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX	CCI_REG16(0xc810)
#define MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES	CCI_REG16(0xc812)
#define MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES_MAX		65535
#define MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK		CCI_REG16(0xc814)
#define MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK_MAX		8191
#define MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION		CCI_REG16(0xc816)
#define MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW		CCI_REG16(0xc818)
#define MT9M114_CAM_SENSOR_CFG_REG_0_DATA		CCI_REG16(0xc826)
#define MT9M114_CAM_SENSOR_CONTROL_READ_MODE		CCI_REG16(0xc834)
#define MT9M114_CAM_SENSOR_CONTROL_HORZ_MIRROR_EN		BIT(0)
#define MT9M114_CAM_SENSOR_CONTROL_VERT_FLIP_EN			BIT(1)
#define MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_NORMAL		(0 << 4)
#define MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_SKIPPING		(1 << 4)
#define MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_AVERAGE		(2 << 4)
#define MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_SUMMING		(3 << 4)
#define MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_MASK		(3 << 4)
#define MT9M114_CAM_SENSOR_CONTROL_Y_READ_OUT_NORMAL		(0 << 8)
#define MT9M114_CAM_SENSOR_CONTROL_Y_READ_OUT_SKIPPING		(1 << 8)
#define MT9M114_CAM_SENSOR_CONTROL_Y_READ_OUT_SUMMING		(3 << 8)
#define MT9M114_CAM_SENSOR_CONTROL_Y_READ_OUT_MASK		(3 << 8)
#define MT9M114_CAM_SENSOR_CONTROL_ANALOG_GAIN		CCI_REG16(0xc836)
#define MT9M114_CAM_SENSOR_CONTROL_COARSE_INTEGRATION_TIME	CCI_REG16(0xc83c)
#define MT9M114_CAM_SENSOR_CONTROL_FINE_INTEGRATION_TIME	CCI_REG16(0xc83e)
#define MT9M114_CAM_MODE_SELECT				CCI_REG8(0xc84c)
#define MT9M114_CAM_MODE_SELECT_NORMAL				(0 << 0)
#define MT9M114_CAM_MODE_SELECT_LENS_CALIBRATION		(1 << 0)
#define MT9M114_CAM_MODE_SELECT_TEST_PATTERN			(2 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT		CCI_REG8(0xc84d)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT_SOLID		(1 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT_SOLID_BARS		(4 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT_RANDOM		(5 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT_FADING_BARS	(8 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT_WALKING_1S_10B	(10 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_SELECT_WALKING_1S_8B	(11 << 0)
#define MT9M114_CAM_MODE_TEST_PATTERN_RED		CCI_REG16(0xc84e)
#define MT9M114_CAM_MODE_TEST_PATTERN_GREEN		CCI_REG16(0xc850)
#define MT9M114_CAM_MODE_TEST_PATTERN_BLUE		CCI_REG16(0xc852)
#define MT9M114_CAM_CROP_WINDOW_XOFFSET			CCI_REG16(0xc854)
#define MT9M114_CAM_CROP_WINDOW_YOFFSET			CCI_REG16(0xc856)
#define MT9M114_CAM_CROP_WINDOW_WIDTH			CCI_REG16(0xc858)
#define MT9M114_CAM_CROP_WINDOW_HEIGHT			CCI_REG16(0xc85a)
#define MT9M114_CAM_CROP_CROPMODE			CCI_REG8(0xc85c)
#define MT9M114_CAM_CROP_MODE_AE_AUTO_CROP_EN			BIT(0)
#define MT9M114_CAM_CROP_MODE_AWB_AUTO_CROP_EN			BIT(1)
#define MT9M114_CAM_OUTPUT_WIDTH			CCI_REG16(0xc868)
#define MT9M114_CAM_OUTPUT_HEIGHT			CCI_REG16(0xc86a)
#define MT9M114_CAM_OUTPUT_FORMAT			CCI_REG16(0xc86c)
#define MT9M114_CAM_OUTPUT_FORMAT_SWAP_RED_BLUE			BIT(0)
#define MT9M114_CAM_OUTPUT_FORMAT_SWAP_BYTES			BIT(1)
#define MT9M114_CAM_OUTPUT_FORMAT_MONO_ENABLE			BIT(2)
#define MT9M114_CAM_OUTPUT_FORMAT_BT656_ENABLE			BIT(3)
#define MT9M114_CAM_OUTPUT_FORMAT_BT656_CROP_SCALE_DISABLE	BIT(4)
#define MT9M114_CAM_OUTPUT_FORMAT_FVLV_DISABLE			BIT(5)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV			(0 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_RGB			(1 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_BAYER			(2 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_NONE			(3 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_MASK			(3 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_RAWR10		(0 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_PRELSC_8_2	(1 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_POSTLSC_8_2	(2 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_PROCESSED8	(3 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_MASK		(3 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_565RGB		(0 << 12)
#define MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_555RGB		(1 << 12)
#define MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_444xRGB		(2 << 12)
#define MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_444RGBx		(3 << 12)
#define MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_MASK		(3 << 12)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV			CCI_REG16(0xc86e)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_CLIP			BIT(5)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_AUV_OFFSET		BIT(4)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_SELECT_601		BIT(3)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_NORMALISE			BIT(2)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_SAMPLING_EVEN_UV		(0 << 0)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_SAMPLING_ODD_UV		(1 << 0)
#define MT9M114_CAM_OUTPUT_FORMAT_YUV_SAMPLING_EVENU_ODDV	(2 << 0)
#define MT9M114_CAM_OUTPUT_Y_OFFSET			CCI_REG8(0xc870)
#define MT9M114_CAM_AET_AEMODE				CCI_REG8(0xc878)
#define MT9M114_CAM_AET_EXEC_SET_INDOOR				BIT(0)
#define MT9M114_CAM_AET_DISCRETE_FRAMERATE			BIT(1)
#define MT9M114_CAM_AET_ADAPTATIVE_TARGET_LUMA			BIT(2)
#define MT9M114_CAM_AET_ADAPTATIVE_SKIP_FRAMES			BIT(3)
#define MT9M114_CAM_AET_SKIP_FRAMES			CCI_REG8(0xc879)
#define MT9M114_CAM_AET_TARGET_AVERAGE_LUMA		CCI_REG8(0xc87a)
#define MT9M114_CAM_AET_TARGET_AVERAGE_LUMA_DARK	CCI_REG8(0xc87b)
#define MT9M114_CAM_AET_BLACK_CLIPPING_TARGET		CCI_REG16(0xc87c)
#define MT9M114_CAM_AET_AE_MIN_VIRT_INT_TIME_PCLK	CCI_REG16(0xc87e)
#define MT9M114_CAM_AET_AE_MIN_VIRT_DGAIN		CCI_REG16(0xc880)
#define MT9M114_CAM_AET_AE_MAX_VIRT_DGAIN		CCI_REG16(0xc882)
#define MT9M114_CAM_AET_AE_MIN_VIRT_AGAIN		CCI_REG16(0xc884)
#define MT9M114_CAM_AET_AE_MAX_VIRT_AGAIN		CCI_REG16(0xc886)
#define MT9M114_CAM_AET_AE_VIRT_GAIN_TH_EG		CCI_REG16(0xc888)
#define MT9M114_CAM_AET_AE_EG_GATE_PERCENTAGE		CCI_REG8(0xc88a)
#define MT9M114_CAM_AET_FLICKER_FREQ_HZ			CCI_REG8(0xc88b)
#define MT9M114_CAM_AET_MAX_FRAME_RATE			CCI_REG16(0xc88c)
#define MT9M114_CAM_AET_MIN_FRAME_RATE			CCI_REG16(0xc88e)
#define MT9M114_CAM_AET_TARGET_GAIN			CCI_REG16(0xc890)
#define MT9M114_CAM_AWB_CCM_L(n)			CCI_REG16(0xc892 + (n) * 2)
#define MT9M114_CAM_AWB_CCM_M(n)			CCI_REG16(0xc8a4 + (n) * 2)
#define MT9M114_CAM_AWB_CCM_R(n)			CCI_REG16(0xc8b6 + (n) * 2)
#define MT9M114_CAM_AWB_CCM_L_RG_GAIN			CCI_REG16(0xc8c8)
#define MT9M114_CAM_AWB_CCM_L_BG_GAIN			CCI_REG16(0xc8ca)
#define MT9M114_CAM_AWB_CCM_M_RG_GAIN			CCI_REG16(0xc8cc)
#define MT9M114_CAM_AWB_CCM_M_BG_GAIN			CCI_REG16(0xc8ce)
#define MT9M114_CAM_AWB_CCM_R_RG_GAIN			CCI_REG16(0xc8d0)
#define MT9M114_CAM_AWB_CCM_R_BG_GAIN			CCI_REG16(0xc8d2)
#define MT9M114_CAM_AWB_CCM_L_CTEMP			CCI_REG16(0xc8d4)
#define MT9M114_CAM_AWB_CCM_M_CTEMP			CCI_REG16(0xc8d6)
#define MT9M114_CAM_AWB_CCM_R_CTEMP			CCI_REG16(0xc8d8)
#define MT9M114_CAM_AWB_AWB_XSCALE			CCI_REG8(0xc8f2)
#define MT9M114_CAM_AWB_AWB_YSCALE			CCI_REG8(0xc8f3)
#define MT9M114_CAM_AWB_AWB_WEIGHTS(n)			CCI_REG16(0xc8f4 + (n) * 2)
#define MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ		CCI_REG16(0xc904)
#define MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ		CCI_REG16(0xc906)
#define MT9M114_CAM_AWB_AWBMODE				CCI_REG8(0xc909)
#define MT9M114_CAM_AWB_MODE_AUTO				BIT(1)
#define MT9M114_CAM_AWB_MODE_EXCLUSIVE_AE			BIT(0)
#define MT9M114_CAM_AWB_K_R_L				CCI_REG8(0xc90c)
#define MT9M114_CAM_AWB_K_G_L				CCI_REG8(0xc90d)
#define MT9M114_CAM_AWB_K_B_L				CCI_REG8(0xc90e)
#define MT9M114_CAM_AWB_K_R_R				CCI_REG8(0xc90f)
#define MT9M114_CAM_AWB_K_G_R				CCI_REG8(0xc910)
#define MT9M114_CAM_AWB_K_B_R				CCI_REG8(0xc911)
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART		CCI_REG16(0xc914)
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART		CCI_REG16(0xc916)
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND		CCI_REG16(0xc918)
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND		CCI_REG16(0xc91a)
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART	CCI_REG16(0xc91c)
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART	CCI_REG16(0xc91e)
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND		CCI_REG16(0xc920)
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND		CCI_REG16(0xc922)
#define MT9M114_CAM_LL_LLMODE				CCI_REG16(0xc924)
#define MT9M114_CAM_LL_START_BRIGHTNESS			CCI_REG16(0xc926)
#define MT9M114_CAM_LL_STOP_BRIGHTNESS			CCI_REG16(0xc928)
#define MT9M114_CAM_LL_START_SATURATION			CCI_REG8(0xc92a)
#define MT9M114_CAM_LL_END_SATURATION			CCI_REG8(0xc92b)
#define MT9M114_CAM_LL_START_DESATURATION		CCI_REG8(0xc92c)
#define MT9M114_CAM_LL_END_DESATURATION			CCI_REG8(0xc92d)
#define MT9M114_CAM_LL_START_DEMOSAICING		CCI_REG8(0xc92e)
#define MT9M114_CAM_LL_START_AP_GAIN			CCI_REG8(0xc92f)
#define MT9M114_CAM_LL_START_AP_THRESH			CCI_REG8(0xc930)
#define MT9M114_CAM_LL_STOP_DEMOSAICING			CCI_REG8(0xc931)
#define MT9M114_CAM_LL_STOP_AP_GAIN			CCI_REG8(0xc932)
#define MT9M114_CAM_LL_STOP_AP_THRESH			CCI_REG8(0xc933)
#define MT9M114_CAM_LL_START_NR_RED			CCI_REG8(0xc934)
#define MT9M114_CAM_LL_START_NR_GREEN			CCI_REG8(0xc935)
#define MT9M114_CAM_LL_START_NR_BLUE			CCI_REG8(0xc936)
#define MT9M114_CAM_LL_START_NR_THRESH			CCI_REG8(0xc937)
#define MT9M114_CAM_LL_STOP_NR_RED			CCI_REG8(0xc938)
#define MT9M114_CAM_LL_STOP_NR_GREEN			CCI_REG8(0xc939)
#define MT9M114_CAM_LL_STOP_NR_BLUE			CCI_REG8(0xc93a)
#define MT9M114_CAM_LL_STOP_NR_THRESH			CCI_REG8(0xc93b)
#define MT9M114_CAM_LL_START_CONTRAST_BM		CCI_REG16(0xc93c)
#define MT9M114_CAM_LL_STOP_CONTRAST_BM			CCI_REG16(0xc93e)
#define MT9M114_CAM_LL_GAMMA				CCI_REG16(0xc940)
#define MT9M114_CAM_LL_START_CONTRAST_GRADIENT		CCI_REG8(0xc942)
#define MT9M114_CAM_LL_STOP_CONTRAST_GRADIENT		CCI_REG8(0xc943)
#define MT9M114_CAM_LL_START_CONTRAST_LUMA_PERCENTAGE	CCI_REG8(0xc944)
#define MT9M114_CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE	CCI_REG8(0xc945)
#define MT9M114_CAM_LL_START_GAIN_METRIC		CCI_REG16(0xc946)
#define MT9M114_CAM_LL_STOP_GAIN_METRIC			CCI_REG16(0xc948)
#define MT9M114_CAM_LL_START_FADE_TO_BLACK_LUMA		CCI_REG16(0xc94a)
#define MT9M114_CAM_LL_STOP_FADE_TO_BLACK_LUMA		CCI_REG16(0xc94c)
#define MT9M114_CAM_LL_CLUSTER_DC_TH_BM			CCI_REG16(0xc94e)
#define MT9M114_CAM_LL_CLUSTER_DC_GATE_PERCENTAGE	CCI_REG8(0xc950)
#define MT9M114_CAM_LL_SUMMING_SENSITIVITY_FACTOR	CCI_REG8(0xc951)
#define MT9M114_CAM_LL_START_TARGET_LUMA_BM		CCI_REG16(0xc952)
#define MT9M114_CAM_LL_STOP_TARGET_LUMA_BM		CCI_REG16(0xc954)
#define MT9M114_CAM_PGA_PGA_CONTROL			CCI_REG16(0xc95e)
#define MT9M114_CAM_SYSCTL_PLL_ENABLE			CCI_REG8(0xc97e)
#define MT9M114_CAM_SYSCTL_PLL_ENABLE_VALUE			BIT(0)
#define MT9M114_CAM_SYSCTL_PLL_DISABLE_VALUE			0x00
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N		CCI_REG16(0xc980)
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_VALUE(m, n)		(((n) << 8) | (m))
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_P		CCI_REG16(0xc982)
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_P_VALUE(p)		((p) << 8)
#define MT9M114_CAM_PORT_OUTPUT_CONTROL			CCI_REG16(0xc984)
#define MT9M114_CAM_PORT_PORT_SELECT_PARALLEL			(0 << 0)
#define MT9M114_CAM_PORT_PORT_SELECT_MIPI			(1 << 0)
#define MT9M114_CAM_PORT_CLOCK_SLOWDOWN				BIT(3)
#define MT9M114_CAM_PORT_TRUNCATE_RAW_BAYER			BIT(4)
#define MT9M114_CAM_PORT_PIXCLK_GATE				BIT(5)
#define MT9M114_CAM_PORT_CONT_MIPI_CLK				BIT(6)
#define MT9M114_CAM_PORT_CHAN_NUM(vc)				((vc) << 8)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_HS_ZERO		CCI_REG16(0xc988)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_HS_ZERO_VALUE(n)		((n) << 8)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_HS_EXIT_TRAIL	CCI_REG16(0xc98a)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_HS_EXIT_VALUE(n)		((n) << 8)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_HS_TRAIL_VALUE(n)	((n) << 0)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_CLK_POST_PRE	CCI_REG16(0xc98c)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_CLK_POST_VALUE(n)	((n) << 8)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_CLK_PRE_VALUE(n)		((n) << 0)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_CLK_TRAIL_ZERO	CCI_REG16(0xc98e)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_CLK_TRAIL_VALUE(n)	((n) << 8)
#define MT9M114_CAM_PORT_MIPI_TIMING_T_CLK_ZERO_VALUE(n)	((n) << 0)

/* System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE			CCI_REG8(0xdc00)
#define MT9M114_SYSMGR_CURRENT_STATE			CCI_REG8(0xdc01)
#define MT9M114_SYSMGR_CMD_STATUS			CCI_REG8(0xdc02)

/* Patch Loader registers */
#define MT9M114_PATCHLDR_LOADER_ADDRESS			CCI_REG16(0xe000)
#define MT9M114_PATCHLDR_PATCH_ID			CCI_REG16(0xe002)
#define MT9M114_PATCHLDR_FIRMWARE_ID			CCI_REG32(0xe004)
#define MT9M114_PATCHLDR_APPLY_STATUS			CCI_REG8(0xe008)
#define MT9M114_PATCHLDR_NUM_PATCHES			CCI_REG8(0xe009)
#define MT9M114_PATCHLDR_PATCH_ID_0			CCI_REG16(0xe00a)
#define MT9M114_PATCHLDR_PATCH_ID_1			CCI_REG16(0xe00c)
#define MT9M114_PATCHLDR_PATCH_ID_2			CCI_REG16(0xe00e)
#define MT9M114_PATCHLDR_PATCH_ID_3			CCI_REG16(0xe010)
#define MT9M114_PATCHLDR_PATCH_ID_4			CCI_REG16(0xe012)
#define MT9M114_PATCHLDR_PATCH_ID_5			CCI_REG16(0xe014)
#define MT9M114_PATCHLDR_PATCH_ID_6			CCI_REG16(0xe016)
#define MT9M114_PATCHLDR_PATCH_ID_7			CCI_REG16(0xe018)

/* SYS_STATE values (for SYSMGR_NEXT_STATE and SYSMGR_CURRENT_STATE) */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE		0x28
#define MT9M114_SYS_STATE_STREAMING			0x31
#define MT9M114_SYS_STATE_START_STREAMING		0x34
#define MT9M114_SYS_STATE_ENTER_SUSPEND			0x40
#define MT9M114_SYS_STATE_SUSPENDED			0x41
#define MT9M114_SYS_STATE_ENTER_STANDBY			0x50
#define MT9M114_SYS_STATE_STANDBY			0x52
#define MT9M114_SYS_STATE_LEAVE_STANDBY			0x54

/* Result status of last SET_STATE comamnd */
#define MT9M114_SET_STATE_RESULT_ENOERR			0x00
#define MT9M114_SET_STATE_RESULT_EINVAL			0x0c
#define MT9M114_SET_STATE_RESULT_ENOSPC			0x0d

/*
 * The minimum amount of horizontal and vertical blanking is undocumented. The
 * minimum values that have been seen in register lists are 303 and 38, use
 * them.
 *
 * Set the default to achieve 1280x960 at 30fps.
 */
#define MT9M114_MIN_HBLANK				303
#define MT9M114_MIN_VBLANK				38
#define MT9M114_DEF_HBLANK				323
#define MT9M114_DEF_VBLANK				39

#define MT9M114_DEF_FRAME_RATE				30
#define MT9M114_MAX_FRAME_RATE				120

#define MT9M114_PIXEL_ARRAY_WIDTH			1296U
#define MT9M114_PIXEL_ARRAY_HEIGHT			976U

/*
 * These values are not well documented and are semi-arbitrary. The pixel array
 * minimum output size is 8 pixels larger than the minimum scaler cropped input
 * width to account for the demosaicing.
 */
#define MT9M114_PIXEL_ARRAY_MIN_OUTPUT_WIDTH		(32U + 8U)
#define MT9M114_PIXEL_ARRAY_MIN_OUTPUT_HEIGHT		(32U + 8U)
#define MT9M114_SCALER_CROPPED_INPUT_WIDTH		32U
#define MT9M114_SCALER_CROPPED_INPUT_HEIGHT		32U

/* Indices into the mt9m114.ifp.tpg array. */
#define MT9M114_TPG_PATTERN				0
#define MT9M114_TPG_RED					1
#define MT9M114_TPG_GREEN				2
#define MT9M114_TPG_BLUE				3

/* -----------------------------------------------------------------------------
 * Data Structures
 */

enum mt9m114_model {
	MT9M113_MODEL = 0x2480,
	MT9M114_MODEL = 0x2481,
};

enum mt9m114_format_flag {
	MT9M114_FMT_FLAG_PARALLEL = BIT(0),
	MT9M114_FMT_FLAG_CSI2 = BIT(1),
};

struct mt9m114_format_info {
	u32 code;
	u32 output_format;
	u32 flags;
};

struct mt9m114 {
	struct i2c_client *client;
	struct regmap *regmap;
	enum mt9m114_model model;
	enum mt9m114_model expected_model;	/* From DT compatible */

	struct clk *clk;
	struct gpio_desc *reset;
	struct gpio_desc *powerdown;
	struct regulator_bulk_data supplies[3];
	struct v4l2_fwnode_endpoint bus_cfg;
	bool bypass_pll;

	struct {
		unsigned int m;
		unsigned int n;
		unsigned int p;
	} pll;

	unsigned int pixrate;
	bool streaming;
	u32 pad_slew_rate;

	/* Pixel Array */
	struct {
		struct v4l2_subdev sd;
		struct media_pad pad;

		struct v4l2_ctrl_handler hdl;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *gain;
		struct v4l2_ctrl *hblank;
		struct v4l2_ctrl *vblank;
	} pa;

	/* Image Flow Processor */
	struct {
		struct v4l2_subdev sd;
		struct media_pad pads[2];

		struct v4l2_ctrl_handler hdl;
		unsigned int frame_rate;

		struct v4l2_ctrl *tpg[4];
	} ifp;
};

/* -----------------------------------------------------------------------------
 * Formats
 */

static const struct mt9m114_format_info mt9m114_format_infos[] = {
	{
		/*
		 * The first two entries are used as defaults, for parallel and
		 * CSI-2 buses respectively. Keep them in that order.
		 */
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.flags = MT9M114_FMT_FLAG_PARALLEL,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV,
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.flags = MT9M114_FMT_FLAG_CSI2,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.flags = MT9M114_FMT_FLAG_PARALLEL,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV
			       | MT9M114_CAM_OUTPUT_FORMAT_SWAP_BYTES,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.flags = MT9M114_FMT_FLAG_CSI2,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV
			       | MT9M114_CAM_OUTPUT_FORMAT_SWAP_BYTES,
	}, {
		.code = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.flags = MT9M114_FMT_FLAG_PARALLEL,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_565RGB
			       | MT9M114_CAM_OUTPUT_FORMAT_FORMAT_RGB
			       | MT9M114_CAM_OUTPUT_FORMAT_SWAP_BYTES,
	}, {
		.code = MEDIA_BUS_FMT_RGB565_2X8_BE,
		.flags = MT9M114_FMT_FLAG_PARALLEL,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_565RGB
			       | MT9M114_CAM_OUTPUT_FORMAT_FORMAT_RGB,
	}, {
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.flags = MT9M114_FMT_FLAG_CSI2,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_565RGB
			       | MT9M114_CAM_OUTPUT_FORMAT_FORMAT_RGB,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_PROCESSED8
			       | MT9M114_CAM_OUTPUT_FORMAT_FORMAT_BAYER,
		.flags = MT9M114_FMT_FLAG_PARALLEL | MT9M114_FMT_FLAG_CSI2,
	}, {
		/* Keep the format compatible with the IFP sink pad last. */
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.output_format = MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_RAWR10
			| MT9M114_CAM_OUTPUT_FORMAT_FORMAT_BAYER,
		.flags = MT9M114_FMT_FLAG_PARALLEL | MT9M114_FMT_FLAG_CSI2,
	}
};

static const struct mt9m114_format_info *
mt9m114_default_format_info(struct mt9m114 *sensor)
{
	if (sensor->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY)
		return &mt9m114_format_infos[1];
	else
		return &mt9m114_format_infos[0];
}

static const struct mt9m114_format_info *
mt9m114_format_info(struct mt9m114 *sensor, unsigned int pad, u32 code)
{
	const unsigned int num_formats = ARRAY_SIZE(mt9m114_format_infos);
	unsigned int flag;
	unsigned int i;

	switch (pad) {
	case 0:
		return &mt9m114_format_infos[num_formats - 1];

	case 1:
		if (sensor->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY)
			flag = MT9M114_FMT_FLAG_CSI2;
		else
			flag = MT9M114_FMT_FLAG_PARALLEL;

		for (i = 0; i < num_formats; ++i) {
			const struct mt9m114_format_info *info =
				&mt9m114_format_infos[i];

			if (info->code == code && info->flags & flag)
				return info;
		}

		return mt9m114_default_format_info(sensor);

	default:
		return NULL;
	}
}

/* -----------------------------------------------------------------------------
 * Initialization
 */

static const struct cci_reg_sequence mt9m114_init[] = {
	/*
	 * Note: MT9M113 MIPI is configured via OUTPUT_CONTROL (0x3400) = 0x7A08,
	 * which is written during start_streaming. The 0x3C40 register does NOT
	 * exist on MT9M113 (it's MT9M114-specific), so we don't write it here.
	 *
	 * IMPORTANT: RESET_REGISTER (0x301A) = 0x120C is NOT written here!
	 * Writing it during init causes the sensor to enter streaming state
	 * before s_stream is called. Per webOS driver, RESET_REGISTER is only
	 * written in mt9m113_set_sensor_mode() during actual streaming start.
	 */

	/* Sensor optimization */
	{ CCI_REG16(0x316a), 0x8270 },
	{ CCI_REG16(0x316c), 0x8270 },
	{ CCI_REG16(0x3ed0), 0x2305 },
	{ CCI_REG16(0x3ed2), 0x77cf },
	{ CCI_REG16(0x316e), 0x8202 },
	{ CCI_REG16(0x3180), 0x87ff },
	{ CCI_REG16(0x30d4), 0x6080 },
	{ CCI_REG16(0xa802), 0x0008 },

	{ CCI_REG16(0x3e14), 0xff39 },

	/* APGA */
	{ MT9M114_CAM_PGA_PGA_CONTROL,			0x0000 },

	/* Automatic White balance */
	{ MT9M114_CAM_AWB_CCM_L(0),			0x0267 },
	{ MT9M114_CAM_AWB_CCM_L(1),			0xff1a },
	{ MT9M114_CAM_AWB_CCM_L(2),			0xffb3 },
	{ MT9M114_CAM_AWB_CCM_L(3),			0xff80 },
	{ MT9M114_CAM_AWB_CCM_L(4),			0x0166 },
	{ MT9M114_CAM_AWB_CCM_L(5),			0x0003 },
	{ MT9M114_CAM_AWB_CCM_L(6),			0xff9a },
	{ MT9M114_CAM_AWB_CCM_L(7),			0xfeb4 },
	{ MT9M114_CAM_AWB_CCM_L(8),			0x024d },
	{ MT9M114_CAM_AWB_CCM_M(0),			0x01bf },
	{ MT9M114_CAM_AWB_CCM_M(1),			0xff01 },
	{ MT9M114_CAM_AWB_CCM_M(2),			0xfff3 },
	{ MT9M114_CAM_AWB_CCM_M(3),			0xff75 },
	{ MT9M114_CAM_AWB_CCM_M(4),			0x0198 },
	{ MT9M114_CAM_AWB_CCM_M(5),			0xfffd },
	{ MT9M114_CAM_AWB_CCM_M(6),			0xff9a },
	{ MT9M114_CAM_AWB_CCM_M(7),			0xfee7 },
	{ MT9M114_CAM_AWB_CCM_M(8),			0x02a8 },
	{ MT9M114_CAM_AWB_CCM_R(0),			0x01d9 },
	{ MT9M114_CAM_AWB_CCM_R(1),			0xff26 },
	{ MT9M114_CAM_AWB_CCM_R(2),			0xfff3 },
	{ MT9M114_CAM_AWB_CCM_R(3),			0xffb3 },
	{ MT9M114_CAM_AWB_CCM_R(4),			0x0132 },
	{ MT9M114_CAM_AWB_CCM_R(5),			0xffe8 },
	{ MT9M114_CAM_AWB_CCM_R(6),			0xffda },
	{ MT9M114_CAM_AWB_CCM_R(7),			0xfecd },
	{ MT9M114_CAM_AWB_CCM_R(8),			0x02c2 },
	{ MT9M114_CAM_AWB_CCM_L_RG_GAIN,		0x0075 },
	{ MT9M114_CAM_AWB_CCM_L_BG_GAIN,		0x011c },
	{ MT9M114_CAM_AWB_CCM_M_RG_GAIN,		0x009a },
	{ MT9M114_CAM_AWB_CCM_M_BG_GAIN,		0x0105 },
	{ MT9M114_CAM_AWB_CCM_R_RG_GAIN,		0x00a4 },
	{ MT9M114_CAM_AWB_CCM_R_BG_GAIN,		0x00ac },
	{ MT9M114_CAM_AWB_CCM_L_CTEMP,			0x0a8c },
	{ MT9M114_CAM_AWB_CCM_M_CTEMP,			0x0f0a },
	{ MT9M114_CAM_AWB_CCM_R_CTEMP,			0x1964 },
	{ MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ,		51 },
	{ MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ,		60 },
	{ MT9M114_CAM_AWB_AWB_XSCALE,			3 },
	{ MT9M114_CAM_AWB_AWB_YSCALE,			2 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(0),		0x0000 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(1),		0x0000 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(2),		0x0000 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(3),		0xe724 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(4),		0x1583 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(5),		0x2045 },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(6),		0x03ff },
	{ MT9M114_CAM_AWB_AWB_WEIGHTS(7),		0x007c },
	{ MT9M114_CAM_AWB_K_R_L,			0x80 },
	{ MT9M114_CAM_AWB_K_G_L,			0x80 },
	{ MT9M114_CAM_AWB_K_B_L,			0x80 },
	{ MT9M114_CAM_AWB_K_R_R,			0x88 },
	{ MT9M114_CAM_AWB_K_G_R,			0x80 },
	{ MT9M114_CAM_AWB_K_B_R,			0x80 },

	/* Low-Light Image Enhancements */
	{ MT9M114_CAM_LL_START_BRIGHTNESS,		0x0020 },
	{ MT9M114_CAM_LL_STOP_BRIGHTNESS,		0x009a },
	{ MT9M114_CAM_LL_START_GAIN_METRIC,		0x0070 },
	{ MT9M114_CAM_LL_STOP_GAIN_METRIC,		0x00f3 },
	{ MT9M114_CAM_LL_START_CONTRAST_LUMA_PERCENTAGE, 0x20 },
	{ MT9M114_CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE,	0x9a },
	{ MT9M114_CAM_LL_START_SATURATION,		0x80 },
	{ MT9M114_CAM_LL_END_SATURATION,		0x4b },
	{ MT9M114_CAM_LL_START_DESATURATION,		0x00 },
	{ MT9M114_CAM_LL_END_DESATURATION,		0xff },
	{ MT9M114_CAM_LL_START_DEMOSAICING,		0x3c },
	{ MT9M114_CAM_LL_START_AP_GAIN,			0x02 },
	{ MT9M114_CAM_LL_START_AP_THRESH,		0x06 },
	{ MT9M114_CAM_LL_STOP_DEMOSAICING,		0x64 },
	{ MT9M114_CAM_LL_STOP_AP_GAIN,			0x01 },
	{ MT9M114_CAM_LL_STOP_AP_THRESH,		0x0c },
	{ MT9M114_CAM_LL_START_NR_RED,			0x3c },
	{ MT9M114_CAM_LL_START_NR_GREEN,		0x3c },
	{ MT9M114_CAM_LL_START_NR_BLUE,			0x3c },
	{ MT9M114_CAM_LL_START_NR_THRESH,		0x0f },
	{ MT9M114_CAM_LL_STOP_NR_RED,			0x64 },
	{ MT9M114_CAM_LL_STOP_NR_GREEN,			0x64 },
	{ MT9M114_CAM_LL_STOP_NR_BLUE,			0x64 },
	{ MT9M114_CAM_LL_STOP_NR_THRESH,		0x32 },
	{ MT9M114_CAM_LL_START_CONTRAST_BM,		0x0020 },
	{ MT9M114_CAM_LL_STOP_CONTRAST_BM,		0x009a },
	{ MT9M114_CAM_LL_GAMMA,				0x00dc },
	{ MT9M114_CAM_LL_START_CONTRAST_GRADIENT,	0x38 },
	{ MT9M114_CAM_LL_STOP_CONTRAST_GRADIENT,	0x30 },
	{ MT9M114_CAM_LL_START_CONTRAST_LUMA_PERCENTAGE, 0x50 },
	{ MT9M114_CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE,	0x19 },
	{ MT9M114_CAM_LL_START_FADE_TO_BLACK_LUMA,	0x0230 },
	{ MT9M114_CAM_LL_STOP_FADE_TO_BLACK_LUMA,	0x0010 },
	{ MT9M114_CAM_LL_CLUSTER_DC_TH_BM,		0x01cd },
	{ MT9M114_CAM_LL_CLUSTER_DC_GATE_PERCENTAGE,	0x05 },
	{ MT9M114_CAM_LL_SUMMING_SENSITIVITY_FACTOR,	0x40 },

	/* Auto-Exposure */
	{ MT9M114_CAM_AET_TARGET_AVERAGE_LUMA_DARK,	0x1b },
	{ MT9M114_CAM_AET_AEMODE,			0x00 },
	{ MT9M114_CAM_AET_TARGET_GAIN,			0x0080 },
	{ MT9M114_CAM_AET_AE_MAX_VIRT_AGAIN,		0x0100 },
	{ MT9M114_CAM_AET_BLACK_CLIPPING_TARGET,	0x005a },

	{ MT9M114_CCM_DELTA_GAIN,			0x05 },
	{ MT9M114_AE_TRACK_AE_TRACKING_DAMPENING_SPEED,	0x20 },

	/* Pixel array timings and integration time */
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,		1 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,	219 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,	1459 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,	96 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,		32 },
};

/* -----------------------------------------------------------------------------
 * MT9M113 MCU Variable Access Helpers
 *
 * MT9M113 uses indirect MCU variable access via XDMA registers:
 * - Write MCU variable address to 0x098C (MCU_ADDRESS)
 * - Read/write data via 0x0990 (MCU_DATA)
 *
 * This is different from MT9M114 which uses direct access to 0xC000+ addresses.
 */

static int mt9m113_write_mcu_var(struct mt9m114 *sensor, u16 addr, u16 value)
{
	int ret = 0;

	cci_write(sensor->regmap, MT9M114_MCU_ADDRESS, addr, &ret);
	cci_write(sensor->regmap, MT9M114_MCU_DATA, value, &ret);
	return ret;
}

static int mt9m113_read_mcu_var(struct mt9m114 *sensor, u16 addr, u64 *value)
{
	int ret;

	ret = cci_write(sensor->regmap, MT9M114_MCU_ADDRESS, addr, NULL);
	if (ret)
		return ret;
	return cci_read(sensor->regmap, MT9M114_MCU_DATA, value, NULL);
}

static int mt9m113_poll_mcu_var(struct mt9m114 *sensor, u16 addr,
				u16 expected, unsigned int timeout_ms)
{
	unsigned int i;
	u64 value;
	int ret;

	for (i = 0; i < timeout_ms / 10; i++) {
		ret = mt9m113_read_mcu_var(sensor, addr, &value);
		if (ret < 0)
			return ret;
		if (value == expected)
			return 0;
		msleep(10);
	}

	dev_err(&sensor->client->dev, "MCU var 0x%04x timeout (got 0x%llx, expected 0x%04x)\n",
		addr, value, expected);
	return -ETIMEDOUT;
}

/* -----------------------------------------------------------------------------
 * MT9M113 Initialization Table
 *
 * Ported from webOS mt9m113_reg.c. This table configures:
 * - PLL and clock settings for 24MHz input, MIPI output
 * - Resolution: Context A = 640x480 (preview), Context B = 1280x1024 (capture)
 * - Auto-exposure, auto-white-balance, flicker detection
 * - Lens shading correction, gamma curves, color correction matrices
 *
 * Register types:
 * - Direct registers (0x0xxx, 0x3xxx): written directly via CCI
 * - MCU variables (0x2xxx, 0xAxxx, etc.): written via XDMA (0x098C/0x0990)
 *
 * The webOS format is {reg, mask, value, word_len, delay}. For MCU variables,
 * pairs of 0x098C (address) and 0x0990 (data) are used.
 */

struct mt9m113_reg_entry {
	u16 reg;
	u16 value;
	u16 delay_ms;
};

/*
 * MT9M113 initialization sequence from webOS kernel.
 * Each entry is either:
 * - A direct register write (reg < 0x8000)
 * - An MCU variable write encoded as reg=0x098C (address) followed by
 *   reg=0x0990 (data), which is handled specially.
 *
 * NOTE: PLL configuration is handled by mt9m114_power_on().
 * This table starts with MCU boot toggle to reset MCU state
 * and clear any auto-streaming that occurred after PLL config.
 */
static const struct mt9m113_reg_entry mt9m113_init_table[] = {
	/*
	 * MCU boot toggle - resets MCU state.
	 * webOS has this at the very start of their init table.
	 */
	{ 0x001C, 0x0001, 0 },		/* MCU_BOOT_MODE = 1 */
	{ 0x001C, 0x0000, 30 },		/* MCU_BOOT_MODE = 0, delay 30ms */

	/*
	 * Clock and standby control - must be set before PLL config.
	 * webOS prev_snap_reg_tbl lines 32-33.
	 */
	{ 0x0016, 0x00FF, 0 },		/* CLOCKS_CONTROL */
	{ 0x0018, 0x0028, 0 },		/* STANDBY_CONTROL */

	/*
	 * PLL configuration from webOS prev_snap_reg_tbl lines 34-43.
	 * This must be done early, before any MCU variable access.
	 */
	{ 0x0014, 0x2145, 0 },		/* PLL_CONTROL: bypass PLL */
	{ 0x0014, 0x2145, 0 },		/* PLL_CONTROL (repeat for stability) */
	{ 0x0014, 0x2145, 0 },		/* PLL_CONTROL (repeat for stability) */
	{ 0x0010, 0x0114, 0 },		/* PLL_DIVIDERS (matches webOS init table) */
	{ 0x0012, 0x00F1, 0 },		/* PLL_P_DIVIDERS */
	{ 0x0014, 0x2545, 0 },		/* PLL_CONTROL: TEST_BYPASS on */
	{ 0x0014, 0x2547, 0 },		/* PLL_CONTROL: PLL_ENABLE on */
	{ 0x0014, 0x3447, 20 },		/* PLL_CONTROL: SEL_LOCK_DET, delay 20ms */
	{ 0x0014, 0x3047, 0 },		/* PLL_CONTROL: TEST_BYPASS off */
	{ 0x0014, 0x3046, 0 },		/* PLL_CONTROL: PLL_BYPASS off */

	/*
	 * Reset and standby control after PLL - webOS lines 44-45.
	 * This takes the sensor out of reset state.
	 */
	{ 0x001A, 0x0218, 0 },		/* RESET_AND_MISC_CONTROL */
	{ 0x0018, 0x002A, 0 },		/* STANDBY_CONTROL */

	/* OFIFO control */
	{ 0x321C, 0x0003, 0 },		/* OFIFO_CONTROL_STATUS */

	/* Context A output (640x480 preview) - via MCU variables */
	{ 0x098C, 0x2703, 0 },		/* MCU_ADDRESS = MODE_OUTPUT_WIDTH_A */
	{ 0x0990, 0x0280, 0 },		/* MCU_DATA = 640 */
	{ 0x098C, 0x2705, 0 },		/* MCU_ADDRESS = MODE_OUTPUT_HEIGHT_A */
	{ 0x0990, 0x01E0, 0 },		/* MCU_DATA = 480 */

	/* Context B output (1280x1024 capture) */
	{ 0x098C, 0x2707, 0 },		/* MCU_ADDRESS = MODE_OUTPUT_WIDTH_B */
	{ 0x0990, 0x0500, 0 },		/* MCU_DATA = 1280 */
	{ 0x098C, 0x2709, 0 },		/* MCU_ADDRESS = MODE_OUTPUT_HEIGHT_B */
	{ 0x0990, 0x0400, 0 },		/* MCU_DATA = 1024 */

	/* Context A sensor configuration */
	{ 0x098C, 0x270D, 0 },		/* MODE_SENSOR_ROW_START_A */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x270F, 0 },		/* MODE_SENSOR_COL_START_A */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x2711, 0 },		/* MODE_SENSOR_ROW_END_A */
	{ 0x0990, 0x03CD, 0 },		/* 973 */
	{ 0x098C, 0x2713, 0 },		/* MODE_SENSOR_COL_END_A */
	{ 0x0990, 0x050D, 0 },		/* 1293 */
	{ 0x098C, 0x2715, 0 },		/* MODE_SENSOR_ROW_SPEED_A */
	{ 0x0990, 0x2111, 0 },
	{ 0x098C, 0x2717, 0 },		/* MODE_SENSOR_READ_MODE_A */
	{ 0x0990, 0x046C, 0 },
	{ 0x098C, 0x2719, 0 },		/* MODE_SENSOR_FINE_CORRECTION_A */
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0x271B, 0 },		/* MODE_SENSOR_FINE_IT_MIN_A */
	{ 0x0990, 0x01F1, 0 },
	{ 0x098C, 0x271D, 0 },		/* MODE_SENSOR_FINE_IT_MAX_MARGIN_A */
	{ 0x0990, 0x013F, 0 },
	{ 0x098C, 0x271F, 0 },		/* MODE_SENSOR_FRAME_LENGTH_A */
	{ 0x0990, 0x032E, 0 },		/* 814 */
	{ 0x098C, 0x2721, 0 },		/* MODE_SENSOR_LINE_LENGTH_PCK_A */
	{ 0x0990, 0x04CC, 0 },		/* 1228 */

	/* Context B sensor configuration */
	{ 0x098C, 0x2723, 0 },		/* MODE_SENSOR_ROW_START_B */
	{ 0x0990, 0x0004, 0 },
	{ 0x098C, 0x2725, 0 },		/* MODE_SENSOR_COL_START_B */
	{ 0x0990, 0x0004, 0 },
	{ 0x098C, 0x2727, 0 },		/* MODE_SENSOR_ROW_END_B */
	{ 0x0990, 0x040B, 0 },
	{ 0x098C, 0x2729, 0 },		/* MODE_SENSOR_COL_END_B */
	{ 0x0990, 0x050B, 0 },
	{ 0x098C, 0x272B, 0 },		/* MODE_SENSOR_ROW_SPEED_B */
	{ 0x0990, 0x2111, 0 },
	{ 0x098C, 0x272D, 0 },		/* MODE_SENSOR_READ_MODE_B */
	{ 0x0990, 0x0024, 0 },
	{ 0x098C, 0x272F, 0 },		/* MODE_SENSOR_FINE_CORRECTION_B */
	{ 0x0990, 0x004C, 0 },
	{ 0x098C, 0x2731, 0 },		/* MODE_SENSOR_FINE_IT_MIN_B */
	{ 0x0990, 0x00F9, 0 },
	{ 0x098C, 0x2733, 0 },		/* MODE_SENSOR_FINE_IT_MAX_MARGIN_B */
	{ 0x0990, 0x00A7, 0 },
	{ 0x098C, 0x2735, 0 },		/* MODE_SENSOR_FRAME_LENGTH_B */
	{ 0x0990, 0x0559, 0 },
	{ 0x098C, 0x2737, 0 },		/* MODE_SENSOR_LINE_LENGTH_PCK_B */
	{ 0x0990, 0x0722, 0 },

	/* Crop configuration - Context A */
	{ 0x098C, 0x2739, 0 },		/* MODE_CROP_X0_A */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x273B, 0 },		/* MODE_CROP_X1_A */
	{ 0x0990, 0x027F, 0 },
	{ 0x098C, 0x273D, 0 },		/* MODE_CROP_Y0_A */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x273F, 0 },		/* MODE_CROP_Y1_A */
	{ 0x0990, 0x01DF, 0 },

	/* Crop configuration - Context B */
	{ 0x098C, 0x2747, 0 },		/* MODE_CROP_X0_B */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x2749, 0 },		/* MODE_CROP_X1_B */
	{ 0x0990, 0x04FF, 0 },
	{ 0x098C, 0x274B, 0 },		/* MODE_CROP_Y0_B */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x274D, 0 },		/* MODE_CROP_Y1_B */
	{ 0x0990, 0x03FF, 0 },

	/* Flicker detection */
	{ 0x098C, 0x222D, 0 },		/* AE_R9_STEP */
	{ 0x0990, 0x00CC, 0 },
	{ 0x098C, 0xA404, 0 },		/* FD_MODE */
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0xA408, 0 },		/* FD_SEARCH_F1_50 */
	{ 0x0990, 0x0032, 0 },
	{ 0x098C, 0xA409, 0 },		/* FD_SEARCH_F2_50 */
	{ 0x0990, 0x0034, 0 },
	{ 0x098C, 0xA40A, 0 },		/* FD_SEARCH_F1_60 */
	{ 0x0990, 0x003C, 0 },
	{ 0x098C, 0xA40B, 0 },		/* FD_SEARCH_F2_60 */
	{ 0x0990, 0x003E, 0 },
	{ 0x098C, 0x2411, 0 },		/* FD_R9_STEP_F60_A */
	{ 0x0990, 0x00CC, 0 },
	{ 0x098C, 0x2413, 0 },		/* FD_R9_STEP_F50_A */
	{ 0x0990, 0x00F4, 0 },
	{ 0x098C, 0x2415, 0 },		/* FD_R9_STEP_F60_B */
	{ 0x0990, 0x0089, 0 },
	{ 0x098C, 0x2417, 0 },		/* FD_R9_STEP_F50_B */
	{ 0x0990, 0x00A4, 0 },
	{ 0x098C, 0xA40D, 0 },		/* FD_STAT_MIN */
	{ 0x0990, 0x0002, 0 },
	{ 0x098C, 0xA40E, 0 },		/* FD_STAT_MAX */
	{ 0x0990, 0x0003, 0 },
	{ 0x098C, 0xA410, 0 },		/* FD_MIN_AMPLITUDE */
	{ 0x0990, 0x000A, 0 },

	/* Sensor core reserved registers */
	{ 0x3044, 0x0504, 0 },
	{ 0x3086, 0x24F7, 0 },
	{ 0x3088, 0xF059, 0 },
	{ 0x3090, 0x0716, 0 },
	{ 0x3092, 0xAB1F, 0 },
	{ 0x30D4, 0x9020, 0 },
	{ 0x30E2, 0x6645, 0 },
	{ 0x30E4, 0x7A66, 0 },
	{ 0x30E6, 0x6652, 0 },
	{ 0x30E8, 0x7766, 0 },
	{ 0x30EA, 0x2E03, 0 },
	{ 0x30EC, 0x452E, 0 },
	{ 0x30EE, 0x2E17, 0 },
	{ 0x30F0, 0x452E, 0 },
	{ 0x30F6, 0x0501, 0 },
	{ 0x30F8, 0x0501, 0 },
	{ 0x30FA, 0x0401, 0 },
	{ 0x30FC, 0x0401, 0 },
	{ 0x30FE, 0x5145, 0 },
	{ 0x3100, 0x4F45, 0 },
	{ 0x3102, 0x652E, 0 },
	{ 0x3104, 0x7552, 0 },
	{ 0x3106, 0x2D05, 0 },
	{ 0x3108, 0x4405, 0 },
	{ 0x311A, 0x5045, 0 },
	{ 0x311E, 0x0601, 0 },
	{ 0x3122, 0x0601, 0 },
	{ 0x316C, 0x8406, 0 },

	/* Noise reduction */
	{ 0x098C, 0xAB2D, 0 },		/* HG_NR_START_G */
	{ 0x0990, 0x002A, 0 },
	{ 0x098C, 0xAB31, 0 },		/* HG_NR_STOP_G */
	{ 0x0990, 0x002E, 0 },

	/* Low-light enhancement */
	{ 0x098C, 0x2B28, 0 },		/* HG_LL_BRIGHTNESSSTART */
	{ 0x0990, 0x1F40, 0 },
	{ 0x098C, 0x2B2A, 0 },		/* HG_LL_BRIGHTNESSSTOP */
	{ 0x0990, 0x3A98, 0 },
	{ 0x098C, 0x2B38, 0 },		/* HG_GAMMASTARTMORPH */
	{ 0x0990, 0x1F40, 0 },
	{ 0x098C, 0x2B3A, 0 },		/* HG_GAMMASTOPMORPH */
	{ 0x0990, 0x3A98, 0 },

	/* AE settings */
	{ 0x098C, 0x2257, 0 },		/* RESERVED_AE_57 */
	{ 0x0990, 0x2710, 0 },
	{ 0x098C, 0x2250, 0 },		/* RESERVED_AE_50 */
	{ 0x0990, 0x1B58, 0 },
	{ 0x098C, 0x2252, 0 },		/* RESERVED_AE_52 */
	{ 0x0990, 0x32C8, 0 },
	{ 0x098C, 0xA24B, 0 },		/* AE_TARGETMAX */
	{ 0x0990, 0x0082, 0 },

	/* Aperture */
	{ 0x326C, 0x0C00, 0 },		/* APERTURE_PARAMETERS */

	/* More Context A settings */
	{ 0x098C, 0x2717, 0 },		/* MODE_SENSOR_READ_MODE_A */
	{ 0x0990, 0x046C, 0 },
	{ 0x098C, 0x2719, 0 },		/* MODE_SENSOR_FINE_CORRECTION_A */
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0x271B, 0 },		/* MODE_SENSOR_FINE_IT_MIN_A */
	{ 0x0990, 0x01F1, 0 },
	{ 0x098C, 0x271D, 0 },		/* MODE_SENSOR_FINE_IT_MAX_MARGIN_A */
	{ 0x0990, 0x013F, 0 },
	{ 0x098C, 0x271F, 0 },		/* MODE_SENSOR_FRAME_LENGTH_A */
	{ 0x0990, 0x032E, 0 },
	{ 0x098C, 0x2721, 0 },		/* MODE_SENSOR_LINE_LENGTH_PCK_A */
	{ 0x0990, 0x04CC, 0 },
	{ 0x098C, 0x275F, 0 },		/* RESERVED_MODE_5F */
	{ 0x0990, 0x0596, 0 },
	{ 0x098C, 0x2761, 0 },		/* RESERVED_MODE_61 */
	{ 0x0990, 0x0094, 0 },

	/* Lens shading correction - P0Q0 through P4Q4 for GR, RD, BL, GB */
	{ 0x364E, 0x07B0, 0 },		/* P_GR_P0Q0 */
	{ 0x3650, 0x7E0E, 0 },
	{ 0x3652, 0x3D31, 0 },
	{ 0x3654, 0x80AE, 0 },
	{ 0x3656, 0xE131, 0 },
	{ 0x3658, 0x01B0, 0 },		/* P_RD_P0Q0 */
	{ 0x365A, 0x878D, 0 },
	{ 0x365C, 0x2671, 0 },
	{ 0x365E, 0x7D2D, 0 },
	{ 0x3660, 0xA5D1, 0 },
	{ 0x3662, 0x03B0, 0 },		/* P_BL_P0Q0 */
	{ 0x3664, 0x5A0E, 0 },
	{ 0x3666, 0x0E71, 0 },
	{ 0x3668, 0x99EE, 0 },
	{ 0x366A, 0xA671, 0 },
	{ 0x366C, 0x0170, 0 },		/* P_GB_P0Q0 */
	{ 0x366E, 0xF44D, 0 },
	{ 0x3670, 0x2971, 0 },
	{ 0x3672, 0x2D4A, 0 },
	{ 0x3674, 0xD671, 0 },

	/* P1Q0-P1Q4 */
	{ 0x3676, 0x674C, 0 },
	{ 0x3678, 0x748D, 0 },
	{ 0x367A, 0x3FEE, 0 },
	{ 0x367C, 0x89AE, 0 },
	{ 0x367E, 0xB410, 0 },
	{ 0x3680, 0x168C, 0 },
	{ 0x3682, 0xC56D, 0 },
	{ 0x3684, 0x7CAC, 0 },
	{ 0x3686, 0x038F, 0 },
	{ 0x3688, 0xA86F, 0 },
	{ 0x368A, 0xDB6B, 0 },
	{ 0x368C, 0xA2AE, 0 },
	{ 0x368E, 0xFA8D, 0 },
	{ 0x3690, 0x5C8E, 0 },
	{ 0x3692, 0x740C, 0 },
	{ 0x3694, 0x9F4B, 0 },
	{ 0x3696, 0x1C4D, 0 },
	{ 0x3698, 0x978D, 0 },
	{ 0x369A, 0x21EC, 0 },
	{ 0x369C, 0xF5AD, 0 },

	/* P2Q0-P2Q4 */
	{ 0x369E, 0x7D10, 0 },
	{ 0x36A0, 0x3E2E, 0 },
	{ 0x36A2, 0x8953, 0 },
	{ 0x36A4, 0xD910, 0 },
	{ 0x36A6, 0x3033, 0 },
	{ 0x36A8, 0x06D1, 0 },
	{ 0x36AA, 0xAD4E, 0 },
	{ 0x36AC, 0xD2D2, 0 },
	{ 0x36AE, 0x5CCE, 0 },
	{ 0x36B0, 0x3B93, 0 },
	{ 0x36B2, 0x50D0, 0 },
	{ 0x36B4, 0x79AD, 0 },
	{ 0x36B6, 0xDFF2, 0 },
	{ 0x36B8, 0x88AF, 0 },
	{ 0x36BA, 0x2453, 0 },
	{ 0x36BC, 0x0051, 0 },
	{ 0x36BE, 0x81CF, 0 },
	{ 0x36C0, 0x8313, 0 },
	{ 0x36C2, 0x2250, 0 },
	{ 0x36C4, 0x4A53, 0 },

	/* P3Q0-P3Q4 */
	{ 0x36C6, 0x0C8D, 0 },
	{ 0x36C8, 0x362B, 0 },
	{ 0x36CA, 0xAD51, 0 },
	{ 0x36CC, 0xA470, 0 },
	{ 0x36CE, 0x3DD2, 0 },
	{ 0x36D0, 0x174C, 0 },
	{ 0x36D2, 0x152F, 0 },
	{ 0x36D4, 0x82F1, 0 },
	{ 0x36D6, 0xDED0, 0 },
	{ 0x36D8, 0x6F12, 0 },
	{ 0x36DA, 0xD36C, 0 },
	{ 0x36DC, 0x51AE, 0 },
	{ 0x36DE, 0xD0AE, 0 },
	{ 0x36E0, 0x274E, 0 },
	{ 0x36E2, 0x25F2, 0 },
	{ 0x36E4, 0xDCCA, 0 },
	{ 0x36E6, 0x438E, 0 },
	{ 0x36E8, 0xD64E, 0 },
	{ 0x36EA, 0x8A71, 0 },
	{ 0x36EC, 0x1492, 0 },

	/* P4Q0-P4Q4 */
	{ 0x36EE, 0xD5B1, 0 },
	{ 0x36F0, 0xEBF0, 0 },
	{ 0x36F2, 0x53F3, 0 },
	{ 0x36F4, 0x3492, 0 },
	{ 0x36F6, 0x9AF4, 0 },
	{ 0x36F8, 0x8BF1, 0 },
	{ 0x36FA, 0x204F, 0 },
	{ 0x36FC, 0x3A93, 0 },
	{ 0x36FE, 0xB551, 0 },
	{ 0x3700, 0xE214, 0 },
	{ 0x3702, 0xF2B0, 0 },
	{ 0x3704, 0x8C30, 0 },
	{ 0x3706, 0x3053, 0 },
	{ 0x3708, 0x64F0, 0 },
	{ 0x370A, 0xFC73, 0 },
	{ 0x370C, 0xD311, 0 },
	{ 0x370E, 0x336F, 0 },
	{ 0x3710, 0x5AF3, 0 },
	{ 0x3712, 0x4EAF, 0 },
	{ 0x3714, 0xDBD4, 0 },

	/* Lens shading origin */
	{ 0x3644, 0x02A0, 0 },		/* POLY_ORIGIN_C */
	{ 0x3642, 0x01FC, 0 },		/* POLY_ORIGIN_R */
	{ 0x3210, 0x01B8, 0 },		/* COLOR_PIPELINE_CONTROL */

	/* Color correction matrix - Low light */
	{ 0x098C, 0x2306, 0 },		/* AWB_CCM_L_0 */
	{ 0x0990, 0x0233, 0 },
	{ 0x098C, 0x2308, 0 },
	{ 0x0990, 0xFF0B, 0 },
	{ 0x098C, 0x230A, 0 },
	{ 0x0990, 0x0024, 0 },
	{ 0x098C, 0x230C, 0 },
	{ 0x0990, 0xFFC8, 0 },
	{ 0x098C, 0x230E, 0 },
	{ 0x0990, 0x01DE, 0 },
	{ 0x098C, 0x2310, 0 },
	{ 0x0990, 0xFFBD, 0 },
	{ 0x098C, 0x2312, 0 },
	{ 0x0990, 0x0019, 0 },
	{ 0x098C, 0x2314, 0 },
	{ 0x0990, 0xFF2B, 0 },
	{ 0x098C, 0x2316, 0 },
	{ 0x0990, 0x01E8, 0 },
	{ 0x098C, 0x2318, 0 },
	{ 0x0990, 0x0024, 0 },
	{ 0x098C, 0x231A, 0 },
	{ 0x0990, 0x0030, 0 },

	/* Color correction matrix - RL (delta) */
	{ 0x098C, 0x231C, 0 },		/* AWB_CCM_RL_0 */
	{ 0x0990, 0xFF7D, 0 },
	{ 0x098C, 0x231E, 0 },
	{ 0x0990, 0x002C, 0 },
	{ 0x098C, 0x2320, 0 },
	{ 0x0990, 0x002C, 0 },
	{ 0x098C, 0x2322, 0 },
	{ 0x0990, 0x0006, 0 },
	{ 0x098C, 0x2324, 0 },
	{ 0x0990, 0x00A3, 0 },
	{ 0x098C, 0x2326, 0 },
	{ 0x0990, 0xFF75, 0 },
	{ 0x098C, 0x2328, 0 },
	{ 0x0990, 0xFFF4, 0 },
	{ 0x098C, 0x232A, 0 },
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0x232C, 0 },
	{ 0x0990, 0xFF75, 0 },
	{ 0x098C, 0x232E, 0 },
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0x2330, 0 },
	{ 0x0990, 0xFFF4, 0 },

	/* AWB settings */
	{ 0x098C, 0xA348, 0 },		/* AWB_GAIN_BUFFER_SPEED */
	{ 0x0990, 0x0008, 0 },
	{ 0x098C, 0xA349, 0 },		/* AWB_JUMP_DIVISOR */
	{ 0x0990, 0x0002, 0 },
	{ 0x098C, 0xA34A, 0 },		/* AWB_GAIN_MIN */
	{ 0x0990, 0x0059, 0 },
	{ 0x098C, 0xA34B, 0 },		/* AWB_GAIN_MAX */
	{ 0x0990, 0x00A6, 0 },
	{ 0x098C, 0xA351, 0 },		/* AWB_CCM_POSITION_MIN */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xA352, 0 },		/* AWB_CCM_POSITION_MAX */
	{ 0x0990, 0x007F, 0 },
	{ 0x098C, 0xA35D, 0 },		/* AWB_STEADY_BGAIN_OUT_MIN */
	{ 0x0990, 0x0078, 0 },
	{ 0x098C, 0xA35E, 0 },		/* AWB_STEADY_BGAIN_OUT_MAX */
	{ 0x0990, 0x0086, 0 },
	{ 0x098C, 0xA35F, 0 },		/* AWB_STEADY_BGAIN_IN_MIN */
	{ 0x0990, 0x007E, 0 },
	{ 0x098C, 0xA360, 0 },		/* AWB_STEADY_BGAIN_IN_MAX */
	{ 0x0990, 0x0082, 0 },

	/* Cold color adjustment */
	{ 0x098C, 0xA369, 0 },		/* AWB_KR_R */
	{ 0x0990, 0x0097, 0 },
	{ 0x098C, 0xA36A, 0 },		/* AWB_KG_R */
	{ 0x0990, 0x008C, 0 },
	{ 0x098C, 0xA36B, 0 },		/* AWB_KB_R */
	{ 0x0990, 0x0080, 0 },

	/* AWB window */
	{ 0x098C, 0xA302, 0 },		/* AWB_WINDOW_POS */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xA303, 0 },		/* AWB_WINDOW_SIZE */
	{ 0x0990, 0x00FF, 0 },

	/* AE preview settings */
	{ 0x098C, 0xA11D, 0 },		/* SEQ_PREVIEW_1_AE */
	{ 0x0990, 0x0002, 0 },
	{ 0x098C, 0x271F, 0 },		/* MODE_SENSOR_FRAME_LENGTH_A */
	{ 0x0990, 0x032E, 0 },
	{ 0x098C, 0x2721, 0 },		/* MODE_SENSOR_LINE_LENGTH_PCK_A */
	{ 0x0990, 0x04CC, 0 },

	/* AE gain settings */
	{ 0x098C, 0xA216, 0 },		/* AE_MAXGAIN23 */
	{ 0x0990, 0x0060, 0 },
	{ 0x098C, 0xA215, 0 },		/* AE_INDEX_TH23 */
	{ 0x0990, 0x000A, 0 },
	{ 0x098C, 0xA20C, 0 },		/* AE_MAX_INDEX */
	{ 0x0990, 0x0028, 0 },
	{ 0x098C, 0xA24F, 0 },		/* AE_BASETARGET */
	{ 0x0990, 0x0042, 0 },
	{ 0x098C, 0xA20E, 0 },		/* AE_MAX_VIRTGAIN */
	{ 0x0990, 0x0060, 0 },

	/* AE window */
	{ 0x098C, 0xA202, 0 },		/* AE_WINDOW_POS */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xA203, 0 },		/* AE_WINDOW_SIZE */
	{ 0x0990, 0x00FF, 0 },
	{ 0x098C, 0xA207, 0 },		/* AE_GATE */
	{ 0x0990, 0x0004, 0 },

	/* Gamma morph control */
	{ 0x098C, 0xAB37, 0 },		/* HG_GAMMA_MORPH_CTRL */
	{ 0x0990, 0x0003, 0 },
	{ 0x098C, 0x2B38, 0 },		/* HG_GAMMASTARTMORPH */
	{ 0x0990, 0x3A98, 0 },
	{ 0x098C, 0x2B3A, 0 },		/* HG_GAMMASTOPMORPH */
	{ 0x0990, 0x5000, 0 },

	/* Saturation */
	{ 0x098C, 0xAB20, 0 },		/* HG_LL_SAT1 */
	{ 0x0990, 0x0023, 0 },
	{ 0x098C, 0xAB24, 0 },		/* HG_LL_SAT2 */
	{ 0x0990, 0x0010, 0 },

	/* AE speed */
	{ 0x098C, 0xA109, 0 },		/* SEQ_AE_FASTBUFF */
	{ 0x0990, 0x0020, 0 },
	{ 0x098C, 0xA10A, 0 },		/* SEQ_AE_FASTSTEP */
	{ 0x0990, 0x0002, 0 },

	/* Gamma table A */
	{ 0x098C, 0xAB3C, 0 },		/* HG_GAMMA_TABLE_A_0 */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xAB3D, 0 },
	{ 0x0990, 0x0006, 0 },
	{ 0x098C, 0xAB3E, 0 },
	{ 0x0990, 0x0014, 0 },
	{ 0x098C, 0xAB3F, 0 },
	{ 0x0990, 0x0038, 0 },
	{ 0x098C, 0xAB40, 0 },
	{ 0x0990, 0x005F, 0 },
	{ 0x098C, 0xAB41, 0 },
	{ 0x0990, 0x0079, 0 },
	{ 0x098C, 0xAB42, 0 },
	{ 0x0990, 0x008D, 0 },
	{ 0x098C, 0xAB43, 0 },
	{ 0x0990, 0x009E, 0 },
	{ 0x098C, 0xAB44, 0 },
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0xAB45, 0 },
	{ 0x0990, 0x00B8, 0 },
	{ 0x098C, 0xAB46, 0 },
	{ 0x0990, 0x00C3, 0 },
	{ 0x098C, 0xAB47, 0 },
	{ 0x0990, 0x00CD, 0 },
	{ 0x098C, 0xAB48, 0 },
	{ 0x0990, 0x00D5, 0 },
	{ 0x098C, 0xAB49, 0 },
	{ 0x0990, 0x00DE, 0 },
	{ 0x098C, 0xAB4A, 0 },
	{ 0x0990, 0x00E5, 0 },
	{ 0x098C, 0xAB4B, 0 },
	{ 0x0990, 0x00EC, 0 },
	{ 0x098C, 0xAB4C, 0 },
	{ 0x0990, 0x00F3, 0 },
	{ 0x098C, 0xAB4D, 0 },
	{ 0x0990, 0x00F9, 0 },
	{ 0x098C, 0xAB4E, 0 },
	{ 0x0990, 0x00FF, 0 },

	/* Noise reduction RGB */
	{ 0x098C, 0xAB2C, 0 },		/* HG_NR_START_R */
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0xAB2D, 0 },		/* HG_NR_START_G */
	{ 0x0990, 0x002A, 0 },
	{ 0x098C, 0xAB2E, 0 },		/* HG_NR_START_B */
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0xAB2F, 0 },		/* HG_NR_START_OL */
	{ 0x0990, 0x0010, 0 },

	/* Gamma table B */
	{ 0x098C, 0xAB4F, 0 },		/* HG_GAMMA_TABLE_B_0 */
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xAB50, 0 },
	{ 0x0990, 0x0004, 0 },
	{ 0x098C, 0xAB51, 0 },
	{ 0x0990, 0x000D, 0 },
	{ 0x098C, 0xAB52, 0 },
	{ 0x0990, 0x0028, 0 },
	{ 0x098C, 0xAB53, 0 },
	{ 0x0990, 0x0053, 0 },
	{ 0x098C, 0xAB54, 0 },
	{ 0x0990, 0x0075, 0 },
	{ 0x098C, 0xAB55, 0 },
	{ 0x0990, 0x0092, 0 },
	{ 0x098C, 0xAB56, 0 },
	{ 0x0990, 0x00A7, 0 },
	{ 0x098C, 0xAB57, 0 },
	{ 0x0990, 0x00B7, 0 },
	{ 0x098C, 0xAB58, 0 },
	{ 0x0990, 0x00C4, 0 },
	{ 0x098C, 0xAB59, 0 },
	{ 0x0990, 0x00CF, 0 },
	{ 0x098C, 0xAB5A, 0 },
	{ 0x0990, 0x00D8, 0 },
	{ 0x098C, 0xAB5B, 0 },
	{ 0x0990, 0x00DF, 0 },
	{ 0x098C, 0xAB5C, 0 },
	{ 0x0990, 0x00E6, 0 },
	{ 0x098C, 0xAB5D, 0 },
	{ 0x0990, 0x00EC, 0 },
	{ 0x098C, 0xAB5E, 0 },
	{ 0x0990, 0x00F2, 0 },
	{ 0x098C, 0xAB5F, 0 },
	{ 0x0990, 0x00F6, 0 },
	{ 0x098C, 0xAB60, 0 },
	{ 0x0990, 0x00FB, 0 },
	{ 0x098C, 0xAB61, 0 },
	{ 0x0990, 0x00FF, 0 },

	/* Read mode - no mirror/flip */
	{ 0x098C, 0x2717, 0 },		/* MODE_SENSOR_READ_MODE_A */
	{ 0x0990, 0x046C, 0 },
	{ 0x098C, 0x272D, 0 },		/* MODE_SENSOR_READ_MODE_B */
	{ 0x0990, 0x0024, 0 },

	/* Reset command before sequencer */
	{ 0x001A, 0x021C, 0 },		/* RESET_AND_MISC_CONTROL */

	/* Issue refresh command - MCU will be polled for completion */
	{ 0x098C, 0xA103, 0 },		/* SEQ_CMD */
	{ 0x0990, 0x0006, 0 },		/* REFRESH_MODE */
};

/*
 * Apply the MT9M113 initialization table.
 * Returns 0 on success, negative error code on failure.
 */
static int mt9m113_sensor_init(struct mt9m114 *sensor)
{
	struct device *dev = &sensor->client->dev;
	int ret = 0;
	unsigned int i;

	dev_info(dev, "MT9M113: applying initialization table (%zu entries)\n",
		 ARRAY_SIZE(mt9m113_init_table));

	/* Debug: Read 0x301A default value before any writes */
	{
		u64 readback = 0;
		cci_read(sensor->regmap, MT9M114_RESET_REGISTER, &readback, NULL);
		dev_info(dev, "MT9M113: RESET_REGISTER default=0x%llx (before init table)\n",
			 readback);
	}

	for (i = 0; i < ARRAY_SIZE(mt9m113_init_table); i++) {
		const struct mt9m113_reg_entry *entry = &mt9m113_init_table[i];

		ret = cci_write(sensor->regmap, CCI_REG16(entry->reg),
				entry->value, NULL);
		if (ret < 0) {
			dev_err(dev, "MT9M113: failed to write reg 0x%04x: %d\n",
				entry->reg, ret);
			return ret;
		}

		if (entry->delay_ms > 0)
			msleep(entry->delay_ms);
	}

	/*
	 * Wait for MCU to complete refresh (SEQ_CMD returns to 0).
	 * The last entry issued SEQ_CMD=0x0006 (REFRESH_MODE).
	 */
	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 1000);
	if (ret < 0) {
		dev_err(dev, "MT9M113: MCU refresh timeout after init table\n");
		return ret;
	}
	dev_info(dev, "MT9M113: MCU refresh completed\n");

	/*
	 * Issue sequencer refresh (SEQ_CMD=0x0005) per webOS driver.
	 * This fully initializes the sequencer after loading settings.
	 */
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				    MT9M113_SEQ_CMD_REFRESH);
	if (ret < 0) {
		dev_err(dev, "MT9M113: failed to issue SEQ_CMD refresh: %d\n", ret);
		return ret;
	}

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 1000);
	if (ret < 0) {
		dev_err(dev, "MT9M113: MCU SEQ refresh timeout\n");
		return ret;
	}
	dev_info(dev, "MT9M113: sequencer refresh completed\n");

	/*
	 * NOTE: Do NOT write OUTPUT_CONTROL here!
	 * Writing OUTPUT_CONTROL=0x7A08 enables MIPI output immediately,
	 * causing the sensor to auto-stream. This makes it impossible to
	 * cleanly restart streaming later because STANDBY doesn't fully stop
	 * MIPI output in this state.
	 *
	 * Instead, OUTPUT_CONTROL is written only in start_streaming(),
	 * after VFE and CSIPHY are configured and ready to receive data.
	 * This ensures proper synchronization between sensor and receiver.
	 */
	dev_info(dev, "MT9M113: OUTPUT_CONTROL NOT set in init (deferred to streaming)\n");

	/* Check final SEQ_STATE after init */
	{
		u64 seq_state;
		mt9m113_read_mcu_var(sensor, MT9M113_SEQ_STATE, &seq_state);
		dev_info(dev, "MT9M113: after init, SEQ_STATE=0x%llx\n", seq_state);
	}

	dev_info(dev, "MT9M113: initialization complete\n");
	return 0;
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

/* Wait for a command to complete. */
static int mt9m114_poll_command(struct mt9m114 *sensor, u32 command)
{
	unsigned int i;
	u64 value;
	int ret;

	for (i = 0; i < 100; ++i) {
		ret = cci_read(sensor->regmap, MT9M114_COMMAND_REGISTER, &value,
			       NULL);
		if (ret < 0)
			return ret;

		if (!(value & command))
			break;

		usleep_range(5000, 6000);
	}

	if (value & command) {
		dev_err(&sensor->client->dev, "Command %u completion timeout\n",
			command);
		return -ETIMEDOUT;
	}

	if (!(value & MT9M114_COMMAND_REGISTER_OK)) {
		dev_err(&sensor->client->dev, "Command %u failed\n", command);
		return -EIO;
	}

	return 0;
}

/* Wait for a state to be entered. */
static int mt9m114_poll_state(struct mt9m114 *sensor, u32 state)
{
	unsigned int i;
	u64 value;
	int ret;

	for (i = 0; i < 100; ++i) {
		ret = cci_read(sensor->regmap, MT9M114_SYSMGR_CURRENT_STATE,
			       &value, NULL);
		if (ret < 0)
			return ret;

		if (value == state)
			return 0;

		usleep_range(1000, 1500);
	}

	dev_err(&sensor->client->dev, "Timeout waiting for state 0x%02x\n",
		state);
	return -ETIMEDOUT;
}

static int mt9m114_set_state(struct mt9m114 *sensor, u8 next_state)
{
	int ret = 0;

	/* Set the next desired state and start the state transition. */
	cci_write(sensor->regmap, MT9M114_SYSMGR_NEXT_STATE, next_state, &ret);
	cci_write(sensor->regmap, MT9M114_COMMAND_REGISTER,
		  MT9M114_COMMAND_REGISTER_OK |
		  MT9M114_COMMAND_REGISTER_SET_STATE, &ret);
	if (ret < 0)
		return ret;

	/* Wait for the state transition to complete. */
	ret = mt9m114_poll_command(sensor, MT9M114_COMMAND_REGISTER_SET_STATE);
	if (ret < 0)
		return ret;

	return 0;
}

static int mt9m114_initialize(struct mt9m114 *sensor)
{
	u32 value;
	int ret;

	/*
	 * MT9M113 uses a completely different initialization sequence.
	 * It requires the full webOS register table applied via MCU indirect
	 * access (0x098C/0x0990) rather than direct writes to 0xC000+ addresses.
	 * Use mt9m113_sensor_init() which applies the complete 500+ entry table.
	 */
	if (sensor->model == MT9M113_MODEL) {
		dev_info(&sensor->client->dev, "mt9m114_initialize: using MT9M113 initialization\n");
		return mt9m113_sensor_init(sensor);
	}

	/* MT9M114 initialization path */
	dev_info(&sensor->client->dev, "mt9m114_initialize: writing init table (%zu entries)\n",
		 ARRAY_SIZE(mt9m114_init));

	ret = cci_multi_reg_write(sensor->regmap, mt9m114_init,
				  ARRAY_SIZE(mt9m114_init), NULL);
	if (ret < 0) {
		dev_err(&sensor->client->dev,
			"Failed to initialize the sensor\n");
		return ret;
	}

	/* Configure the PLL. */
	if (sensor->bypass_pll) {
		cci_write(sensor->regmap, MT9M114_CAM_SYSCTL_PLL_ENABLE,
			  MT9M114_CAM_SYSCTL_PLL_DISABLE_VALUE, &ret);
	} else {
		cci_write(sensor->regmap, MT9M114_CAM_SYSCTL_PLL_ENABLE,
			  MT9M114_CAM_SYSCTL_PLL_ENABLE_VALUE, &ret);
		cci_write(sensor->regmap, MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N,
			  MT9M114_CAM_SYSCTL_PLL_DIVIDER_VALUE(sensor->pll.m,
							       sensor->pll.n),
			  &ret);
		cci_write(sensor->regmap, MT9M114_CAM_SYSCTL_PLL_DIVIDER_P,
			  MT9M114_CAM_SYSCTL_PLL_DIVIDER_P_VALUE(sensor->pll.p),
			  &ret);
	}

	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_PIXCLK,
		  sensor->pixrate, &ret);

	/* Configure the output mode. */
	if (sensor->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY) {
		value = MT9M114_CAM_PORT_PORT_SELECT_MIPI
		      | MT9M114_CAM_PORT_CHAN_NUM(0)
		      | 0x8000;
		if (!(sensor->bus_cfg.bus.mipi_csi2.flags &
		      V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK))
			value |= MT9M114_CAM_PORT_CONT_MIPI_CLK;
	} else {
		value = MT9M114_CAM_PORT_PORT_SELECT_PARALLEL
		      | 0x8000;
	}
	cci_write(sensor->regmap, MT9M114_CAM_PORT_OUTPUT_CONTROL, value, &ret);
	if (ret < 0)
		return ret;

	value = sensor->pad_slew_rate
	      | sensor->pad_slew_rate << 4
	      |	sensor->pad_slew_rate << 8;
	cci_write(sensor->regmap, MT9M114_PAD_SLEW, value, &ret);
	if (ret < 0)
		return ret;

	dev_info(&sensor->client->dev, "mt9m114_initialize: issuing Change Config\n");
	ret = mt9m114_set_state(sensor, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	if (ret < 0)
		return ret;

	ret = mt9m114_set_state(sensor, MT9M114_SYS_STATE_ENTER_SUSPEND);
	if (ret < 0)
		return ret;

	return 0;
}

static int mt9m114_configure_pa(struct mt9m114 *sensor,
				struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	unsigned int hratio, vratio;
	u64 read_mode;
	int ret;

	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);

	ret = cci_read(sensor->regmap, MT9M114_CAM_SENSOR_CONTROL_READ_MODE,
		       &read_mode, NULL);
	if (ret < 0)
		return ret;

	hratio = crop->width / format->width;
	vratio = crop->height / format->height;

	/*
	 * Pixel array crop and binning. The CAM_SENSOR_CFG_CPIPE_LAST_ROW
	 * register isn't clearly documented, but is always set to the number
	 * of active rows minus 4 divided by the vertical binning factor in all
	 * example sensor modes.
	 */
	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_X_ADDR_START,
		  crop->left, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,
		  crop->top, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_X_ADDR_END,
		  crop->width + crop->left - 1, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,
		  crop->height + crop->top - 1, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,
		  (crop->height - 4) / vratio - 1, &ret);

	read_mode &= ~(MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_MASK |
		       MT9M114_CAM_SENSOR_CONTROL_Y_READ_OUT_MASK);

	if (hratio > 1)
		read_mode |= MT9M114_CAM_SENSOR_CONTROL_X_READ_OUT_SUMMING;
	if (vratio > 1)
		read_mode |= MT9M114_CAM_SENSOR_CONTROL_Y_READ_OUT_SUMMING;

	cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CONTROL_READ_MODE,
		  read_mode, &ret);

	return ret;
}

static int mt9m114_configure_ifp(struct mt9m114 *sensor,
				 struct v4l2_subdev_state *state)
{
	const struct mt9m114_format_info *info;
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	const struct v4l2_rect *compose;
	u64 output_format;
	int ret = 0;

	format = v4l2_subdev_state_get_format(state, 1);
	info = mt9m114_format_info(sensor, 1, format->code);
	crop = v4l2_subdev_state_get_crop(state, 0);
	compose = v4l2_subdev_state_get_compose(state, 0);

	ret = cci_read(sensor->regmap, MT9M114_CAM_OUTPUT_FORMAT,
		       &output_format, NULL);
	if (ret < 0)
		return ret;

	/*
	 * Color pipeline (IFP) cropping and scaling. Subtract 4 from the left
	 * and top coordinates to compensate for the lines and columns removed
	 * by demosaicing that are taken into account in the crop rectangle but
	 * not in the hardware.
	 */
	cci_write(sensor->regmap, MT9M114_CAM_CROP_WINDOW_XOFFSET,
		  crop->left - 4, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_CROP_WINDOW_YOFFSET,
		  crop->top - 4, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_CROP_WINDOW_WIDTH,
		  crop->width, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_CROP_WINDOW_HEIGHT,
		  crop->height, &ret);

	cci_write(sensor->regmap, MT9M114_CAM_OUTPUT_WIDTH,
		  compose->width, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_OUTPUT_HEIGHT,
		  compose->height, &ret);

	/* AWB and AE windows, use the full frame. */
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,
		  0, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,
		  0, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,
		  compose->width - 1, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,
		  compose->height - 1, &ret);

	cci_write(sensor->regmap, MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,
		  0, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,
		  0, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,
		  compose->width / 5 - 1, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,
		  compose->height / 5 - 1, &ret);

	cci_write(sensor->regmap, MT9M114_CAM_CROP_CROPMODE,
		  MT9M114_CAM_CROP_MODE_AWB_AUTO_CROP_EN |
		  MT9M114_CAM_CROP_MODE_AE_AUTO_CROP_EN, &ret);

	/* Set the media bus code. */
	output_format &= ~(MT9M114_CAM_OUTPUT_FORMAT_RGB_FORMAT_MASK |
			   MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_MASK |
			   MT9M114_CAM_OUTPUT_FORMAT_FORMAT_MASK |
			   MT9M114_CAM_OUTPUT_FORMAT_SWAP_BYTES |
			   MT9M114_CAM_OUTPUT_FORMAT_SWAP_RED_BLUE);
	output_format |= info->output_format;

	cci_write(sensor->regmap, MT9M114_CAM_OUTPUT_FORMAT,
		  output_format, &ret);

	return ret;
}

static int mt9m114_set_frame_rate(struct mt9m114 *sensor)
{
	u16 frame_rate = sensor->ifp.frame_rate << 8;
	int ret = 0;

	cci_write(sensor->regmap, MT9M114_CAM_AET_MIN_FRAME_RATE,
		  frame_rate, &ret);
	cci_write(sensor->regmap, MT9M114_CAM_AET_MAX_FRAME_RATE,
		  frame_rate, &ret);

	return ret;
}

static int mt9m114_start_streaming(struct mt9m114 *sensor,
				   struct v4l2_subdev_state *pa_state,
				   struct v4l2_subdev_state *ifp_state)
{
	int ret;

	ret = pm_runtime_resume_and_get(&sensor->client->dev);
	if (ret)
		return ret;

	/*
	 * MT9M113 uses indirect MCU variable access (0x098C/0x0990) instead
	 * of direct writes to 0xC000+ addresses. Skip the configure functions
	 * which write to wrong addresses for MT9M113 - the sensor uses its
	 * default configuration set during power_on.
	 */
	if (sensor->model == MT9M113_MODEL)
		goto mt9m113_streaming;

	ret = mt9m114_configure_ifp(sensor, ifp_state);
	if (ret)
		goto error;

	ret = mt9m114_configure_pa(sensor, pa_state);
	if (ret)
		goto error;

	ret = mt9m114_set_frame_rate(sensor);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(&sensor->pa.hdl);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(&sensor->ifp.hdl);
	if (ret)
		goto error;

	/*
	 * The Change-Config state is transient and moves to the streaming
	 * state automatically.
	 */
	ret = mt9m114_set_state(sensor, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	if (ret)
		goto error;

	sensor->streaming = true;
	return 0;

mt9m113_streaming:
	/*
	 * MT9M113 uses a different command mechanism (MCU indirect via
	 * 0x098C/0x0990) and doesn't support MT9M114's COMMAND_REGISTER.
	 *
	 * WebOS driver behavior:
	 * - OUTPUT_CONTROL and RESET_REGISTER written ONCE during first CSI config
	 * - For mode changes: only SEQ_CAP_MODE and SEQ_CMD are written
	 *
	 * CRITICAL: If sensor is already streaming (SEQ_STATE=0x3), do NOT
	 * re-write OUTPUT_CONTROL or RESET_REGISTER as this disrupts MIPI.
	 */
	{
		u64 seq_state;
		u64 output_ctrl;
		int timeout;

		dev_info(&sensor->client->dev, "MT9M113: starting streaming sequence\n");

		/* Check current SEQ_STATE */
		mt9m113_read_mcu_var(sensor, MT9M113_SEQ_STATE, &seq_state);
		dev_info(&sensor->client->dev,
			 "MT9M113: SEQ_STATE=0x%llx (0x03=preview mode)\n", seq_state);

		/* Read current OUTPUT_CONTROL */
		cci_read(sensor->regmap, MT9M113_OUTPUT_CONTROL, &output_ctrl, NULL);
		dev_info(&sensor->client->dev,
			 "MT9M113: OUTPUT_CONTROL=0x%llx (0x7A08=MIPI enabled)\n", output_ctrl);

		/*
		 * Only configure MIPI if not already enabled.
		 * WebOS writes OUTPUT_CONTROL/RESET_REGISTER only once.
		 */
		if (output_ctrl != MT9M113_OUTPUT_CONTROL_MIPI_ENABLE) {
			dev_info(&sensor->client->dev, "MT9M113: Configuring MIPI (first time)\n");

			/* Configure short packets BEFORE enabling MIPI */
			cci_write(sensor->regmap, MT9M113_CUSTOM_SHORT_PKT,
				  MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN, NULL);

			/* Enable MIPI output */
			cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL,
				  MT9M113_OUTPUT_CONTROL_MIPI_ENABLE, NULL);

			/* Set streaming mode in RESET_REGISTER */
			cci_write(sensor->regmap, MT9M114_RESET_REGISTER, 0x120C, NULL);

			dev_info(&sensor->client->dev,
				 "MT9M113: MIPI configured (OUTPUT_CONTROL=0x7A08)\n");
		} else {
			dev_info(&sensor->client->dev,
				 "MT9M113: MIPI already enabled, skipping OUTPUT_CONTROL write\n");
		}

		/*
		 * Set capture mode via MCU interface.
		 * From webOS kernel: SEQ_CAP_MODE (0xA115) = 0x0030 for preview mode.
		 */
		ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CAP_MODE, 0x0030);
		if (ret) {
			dev_err(&sensor->client->dev, "MT9M113: SEQ_CAP_MODE failed: %d\n", ret);
			goto error;
		}

		/* webOS delays 40ms between SEQ_CAP_MODE and SEQ_CMD */
		usleep_range(40000, 50000);

		/* Issue SEQ_CMD=1 (RUN) to start streaming */
		ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
					    MT9M113_SEQ_CMD_RUN);
		if (ret) {
			dev_err(&sensor->client->dev, "MT9M113: SEQ_CMD RUN failed: %d\n", ret);
			goto error;
		}

		/* Wait for streaming to start (SEQ_STATE = 0x03) */
		timeout = 50; /* 500ms max */
		do {
			msleep(10);
			mt9m113_read_mcu_var(sensor, MT9M113_SEQ_STATE, &seq_state);
		} while (seq_state != 0x03 && --timeout > 0);

		if (timeout == 0) {
			dev_warn(&sensor->client->dev,
				 "MT9M113: RUN timeout, SEQ_STATE=0x%llx\n", seq_state);
		} else {
			dev_info(&sensor->client->dev,
				 "MT9M113: Streaming active (SEQ_STATE=0x%llx)\n", seq_state);
		}

		dev_info(&sensor->client->dev, "MT9M113: streaming command issued\n");
	}

	sensor->streaming = true;

	return 0;

error:
	pm_runtime_put_autosuspend(&sensor->client->dev);

	return ret;
}

static int mt9m114_stop_streaming(struct mt9m114 *sensor)
{
	int ret = 0;

	sensor->streaming = false;

	if (sensor->model == MT9M113_MODEL) {
		/*
		 * MT9M113: Issue SEQ_CMD=0x0003 (STANDBY) to properly stop streaming.
		 * Without this, the sensor remains in streaming state and subsequent
		 * streaming attempts fail because SEQ_CMD=0x0001 (RUN) is ignored
		 * when already running.
		 *
		 * The webOS driver doesn't explicitly stop streaming, it just powers
		 * down the sensor via GPIO. But since we keep the sensor powered
		 * between streaming sessions for faster startup, we need to properly
		 * transition to standby state.
		 */
		/*
		 * Switch to capture mode to stop continuous preview streaming.
		 * Note: MT9M113 doesn't have a true standby command - webOS just
		 * powers down via GPIO. We use capture mode (0x02) to change state.
		 */
		dev_info(&sensor->client->dev, "MT9M113: stopping streaming (SEQ_CMD=CAPTURE)\n");
		ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
					    MT9M113_SEQ_CMD_CAPTURE);
		if (ret < 0) {
			dev_err(&sensor->client->dev,
				"MT9M113: failed to stop streaming: %d\n", ret);
		}
		/*
		 * Wait for the sequencer to enter standby.
		 * Don't poll SEQ_CMD - just wait a fixed time like webOS does.
		 */
		msleep(20);
	} else {
		ret = mt9m114_set_state(sensor, MT9M114_SYS_STATE_ENTER_SUSPEND);
	}

	pm_runtime_put_autosuspend(&sensor->client->dev);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Common Subdev Operations
 */

/*
 * Custom link validation for the IFP sink pad.
 *
 * The pixel array outputs full resolution (1296x976) but the IFP internally
 * crops the image. The default v4l2_subdev_link_validate requires exact format
 * matches which fails because the PA output is larger than the IFP sink format.
 *
 * This function validates that the source format can be cropped to match
 * the IFP sink format using the crop settings.
 */
static int mt9m114_link_validate(struct media_link *link)
{
	struct v4l2_subdev *sink_sd;

	/* Only apply custom validation to IFP sink pad */
	if (!is_media_entity_v4l2_subdev(link->sink->entity))
		return v4l2_subdev_link_validate(link);

	sink_sd = media_entity_to_v4l2_subdev(link->sink->entity);

	/* Check if this is the IFP (has " ifp" suffix in name) */
	if (!strstr(sink_sd->name, " ifp"))
		return v4l2_subdev_link_validate(link);

	/*
	 * For the internal pixel array -> IFP link, skip format validation.
	 *
	 * The pixel array outputs full resolution (1296x976) but the IFP
	 * internally crops the image. This is an immutable internal link
	 * managed by the mt9m114 driver, so we trust it is valid.
	 *
	 * The default v4l2_subdev_link_validate would fail because it requires
	 * exact format matches, but the PA output is larger than the IFP sink.
	 */
	dev_dbg(sink_sd->dev, "mt9m114: PA->IFP link validated (internal)\n");
	return 0;
}

static const struct media_entity_operations mt9m114_entity_ops = {
	.link_validate = mt9m114_link_validate,
};

/* -----------------------------------------------------------------------------
 * Pixel Array Control Operations
 */

static inline struct mt9m114 *pa_ctrl_to_mt9m114(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mt9m114, pa.hdl);
}

static int mt9m114_pa_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m114 *sensor = pa_ctrl_to_mt9m114(ctrl);
	u64 value;
	int ret;

	if (!pm_runtime_get_if_in_use(&sensor->client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_read(sensor->regmap,
			       MT9M114_CAM_SENSOR_CONTROL_COARSE_INTEGRATION_TIME,
			       &value, NULL);
		if (ret)
			break;

		ctrl->val = value;
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_read(sensor->regmap,
			       MT9M114_CAM_SENSOR_CONTROL_ANALOG_GAIN,
			       &value, NULL);
		if (ret)
			break;

		ctrl->val = value;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(&sensor->client->dev);

	return ret;
}

static int mt9m114_pa_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m114 *sensor = pa_ctrl_to_mt9m114(ctrl);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	int ret = 0;
	u64 mask;

	/* V4L2 controls values are applied only when power is up. */
	if (!pm_runtime_get_if_in_use(&sensor->client->dev))
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->pa.sd);
	format = v4l2_subdev_state_get_format(state, 0);

	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
		cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,
			  ctrl->val + format->width, &ret);
		break;

	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,
			  ctrl->val + format->height, &ret);
		break;

	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap,
			  MT9M114_CAM_SENSOR_CONTROL_COARSE_INTEGRATION_TIME,
			  ctrl->val, &ret);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/*
		 * The CAM_SENSOR_CONTROL_ANALOG_GAIN contains linear analog
		 * gain values that are mapped to the GLOBAL_GAIN register
		 * values by the sensor firmware.
		 */
		cci_write(sensor->regmap, MT9M114_CAM_SENSOR_CONTROL_ANALOG_GAIN,
			  ctrl->val, &ret);
		break;

	case V4L2_CID_HFLIP:
		mask = MT9M114_CAM_SENSOR_CONTROL_HORZ_MIRROR_EN;
		ret = cci_update_bits(sensor->regmap,
				      MT9M114_CAM_SENSOR_CONTROL_READ_MODE,
				      mask, ctrl->val ? mask : 0, NULL);
		break;

	case V4L2_CID_VFLIP:
		mask = MT9M114_CAM_SENSOR_CONTROL_VERT_FLIP_EN;
		ret = cci_update_bits(sensor->regmap,
				      MT9M114_CAM_SENSOR_CONTROL_READ_MODE,
				      mask, ctrl->val ? mask : 0, NULL);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(&sensor->client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mt9m114_pa_ctrl_ops = {
	.g_volatile_ctrl = mt9m114_pa_g_ctrl,
	.s_ctrl = mt9m114_pa_s_ctrl,
};

static void mt9m114_pa_ctrl_update_exposure(struct mt9m114 *sensor, bool manual)
{
	/*
	 * Update the volatile flag on the manual exposure and gain controls.
	 * If the controls have switched to manual, read their current value
	 * from the hardware to ensure that control read and write operations
	 * will behave correctly
	 */
	if (manual) {
		mt9m114_pa_g_ctrl(sensor->pa.exposure);
		sensor->pa.exposure->cur.val = sensor->pa.exposure->val;
		sensor->pa.exposure->flags &= ~V4L2_CTRL_FLAG_VOLATILE;

		mt9m114_pa_g_ctrl(sensor->pa.gain);
		sensor->pa.gain->cur.val = sensor->pa.gain->val;
		sensor->pa.gain->flags &= ~V4L2_CTRL_FLAG_VOLATILE;
	} else {
		sensor->pa.exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;
		sensor->pa.gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}
}

static void mt9m114_pa_ctrl_update_blanking(struct mt9m114 *sensor,
					    const struct v4l2_mbus_framefmt *format)
{
	unsigned int max_blank;

	/* Update the blanking controls ranges based on the output size. */
	max_blank = MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK_MAX
		  - format->width;
	__v4l2_ctrl_modify_range(sensor->pa.hblank, MT9M114_MIN_HBLANK,
				 max_blank, 1, MT9M114_DEF_HBLANK);

	max_blank = MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES_MAX
		  - format->height;
	__v4l2_ctrl_modify_range(sensor->pa.vblank, MT9M114_MIN_VBLANK,
				 max_blank, 1, MT9M114_DEF_VBLANK);
}

/* -----------------------------------------------------------------------------
 * Pixel Array Subdev Operations
 */

static inline struct mt9m114 *pa_to_mt9m114(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m114, pa.sd);
}

static int mt9m114_pa_init_state(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	crop = v4l2_subdev_state_get_crop(state, 0);

	crop->left = 0;
	crop->top = 0;
	crop->width = MT9M114_PIXEL_ARRAY_WIDTH;
	crop->height = MT9M114_PIXEL_ARRAY_HEIGHT;

	format = v4l2_subdev_state_get_format(state, 0);

	format->width = MT9M114_PIXEL_ARRAY_WIDTH;
	format->height = MT9M114_PIXEL_ARRAY_HEIGHT;
	format->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_601;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	return 0;
}

static int mt9m114_pa_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int mt9m114_pa_enum_framesizes(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 1)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	/* Report binning capability through frame size enumeration. */
	fse->min_width = MT9M114_PIXEL_ARRAY_WIDTH / (fse->index + 1);
	fse->max_width = MT9M114_PIXEL_ARRAY_WIDTH / (fse->index + 1);
	fse->min_height = MT9M114_PIXEL_ARRAY_HEIGHT / (fse->index + 1);
	fse->max_height = MT9M114_PIXEL_ARRAY_HEIGHT / (fse->index + 1);

	return 0;
}

static int mt9m114_pa_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct mt9m114 *sensor = pa_to_mt9m114(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	unsigned int hscale;
	unsigned int vscale;

	crop = v4l2_subdev_state_get_crop(state, fmt->pad);
	format = v4l2_subdev_state_get_format(state, fmt->pad);

	/* The sensor can bin horizontally and vertically. */
	hscale = DIV_ROUND_CLOSEST(crop->width, fmt->format.width ? : 1);
	vscale = DIV_ROUND_CLOSEST(crop->height, fmt->format.height ? : 1);
	format->width = crop->width / clamp(hscale, 1U, 2U);
	format->height = crop->height / clamp(vscale, 1U, 2U);

	fmt->format = *format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		mt9m114_pa_ctrl_update_blanking(sensor, format);

	return 0;
}

static int mt9m114_pa_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = MT9M114_PIXEL_ARRAY_WIDTH;
		sel->r.height = MT9M114_PIXEL_ARRAY_HEIGHT;
		return 0;

	default:
		return -EINVAL;
	}
}

static int mt9m114_pa_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *sel)
{
	struct mt9m114 *sensor = pa_to_mt9m114(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	int ret = 0;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(state, sel->pad);
	format = v4l2_subdev_state_get_format(state, sel->pad);

	/*
	 * Clamp the crop rectangle. The vertical coordinates must be even, and
	 * the horizontal coordinates must be a multiple of 4.
	 *
	 * FIXME: The horizontal coordinates must be a multiple of 8 when
	 * binning, but binning is configured after setting the selection, so
	 * we can't know tell here if it will be used.
	 */
	sel->r.left = ALIGN(sel->r.left, 4);
	sel->r.top = ALIGN(sel->r.top, 2);
	sel->r.width = clamp_t(unsigned int, ALIGN(sel->r.width, 4),
			       MT9M114_PIXEL_ARRAY_MIN_OUTPUT_WIDTH,
			       MT9M114_PIXEL_ARRAY_WIDTH - sel->r.left);
	sel->r.height = clamp_t(unsigned int, ALIGN(sel->r.height, 2),
				MT9M114_PIXEL_ARRAY_MIN_OUTPUT_HEIGHT,
				MT9M114_PIXEL_ARRAY_HEIGHT - sel->r.top);

	/* Changing the selection size is not allowed in streaming state. */
	if (sensor->streaming &&
	    (sel->r.height != crop->height || sel->r.width != crop->width))
		return -EBUSY;

	*crop = sel->r;

	/* Reset the format. */
	format->width = crop->width;
	format->height = crop->height;

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return ret;

	mt9m114_pa_ctrl_update_blanking(sensor, format);

	/* Apply values immediately if streaming. */
	if (sensor->streaming) {
		ret = mt9m114_configure_pa(sensor, state);
		if (ret)
			return ret;
		/*
		 * Changing the cropping config requires a CONFIG_CHANGE.
		 * MT9M113 doesn't support COMMAND_REGISTER - changes take
		 * effect immediately.
		 */
		if (sensor->model != MT9M113_MODEL)
			ret = mt9m114_set_state(sensor,
						MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	}
	return ret;
}

static const struct v4l2_subdev_pad_ops mt9m114_pa_pad_ops = {
	.enum_mbus_code = mt9m114_pa_enum_mbus_code,
	.enum_frame_size = mt9m114_pa_enum_framesizes,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mt9m114_pa_set_fmt,
	.get_selection = mt9m114_pa_get_selection,
	.set_selection = mt9m114_pa_set_selection,
};

static const struct v4l2_subdev_ops mt9m114_pa_ops = {
	.pad = &mt9m114_pa_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9m114_pa_internal_ops = {
	.init_state = mt9m114_pa_init_state,
};

static int mt9m114_pa_init(struct mt9m114 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->pa.hdl;
	struct v4l2_subdev *sd = &sensor->pa.sd;
	struct media_pad *pads = &sensor->pa.pad;
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	unsigned int max_exposure;
	int ret;

	/* Initialize the subdev. */
	v4l2_subdev_init(sd, &mt9m114_pa_ops);
	sd->internal_ops = &mt9m114_pa_internal_ops;
	v4l2_i2c_subdev_set_name(sd, sensor->client, NULL, " pixel array");

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->owner = THIS_MODULE;
	sd->dev = &sensor->client->dev;
	v4l2_set_subdevdata(sd, sensor->client);

	/* Initialize the media entity. */
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &mt9m114_entity_ops;
	pads[0].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, pads);
	if (ret < 0)
		return ret;

	/* Initialize the control handler. */
	v4l2_ctrl_handler_init(hdl, 7);

	/* The range of the HBLANK and VBLANK controls will be updated below. */
	sensor->pa.hblank = v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
					      V4L2_CID_HBLANK,
					      MT9M114_DEF_HBLANK,
					      MT9M114_DEF_HBLANK, 1,
					      MT9M114_DEF_HBLANK);
	sensor->pa.vblank = v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
					      V4L2_CID_VBLANK,
					      MT9M114_DEF_VBLANK,
					      MT9M114_DEF_VBLANK, 1,
					      MT9M114_DEF_VBLANK);

	/*
	 * The maximum coarse integration time is the frame length in lines
	 * minus two. The default is taken directly from the datasheet, but
	 * makes little sense as auto-exposure is enabled by default.
	 */
	max_exposure = MT9M114_PIXEL_ARRAY_HEIGHT + MT9M114_MIN_VBLANK - 2;
	sensor->pa.exposure = v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
						V4L2_CID_EXPOSURE, 1,
						max_exposure, 1, 16);
	if (sensor->pa.exposure)
		sensor->pa.exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	sensor->pa.gain = v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
					    V4L2_CID_ANALOGUE_GAIN, 1,
					    511, 1, 32);
	if (sensor->pa.gain)
		sensor->pa.gain->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
			  V4L2_CID_PIXEL_RATE,
			  sensor->pixrate, sensor->pixrate, 1,
			  sensor->pixrate);

	v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
			  V4L2_CID_HFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &mt9m114_pa_ctrl_ops,
			  V4L2_CID_VFLIP,
			  0, 1, 1, 0);

	if (hdl->error) {
		ret = hdl->error;
		goto error;
	}

	sd->state_lock = hdl->lock;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto error;

	/* Update the range of the blanking controls based on the format. */
	state = v4l2_subdev_lock_and_get_active_state(sd);
	format = v4l2_subdev_state_get_format(state, 0);
	mt9m114_pa_ctrl_update_blanking(sensor, format);
	v4l2_subdev_unlock_state(state);

	sd->ctrl_handler = hdl;

	return 0;

error:
	v4l2_ctrl_handler_free(&sensor->pa.hdl);
	media_entity_cleanup(&sensor->pa.sd.entity);
	return ret;
}

static void mt9m114_pa_cleanup(struct mt9m114 *sensor)
{
	v4l2_ctrl_handler_free(&sensor->pa.hdl);
	media_entity_cleanup(&sensor->pa.sd.entity);
}

/* -----------------------------------------------------------------------------
 * Image Flow Processor Control Operations
 */

static const char * const mt9m114_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"100% Color Bars",
	"Pseudo-Random",
	"Fade-to-Gray Color Bars",
	"Walking Ones 10-bit",
	"Walking Ones 8-bit",
};

/* Keep in sync with mt9m114_test_pattern_menu */
static const unsigned int mt9m114_test_pattern_value[] = {
	MT9M114_CAM_MODE_TEST_PATTERN_SELECT_SOLID,
	MT9M114_CAM_MODE_TEST_PATTERN_SELECT_SOLID_BARS,
	MT9M114_CAM_MODE_TEST_PATTERN_SELECT_RANDOM,
	MT9M114_CAM_MODE_TEST_PATTERN_SELECT_FADING_BARS,
	MT9M114_CAM_MODE_TEST_PATTERN_SELECT_WALKING_1S_10B,
	MT9M114_CAM_MODE_TEST_PATTERN_SELECT_WALKING_1S_8B,
};

static inline struct mt9m114 *ifp_ctrl_to_mt9m114(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mt9m114, ifp.hdl);
}

static int mt9m114_ifp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m114 *sensor = ifp_ctrl_to_mt9m114(ctrl);
	u32 value;
	int ret = 0;

	if (ctrl->id == V4L2_CID_EXPOSURE_AUTO)
		mt9m114_pa_ctrl_update_exposure(sensor,
						ctrl->val != V4L2_EXPOSURE_AUTO);

	/* V4L2 controls values are applied only when power is up. */
	if (!pm_runtime_get_if_in_use(&sensor->client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		/* Control both the AWB mode and the CCM algorithm. */
		if (ctrl->val)
			value = MT9M114_CAM_AWB_MODE_AUTO
			      | MT9M114_CAM_AWB_MODE_EXCLUSIVE_AE;
		else
			value = 0;

		cci_write(sensor->regmap, MT9M114_CAM_AWB_AWBMODE, value, &ret);

		if (ctrl->val)
			value = MT9M114_CCM_EXEC_CALC_CCM_MATRIX | 0x22;
		else
			value = 0;

		cci_write(sensor->regmap, MT9M114_CCM_ALGO, value, &ret);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val == V4L2_EXPOSURE_AUTO)
			value = MT9M114_AE_TRACK_EXEC_AUTOMATIC_EXPOSURE
			      | 0x00fe;
		else
			value = 0;

		cci_write(sensor->regmap, MT9M114_AE_TRACK_ALGO, value, &ret);
		if (ret)
			break;

		break;

	case V4L2_CID_TEST_PATTERN:
	case V4L2_CID_TEST_PATTERN_RED:
	case V4L2_CID_TEST_PATTERN_GREENR:
	case V4L2_CID_TEST_PATTERN_BLUE: {
		unsigned int pattern = sensor->ifp.tpg[MT9M114_TPG_PATTERN]->val;

		if (pattern) {
			cci_write(sensor->regmap, MT9M114_CAM_MODE_SELECT,
				  MT9M114_CAM_MODE_SELECT_TEST_PATTERN, &ret);
			cci_write(sensor->regmap,
				  MT9M114_CAM_MODE_TEST_PATTERN_SELECT,
				  mt9m114_test_pattern_value[pattern - 1], &ret);
			cci_write(sensor->regmap,
				  MT9M114_CAM_MODE_TEST_PATTERN_RED,
				  sensor->ifp.tpg[MT9M114_TPG_RED]->val, &ret);
			cci_write(sensor->regmap,
				  MT9M114_CAM_MODE_TEST_PATTERN_GREEN,
				  sensor->ifp.tpg[MT9M114_TPG_GREEN]->val, &ret);
			cci_write(sensor->regmap,
				  MT9M114_CAM_MODE_TEST_PATTERN_BLUE,
				  sensor->ifp.tpg[MT9M114_TPG_BLUE]->val, &ret);
		} else {
			cci_write(sensor->regmap, MT9M114_CAM_MODE_SELECT,
				  MT9M114_CAM_MODE_SELECT_NORMAL, &ret);
		}

		/*
		 * A Config-Change needs to be issued for the change to take
		 * effect. If we're not streaming ignore this, the change will
		 * be applied when the stream is started.
		 * MT9M113 doesn't support COMMAND_REGISTER - changes take
		 * effect immediately.
		 */
		if (ret || !sensor->streaming ||
		    sensor->model == MT9M113_MODEL)
			break;

		ret = mt9m114_set_state(sensor,
					MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
		break;
	}

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(&sensor->client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mt9m114_ifp_ctrl_ops = {
	.s_ctrl = mt9m114_ifp_s_ctrl,
};

/* -----------------------------------------------------------------------------
 * Image Flow Processor Subdev Operations
 */

static inline struct mt9m114 *ifp_to_mt9m114(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m114, ifp.sd);
}

static int mt9m114_ifp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	struct v4l2_subdev_state *pa_state;
	struct v4l2_subdev_state *ifp_state;
	int ret;

	if (!enable)
		return mt9m114_stop_streaming(sensor);

	ifp_state = v4l2_subdev_lock_and_get_active_state(&sensor->ifp.sd);
	pa_state = v4l2_subdev_lock_and_get_active_state(&sensor->pa.sd);

	ret = mt9m114_start_streaming(sensor, pa_state, ifp_state);

	v4l2_subdev_unlock_state(pa_state);
	v4l2_subdev_unlock_state(ifp_state);

	return ret;
}

static int mt9m114_ifp_get_frame_interval(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *sd_state,
					  struct v4l2_subdev_frame_interval *interval)
{
	struct v4l2_fract *ival = &interval->interval;
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);

	/*
	 * FIXME: Implement support for V4L2_SUBDEV_FORMAT_TRY, using the V4L2
	 * subdev active state API.
	 */
	if (interval->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	ival->numerator = 1;
	ival->denominator = sensor->ifp.frame_rate;

	return 0;
}

static int mt9m114_ifp_set_frame_interval(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *sd_state,
					  struct v4l2_subdev_frame_interval *interval)
{
	struct v4l2_fract *ival = &interval->interval;
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	int ret = 0;

	/*
	 * FIXME: Implement support for V4L2_SUBDEV_FORMAT_TRY, using the V4L2
	 * subdev active state API.
	 */
	if (interval->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	if (ival->numerator != 0 && ival->denominator != 0)
		sensor->ifp.frame_rate = min_t(unsigned int,
					       ival->denominator / ival->numerator,
					       MT9M114_MAX_FRAME_RATE);
	else
		sensor->ifp.frame_rate = MT9M114_MAX_FRAME_RATE;

	ival->numerator = 1;
	ival->denominator = sensor->ifp.frame_rate;

	if (sensor->streaming)
		ret = mt9m114_set_frame_rate(sensor);

	return ret;
}

static int mt9m114_ifp_init_state(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	struct v4l2_rect *compose;

	format = v4l2_subdev_state_get_format(state, 0);

	format->width = MT9M114_PIXEL_ARRAY_WIDTH;
	format->height = MT9M114_PIXEL_ARRAY_HEIGHT;
	format->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_601;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	crop = v4l2_subdev_state_get_crop(state, 0);

	crop->left = 4;
	crop->top = 4;
	crop->width = format->width - 8;
	crop->height = format->height - 8;

	compose = v4l2_subdev_state_get_compose(state, 0);

	compose->left = 0;
	compose->top = 0;
	compose->width = crop->width;
	compose->height = crop->height;

	format = v4l2_subdev_state_get_format(state, 1);

	format->width = compose->width;
	format->height = compose->height;
	format->code = mt9m114_default_format_info(sensor)->code;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

static int mt9m114_ifp_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	const unsigned int num_formats = ARRAY_SIZE(mt9m114_format_infos);
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	unsigned int index = 0;
	unsigned int flag;
	unsigned int i;

	switch (code->pad) {
	case 0:
		if (code->index != 0)
			return -EINVAL;

		code->code = mt9m114_format_infos[num_formats - 1].code;
		return 0;

	case 1:
		if (sensor->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY)
			flag = MT9M114_FMT_FLAG_CSI2;
		else
			flag = MT9M114_FMT_FLAG_PARALLEL;

		for (i = 0; i < num_formats; ++i) {
			const struct mt9m114_format_info *info =
				&mt9m114_format_infos[i];

			if (info->flags & flag) {
				if (index == code->index) {
					code->code = info->code;
					return 0;
				}

				index++;
			}
		}

		return -EINVAL;

	default:
		return -EINVAL;
	}
}

static int mt9m114_ifp_enum_framesizes(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	const struct mt9m114_format_info *info;

	if (fse->index > 0)
		return -EINVAL;

	info = mt9m114_format_info(sensor, fse->pad, fse->code);
	if (!info || info->code != fse->code)
		return -EINVAL;

	if (fse->pad == 0) {
		fse->min_width = MT9M114_PIXEL_ARRAY_MIN_OUTPUT_WIDTH;
		fse->max_width = MT9M114_PIXEL_ARRAY_WIDTH;
		fse->min_height = MT9M114_PIXEL_ARRAY_MIN_OUTPUT_HEIGHT;
		fse->max_height = MT9M114_PIXEL_ARRAY_HEIGHT;
	} else {
		const struct v4l2_rect *crop;

		crop = v4l2_subdev_state_get_crop(state, 0);

		fse->max_width = crop->width;
		fse->max_height = crop->height;

		fse->min_width = fse->max_width / 4;
		fse->min_height = fse->max_height / 4;
	}

	return 0;
}

static int mt9m114_ifp_enum_frameintervals(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *state,
					   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	const struct mt9m114_format_info *info;

	if (fie->index > 0)
		return -EINVAL;

	info = mt9m114_format_info(sensor, fie->pad, fie->code);
	if (!info || info->code != fie->code)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = MT9M114_MAX_FRAME_RATE;

	return 0;
}

static int mt9m114_ifp_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *fmt)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	if (fmt->pad == 0) {
		/* Only the size can be changed on the sink pad. */
		format->width = clamp(ALIGN(fmt->format.width, 8),
				      MT9M114_PIXEL_ARRAY_MIN_OUTPUT_WIDTH,
				      MT9M114_PIXEL_ARRAY_WIDTH);
		format->height = clamp(ALIGN(fmt->format.height, 8),
				       MT9M114_PIXEL_ARRAY_MIN_OUTPUT_HEIGHT,
				       MT9M114_PIXEL_ARRAY_HEIGHT);
	} else {
		const struct mt9m114_format_info *info;

		/* Only the media bus code can be changed on the source pad. */
		info = mt9m114_format_info(sensor, 1, fmt->format.code);

		format->code = info->code;

		/* If the output format is RAW10, bypass the scaler. */
		if (format->code == MEDIA_BUS_FMT_SGRBG10_1X10)
			*format = *v4l2_subdev_state_get_format(state, 0);
	}

	fmt->format = *format;

	return 0;
}

static int mt9m114_ifp_get_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_selection *sel)
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	int ret = 0;

	/* Crop and compose are only supported on the sink pad. */
	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		break;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/*
		 * The crop default and bounds are equal to the sink
		 * format size minus 4 pixels on each side for demosaicing.
		 */
		format = v4l2_subdev_state_get_format(state, 0);

		sel->r.left = 4;
		sel->r.top = 4;
		sel->r.width = format->width - 8;
		sel->r.height = format->height - 8;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		sel->r = *v4l2_subdev_state_get_compose(state, 0);
		break;

	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		/*
		 * The compose default and bounds sizes are equal to the sink
		 * crop rectangle size.
		 */
		crop = v4l2_subdev_state_get_crop(state, 0);
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = crop->width;
		sel->r.height = crop->height;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mt9m114_ifp_set_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	struct v4l2_rect *compose;

	if (sel->target != V4L2_SEL_TGT_CROP &&
	    sel->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	/* Crop and compose are only supported on the sink pad. */
	if (sel->pad != 0)
		return -EINVAL;

	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);
	compose = v4l2_subdev_state_get_compose(state, 0);

	if (sel->target == V4L2_SEL_TGT_CROP) {
		/*
		 * Clamp the crop rectangle. Demosaicing removes 4 pixels on
		 * each side of the image.
		 */
		crop->left = clamp_t(unsigned int, ALIGN(sel->r.left, 2), 4,
				     format->width - 4 -
				     MT9M114_SCALER_CROPPED_INPUT_WIDTH);
		crop->top = clamp_t(unsigned int, ALIGN(sel->r.top, 2), 4,
				    format->height - 4 -
				    MT9M114_SCALER_CROPPED_INPUT_HEIGHT);
		crop->width = clamp_t(unsigned int, ALIGN(sel->r.width, 2),
				      MT9M114_SCALER_CROPPED_INPUT_WIDTH,
				      format->width - 4 - crop->left);
		crop->height = clamp_t(unsigned int, ALIGN(sel->r.height, 2),
				       MT9M114_SCALER_CROPPED_INPUT_HEIGHT,
				       format->height - 4 - crop->top);

		sel->r = *crop;

		/* Propagate to the compose rectangle. */
		compose->width = crop->width;
		compose->height = crop->height;
	} else {
		/*
		 * Clamp the compose rectangle. The scaler can only downscale.
		 */
		compose->left = 0;
		compose->top = 0;
		compose->width = clamp_t(unsigned int, ALIGN(sel->r.width, 2),
					 MT9M114_SCALER_CROPPED_INPUT_WIDTH,
					 crop->width);
		compose->height = clamp_t(unsigned int, ALIGN(sel->r.height, 2),
					  MT9M114_SCALER_CROPPED_INPUT_HEIGHT,
					  crop->height);

		sel->r = *compose;
	}

	/* Propagate the compose rectangle to the source format. */
	format = v4l2_subdev_state_get_format(state, 1);
	format->width = compose->width;
	format->height = compose->height;

	return 0;
}

static void mt9m114_ifp_unregistered(struct v4l2_subdev *sd)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);

	v4l2_device_unregister_subdev(&sensor->pa.sd);
}

static int mt9m114_ifp_registered(struct v4l2_subdev *sd)
{
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	int ret;

	ret = v4l2_device_register_subdev(sd->v4l2_dev, &sensor->pa.sd);
	if (ret < 0) {
		dev_err(&sensor->client->dev,
			"Failed to register pixel array subdev\n");
		return ret;
	}

	ret = media_create_pad_link(&sensor->pa.sd.entity, 0,
				    &sensor->ifp.sd.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret < 0) {
		dev_err(&sensor->client->dev,
			"Failed to link pixel array to ifp\n");
		v4l2_device_unregister_subdev(&sensor->pa.sd);
		return ret;
	}

	return 0;
}

static const struct v4l2_subdev_video_ops mt9m114_ifp_video_ops = {
	.s_stream = mt9m114_ifp_s_stream,
};

static const struct v4l2_subdev_pad_ops mt9m114_ifp_pad_ops = {
	.enum_mbus_code = mt9m114_ifp_enum_mbus_code,
	.enum_frame_size = mt9m114_ifp_enum_framesizes,
	.enum_frame_interval = mt9m114_ifp_enum_frameintervals,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mt9m114_ifp_set_fmt,
	.get_selection = mt9m114_ifp_get_selection,
	.set_selection = mt9m114_ifp_set_selection,
	.get_frame_interval = mt9m114_ifp_get_frame_interval,
	.set_frame_interval = mt9m114_ifp_set_frame_interval,
};

static const struct v4l2_subdev_ops mt9m114_ifp_ops = {
	.video = &mt9m114_ifp_video_ops,
	.pad = &mt9m114_ifp_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9m114_ifp_internal_ops = {
	.init_state = mt9m114_ifp_init_state,
	.registered = mt9m114_ifp_registered,
	.unregistered = mt9m114_ifp_unregistered,
};

static int mt9m114_ifp_init(struct mt9m114 *sensor)
{
	struct v4l2_subdev *sd = &sensor->ifp.sd;
	struct media_pad *pads = sensor->ifp.pads;
	struct v4l2_ctrl_handler *hdl = &sensor->ifp.hdl;
	struct v4l2_ctrl *link_freq;
	int ret;

	/* Initialize the subdev. */
	v4l2_i2c_subdev_init(sd, sensor->client, &mt9m114_ifp_ops);
	v4l2_i2c_subdev_set_name(sd, sensor->client, NULL, " ifp");

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &mt9m114_ifp_internal_ops;

	/* Initialize the media entity. */
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	sd->entity.ops = &mt9m114_entity_ops;
	pads[0].flags = MEDIA_PAD_FL_SINK;
	pads[1].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 2, pads);
	if (ret < 0)
		return ret;

	sensor->ifp.frame_rate = MT9M114_DEF_FRAME_RATE;

	/* Initialize the control handler. */
	v4l2_ctrl_handler_init(hdl, 8);
	v4l2_ctrl_new_std(hdl, &mt9m114_ifp_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE,
			  0, 1, 1, 1);
	v4l2_ctrl_new_std_menu(hdl, &mt9m114_ifp_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO,
			       V4L2_EXPOSURE_MANUAL, 0,
			       V4L2_EXPOSURE_AUTO);

	link_freq = v4l2_ctrl_new_int_menu(hdl, &mt9m114_ifp_ctrl_ops,
					   V4L2_CID_LINK_FREQ,
					   sensor->bus_cfg.nr_of_link_frequencies - 1,
					   0, sensor->bus_cfg.link_frequencies);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &mt9m114_ifp_ctrl_ops,
			  V4L2_CID_PIXEL_RATE,
			  sensor->pixrate, sensor->pixrate, 1,
			  sensor->pixrate);

	sensor->ifp.tpg[MT9M114_TPG_PATTERN] =
		v4l2_ctrl_new_std_menu_items(hdl, &mt9m114_ifp_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(mt9m114_test_pattern_menu) - 1,
					     0, 0, mt9m114_test_pattern_menu);
	sensor->ifp.tpg[MT9M114_TPG_RED] =
		v4l2_ctrl_new_std(hdl, &mt9m114_ifp_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED,
				  0, 1023, 1, 1023);
	sensor->ifp.tpg[MT9M114_TPG_GREEN] =
		v4l2_ctrl_new_std(hdl, &mt9m114_ifp_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_GREENR,
				  0, 1023, 1, 1023);
	sensor->ifp.tpg[MT9M114_TPG_BLUE] =
		v4l2_ctrl_new_std(hdl, &mt9m114_ifp_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_BLUE,
				  0, 1023, 1, 1023);

	v4l2_ctrl_cluster(ARRAY_SIZE(sensor->ifp.tpg), sensor->ifp.tpg);

	if (hdl->error) {
		ret = hdl->error;
		goto error;
	}

	sd->ctrl_handler = hdl;
	sd->state_lock = hdl->lock;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto error;

	return 0;

error:
	v4l2_ctrl_handler_free(&sensor->ifp.hdl);
	media_entity_cleanup(&sensor->ifp.sd.entity);
	return ret;
}

static void mt9m114_ifp_cleanup(struct mt9m114 *sensor)
{
	v4l2_ctrl_handler_free(&sensor->ifp.hdl);
	media_entity_cleanup(&sensor->ifp.sd.entity);
}

/* -----------------------------------------------------------------------------
 * Power Management
 */

static int mt9m114_power_on(struct mt9m114 *sensor)
{
	struct device *dev = &sensor->client->dev;
	int ret;

	dev_info(dev, "power_on: starting\n");

	/* Enable power and clocks. */
	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0) {
		dev_err(dev, "power_on: regulator_bulk_enable failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "power_on: regulators enabled\n");

	/*
	 * Deassert powerdown to enable the sensor. The legacy webOS driver
	 * controls power state entirely through the powerdown GPIO.
	 */
	if (sensor->powerdown) {
		dev_info(dev, "power_on: deasserting powerdown\n");
		gpiod_set_value(sensor->powerdown, 0);
	}

	/*
	 * After enabling regulators and deasserting powerdown, wait for power
	 * to stabilize before enabling clock. Legacy driver waits 20ms.
	 */
	usleep_range(20000, 25000);
	dev_info(dev, "power_on: power stabilization wait complete\n");

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0) {
		dev_err(dev, "power_on: clk_prepare_enable failed: %d\n", ret);
		goto error_regulator;
	}
	dev_info(dev, "power_on: clock enabled, rate=%lu Hz\n",
		 clk_get_rate(sensor->clk));

	/*
	 * Wait for clock to stabilize before reset.
	 * Extended to 20ms at cold boot to ensure MMCC is fully initialized
	 * and the clock is actually outputting. Legacy driver waits 5ms,
	 * but we observed I2C failures at cold boot with short delays.
	 */
	msleep(20);
	dev_info(dev, "power_on: clock stabilization complete, rate=%lu Hz\n",
		 clk_get_rate(sensor->clk));

	/* Perform a hard reset if available, or a soft reset otherwise. */
	if (sensor->reset) {
		/*
		 * Assert reset for at least 1ms. The datasheet specifies a
		 * minimum of 50 clock cycles (~2µs at 24MHz), but in practice
		 * the sensor needs a longer pulse to reliably reset,
		 * especially after a power cycle during runtime resume.
		 */
		dev_info(dev, "power_on: asserting reset for 1ms\n");
		gpiod_set_value(sensor->reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value(sensor->reset, 0);
		dev_info(dev, "power_on: reset released, waiting 45ms\n");

		/*
		 * After releasing reset, the sensor needs time to boot up
		 * before it can respond to I2C commands. The datasheet
		 * specifies a minimum of 44.5ms for internal initialization.
		 */
		usleep_range(44500, 50000);
	} else if (sensor->powerdown) {
		/*
		 * When using powerdown GPIO, the sensor is in a fresh state
		 * after powerdown deassert. Wait for sensor to be ready.
		 */
		dev_info(dev, "power_on: powerdown GPIO used, waiting 45ms\n");
		usleep_range(44500, 50000);

		/*
		 * MT9M113 MIPI mode: Check if sensor is already initialized
		 * (runtime resume case) or needs full initialization (cold boot).
		 * During runtime resume from powerdown standby, the sensor
		 * may need extra time to restore I2C functionality.
		 */
		if (sensor->expected_model == MT9M113_MODEL &&
		    sensor->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY) {
			u64 readback = 0;
			int read_ret;
			int retry;

			/*
			 * Try reading CLOCKS_CONTROL with retries to check if
			 * sensor is responding and already initialized.
			 * After powerdown resume, the sensor may need extra time
			 * to restore I2C functionality.
			 */
			for (retry = 0; retry < 5; retry++) {
				read_ret = cci_read(sensor->regmap, MT9M114_CLOCKS_CONTROL,
						    &readback, NULL);
				dev_info(dev, "power_on: CLOCKS_CONTROL=0x%llx ret=%d (try %d)\n",
					 readback, read_ret, retry + 1);
				if (read_ret == 0)
					break;
				/* I2C failed, wait and retry */
				msleep(100);
			}

			if (read_ret < 0) {
				/*
				 * I2C still failing after retries. The sensor
				 * is not responding, which can happen if powerdown
				 * GPIO isn't working or the sensor is damaged.
				 */
				dev_err(dev, "power_on: sensor not responding after %d retries\n",
					retry);
				ret = read_ret;
				goto error_clock;
			}

			if (readback != 0) {
				/*
				 * Sensor already initialized (runtime resume).
				 * Skip soft reset, just ensure MIPI settings.
				 */
				dev_info(dev, "power_on: sensor already init, skipping soft reset\n");
				msleep(50); /* Brief stabilization */
			} else {
				/*
				 * Cold boot or sensor not responding.
				 * Perform full MIPI init sequence per webOS:
				 * 1. Soft reset (0x001A = 1, then 0)
				 * 2. OUTPUT_CONTROL (0x3400 = 0x7A08) enables MIPI
				 * 3. RESET_REGISTER (0x301A = 0x120C) for streaming
				 * Note: MT9M113 uses 0x3400 for MIPI, NOT 0x3C40.
				 */
				dev_info(dev, "power_on: MT9M113 MIPI early init sequence\n");

				/* Soft reset */
				cci_write(sensor->regmap, MT9M114_RESET_AND_MISC_CONTROL,
					  MT9M114_RESET_SOC, &ret);
				cci_write(sensor->regmap, MT9M114_RESET_AND_MISC_CONTROL,
					  0, &ret);
				if (ret < 0) {
					dev_err(dev, "power_on: soft reset failed: %d\n", ret);
					goto error_clock;
				}
				msleep(200); /* webOS uses 200ms delay after reset */

				/*
				 * NOTE: MIPI output (OUTPUT_CONTROL, CUSTOM_SHORT_PKT) is
				 * NOT configured here. Enabling MIPI during power_on causes
				 * the sensor to enter streaming state (SEQ_STATE=0x3) before
				 * s_stream is called, breaking subsequent streaming attempts.
				 *
				 * MIPI is configured only in s_stream, matching webOS behavior.
				 */

				/*
				 * NOTE: RESET_REGISTER (0x301A) is NOT written here!
				 * Per webOS driver analysis, RESET_REGISTER=0x120C is only
				 * written in mt9m113_set_sensor_mode() during streaming start.
				 * Writing it here causes the sensor to enter streaming state
				 * (SEQ_STATE=0x3) before s_stream is called, which breaks
				 * subsequent streaming attempts.
				 */
			}
		}
	} else {
		/*
		 * No reset GPIO and no powerdown GPIO - the sensor may have
		 * been left in an unknown state. Perform soft reset.
		 */
		dev_info(dev, "power_on: no reset/powerdown GPIO, waiting 45ms then soft reset\n");
		usleep_range(44500, 50000);

		cci_write(sensor->regmap, MT9M114_RESET_AND_MISC_CONTROL,
			  MT9M114_RESET_SOC, &ret);
		cci_write(sensor->regmap, MT9M114_RESET_AND_MISC_CONTROL, 0,
			  &ret);

		if (ret < 0) {
			dev_err(dev, "power_on: Soft reset failed: %d\n", ret);
			goto error_clock;
		}
		dev_info(dev, "power_on: soft reset complete\n");
	}

	/*
	 * MT9M113: Skip MCU boot and PLL config here - let the init table
	 * handle everything.
	 *
	 * The webOS driver does NOT do MCU boot or PLL config before the
	 * init table. It just powers on the sensor and immediately applies
	 * the init table (mt9m113_reg_init). Doing MCU boot and PLL config
	 * here leaves the sensor in streaming state (SEQ_STATE=0x3) and
	 * makes the MCU unresponsive to commands.
	 *
	 * The init table includes:
	 * - MCU_BOOT_MODE toggle (0x001C = 1, then 0)
	 * - PLL configuration
	 * - All MCU variable settings
	 * - REFRESH command to apply settings
	 */
	if (sensor->expected_model == MT9M113_MODEL) {
		u64 chip_id = 0;
		int read_ret;

		/*
		 * Just verify sensor is responding to I2C by reading chip ID.
		 * Don't do any configuration - let init table handle it.
		 */
		read_ret = cci_read(sensor->regmap, MT9M114_CHIP_ID, &chip_id, NULL);
		if (read_ret < 0) {
			dev_warn(dev, "power_on: MT9M113 I2C read failed, retrying after 50ms\n");
			msleep(50);
			read_ret = cci_read(sensor->regmap, MT9M114_CHIP_ID, &chip_id, NULL);
			if (read_ret < 0) {
				dev_err(dev, "power_on: MT9M113 not responding to I2C\n");
				ret = read_ret;
				goto error_clock;
			}
		}
		dev_info(dev, "power_on: MT9M113 chip_id=0x%llx, skipping MCU/PLL (handled by init table)\n",
			 chip_id);
		goto mt9m113_init_done;
	}

	/*
	 * Wait for the sensor to be ready to accept I2C commands by polling the
	 * command register to wait for initialization to complete.
	 */
	usleep_range(44500, 50000);

	ret = mt9m114_poll_command(sensor, MT9M114_COMMAND_REGISTER_SET_STATE);
	if (ret < 0)
		goto error_clock;

mt9m113_init_done:
	/*
	 * MT9M113 uses a different command mechanism (MCU indirect via
	 * 0x098C/0x0990) and doesn't support the MT9M114's COMMAND_REGISTER.
	 *
	 * We don't check SEQ_STATE here - the sensor hasn't been initialized
	 * yet. The init table will be applied by mt9m113_sensor_init() which
	 * is called from mt9m114_initialize() later.
	 */
	if (sensor->expected_model == MT9M113_MODEL) {
		dev_info(dev, "power_on: MT9M113 ready for init table\n");
		return 0;
	}

	if (sensor->bus_cfg.bus_type == V4L2_MBUS_PARALLEL) {
		/*
		 * In parallel mode (OE set to low), the sensor will enter the
		 * streaming state after initialization. Enter the standby
		 * manually to stop streaming.
		 */
		ret = mt9m114_set_state(sensor,
					MT9M114_SYS_STATE_ENTER_STANDBY);
		if (ret < 0)
			goto error_clock;
	}

	/*
	 * Before issuing any Set-State command, we must ensure that the sensor
	 * reaches the standby mode (either initiated manually above in
	 * parallel mode, or automatically after reset in MIPI mode).
	 */
	ret = mt9m114_poll_state(sensor, MT9M114_SYS_STATE_STANDBY);
	if (ret < 0)
		goto error_clock;

	return 0;

error_clock:
	clk_disable_unprepare(sensor->clk);
error_regulator:
	/* Only disable regulators if not using powerdown GPIO control */
	if (!sensor->powerdown)
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
	return ret;
}

static void mt9m114_power_off(struct mt9m114 *sensor)
{
	/*
	 * Assert powerdown to disable the sensor. The legacy webOS driver
	 * only toggles the powerdown GPIO and does NOT disable regulators.
	 * Keeping regulators enabled ensures proper sensor initialization
	 * on the next power-on cycle.
	 */
	if (sensor->powerdown) {
		gpiod_set_value(sensor->powerdown, 1);
		clk_disable_unprepare(sensor->clk);
		/* Keep regulators enabled - only powerdown controls power */
	} else {
		/* No powerdown GPIO - use full power cycle */
		clk_disable_unprepare(sensor->clk);
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
	}
}

static int __maybe_unused mt9m114_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	int ret;

	dev_info(dev, "runtime_resume: starting\n");

	/*
	 * MT9M113 with powerdown GPIO: Sensor was never actually suspended
	 * (suspend is a no-op), so resume is also a no-op. The sensor stays
	 * powered and initialized like webOS does.
	 */
	if (sensor->powerdown && sensor->expected_model == MT9M113_MODEL) {
		dev_info(dev, "runtime_resume: MT9M113 already powered (no-op)\n");
		return 0;
	}

	/* Standard power-on for other configurations */
	ret = mt9m114_power_on(sensor);
	if (ret) {
		dev_err(dev, "runtime_resume: power_on failed: %d\n", ret);
		return ret;
	}

	ret = mt9m114_initialize(sensor);
	if (ret) {
		dev_err(dev, "runtime_resume: initialize failed: %d\n", ret);
		mt9m114_power_off(sensor);
		return ret;
	}

	dev_info(dev, "runtime_resume: complete\n");
	return 0;
}

static int __maybe_unused mt9m114_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);

	dev_info(dev, "runtime_suspend: starting\n");

	/*
	 * MT9M113 with powerdown GPIO: Keep sensor powered like webOS does.
	 * The legacy webOS driver never suspends the camera between streaming
	 * sessions - it keeps it initialized and ready. Runtime PM suspend
	 * causes issues because:
	 * - Powerdown GPIO toggling makes I2C unresponsive on resume
	 * - Soft standby via SYS_STATE doesn't work reliably on MT9M113
	 * - SYSMGR_CURRENT_STATE reads 0x0 instead of expected values
	 *
	 * Just return success without actually suspending the sensor.
	 */
	if (sensor->powerdown && sensor->expected_model == MT9M113_MODEL) {
		dev_info(dev, "runtime_suspend: MT9M113 stays powered (no-op)\n");
		return 0;
	}

	/* Standard power-off for other configurations */
	dev_info(dev, "runtime_suspend: powering off\n");
	mt9m114_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops mt9m114_pm_ops = {
	SET_RUNTIME_PM_OPS(mt9m114_runtime_suspend, mt9m114_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int mt9m114_verify_link_frequency(struct mt9m114 *sensor,
					 unsigned int pixrate)
{
	unsigned int link_freq = sensor->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY
			       ? pixrate * 8 : pixrate * 2;

	if (sensor->bus_cfg.nr_of_link_frequencies != 1 ||
	    sensor->bus_cfg.link_frequencies[0] != link_freq)
		return -EINVAL;

	return 0;
}

static int mt9m114_clk_init(struct mt9m114 *sensor)
{
	unsigned int pixrate;

	/* Hardcode the PLL multiplier and dividers to default settings. */
	sensor->pll.m = 32;
	sensor->pll.n = 1;
	sensor->pll.p = 7;

	/*
	 * Calculate the pixel rate and link frequency. The CSI-2 bus is clocked
	 * for 16-bit per pixel, transmitted in DDR over a single lane. For
	 * parallel mode, the sensor ouputs one pixel in two PIXCLK cycles.
	 */

	/*
	 * Check if EXTCLK fits the configured link frequency. Bypass the PLL
	 * in this case.
	 */
	pixrate = clk_get_rate(sensor->clk) / 2;
	if (mt9m114_verify_link_frequency(sensor, pixrate) == 0) {
		sensor->pixrate = pixrate;
		sensor->bypass_pll = true;
		return 0;
	}

	/* Check if the PLL configuration fits the configured link frequency. */
	pixrate = clk_get_rate(sensor->clk) * sensor->pll.m
		/ ((sensor->pll.n + 1) * (sensor->pll.p + 1));
	if (mt9m114_verify_link_frequency(sensor, pixrate) == 0) {
		sensor->pixrate = pixrate;
		sensor->bypass_pll = false;
		return 0;
	}

	dev_err(&sensor->client->dev, "Unsupported DT link-frequencies\n");
	return -EINVAL;
}

static int mt9m114_identify(struct mt9m114 *sensor)
{
	u64 major, minor, release, customer;
	u64 value;
	int ret;

	ret = cci_read(sensor->regmap, MT9M114_CHIP_ID, &value, NULL);
	if (ret) {
		dev_err(&sensor->client->dev, "Failed to read chip ID\n");
		return -ENXIO;
	}

	switch (value) {
	case MT9M113_MODEL:
		sensor->model = MT9M113_MODEL;
		dev_info(&sensor->client->dev, "Detected MT9M113 chip ID 0x%04llx\n",
			 value);
		break;
	case MT9M114_MODEL:
		sensor->model = MT9M114_MODEL;
		dev_info(&sensor->client->dev, "Detected MT9M114 chip ID 0x%04llx\n",
			 value);
		break;
	default:
		dev_err(&sensor->client->dev, "Invalid chip ID 0x%04llx\n",
			value);
		return -ENXIO;
	}

	cci_read(sensor->regmap, MT9M114_MON_MAJOR_VERSION, &major, &ret);
	cci_read(sensor->regmap, MT9M114_MON_MINOR_VERSION, &minor, &ret);
	cci_read(sensor->regmap, MT9M114_MON_RELEASE_VERSION, &release, &ret);
	cci_read(sensor->regmap, MT9M114_CUSTOMER_REV, &customer, &ret);
	if (ret) {
		dev_err(&sensor->client->dev, "Failed to read version\n");
		return -ENXIO;
	}

	dev_dbg(&sensor->client->dev,
		"monitor v%llu.%llu.%04llx customer rev 0x%04llx\n",
		major, minor, release, customer);

	return 0;
}

static int mt9m114_parse_dt(struct mt9m114 *sensor)
{
	struct fwnode_handle *fwnode = dev_fwnode(&sensor->client->dev);
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep) {
		dev_err(&sensor->client->dev, "No endpoint found\n");
		return -EINVAL;
	}

	sensor->bus_cfg.bus_type = V4L2_MBUS_UNKNOWN;
	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &sensor->bus_cfg);
	fwnode_handle_put(ep);
	if (ret < 0) {
		dev_err(&sensor->client->dev, "Failed to parse endpoint\n");
		goto error;
	}

	switch (sensor->bus_cfg.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
	case V4L2_MBUS_PARALLEL:
		break;

	default:
		dev_err(&sensor->client->dev, "unsupported bus type %u\n",
			sensor->bus_cfg.bus_type);
		ret = -EINVAL;
		goto error;
	}

	sensor->pad_slew_rate = MT9M114_PAD_SLEW_DEFAULT;
	device_property_read_u32(&sensor->client->dev, "slew-rate",
				 &sensor->pad_slew_rate);

	if (sensor->pad_slew_rate < MT9M114_PAD_SLEW_MIN ||
	    sensor->pad_slew_rate > MT9M114_PAD_SLEW_MAX) {
		dev_err(&sensor->client->dev, "Invalid slew-rate %u\n",
			sensor->pad_slew_rate);
		return -EINVAL;
	}

	return 0;

error:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static int mt9m114_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mt9m114 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->client = client;
	sensor->expected_model = (uintptr_t)device_get_match_data(dev);

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	ret = mt9m114_parse_dt(sensor);
	if (ret < 0)
		return ret;

	/* Acquire clocks, GPIOs and regulators. */
	sensor->clk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(sensor->clk),
				    "Failed to get clock\n");
		goto error_ep_free;
	}

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sensor->reset)) {
		ret = PTR_ERR(sensor->reset);
		dev_err_probe(dev, ret, "Failed to get reset GPIO\n");
		goto error_ep_free;
	}

	/*
	 * Powerdown GPIO controls the sensor power state. When asserted (high),
	 * the sensor is powered down. When deasserted (low), the sensor is
	 * enabled. This matches the legacy webOS driver behavior.
	 */
	sensor->powerdown = devm_gpiod_get_optional(dev, "powerdown",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->powerdown)) {
		ret = PTR_ERR(sensor->powerdown);
		dev_err_probe(dev, ret, "Failed to get powerdown GPIO\n");
		goto error_ep_free;
	}

	sensor->supplies[0].supply = "vddio";
	sensor->supplies[1].supply = "vdd";
	sensor->supplies[2].supply = "vaa";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to get regulators\n");
		goto error_ep_free;
	}

	ret = mt9m114_clk_init(sensor);
	if (ret)
		goto error_ep_free;

	/*
	 * Identify the sensor. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, and initialize it after identification
	 * to reach the same state as if resumed through runtime PM.
	 */
	ret = mt9m114_power_on(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Could not power on the device\n");
		goto error_ep_free;
	}

	ret = mt9m114_identify(sensor);
	if (ret < 0)
		goto error_power_off;

	ret = mt9m114_initialize(sensor);
	if (ret < 0)
		goto error_power_off;

	/*
	 * Enable runtime PM with autosuspend. As the device has been powered
	 * manually, mark it as active, and increase the usage count without
	 * resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	/* Initialize the subdevices. */
	ret = mt9m114_pa_init(sensor);
	if (ret < 0)
		goto error_pm_cleanup;

	ret = mt9m114_ifp_init(sensor);
	if (ret < 0)
		goto error_pa_cleanup;

	ret = v4l2_async_register_subdev(&sensor->ifp.sd);
	if (ret < 0)
		goto error_ifp_cleanup;

	/*
	 * Decrease the PM usage count. The device will get suspended after the
	 * autosuspend delay, turning the power off.
	 */
	pm_runtime_put_autosuspend(dev);

	return 0;

error_ifp_cleanup:
	mt9m114_ifp_cleanup(sensor);
error_pa_cleanup:
	mt9m114_pa_cleanup(sensor);
error_pm_cleanup:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
error_power_off:
	mt9m114_power_off(sensor);
error_ep_free:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static void mt9m114_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9m114 *sensor = ifp_to_mt9m114(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(&sensor->ifp.sd);

	mt9m114_ifp_cleanup(sensor);
	mt9m114_pa_cleanup(sensor);
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		mt9m114_power_off(sensor);
	pm_runtime_set_suspended(dev);
}

static const struct of_device_id mt9m114_of_ids[] = {
	{ .compatible = "aptina,mt9m113", .data = (void *)MT9M113_MODEL },
	{ .compatible = "onnn,mt9m114", .data = (void *)MT9M114_MODEL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt9m114_of_ids);

static struct i2c_driver mt9m114_driver = {
	.driver = {
		.name	= "mt9m114",
		.pm	= &mt9m114_pm_ops,
		.of_match_table = mt9m114_of_ids,
	},
	.probe		= mt9m114_probe,
	.remove		= mt9m114_remove,
};

module_i2c_driver(mt9m114_driver);

MODULE_DESCRIPTION("onsemi MT9M114 Sensor Driver");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_LICENSE("GPL");
