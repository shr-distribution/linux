/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Palm A6 Battery Controller Driver - private definitions
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 */

#ifndef _A6_INTERNAL_H_
#define _A6_INTERNAL_H_

#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/power_supply.h>
#include <linux/gpio/consumer.h>

#include <linux/a6.h>
#include <linux/a6_sbw_interface.h>

#define A2A_RD_BUFF_SIZE (4 * 1024)
#define A2A_WR_BUFF_SIZE (4 * 1024)

#define A6_DEBUG
#define A6_PQ
/* #define A6_I2C_RETRY */

#ifdef A6_DEBUG
#define ASSERT(i)  WARN_ON_ONCE(!(i))
#else
#define ASSERT(x)  ((void)(x))
#endif

enum {
	A6_DEBUG_VERBOSE = 0x01,
};

extern int a6_debug_mask;
extern int a6_tp_irq_count;
extern int a6_t2s_dup_correct;

#define A6_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & a6_debug_mask) \
			printk(level "%s" message, "", ##__VA_ARGS__); \
	} while (0)

#define PROFILE_USAGE
#if defined PROFILE_USAGE
extern bool reset_active;
extern uint32_t start_time;
extern int32_t diff_time;

extern uint32_t start_last_a6_activity;
#endif

/* page 0x00 */
/* host interrupts */
#define TS2_I2C_INT_MASK_0                             0x0000
#define TS2_I2C_INT_MASK_1                             0x0001
#define TS2_I2C_INT_MASK_2                             0x0002
#define TS2_I2C_INT_MASK_3                             0x0003
#define TS2_I2C_INT_STATUS_0                           0x0004
#define TS2_I2C_INT_STATUS_1                           0x0005
#define TS2_I2C_INT_STATUS_2                           0x0006
#define TS2_I2C_INT_STATUS_3                           0x0007
#define TS2_I2C_INT_0_GPIOA_7                          0x80
#define TS2_I2C_INT_0_GPIOA_6                          0x40
#define TS2_I2C_INT_0_GPIOA_5                          0x20
#define TS2_I2C_INT_0_GPIOA_4                          0x10
#define TS2_I2C_INT_0_GPIOA_3                          0x08
#define TS2_I2C_INT_0_GPIOA_2                          0x04
#define TS2_I2C_INT_0_GPIOA_1                          0x02
#define TS2_I2C_INT_0_GPIOA_0                          0x01
#define TS2_I2C_INT_0_GPIOA_MASK                       0xff
#define TS2_I2C_INT_1_COMM_RX_FULL                     0x02
#define TS2_I2C_INT_1_COMM_TX_EMPTY                    0x01
#define TS2_I2C_INT_2_BAT_TEMP_HIGH                    0x20
#define TS2_I2C_INT_2_BAT_TEMP_LOW                     0x10
#define TS2_I2C_INT_2_BAT_VOLT_LOW                     0x08
#define TS2_I2C_INT_2_BAT_RARC_CRIT                    0x04
#define TS2_I2C_INT_2_BAT_RARC_LOW2                    0x02
#define TS2_I2C_INT_2_BAT_RARC_LOW1                    0x01
#define TS2_I2C_INT_3_A2A_CONNECT_CHANGE               0x08
#define TS2_I2C_INT_3_FLAGS_CHANGE                     0x04
#define TS2_I2C_INT_3_LOG                              0x02
#define TS2_I2C_INT_3_RESET                            0x01


/* page 0x01 */
/* battery (airboard only); interface defined by phone teams */
#define TS2_I2C_BAT_STATUS                             0x0100
#define TS2_I2C_BAT_RARC                               0x0101
#define TS2_I2C_BAT_RSRC                               0x0102
#define TS2_I2C_BAT_AVG_CUR_MSB                        0x0103
#define TS2_I2C_BAT_AVG_CUR_LSB                        0x0104
#define TS2_I2C_BAT_TEMP_MSB                           0x0105
#define TS2_I2C_BAT_TEMP_LSB                           0x0106
#define TS2_I2C_BAT_VOLT_MSB                           0x0107
#define TS2_I2C_BAT_VOLT_LSB                           0x0108
#define TS2_I2C_BAT_CUR_MSB                            0x0109
#define TS2_I2C_BAT_CUR_LSB                            0x010a
#define TS2_I2C_BAT_COULOMB_MSB                        0x010b
#define TS2_I2C_BAT_COULOMB_LSB                        0x010c
#define TS2_I2C_BAT_AS                                 0x010d
#define TS2_I2C_BAT_FULL_MSB                           0x010e
#define TS2_I2C_BAT_FULL_LSB                           0x010f
#define TS2_I2C_BAT_FULL40_MSB                         0x0110
#define TS2_I2C_BAT_FULL40_LSB                         0x0111
#define TS2_I2C_BAT_RSNSP                              0x0112
#define TS2_I2C_BAT_RAAC_MSB                           0x0113
#define TS2_I2C_BAT_RAAC_LSB                           0x0114
#define TS2_I2C_BAT_SACR_MSB                           0x0115
#define TS2_I2C_BAT_SACR_LSB                           0x0116
#define TS2_I2C_BAT_ASL                                0x0117
#define TS2_I2C_BAT_FAC_MSB                            0x0118
#define TS2_I2C_BAT_FAC_LSB                            0x0119

#define TS2_I2C_BAT_ROMID_0                            0x0120
#define TS2_I2C_BAT_ROMID(x) \
	(TS2_I2C_BAT_ROMID_0 + (x))

#define TS2_I2C_BAT_COMMAND_STATUS                     0x0140
#define TS2_I2C_BAT_COMMAND_AUTH			0x81
#define TS2_I2C_BAT_COMMAND_REFRESH		0x82
#define TS2_I2C_BAT_COMMAND_WAKE			0x83
#define TS2_I2C_BAT_COMMAND_OFF			0xe9
#define TS2_I2C_BAT_STATUS_AUTH_FAIL		0x08
#define TS2_I2C_BAT_STATUS_AUTH_PASS		0x04
#define TS2_I2C_BAT_STATUS_REGS_VALID		0x02
#define TS2_I2C_BAT_STATUS_BUSY			0x01


/* battery configuration (airboard only) */
#define TS2_I2C_BAT_TEMP_LOW_MSB                       0x0180
#define TS2_I2C_BAT_TEMP_LOW_LSB                       0x0181
#define TS2_I2C_BAT_TEMP_HIGH_MSB                      0x0182
#define TS2_I2C_BAT_TEMP_HIGH_LSB                      0x0183
#define TS2_I2C_BAT_VOLT_LOW_MSB                       0x0184
#define TS2_I2C_BAT_VOLT_LOW_LSB                       0x0185
#define TS2_I2C_BAT_RARC_CRIT                          0x0186
#define TS2_I2C_BAT_RARC_LOW_2                         0x0187
#define TS2_I2C_BAT_RARC_LOW_1                         0x0188

#define TS2_I2C_BAT_CHALLENGE_0                        0x01e0
#define TS2_I2C_BAT_CHALLENGE(x) \
	(TS2_I2C_BAT_CHALLENGE_0 + (x))
#define TS2_I2C_BAT_RESPONSE_0 \
	(TS2_I2C_BAT_CHALLENGE_0 + 8)
#define TS2_I2C_BAT_RESPONSE(x) \
	(TS2_I2C_BAT_RESPONSE_0 + (x))


/* page 0x02 */
/* comms */
#define TS2_I2C_COMM_STATUS                            0x0200
#define TS2_I2C_COMM_STATUS_RX_FULL			0x02
#define TS2_I2C_COMM_STATUS_TX_EMPTY			0x01
#define TS2_I2C_COMM_TXDATA_RXDATA_RESERVED_1          0x0201 /* reserved 1 */
#define TS2_I2C_COMM_TXDATA_RXDATA_RESERVED_2          0x0202 /* reserved 2 */
#define TS2_I2C_COMM_TXDATA_RXDATA                     0x0203 /* side-effect */

/* page 0x03 */
/* log */
#define TS2_I2C_LOG_LEVEL                              0x0300
#define TS2_I2C_LOG_INT_THRESHOLD                      0x0301
#define TS2_I2C_LOG_LOST_MSB_RESERVED_1                0x0302 /* reserved 1 */
#define TS2_I2C_LOG_LOST_MSB_RESERVED_2                0x0303 /* reserved 2 */
#define TS2_I2C_LOG_LOST_MSB                           0x0304 /* side-effect */
#define TS2_I2C_LOG_LOST_LSB                           0x0305
#define TS2_I2C_LOG_COUNT_MSB                          0x0306
#define TS2_I2C_LOG_COUNT_LSB                          0x0307
#define TS2_I2C_LOG_ENTRY_MSB_RESERVED_1               0x0308 /* reserved 1 */
#define TS2_I2C_LOG_ENTRY_MSB_RESERVED_2               0x0309 /* reserved 2 */
#define TS2_I2C_LOG_ENTRY_MSB                          0x030a /* side-effect */
#define TS2_I2C_LOG_ENTRY_LSB                          0x030b


/* page 0x04 */
/* local enumeration structure */
/* Enumeration registers (local) */
#define	TS2_I2C_ENUM_STRUCT_VER                        0x0400
#define	TS2_I2C_ENUM_STRUCT_LEN                        0x0401
#define	TS2_I2C_ENUM_PROTOCOL_MAP_VER                  0x0402
#define	TS2_I2C_ENUM_MFGR_ID_HI                        0x0403
#define	TS2_I2C_ENUM_MFGR_ID_LO                        0x0404
#define	TS2_I2C_ENUM_PRODUCT_TYPE_HI                   0x0405
#define	TS2_I2C_ENUM_PRODUCT_TYPE_LO                   0x0406
#define	TS2_I2C_ENUM_SERNO_7                           0x0407
#define	TS2_I2C_ENUM_SERNO_6                           0x0408
#define	TS2_I2C_ENUM_SERNO_5                           0x0409
#define	TS2_I2C_ENUM_SERNO_4                           0x040A
#define	TS2_I2C_ENUM_SERNO_3                           0x040B
#define	TS2_I2C_ENUM_SERNO_2                           0x040C
#define	TS2_I2C_ENUM_SERNO_1                           0x040D
#define	TS2_I2C_ENUM_SERNO_0                           0x040E
#define	TS2_I2C_ENUM_ASSY_REV                          0x040F
#define	TS2_I2C_ENUM_FW_VER_2                          0x0410
#define	TS2_I2C_ENUM_FW_VER_1                          0x0411
#define	TS2_I2C_ENUM_FW_VER_0                          0x0412
#define	TS2_I2C_ENUM_VNODE_MIN_HI                      0x0413
#define	TS2_I2C_ENUM_VNODE_MIN_LO                      0x0414
#define	TS2_I2C_ENUM_VNODE_MAX_HI                      0x0415
#define	TS2_I2C_ENUM_VNODE_MAX_LO                      0x0416
#define	TS2_I2C_ENUM_INODE_MIN_HI                      0x0417
#define	TS2_I2C_ENUM_INODE_MIN_LO                      0x0418
#define	TS2_I2C_ENUM_INODE_MAX_HI                      0x0419
#define	TS2_I2C_ENUM_INODE_MAX_LO                      0x041A
#define	TS2_I2C_ENUM_TNODE_MIN                         0x041B
#define	TS2_I2C_ENUM_TNODE_MAX                         0x041C
#define	TS2_I2C_ENUM_POWER_MAX                         0x041D

#define	TS2_I2C_ENUM_ACCE_0                            0x0428
#define	TS2_I2C_ENUM_ACCE_1                            0x0429
#define	TS2_I2C_ENUM_ACCE_2                            0x042A
#define	TS2_I2C_ENUM_ACCE_3                            0x042B
#define	TS2_I2C_ENUM_ACCE_4                            0x042C
#define	TS2_I2C_ENUM_ACCE_5                            0x042D
#define	TS2_I2C_ENUM_ACCE_6                            0x042E
#define	TS2_I2C_ENUM_ACCE_7                            0x042F
#define	TS2_I2C_ENUM_ACCE_8                            0x0430
#define	TS2_I2C_ENUM_ACCE_9                            0x0431
#define	TS2_I2C_ENUM_ACCE_10                           0x0432
#define	TS2_I2C_ENUM_ACCE_11                           0x0433
#define	TS2_I2C_ENUM_ACCE_12                           0x0434
#define	TS2_I2C_ENUM_ACCE_13                           0x0435
#define	TS2_I2C_ENUM_ACCE_14                           0x0436
#define	TS2_I2C_ENUM_ACCE_15                           0x0437
#define	TS2_I2C_ENUM_MIN_PWM                           0x0438


/* page 0x05 */
/* remote enumeration structure */
#define	TS2_I2C_ENUM_REMOTE_STRUCT_VER                 0x0500
#define	TS2_I2C_ENUM_REMOTE_STRUCT_LEN                 0x0501
#define	TS2_I2C_ENUM_REMOTE_PROTOCOL_MAP_VER           0x0502
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_HI                 0x0503
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_LO                 0x0504
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_HI_V1              0x0505
#define	TS2_I2C_ENUM_REMOTE_MFGR_ID_LO_V1              0x0506
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI            0x0505
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO            0x0506
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_HI_V1         0x0507
#define	TS2_I2C_ENUM_REMOTE_PRODUCT_TYPE_LO_V1         0x0508
#define	TS2_I2C_ENUM_REMOTE_SERNO_7                    0x0507
#define	TS2_I2C_ENUM_REMOTE_SERNO_6                    0x0508
#define	TS2_I2C_ENUM_REMOTE_SERNO_5                    0x0509
#define	TS2_I2C_ENUM_REMOTE_SERNO_4                    0x050A
#define	TS2_I2C_ENUM_REMOTE_SERNO_3                    0x050B
#define	TS2_I2C_ENUM_REMOTE_SERNO_2                    0x050C
#define	TS2_I2C_ENUM_REMOTE_SERNO_1                    0x050D
#define	TS2_I2C_ENUM_REMOTE_SERNO_0                    0x050E
#define	TS2_I2C_ENUM_REMOTE_ASSY_REV                   0x050F
#define	TS2_I2C_ENUM_REMOTE_FW_VER_2                   0x0510
#define	TS2_I2C_ENUM_REMOTE_FW_VER_1                   0x0511
#define	TS2_I2C_ENUM_REMOTE_FW_VER_0                   0x0512
#define	TS2_I2C_ENUM_REMOTE_VNODE_MIN_HI               0x0513
#define	TS2_I2C_ENUM_REMOTE_VNODE_MIN_LO               0x0514
#define	TS2_I2C_ENUM_REMOTE_VNODE_MAX_HI               0x0515
#define	TS2_I2C_ENUM_REMOTE_VNODE_MAX_LO               0x0516
#define	TS2_I2C_ENUM_REMOTE_INODE_MIN_HI               0x0517
#define	TS2_I2C_ENUM_REMOTE_INODE_MIN_LO               0x0518
#define	TS2_I2C_ENUM_REMOTE_INODE_MAX_HI               0x0519
#define	TS2_I2C_ENUM_REMOTE_INODE_MAX_LO               0x051A
#define	TS2_I2C_ENUM_REMOTE_TNODE_MIN                  0x051B
#define	TS2_I2C_ENUM_REMOTE_TNODE_MAX                  0x051C
#define	TS2_I2C_ENUM_REMOTE_POWER_MAX                  0x051D



#define	TS2_I2C_ENUM_REMOTE_ACCE_0                     0x0528
#define	TS2_I2C_ENUM_REMOTE_ACCE_1                     0x0529
#define	TS2_I2C_ENUM_REMOTE_ACCE_2                     0x052A
#define	TS2_I2C_ENUM_REMOTE_ACCE_3                     0x052B
#define	TS2_I2C_ENUM_REMOTE_ACCE_4                     0x052C
#define	TS2_I2C_ENUM_REMOTE_ACCE_5                     0x052D
#define	TS2_I2C_ENUM_REMOTE_ACCE_6                     0x052E
#define	TS2_I2C_ENUM_REMOTE_ACCE_7                     0x052F
#define	TS2_I2C_ENUM_REMOTE_ACCE_8                     0x0530
#define	TS2_I2C_ENUM_REMOTE_ACCE_9                     0x0531
#define	TS2_I2C_ENUM_REMOTE_ACCE_10                    0x0532
#define	TS2_I2C_ENUM_REMOTE_ACCE_11                    0x0533
#define	TS2_I2C_ENUM_REMOTE_ACCE_12                    0x0534
#define	TS2_I2C_ENUM_REMOTE_ACCE_13                    0x0535
#define	TS2_I2C_ENUM_REMOTE_ACCE_14                    0x0536
#define	TS2_I2C_ENUM_REMOTE_ACCE_15                    0x0537


/* page 0x06 */
/* reserved */


/* page 0x07 */
/* voltage, current, temperature, power (puck only), pwm (puck only) */
#define TS2_I2C_ID                                     0x0700
#define TS2_I2C_ID_AIRBOARD_0				0x2c
#define TS2_I2C_FLAGS_0                                0x0701
#define	TS2_I2C_FLAGS_0_PUCK_PRIORITY			(1<<0)
#define TS2_I2C_FLAGS_1                                0x0702
#define TS2_I2C_FLAGS_2                                0x0703
#define	TS2_I2C_FLAGS_2_A2A_CONNECT			0x80
#define	TS2_I2C_FLAGS_2_A2A_ATTEMPT			0x40
#define	TS2_I2C_FLAGS_2_BATTERY_DETECT			0x20
#define	TS2_I2C_FLAGS_2_WIRED_CHARGE			0x10
#define	TS2_I2C_FLAGS_2_PUCK_CHARGE			0x08
#define TS2_I2C_FLAGS_2_WIRED				0x04
#define TS2_I2C_FLAGS_2_PUCK				0x02
#define TS2_I2C_FLAGS_2_PUCK_DETECT			0x01
#define TS2_I2C_V_LOCAL                                0x0704
#define TS2_I2C_I_LOCAL                                0x0705
#define TS2_I2C_T_LOCAL                                0x0706
#define TS2_I2C_P_LOCAL                                0x0707 /* puck only */
#define TS2_I2C_V_REMOTE                               0x0708
#define TS2_I2C_I_REMOTE                               0x0709
#define TS2_I2C_T_REMOTE                               0x070a
#define TS2_I2C_P_REMOTE                               0x070b /* puck only */
#define TS2_I2C_PWM                                    0x070c /* puck only */
#define TS2_I2C_PERIOD                                 0x070d /* puck only */

/* config */
#define TS2_I2C_TX_POWER_V1                            0x0740
#define TS2_I2C_TX_POWER_V2                            0x0741

/* puck only config */
#define TS2_I2C_PWM_PERIOD                             0x0780
#define TS2_I2C_PWM_DUTY_ENUM                          0x0781
#define TS2_I2C_PWM_DUTY_MIN_RUN                       0x0782
#define TS2_I2C_PWM_DUTY_MIN_BOOST                     0x0783
#define TS2_I2C_PWM_DEADTIME                           0x0784
#define TS2_I2C_PWM_EFFICIENCY_OFFSET                  0x0785
#define TS2_I2C_PWM_EFFICIENCY_SCALE                   0x0786

/* airboard only */
#define TS2_I2C_V_OFFSET                               0x07c0
#define TS2_I2C_WAKEUP_PERIOD                          0x07c1
#define TS2_I2C_CURRENT_ADJ_FOR_V1_SLOPE               0x07c2
#define TS2_I2C_CURRENT_ADJ_FOR_V1_OFFSET              0x07c3
#define TS2_I2C_VOLTAGE_ADJ_FOR_V1_SLOPE               0x07c4
#define TS2_I2C_VOLTAGE_ADJ_FOR_V1_OFFSET              0x07c5

/* misc registers */
#define TS2_I2C_COMMAND                                0x1000
#define TS2_I2C_COMMAND_RESET_HOST		 0x01
#define TS2_I2C_COMMAND_CLEAR_BTN_SEQ_RESET_FLAG	 0x02

#define TS2_I2C_COMMAND_COMM_CONNECT		 0x10
#define TS2_I2C_COMMAND_COMM_DISCONNECT		 0x11
#define TS2_I2C_COMMAND_REENUMERATE		 0x12

#define TS2_I2C_COMMAND_FRAM_CHECKSUM_1400_READ	 0x20
#define TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_1	 0x21
#define TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_2	 0x22

#define TS2_I2C_COMMAND_MEM_READ_BYTE		 0x80
#define TS2_I2C_COMMAND_MEM_READ_CHAWMP		 0x81
#define TS2_I2C_COMMAND_MEM_WRITE_BYTE		 0x82
#define TS2_I2C_COMMAND_MEM_WRITE_CHAWMP		 0x83
#define TS2_I2C_COMMAND_PMIC_READ		 0x84
#define TS2_I2C_COMMAND_PMIC_WRITE		 0x85
#define TS2_I2C_COMMAND_SP_READ			 0x86
#define TS2_I2C_COMMAND_SP_INDIRECT_READ		 0x87
#define TS2_I2C_COMMAND_ADDR_MSB                       0x1001
#define TS2_I2C_COMMAND_ADDR_LSB                       0x1002
#define TS2_I2C_COMMAND_DATA_MSB                       0x1003
#define TS2_I2C_COMMAND_DATA_LSB                       0x1004

#define TS2_I2C_DEBUG_TRACE_DATA_RESERVED_1            0x101e /* reserved 1 */
#define TS2_I2C_DEBUG_TRACE_DATA_RESERVED_2            0x101f /* reserved 2 */
#define TS2_I2C_DEBUG_TRACE_DATA                       0x1020 /* side-effect */


#define RSENSE_DEFAULT	20
#define FORCE_WAKE_TIMER_EXPIRY (HZ/20)

#define a6_wait_event_ex(wq, condition)					\
do {									\
	if (condition)							\
		break;							\
	do {								\
		DEFINE_WAIT(__wait);					\
									\
		for (;;) {						\
			prepare_to_wait_exclusive(&wq, &__wait,		\
				TASK_UNINTERRUPTIBLE);			\
			if (condition)					\
				break;					\
			schedule();					\
		}							\
		finish_wait(&wq, &__wait);				\
	} while (0);							\
} while (0)


enum {
	DEVICE_BUSY_BIT = 0,
	IS_OPENED,
	IS_INITIALIZED_BIT,
	BOOTLOAD_ACTIVE_BIT,
	FORCE_WAKE_ACTIVE_BIT,
	READ_ACTIVE_BIT,
	WRITE_ACTIVE_BIT,
	A2A_CONNECTED,
	EXTRACT_INITIATED,
	/* capabilities */
	CAP_PERIODIC_WAKE,
#ifdef A6_PQ
	STARTING_AID_TASK,
	KILLING_AID_TASK,
	IS_QUIESCED,
#endif
	IS_SUSPENDED,
	INT_PENDING,
	SIZE_FLAGS
};


struct a6_device_state {
	struct i2c_client *i2c_dev;
	struct a6_platform_data *plat_data;
	struct miscdevice mdev;

	struct mutex dev_mutex;
	unsigned int timestamping;

	struct timer_list	a6_force_wake_timer;
	struct work_struct	a6_force_wake_work;
	struct mutex		a6_force_wake_mutex;

	wait_queue_head_t	dev_busyq;
	struct work_struct	a6_irq_work;

	int32_t cpufreq_hold_flag;
	struct workqueue_struct *ka6d_workqueue;
	struct workqueue_struct *ka6d_fw_workqueue;

	uint32_t cached_rsense_val:	16;
	uint32_t busy_count:		 8;
	DECLARE_BITMAP(flags, SIZE_FLAGS);

#ifdef A6_PQ
	struct completion aq_enq_complete;
	struct completion aid_exit_complete;
	struct mutex aq_mutex;
	struct list_head aq_head;
	struct task_struct *ai_dispatch_task;
#ifdef A6_DEBUG
	uint32_t dbgflg_kill_raid:	1;

	uint8_t debug_restart_aid;
	uint8_t debug_flush_aiq;
	uint8_t debug_unused_01;
	uint8_t debug_unused_02;
	struct task_struct *restart_aid_task;
#endif
#endif

	int cpufreq_hold;

	/* pmem extract */
	struct miscdevice pmem_mdev;
	char pmem_dev_name[16];

	char *a2a_rd_buf, *a2a_wr_buf;
	char *a2a_rp, *a2a_wp;

	/* Modern kernel API support */
	struct gpio_desc *tck_gpio;	/* SBW test clock */
	struct gpio_desc *tdio_gpio;	/* SBW test data I/O */
	struct gpio_desc *wakeup_gpio;	/* Wakeup signal to A6 */
	struct gpio_desc *irq_gpio;	/* Interrupt from A6 (optional) */
	struct power_supply *battery;	/* Power supply device */
	struct power_supply_desc battery_desc; /* Power supply descriptor */
	int device_index;		/* Device index (0 or 1) */
};

/*
 * Note: 16-bit accesses comprising an LSB, MSB register pair must be specified
 * in LSB:MSB order in the corresponding a6_register_desc struct. This is
 * because an a6-fw constraint restricts 16-bit value accesses to MSB-leading
 * only. The a6_i2c_read_reg and the a6_i2c_write_reg functions take the
 * LSB:MSB definition in the a6_register_desc struct and re-orders the i2c txn
 * to read in MSB:LSB order.
 */
#define id_size 20

typedef int32_t (*format_show_fn)(const struct a6_device_state *state, const uint8_t *val,
		  uint8_t *fmt_buffer, uint32_t size_buffer);
typedef int32_t (*format_store_fn)(const struct a6_device_state *state, const uint8_t *fmt_buffer,
		  uint8_t *val, uint32_t size_buffer);

struct a6_register_desc {
	const char *debug_name;
	struct device_attribute dev_attr;
	format_show_fn format;
	format_store_fn r_format;
	uint16_t id[id_size];
	uint32_t num_ids: 5;
	uint32_t ro: 1;
};

/* Shared register descriptor tables (defined in a6_regs.c) */
extern struct a6_register_desc a6_register_desc_arr[];
extern const size_t a6_register_desc_arr_size;
extern struct device_attribute custom_devattr[];
extern const size_t custom_devattr_size;

/* Shared SBW state (defined in a6_sbw_gpio.c) */
extern struct a6_device_state *current_sbw_state;
extern struct a6_sbw_interface a6_gpio_sbw_ops;

/* Misc helpers (defined in a6_sbw_gpio.c) */
uint8_t _convert_hex_char_to_decimal(uint8_t x);

/* i2c register access (defined in a6_dispatch.c) */
int32_t a6_i2c_read_reg(struct i2c_client *client, const uint16_t *ids,
			uint32_t num_ids, uint8_t *out);
int32_t a6_i2c_write_reg(struct i2c_client *client, const uint16_t *ids,
			 uint32_t num_ids, const uint8_t *in);

#ifdef A6_PQ
/* action item dispatcher (defined in a6_dispatch.c) */
int32_t a6_start_ai_dispatch_task(struct a6_device_state *state);
int32_t a6_stop_ai_dispatch_task(struct a6_device_state *state);
int32_t flush_a6_action_items(struct a6_device_state *state);
#ifdef A6_DEBUG
int32_t a6_create_debug_interface(struct a6_device_state *state);
#endif
#endif

/* device state init / power supply (defined in a6.c, a6_power_supply.c) */
int32_t a6_init_state(struct i2c_client *client);
int a6_register_power_supply(struct a6_device_state *state);

/* sysfs / dev attribute handlers (defined in a6_sysfs.c) */
ssize_t a6_generic_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t a6_generic_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);
ssize_t a6_val_cksum_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t a6_diag_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t a6_diag_store(struct device *dev, struct device_attribute *attr,
		      const char *buf, size_t count);
int32_t a6_create_dev_files(struct a6_device_state *state, struct device *dev);
void a6_remove_dev_files(struct a6_device_state *state, struct device *dev);

/* format helpers (defined in a6_sysfs.c) */
int32_t format_current(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_voltage(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t r_format_voltage(const struct a6_device_state *state, const uint8_t *fmt_buffer,
			 uint8_t *val, uint32_t size_buffer);
int32_t format_rawcoulomb(const struct a6_device_state *state, const uint8_t *val,
			  uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_coulomb(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_age(const struct a6_device_state *state, const uint8_t *val,
		   uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_fullx(const struct a6_device_state *state, const uint8_t *val,
		     uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_temp(const struct a6_device_state *state, const uint8_t *val,
		    uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t r_format_temp(const struct a6_device_state *state, const uint8_t *fmt_buffer,
		      uint8_t *val, uint32_t size_buffer);
int32_t format_status(const struct a6_device_state *state, const uint8_t *val,
		      uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_rsense(const struct a6_device_state *state, const uint8_t *val,
		      uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_charge_source_status(const struct a6_device_state *state, const uint8_t *val,
				    uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_version(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_v_offset(const struct a6_device_state *state, const uint8_t *val,
			uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_accessory(const struct a6_device_state *state, const uint8_t *val,
			 uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_comm_status(const struct a6_device_state *state, const uint8_t *val,
			   uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_raw_unsigned(const struct a6_device_state *state, const uint8_t *val,
			    uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_max_power_available(const struct a6_device_state *state, const uint8_t *val,
				   uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_accessory_combo(const struct a6_device_state *state, const uint8_t *val,
			       uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_u16_hex(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_serno_v1(const struct a6_device_state *state, const uint8_t *val,
			uint8_t *fmt_buffer, uint32_t size_buffer);
int32_t format_serno_v2(const struct a6_device_state *state, const uint8_t *val,
			uint8_t *fmt_buffer, uint32_t size_buffer);

/* character device & firmware update (defined in a6_cdev.c) */
extern const struct file_operations a6_fops;
extern const struct file_operations a6_pmem_fops;

/* IRQ + force wake handlers (defined in a6.c) */
irqreturn_t a6_irq(int irq, void *dev_id);
void a6_irq_work_handler(struct work_struct *work);
void a6_force_wake_work_handler(struct work_struct *work);
void a6_force_wake_timer_callback(struct timer_list *t);

#endif /* _A6_INTERNAL_H_ */
