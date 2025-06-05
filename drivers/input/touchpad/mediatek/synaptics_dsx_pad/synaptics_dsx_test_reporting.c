/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012-2016 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"
#include <asm/uaccess.h>

#include "cei_touch_log.h"

#define WATCHDOG_HRTIMER
#define WATCHDOG_TIMEOUT_S 3
#define CALIBRATION_TIMEOUT_S 10
#define FORCE_TIMEOUT_100MS 10
#define STATUS_WORK_INTERVAL 20 /* ms */

#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)

#define GET_REPORT_TIMEOUT_S 3

/*
#define RAW_HEX
#define HUMAN_READABLE
*/

#define STATUS_IDLE 0
#define STATUS_BUSY 1
#define STATUS_ERROR 2

#define DATA_REPORT_INDEX_OFFSET 1
#define DATA_REPORT_DATA_OFFSET 3

#define SENSOR_RX_MAPPING_OFFSET 1
#define SENSOR_TX_MAPPING_OFFSET 2

#define COMMAND_GET_REPORT 1
#define COMMAND_FORCE_CAL 2
#define COMMAND_FORCE_UPDATE 4

#define CONTROL_NO_AUTO_CAL 1

#define CONTROL_0_SIZE 1
#define CONTROL_1_SIZE 1
#define CONTROL_2_SIZE 2
#define CONTROL_3_SIZE 1
#define CONTROL_4_6_SIZE 3
#define CONTROL_7_SIZE 1
#define CONTROL_8_9_SIZE 3
#define CONTROL_10_SIZE 1
#define CONTROL_11_SIZE 2
#define CONTROL_12_13_SIZE 2
#define CONTROL_14_SIZE 1
#define CONTROL_15_SIZE 1
#define CONTROL_16_SIZE 1
#define CONTROL_17_SIZE 1
#define CONTROL_18_SIZE 1
#define CONTROL_19_SIZE 1
#define CONTROL_20_SIZE 1
#define CONTROL_21_SIZE 2
#define CONTROL_22_26_SIZE 7
#define CONTROL_27_SIZE 1
#define CONTROL_28_SIZE 2
#define CONTROL_29_SIZE 1
#define CONTROL_30_SIZE 1
#define CONTROL_31_SIZE 1
#define CONTROL_32_35_SIZE 8
#define CONTROL_36_SIZE 1
#define CONTROL_37_SIZE 1
#define CONTROL_38_SIZE 1
#define CONTROL_39_SIZE 1
#define CONTROL_40_SIZE 1
#define CONTROL_41_SIZE 1
#define CONTROL_42_SIZE 2
#define CONTROL_43_54_SIZE 13
#define CONTROL_55_56_SIZE 2
#define CONTROL_57_SIZE 1
#define CONTROL_58_SIZE 1
#define CONTROL_59_SIZE 2
#define CONTROL_60_62_SIZE 3
#define CONTROL_63_SIZE 1
#define CONTROL_64_67_SIZE 4
#define CONTROL_68_73_SIZE 8
#define CONTROL_74_SIZE 2
#define CONTROL_75_SIZE 1
#define CONTROL_76_SIZE 1
#define CONTROL_77_78_SIZE 2
#define CONTROL_79_83_SIZE 5
#define CONTROL_84_85_SIZE 2
#define CONTROL_86_SIZE 1
#define CONTROL_87_SIZE 1
#define CONTROL_88_SIZE 1
#define CONTROL_89_SIZE 1
#define CONTROL_90_SIZE 1
#define CONTROL_91_SIZE 1
#define CONTROL_92_SIZE 1
#define CONTROL_93_SIZE 1
#define CONTROL_94_SIZE 1
#define CONTROL_95_SIZE 1
#define CONTROL_96_SIZE 1
#define CONTROL_97_SIZE 1
#define CONTROL_98_SIZE 1
#define CONTROL_99_SIZE 1
#define CONTROL_100_SIZE 1
#define CONTROL_101_SIZE 1
#define CONTROL_102_SIZE 1
#define CONTROL_103_SIZE 1
#define CONTROL_104_SIZE 1
#define CONTROL_105_SIZE 1
#define CONTROL_106_SIZE 1
#define CONTROL_107_SIZE 1
#define CONTROL_108_SIZE 1
#define CONTROL_109_SIZE 1
#define CONTROL_110_SIZE 1
#define CONTROL_111_SIZE 1
#define CONTROL_112_SIZE 1
#define CONTROL_113_SIZE 1
#define CONTROL_114_SIZE 1
#define CONTROL_115_SIZE 1
#define CONTROL_116_SIZE 1
#define CONTROL_117_SIZE 1
#define CONTROL_118_SIZE 1
#define CONTROL_119_SIZE 1
#define CONTROL_120_SIZE 1
#define CONTROL_121_SIZE 1
#define CONTROL_122_SIZE 1
#define CONTROL_123_SIZE 1
#define CONTROL_124_SIZE 1
#define CONTROL_125_SIZE 1
#define CONTROL_126_SIZE 1
#define CONTROL_127_SIZE 1
#define CONTROL_128_SIZE 1
#define CONTROL_129_SIZE 1
#define CONTROL_130_SIZE 1
#define CONTROL_131_SIZE 1
#define CONTROL_132_SIZE 1
#define CONTROL_133_SIZE 1
#define CONTROL_134_SIZE 1
#define CONTROL_135_SIZE 1
#define CONTROL_136_SIZE 1
#define CONTROL_137_SIZE 1
#define CONTROL_138_SIZE 1
#define CONTROL_139_SIZE 1
#define CONTROL_140_SIZE 1
#define CONTROL_141_SIZE 1
#define CONTROL_142_SIZE 1
#define CONTROL_143_SIZE 1
#define CONTROL_144_SIZE 1
#define CONTROL_145_SIZE 1
#define CONTROL_146_SIZE 1
#define CONTROL_147_SIZE 1
#define CONTROL_148_SIZE 1
#define CONTROL_149_SIZE 1
#define CONTROL_150_SIZE 1
#define CONTROL_151_SIZE 1
#define CONTROL_152_SIZE 1
#define CONTROL_153_SIZE 1
#define CONTROL_154_SIZE 1
#define CONTROL_155_SIZE 1
#define CONTROL_156_SIZE 1
#define CONTROL_157_158_SIZE 2
#define CONTROL_163_SIZE 1
#define CONTROL_165_SIZE 1
#define CONTROL_166_SIZE 1
#define CONTROL_167_SIZE 1
#define CONTROL_168_SIZE 1
#define CONTROL_169_SIZE 1
#define CONTROL_171_SIZE 1
#define CONTROL_172_SIZE 1
#define CONTROL_173_SIZE 1
#define CONTROL_174_SIZE 1
#define CONTROL_175_SIZE 1
#define CONTROL_176_SIZE 1
#define CONTROL_177_178_SIZE 2
#define CONTROL_179_SIZE 1
#define CONTROL_182_SIZE 1
#define CONTROL_183_SIZE 1
#define CONTROL_185_SIZE 1
#define CONTROL_186_SIZE 1
#define CONTROL_187_SIZE 1
#define CONTROL_188_SIZE 1

#define HIGH_RESISTANCE_DATA_SIZE 6
#define FULL_RAW_CAP_MIN_MAX_DATA_SIZE 4
#define TRX_OPEN_SHORT_DATA_SIZE 7

#define NO_AUTO_CAL_MASK 0x01

/* tddi f54 test reporting + */

#define F54_POLLING_GET_REPORT

/* test limit config */
#define FULL_RAW_CAP_TEST_LIMIT_LOWER 300
#define FULL_RAW_CAP_TEST_LIMIT_UPPER 70000
#define EE_SHORT_TEST_LIMIT_PART1  100
#define EE_SHORT_TEST_LIMIT_PART2  90  // ( unit = ratio )
#define AMP_OPEN_INT_DUR_ONE 90
#define AMP_OPEN_INT_DUR_TWO 20
#define AMP_OPEN_TEST_LIMIT_PHASE1_LOWER 500  // ( unit = femtofarad (fF) )
#define AMP_OPEN_TEST_LIMIT_PHASE1_UPPER 8000 // ( unit = femtofarad (fF) )
#define AMP_OPEN_TEST_LIMIT_PHASE2_LOWER 70   // ( unit = ratio )
#define AMP_OPEN_TEST_LIMIT_PHASE2_UPPER 130  // ( unit = ratio )
/* tddi f54 test reporting - */

#define TEST_DEFAULT_SAVE_PATH "/data/"
#define TEST_DEFAULT_RESULT_FILE_NAME "td4322_self_test_data.txt"
#define TEST_DATA_SIZE 40 * 1024
#define TEST_TEMP_DATA_SIZE 20 * 1024

#define concat(a, b) a##b

#define GROUP(_attrs) {\
	.attrs = _attrs,\
}

#define attrify(propname) (&dev_attr_##propname.attr)

#define show_prototype(propname)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf);\
\
static struct device_attribute dev_attr_##propname =\
		__ATTR(propname, S_IRUGO,\
		concat(synaptics_rmi4_f54, _##propname##_show),\
		synaptics_rmi4_store_error);

#define store_prototype(propname)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t count);\
\
static struct device_attribute dev_attr_##propname =\
		__ATTR(propname, S_IWUSR | S_IWGRP,\
		synaptics_rmi4_show_error,\
		concat(synaptics_rmi4_f54, _##propname##_store));

#define show_store_prototype(propname)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf);\
\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t count);\
\
static struct device_attribute dev_attr_##propname =\
		__ATTR(propname, (S_IRUGO | S_IWUSR | S_IWGRP),\
		concat(synaptics_rmi4_f54, _##propname##_show),\
		concat(synaptics_rmi4_f54, _##propname##_store));

#define simple_show_func(rtype, propname, fmt)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf)\
{\
	return snprintf(buf, PAGE_SIZE, fmt, f54->rtype.propname);\
} \

#define simple_show_func_unsigned(rtype, propname)\
simple_show_func(rtype, propname, "%u\n")

#define show_func(rtype, rgrp, propname, fmt)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf)\
{\
	int retval;\
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;\
\
	mutex_lock(&f54->rtype##_mutex);\
\
	retval = f54->fn_ptr->read(rmi4_data,\
			f54->rtype.rgrp->address,\
			f54->rtype.rgrp->data,\
			sizeof(f54->rtype.rgrp->data));\
	mutex_unlock(&f54->rtype##_mutex);\
	if (retval < 0) {\
		TP_LOGE("Failed to read " #rtype\
				" " #rgrp "\n");\
		return retval;\
	} \
\
	return snprintf(buf, PAGE_SIZE, fmt,\
			f54->rtype.rgrp->propname);\
} \

#define show_store_func(rtype, rgrp, propname, fmt)\
show_func(rtype, rgrp, propname, fmt)\
\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t count)\
{\
	int retval;\
	unsigned long setting;\
	unsigned long o_setting;\
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;\
\
	retval = sstrtoul(buf, 10, &setting);\
	if (retval)\
		return retval;\
\
	mutex_lock(&f54->rtype##_mutex);\
	retval = f54->fn_ptr->read(rmi4_data,\
			f54->rtype.rgrp->address,\
			f54->rtype.rgrp->data,\
			sizeof(f54->rtype.rgrp->data));\
	if (retval < 0) {\
		mutex_unlock(&f54->rtype##_mutex);\
		TP_LOGE("Failed to read " #rtype\
				" " #rgrp "\n");\
		return retval;\
	} \
\
	if (f54->rtype.rgrp->propname == setting) {\
		mutex_unlock(&f54->rtype##_mutex);\
		return count;\
	} \
\
	o_setting = f54->rtype.rgrp->propname;\
	f54->rtype.rgrp->propname = setting;\
\
	retval = f54->fn_ptr->write(rmi4_data,\
			f54->rtype.rgrp->address,\
			f54->rtype.rgrp->data,\
			sizeof(f54->rtype.rgrp->data));\
	if (retval < 0) {\
		TP_LOGE("Failed to write " #rtype\
				" " #rgrp "\n");\
		f54->rtype.rgrp->propname = o_setting;\
		mutex_unlock(&f54->rtype##_mutex);\
		return retval;\
	} \
\
	mutex_unlock(&f54->rtype##_mutex);\
	return count;\
} \

#define show_store_func_unsigned(rtype, rgrp, propname)\
show_store_func(rtype, rgrp, propname, "%u\n")

#define show_replicated_func(rtype, rgrp, propname, fmt)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf)\
{\
	int retval;\
	int size = 0;\
	unsigned char ii;\
	unsigned char length;\
	unsigned char *temp;\
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;\
\
	mutex_lock(&f54->rtype##_mutex);\
\
	length = f54->rtype.rgrp->length;\
\
	retval = f54->fn_ptr->read(rmi4_data,\
			f54->rtype.rgrp->address,\
			(unsigned char *)f54->rtype.rgrp->data,\
			length);\
	mutex_unlock(&f54->rtype##_mutex);\
	if (retval < 0) {\
		TP_LOGI("Failed to read " #rtype\
				" " #rgrp "\n");\
	} \
\
	temp = buf;\
\
	for (ii = 0; ii < length; ii++) {\
		retval = snprintf(temp, PAGE_SIZE - size, fmt " ",\
				f54->rtype.rgrp->data[ii].propname);\
		if (retval < 0) {\
			TP_LOGE("Faild to write output\n");\
			return retval;\
		} \
		size += retval;\
		temp += retval;\
	} \
\
	retval = snprintf(temp, PAGE_SIZE - size, "\n");\
	if (retval < 0) {\
		TP_LOGE("Faild to write null terminator\n");\
		return retval;\
	} \
\
	return size + retval;\
} \

#define show_replicated_func_unsigned(rtype, rgrp, propname)\
show_replicated_func(rtype, rgrp, propname, "%u")

#define show_store_replicated_func(rtype, rgrp, propname, fmt)\
show_replicated_func(rtype, rgrp, propname, fmt)\
\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t count)\
{\
	int retval;\
	unsigned int setting;\
	unsigned char ii;\
	unsigned char length;\
	const unsigned char *temp;\
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;\
\
	mutex_lock(&f54->rtype##_mutex);\
\
	length = f54->rtype.rgrp->length;\
\
	retval = f54->fn_ptr->read(rmi4_data,\
			f54->rtype.rgrp->address,\
			(unsigned char *)f54->rtype.rgrp->data,\
			length);\
	if (retval < 0) {\
		TP_LOGI("Failed to read " #rtype\
				" " #rgrp "\n");\
	} \
\
	temp = buf;\
\
	for (ii = 0; ii < length; ii++) {\
		if (sscanf(temp, fmt, &setting) == 1) {\
			f54->rtype.rgrp->data[ii].propname = setting;\
		} else {\
			retval = f54->fn_ptr->read(rmi4_data,\
					f54->rtype.rgrp->address,\
					(unsigned char *)f54->rtype.rgrp->data,\
					length);\
			mutex_unlock(&f54->rtype##_mutex);\
			return -EINVAL;\
		} \
\
		while (*temp != 0) {\
			temp++;\
			if (isspace(*(temp - 1)) && !isspace(*temp))\
				break;\
		} \
	} \
\
	retval = f54->fn_ptr->write(rmi4_data,\
			f54->rtype.rgrp->address,\
			(unsigned char *)f54->rtype.rgrp->data,\
			length);\
	mutex_unlock(&f54->rtype##_mutex);\
	if (retval < 0) {\
		TP_LOGE("Failed to write " #rtype\
				" " #rgrp "\n");\
		return retval;\
	} \
\
	return count;\
} \

#define show_store_replicated_func_unsigned(rtype, rgrp, propname)\
show_store_replicated_func(rtype, rgrp, propname, "%u")

enum f54_report_types {
	F54_8BIT_IMAGE = 1,
	F54_16BIT_IMAGE = 2,
	F54_RAW_16BIT_IMAGE = 3,
	F54_HIGH_RESISTANCE = 4,
	F54_TX_TO_TX_SHORTS = 5,
	F54_RX_TO_RX_SHORTS_1 = 7,
	F54_TRUE_BASELINE = 9,
	F54_FULL_RAW_CAP_MIN_MAX = 13,
	F54_RX_OPENS_1 = 14,
	F54_TX_OPENS = 15,
	F54_TX_TO_GND_SHORTS = 16,
	F54_RX_TO_RX_SHORTS_2 = 17,
	F54_RX_OPENS_2 = 18,
	F54_FULL_RAW_CAP = 19,
	F54_FULL_RAW_CAP_NO_RX_COUPLING = 20,
	F54_SENSOR_SPEED = 22,
	F54_ADC_RANGE = 23,
	F54_TRX_OPENS = 24,
	F54_TRX_TO_GND_SHORTS = 25,
	F54_TRX_SHORTS = 26,
	F54_ABS_RAW_CAP = 38,
	F54_ABS_DELTA_CAP = 40,
	F54_ABS_HYBRID_DELTA_CAP = 59,
	F54_ABS_HYBRID_RAW_CAP = 63,
	F54_AMP_FULL_RAW_CAP = 78,
	F54_AMP_RAW_ADC = 83,
	/* tddi f54 test reporting + */
	F54_FULL_RAW_CAP_TDDI = 92,
	F54_NOISE_TDDI = 94,
	F54_EE_SHORT_TDDI = 95,
	/* tddi f54 test reporting - */

	INVALID_REPORT_TYPE = -1,
};

enum f54_afe_cal {
	F54_AFE_CAL,
	F54_AFE_IS_CAL,
};

struct f54_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;

			/* query 1 */
			unsigned char num_of_tx_electrodes;

			/* query 2 */
			unsigned char f54_query2_b0__1:2;
			unsigned char has_baseline:1;
			unsigned char has_image8:1;
			unsigned char f54_query2_b4__5:2;
			unsigned char has_image16:1;
			unsigned char f54_query2_b7:1;

			/* queries 3.0 and 3.1 */
			unsigned short clock_rate;

			/* query 4 */
			unsigned char touch_controller_family;

			/* query 5 */
			unsigned char has_pixel_touch_threshold_adjustment:1;
			unsigned char f54_query5_b1__7:7;

			/* query 6 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_interference_metric:1;
			unsigned char has_sense_frequency_control:1;
			unsigned char has_firmware_noise_mitigation:1;
			unsigned char has_ctrl11:1;
			unsigned char has_two_byte_report_rate:1;
			unsigned char has_one_byte_report_rate:1;
			unsigned char has_relaxation_control:1;

			/* query 7 */
			unsigned char curve_compensation_mode:2;
			unsigned char f54_query7_b2__7:6;

			/* query 8 */
			unsigned char f54_query8_b0:1;
			unsigned char has_iir_filter:1;
			unsigned char has_cmn_removal:1;
			unsigned char has_cmn_maximum:1;
			unsigned char has_touch_hysteresis:1;
			unsigned char has_edge_compensation:1;
			unsigned char has_per_frequency_noise_control:1;
			unsigned char has_enhanced_stretch:1;

			/* query 9 */
			unsigned char has_force_fast_relaxation:1;
			unsigned char has_multi_metric_state_machine:1;
			unsigned char has_signal_clarity:1;
			unsigned char has_variance_metric:1;
			unsigned char has_0d_relaxation_control:1;
			unsigned char has_0d_acquisition_control:1;
			unsigned char has_status:1;
			unsigned char has_slew_metric:1;

			/* query 10 */
			unsigned char has_h_blank:1;
			unsigned char has_v_blank:1;
			unsigned char has_long_h_blank:1;
			unsigned char has_startup_fast_relaxation:1;
			unsigned char has_esd_control:1;
			unsigned char has_noise_mitigation2:1;
			unsigned char has_noise_state:1;
			unsigned char has_energy_ratio_relaxation:1;

			/* query 11 */
			unsigned char has_excessive_noise_reporting:1;
			unsigned char has_slew_option:1;
			unsigned char has_two_overhead_bursts:1;
			unsigned char has_query13:1;
			unsigned char has_one_overhead_burst:1;
			unsigned char f54_query11_b5:1;
			unsigned char has_ctrl88:1;
			unsigned char has_query15:1;

			/* query 12 */
			unsigned char number_of_sensing_frequencies:4;
			unsigned char f54_query12_b4__7:4;
		} __packed;
		unsigned char data[14];
	};
};

struct f54_query_13 {
	union {
		struct {
			unsigned char has_ctrl86:1;
			unsigned char has_ctrl87:1;
			unsigned char has_ctrl87_sub0:1;
			unsigned char has_ctrl87_sub1:1;
			unsigned char has_ctrl87_sub2:1;
			unsigned char has_cidim:1;
			unsigned char has_noise_mitigation_enhancement:1;
			unsigned char has_rail_im:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_15 {
	union {
		struct {
			unsigned char has_ctrl90:1;
			unsigned char has_transmit_strength:1;
			unsigned char has_ctrl87_sub3:1;
			unsigned char has_query16:1;
			unsigned char has_query20:1;
			unsigned char has_query21:1;
			unsigned char has_query22:1;
			unsigned char has_query25:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_16 {
	union {
		struct {
			unsigned char has_query17:1;
			unsigned char has_data17:1;
			unsigned char has_ctrl92:1;
			unsigned char has_ctrl93:1;
			unsigned char has_ctrl94_query18:1;
			unsigned char has_ctrl95_query19:1;
			unsigned char has_ctrl99:1;
			unsigned char has_ctrl100:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_21 {
	union {
		struct {
			unsigned char has_abs_rx:1;
			unsigned char has_abs_tx:1;
			unsigned char has_ctrl91:1;
			unsigned char has_ctrl96:1;
			unsigned char has_ctrl97:1;
			unsigned char has_ctrl98:1;
			unsigned char has_data19:1;
			unsigned char has_query24_data18:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_22 {
	union {
		struct {
			unsigned char has_packed_image:1;
			unsigned char has_ctrl101:1;
			unsigned char has_dynamic_sense_display_ratio:1;
			unsigned char has_query23:1;
			unsigned char has_ctrl103_query26:1;
			unsigned char has_ctrl104:1;
			unsigned char has_ctrl105:1;
			unsigned char has_query28:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_23 {
	union {
		struct {
			unsigned char has_ctrl102:1;
			unsigned char has_ctrl102_sub1:1;
			unsigned char has_ctrl102_sub2:1;
			unsigned char has_ctrl102_sub4:1;
			unsigned char has_ctrl102_sub5:1;
			unsigned char has_ctrl102_sub9:1;
			unsigned char has_ctrl102_sub10:1;
			unsigned char has_ctrl102_sub11:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_25 {
	union {
		struct {
			unsigned char has_ctrl106:1;
			unsigned char has_ctrl102_sub12:1;
			unsigned char has_ctrl107:1;
			unsigned char has_ctrl108:1;
			unsigned char has_ctrl109:1;
			unsigned char has_data20:1;
			unsigned char f54_query25_b6:1;
			unsigned char has_query27:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_27 {
	union {
		struct {
			unsigned char has_ctrl110:1;
			unsigned char has_data21:1;
			unsigned char has_ctrl111:1;
			unsigned char has_ctrl112:1;
			unsigned char has_ctrl113:1;
			unsigned char has_data22:1;
			unsigned char has_ctrl114:1;
			unsigned char has_query29:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_29 {
	union {
		struct {
			unsigned char has_ctrl115:1;
			unsigned char has_ground_ring_options:1;
			unsigned char has_lost_bursts_tuning:1;
			unsigned char has_aux_exvcom2_select:1;
			unsigned char has_ctrl116:1;
			unsigned char has_data23:1;
			unsigned char has_ctrl117:1;
			unsigned char has_query30:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_30 {
	union {
		struct {
			unsigned char has_ctrl118:1;
			unsigned char has_ctrl119:1;
			unsigned char has_ctrl120:1;
			unsigned char has_ctrl121:1;
			unsigned char has_ctrl122_query31:1;
			unsigned char has_ctrl123:1;
			unsigned char has_ctrl124:1;
			unsigned char has_query32:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_32 {
	union {
		struct {
			unsigned char has_ctrl125:1;
			unsigned char has_ctrl126:1;
			unsigned char has_ctrl127:1;
			unsigned char has_abs_charge_pump_disable:1;
			unsigned char has_query33:1;
			unsigned char has_data24:1;
			unsigned char has_query34:1;
			unsigned char has_query35:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_33 {
	union {
		struct {
			unsigned char has_ctrl128:1;
			unsigned char has_ctrl129:1;
			unsigned char has_ctrl130:1;
			unsigned char has_ctrl131:1;
			unsigned char has_ctrl132:1;
			unsigned char has_ctrl133:1;
			unsigned char has_ctrl134:1;
			unsigned char has_query36:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_35 {
	union {
		struct {
			unsigned char has_data25:1;
			unsigned char has_ctrl135:1;
			unsigned char has_ctrl136:1;
			unsigned char has_ctrl137:1;
			unsigned char has_ctrl138:1;
			unsigned char has_ctrl139:1;
			unsigned char has_data26:1;
			unsigned char has_ctrl140:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_36 {
	union {
		struct {
			unsigned char has_ctrl141:1;
			unsigned char has_ctrl142:1;
			unsigned char has_query37:1;
			unsigned char has_ctrl143:1;
			unsigned char has_ctrl144:1;
			unsigned char has_ctrl145:1;
			unsigned char has_ctrl146:1;
			unsigned char has_query38:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_38 {
	union {
		struct {
			unsigned char has_ctrl147:1;
			unsigned char has_ctrl148:1;
			unsigned char has_ctrl149:1;
			unsigned char has_ctrl150:1;
			unsigned char has_ctrl151:1;
			unsigned char has_ctrl152:1;
			unsigned char has_ctrl153:1;
			unsigned char has_query39:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_39 {
	union {
		struct {
			unsigned char has_ctrl154:1;
			unsigned char has_ctrl155:1;
			unsigned char has_ctrl156:1;
			unsigned char has_ctrl160:1;
			unsigned char has_ctrl157_ctrl158:1;
			unsigned char f54_query39_b5__6:2;
			unsigned char has_query40:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_40 {
	union {
		struct {
			unsigned char has_ctrl169:1;
			unsigned char has_ctrl163_query41:1;
			unsigned char f54_query40_b2:1;
			unsigned char has_ctrl165_query42:1;
			unsigned char has_ctrl166:1;
			unsigned char has_ctrl167:1;
			unsigned char has_ctrl168:1;
			unsigned char has_query43:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_43 {
	union {
		struct {
			unsigned char f54_query43_b0__1:2;
			unsigned char has_ctrl171:1;
			unsigned char has_ctrl172_query44_query45:1;
			unsigned char has_ctrl173:1;
			unsigned char has_ctrl174:1;
			unsigned char has_ctrl175:1;
			unsigned char has_query46:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_46 {
	union {
		struct {
			unsigned char has_ctrl176:1;
			unsigned char has_ctrl177_ctrl178:1;
			unsigned char has_ctrl179:1;
			unsigned char f54_query46_b3:1;
			unsigned char has_data27:1;
			unsigned char has_data28:1;
			unsigned char f54_query46_b6:1;
			unsigned char has_query47:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_47 {
	union {
		struct {
			unsigned char f54_query47_b0:1;
			unsigned char has_ctrl182:1;
			unsigned char has_ctrl183:1;
			unsigned char f54_query47_b3:1;
			unsigned char has_ctrl185:1;
			unsigned char has_ctrl186:1;
			unsigned char has_ctrl187:1;
			unsigned char has_query49:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_49 {
	union {
		struct {
			unsigned char f54_query49_b0__1:2;
			unsigned char has_ctrl188:1;
			unsigned char has_data31:1;
			unsigned char f54_query49_b4__6:3;
			unsigned char has_query50:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_50 {
	union {
		struct {
			unsigned char f54_query50_b0__6:7;
			unsigned char has_query51:1;
		} __packed;
		unsigned char data[1];
	};
};

/* tddi f54 test reporting + */
struct f54_query_51 {
	union {
		struct {
			unsigned char f54_query51_b0__4:5;
			unsigned char has_query53_query54_ctrl198:1;
			unsigned char has_ctrl199:1;
			unsigned char has_query55:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_55 {
	union {
		struct {
			unsigned char has_query56:1;
			unsigned char has_data33_data34:1;
			unsigned char has_alternate_report_rate:1;
			unsigned char has_ctrl200:1;
			unsigned char has_ctrl201_ctrl202:1;
			unsigned char has_ctrl203:1;
			unsigned char has_ctrl204:1;
			unsigned char has_query57:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_57 {
	union {
		struct {
			unsigned char has_ctrl205:1;
			unsigned char has_ctrl206:1;
			unsigned char has_usb_bulk_read:1;
			unsigned char has_ctrl207:1;
			unsigned char has_ctrl208:1;
			unsigned char has_ctrl209:1;
			unsigned char has_ctrl210:1;
			unsigned char has_query58:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_58 {
	union {
		struct {
			unsigned char has_query59:1;
			unsigned char has_query60:1;
			unsigned char has_ctrl211:1;
			unsigned char has_ctrl212:1;
			unsigned char has_hybrid_abs_tx_axis_filtering:1;
			unsigned char f54_query58_b5:1;
			unsigned char has_ctrl213:1;
			unsigned char has_query61:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_61 {
	union {
		struct {
			unsigned char has_ctrl214:1;
			unsigned char has_ctrl215_query62_query63:1;
			unsigned char f54_query61_b2__4:3;
			unsigned char has_misc_host_ctrl:1;
			unsigned char has_hybrid_abs_buttons:1;
			unsigned char has_query64:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_64 {
	union {
		struct {
			unsigned char f54_query64_b0:1;
			unsigned char has_ctrl220:1;
			unsigned char f54_query64_b2__3:2;
			unsigned char has_ctrl219_sub1:1;
			unsigned char f54_query64_b5:1;
			unsigned char has_ctrl224_ctrl226_ctrl227:1;
			unsigned char has_query65:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_65 {
	union {
		struct {
			unsigned char f54_query65_b0__4:5;
			unsigned char has_query66_ctrl231:1;
			unsigned char has_ctrl232:1;
			unsigned char has_query67:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_67 {
	union {
		struct {
			unsigned char has_abs_doze_spatial_filter_enable:1;
			unsigned char has_abs_doze_average_filter_enable:1;
			unsigned char has_single_display_pulse:1;
			unsigned char f54_query67_b3__6:4;
			unsigned char has_query68:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_68 {
	union {
		struct {
			unsigned char f54_query68_b0__4:5;
			unsigned char has_freq_filter_bw_ext:1;
			unsigned char f54_query68_b6:1;
			unsigned char has_query69:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_69 {
	union {
		struct {
			unsigned char has_ctrl240_sub0:1;
			unsigned char has_ctrl240_sub1_sub2:1;
			unsigned char has_ctrl240_sub3:1;
			unsigned char has_ctrl240_sub4:1;
			unsigned char burst_mode_report_type_enabled:1;
			unsigned char f54_query69_b5__7:3;
		} __packed;
		unsigned char data[1];
	};
};
/* tddi f54 test reporting - */

struct f54_data_31 {
	union {
		struct {
			unsigned char is_calibration_crc:1;
			unsigned char calibration_crc:1;
			unsigned char short_test_row_number:5;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_0 {
	union {
		struct {
			unsigned char no_relax:1;
			unsigned char no_scan:1;
			unsigned char force_fast_relaxation:1;
			unsigned char startup_fast_relaxation:1;
			unsigned char gesture_cancels_sfr:1;
			unsigned char enable_energy_ratio_relaxation:1;
			unsigned char excessive_noise_attn_enable:1;
			unsigned char f54_control0_b7:1;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_1 {
	union {
		struct {
			unsigned char bursts_per_cluster:4;
			unsigned char f54_ctrl1_b4__7:4;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_2 {
	union {
		struct {
			unsigned short saturation_cap;
		} __packed;
		struct {
			unsigned char data[2];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_3 {
	union {
		struct {
			unsigned char pixel_touch_threshold;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_4__6 {
	union {
		struct {
			/* control 4 */
			unsigned char rx_feedback_cap:2;
			unsigned char bias_current:2;
			unsigned char f54_ctrl4_b4__7:4;

			/* control 5 */
			unsigned char low_ref_cap:2;
			unsigned char low_ref_feedback_cap:2;
			unsigned char low_ref_polarity:1;
			unsigned char f54_ctrl5_b5__7:3;

			/* control 6 */
			unsigned char high_ref_cap:2;
			unsigned char high_ref_feedback_cap:2;
			unsigned char high_ref_polarity:1;
			unsigned char f54_ctrl6_b5__7:3;
		} __packed;
		struct {
			unsigned char data[3];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_7 {
	union {
		struct {
			unsigned char cbc_cap:3;
			unsigned char cbc_polarity:1;
			unsigned char cbc_tx_carrier_selection:1;
			unsigned char f54_ctrl7_b5__7:3;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_8__9 {
	union {
		struct {
			/* control 8 */
			unsigned short integration_duration:10;
			unsigned short f54_ctrl8_b10__15:6;

			/* control 9 */
			unsigned char reset_duration;
		} __packed;
		struct {
			unsigned char data[3];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_10 {
	union {
		struct {
			unsigned char noise_sensing_bursts_per_image:4;
			unsigned char f54_ctrl10_b4__7:4;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_11 {
	union {
		struct {
			unsigned short f54_ctrl11;
		} __packed;
		struct {
			unsigned char data[2];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_12__13 {
	union {
		struct {
			/* control 12 */
			unsigned char slow_relaxation_rate;

			/* control 13 */
			unsigned char fast_relaxation_rate;
		} __packed;
		struct {
			unsigned char data[2];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_14 {
	union {
		struct {
			unsigned char rxs_on_xaxis:1;
			unsigned char curve_comp_on_txs:1;
			unsigned char f54_ctrl14_b2__7:6;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_15n {
	unsigned char sensor_rx_assignment;
};

struct f54_control_15 {
	struct f54_control_15n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_16n {
	unsigned char sensor_tx_assignment;
};

struct f54_control_16 {
	struct f54_control_16n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_17n {
	unsigned char burst_count_b8__10:3;
	unsigned char disable:1;
	unsigned char f54_ctrl17_b4:1;
	unsigned char filter_bandwidth:3;
};

struct f54_control_17 {
	struct f54_control_17n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_18n {
	unsigned char burst_count_b0__7;
};

struct f54_control_18 {
	struct f54_control_18n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_19n {
	unsigned char stretch_duration;
};

struct f54_control_19 {
	struct f54_control_19n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_20 {
	union {
		struct {
			unsigned char disable_noise_mitigation:1;
			unsigned char f54_ctrl20_b1__7:7;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_21 {
	union {
		struct {
			unsigned short freq_shift_noise_threshold;
		} __packed;
		struct {
			unsigned char data[2];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_22__26 {
	union {
		struct {
			/* control 22 */
			unsigned char f54_ctrl22;

			/* control 23 */
			unsigned short medium_noise_threshold;

			/* control 24 */
			unsigned short high_noise_threshold;

			/* control 25 */
			unsigned char noise_density;

			/* control 26 */
			unsigned char frame_count;
		} __packed;
		struct {
			unsigned char data[7];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_27 {
	union {
		struct {
			unsigned char iir_filter_coef;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_28 {
	union {
		struct {
			unsigned short quiet_threshold;
		} __packed;
		struct {
			unsigned char data[2];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_29 {
	union {
		struct {
			/* control 29 */
			unsigned char f54_ctrl29_b0__6:7;
			unsigned char cmn_filter_disable:1;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_30 {
	union {
		struct {
			unsigned char cmn_filter_max;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_31 {
	union {
		struct {
			unsigned char touch_hysteresis;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_32__35 {
	union {
		struct {
			/* control 32 */
			unsigned short rx_low_edge_comp;

			/* control 33 */
			unsigned short rx_high_edge_comp;

			/* control 34 */
			unsigned short tx_low_edge_comp;

			/* control 35 */
			unsigned short tx_high_edge_comp;
		} __packed;
		struct {
			unsigned char data[8];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_36n {
	unsigned char axis1_comp;
};

struct f54_control_36 {
	struct f54_control_36n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_37n {
	unsigned char axis2_comp;
};

struct f54_control_37 {
	struct f54_control_37n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_38n {
	unsigned char noise_control_1;
};

struct f54_control_38 {
	struct f54_control_38n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_39n {
	unsigned char noise_control_2;
};

struct f54_control_39 {
	struct f54_control_39n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_40n {
	unsigned char noise_control_3;
};

struct f54_control_40 {
	struct f54_control_40n *data;
	unsigned short address;
	unsigned char length;
};

struct f54_control_41 {
	union {
		struct {
			unsigned char no_signal_clarity:1;
			unsigned char f54_ctrl41_b1__7:7;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_57 {
	union {
		struct {
			unsigned char cbc_cap_0d:3;
			unsigned char cbc_polarity_0d:1;
			unsigned char cbc_tx_carrier_selection_0d:1;
			unsigned char f54_ctrl57_b5__7:3;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_86 {
	union {
		struct {
			unsigned char enable_high_noise_state:1;
			unsigned char dynamic_sense_display_ratio:2;
			unsigned char f54_ctrl86_b3__7:5;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_88 {
	union {
		struct {
			unsigned char tx_low_reference_polarity:1;
			unsigned char tx_high_reference_polarity:1;
			unsigned char abs_low_reference_polarity:1;
			unsigned char abs_polarity:1;
			unsigned char cbc_polarity:1;
			unsigned char cbc_tx_carrier_selection:1;
			unsigned char charge_pump_enable:1;
			unsigned char cbc_abs_auto_servo:1;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

/* tddi f54 test reporting + */
struct f54_control_91 {
	union {
		struct {
			unsigned char reflo_transcap_capacitance;
			unsigned char refhi_transcap_capacitance;
			unsigned char receiver_feedback_capacitance;
			unsigned char reference_receiver_feedback_capacitance;
			unsigned char gain_ctrl;
		} __packed;
		struct {
			unsigned char data[5];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_96 {
	union {
		struct {
			unsigned char cbc_transcap[64];
		} __packed;
		struct {
			unsigned char data[64];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_99 {
	union {
		struct {
			unsigned char integration_duration_lsb;
			unsigned char integration_duration_msb;
			unsigned char reset_duration;
		} __packed;
		struct {
			unsigned char data[3];
			unsigned short address;
		} __packed;
	};
};
/* tddi f54 test reporting - */

struct f54_control_110 {
	union {
		struct {
			unsigned char active_stylus_rx_feedback_cap;
			unsigned char active_stylus_rx_feedback_cap_reference;
			unsigned char active_stylus_low_reference;
			unsigned char active_stylus_high_reference;
			unsigned char active_stylus_gain_control;
			unsigned char active_stylus_gain_control_reference;
			unsigned char active_stylus_timing_mode;
			unsigned char active_stylus_discovery_bursts;
			unsigned char active_stylus_detection_bursts;
			unsigned char active_stylus_discovery_noise_multiplier;
			unsigned char active_stylus_detection_envelope_min;
			unsigned char active_stylus_detection_envelope_max;
			unsigned char active_stylus_lose_count;
		} __packed;
		struct {
			unsigned char data[13];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_149 {
	union {
		struct {
			unsigned char trans_cbc_global_cap_enable:1;
			unsigned char f54_ctrl149_b1__7:7;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_188 {
	union {
		struct {
			unsigned char start_calibration:1;
			unsigned char start_is_calibration:1;
			unsigned char frequency:2;
			unsigned char start_production_test:1;
			unsigned char short_test_calibration:1;
			unsigned char f54_ctrl188_b7:1;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control {
	struct f54_control_0 *reg_0;
	struct f54_control_1 *reg_1;
	struct f54_control_2 *reg_2;
	struct f54_control_3 *reg_3;
	struct f54_control_4__6 *reg_4__6;
	struct f54_control_7 *reg_7;
	struct f54_control_8__9 *reg_8__9;
	struct f54_control_10 *reg_10;
	struct f54_control_11 *reg_11;
	struct f54_control_12__13 *reg_12__13;
	struct f54_control_14 *reg_14;
	struct f54_control_15 *reg_15;
	struct f54_control_16 *reg_16;
	struct f54_control_17 *reg_17;
	struct f54_control_18 *reg_18;
	struct f54_control_19 *reg_19;
	struct f54_control_20 *reg_20;
	struct f54_control_21 *reg_21;
	struct f54_control_22__26 *reg_22__26;
	struct f54_control_27 *reg_27;
	struct f54_control_28 *reg_28;
	struct f54_control_29 *reg_29;
	struct f54_control_30 *reg_30;
	struct f54_control_31 *reg_31;
	struct f54_control_32__35 *reg_32__35;
	struct f54_control_36 *reg_36;
	struct f54_control_37 *reg_37;
	struct f54_control_38 *reg_38;
	struct f54_control_39 *reg_39;
	struct f54_control_40 *reg_40;
	struct f54_control_41 *reg_41;
	struct f54_control_57 *reg_57;
	struct f54_control_86 *reg_86;
	struct f54_control_88 *reg_88;
	/* tddi f54 test reporting + */
	struct f54_control_91 *reg_91;
	struct f54_control_96 *reg_96;
    struct f54_control_99 *reg_99;
	/* tddi f54 test reporting - */
	struct f54_control_110 *reg_110;
	struct f54_control_149 *reg_149;
	struct f54_control_188 *reg_188;
};

struct synaptics_rmi4_f54_handle {
	bool no_auto_cal;
	bool skip_preparation;
	unsigned char status;
	unsigned char intr_mask;
	unsigned char intr_reg_num;
	unsigned char tx_assigned;
	unsigned char rx_assigned;
	/* tddi f54 test reporting + */
	unsigned char swap_sensor_side;
	unsigned char left_mux_size;
	unsigned char right_mux_size;
	/*tddi f54 test reporting - */
	unsigned char *report_data;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	unsigned short fifoindex;
	unsigned int report_size;
	unsigned int data_buffer_size;
	enum f54_report_types report_type;
	struct mutex status_mutex;
	struct mutex data_mutex;
	struct mutex control_mutex;
	struct f54_query query;
	struct f54_query_13 query_13;
	struct f54_query_15 query_15;
	struct f54_query_16 query_16;
	struct f54_query_21 query_21;
	struct f54_query_22 query_22;
	struct f54_query_23 query_23;
	struct f54_query_25 query_25;
	struct f54_query_27 query_27;
	struct f54_query_29 query_29;
	struct f54_query_30 query_30;
	struct f54_query_32 query_32;
	struct f54_query_33 query_33;
	struct f54_query_35 query_35;
	struct f54_query_36 query_36;
	struct f54_query_38 query_38;
	struct f54_query_39 query_39;
	struct f54_query_40 query_40;
	struct f54_query_43 query_43;
	struct f54_query_46 query_46;
	struct f54_query_47 query_47;
	struct f54_query_49 query_49;
	struct f54_query_50 query_50;
	struct f54_query_51 query_51;
	/* tddi f54 test reporting + */
	struct f54_query_55 query_55;
	struct f54_query_57 query_57;
	struct f54_query_58 query_58;
	struct f54_query_61 query_61;
	struct f54_query_64 query_64;
	struct f54_query_65 query_65;
	struct f54_query_67 query_67;
	struct f54_query_68 query_68;
	struct f54_query_69 query_69;
	/* tddi f54 test reporting - */
	struct f54_data_31 data_31;
	struct f54_control control;
	struct kobject *attr_dir;
	struct hrtimer watchdog;
	struct work_struct timeout_work;
	struct delayed_work status_work;
	struct workqueue_struct *status_workqueue;
	struct synaptics_rmi4_access_ptr *fn_ptr;
	struct synaptics_rmi4_data *rmi4_data;
};

struct f55_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;

			/* query 1 */
			unsigned char num_of_tx_electrodes;

			/* query 2 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_edge_compensation:1;
			unsigned char curve_compensation_mode:2;
			unsigned char has_ctrl6:1;
			unsigned char has_alternate_transmitter_assignment:1;
			unsigned char has_single_layer_multi_touch:1;
			unsigned char has_query5:1;
		} __packed;
		unsigned char data[3];
	};
};

struct f55_query_3 {
	union {
		struct {
			unsigned char has_ctrl8:1;
			unsigned char has_ctrl9:1;
			unsigned char has_oncell_pattern_support:1;
			unsigned char has_data0:1;
			unsigned char has_single_wide_pattern_support:1;
			unsigned char has_mirrored_tx_pattern_support:1;
			unsigned char has_discrete_pattern_support:1;
			unsigned char has_query9:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_5 {
	union {
		struct {
			unsigned char has_corner_compensation:1;
			unsigned char has_ctrl12:1;
			unsigned char has_trx_configuration:1;
			unsigned char has_ctrl13:1;
			unsigned char f55_query5_b4:1;
			unsigned char has_ctrl14:1;
			unsigned char has_basis_function:1;
			unsigned char has_query17:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_17 {
	union {
		struct {
			unsigned char f55_query17_b0:1;
			unsigned char has_ctrl16:1;
			unsigned char has_ctrl18_ctrl19:1;
			unsigned char has_ctrl17:1;
			unsigned char has_ctrl20:1;
			unsigned char has_ctrl21:1;
			unsigned char has_ctrl22:1;
			unsigned char has_query18:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_18 {
	union {
		struct {
			unsigned char has_ctrl23:1;
			unsigned char has_ctrl24:1;
			unsigned char has_query19:1;
			unsigned char has_ctrl25:1;
			unsigned char has_ctrl26:1;
			unsigned char has_ctrl27_query20:1;
			unsigned char has_ctrl28_query21:1;
			unsigned char has_query22:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_22 {
	union {
		struct {
			unsigned char has_ctrl29:1;
			unsigned char has_query23:1;
			unsigned char has_guard_disable:1;
			unsigned char has_ctrl30:1;
			unsigned char has_ctrl31:1;
			unsigned char has_ctrl32:1;
			unsigned char has_query24_through_query27:1;
			unsigned char has_query28:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_23 {
	union {
		struct {
			unsigned char amp_sensor_enabled:1;
			unsigned char image_transposed:1;
			unsigned char first_column_at_left_side:1;
			unsigned char size_of_column2mux:5;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_28 {
	union {
		struct {
			unsigned char f55_query28_b0__4:5;
			unsigned char has_ctrl37:1;
			unsigned char has_query29:1;
			unsigned char has_query30:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_30 {
	union {
		struct {
			unsigned char has_ctrl38:1;
			unsigned char has_query31_query32:1;
			unsigned char has_ctrl39:1;
			unsigned char has_ctrl40:1;
			unsigned char has_ctrl41:1;
			unsigned char has_ctrl42:1;
			unsigned char has_ctrl43_ctrl44:1;
			unsigned char has_query33:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_query_33 {
	union {
		struct {
			unsigned char has_extended_amp_pad:1;
			unsigned char has_extended_amp_btn:1;
			unsigned char has_ctrl45_ctrl46:1;
			unsigned char f55_query33_b3:1;
			unsigned char has_ctrl47_sub0_sub1:1;
			unsigned char f55_query33_b5__7:3;
		} __packed;
		unsigned char data[1];
	};
};

struct f55_control_43 {
	union {
		struct {
			unsigned char swap_sensor_side:1;
			unsigned char f55_ctrl43_b1__7:7;
			unsigned char afe_l_mux_size:4;
			unsigned char afe_r_mux_size:4;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f55_handle {
	bool amp_sensor;
	bool extended_amp;
	bool has_force;
	unsigned char size_of_column2mux;
	unsigned char afe_mux_offset;
	unsigned char force_tx_offset;
	unsigned char force_rx_offset;
	unsigned char *tx_assignment;
	unsigned char *rx_assignment;
	unsigned char *force_tx_assignment;
	unsigned char *force_rx_assignment;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	struct f55_query query;
	struct f55_query_3 query_3;
	struct f55_query_5 query_5;
	struct f55_query_17 query_17;
	struct f55_query_18 query_18;
	struct f55_query_22 query_22;
	struct f55_query_23 query_23;
	struct f55_query_28 query_28;
	struct f55_query_30 query_30;
	struct f55_query_33 query_33;
};

show_prototype(num_of_mapped_tx)
show_prototype(num_of_mapped_rx)
show_prototype(report_size)
show_prototype(status)
store_prototype(do_preparation)
store_prototype(force_cal)
store_prototype(get_report)
store_prototype(resume_touch)
store_prototype(do_afe_calibration)
show_store_prototype(report_type)
show_store_prototype(fifoindex)
show_store_prototype(no_auto_cal)
show_store_prototype(read_report)
show_prototype(num_of_rx_electrodes)
show_prototype(num_of_tx_electrodes)
show_prototype(has_image16)
show_prototype(has_image8)
show_prototype(has_baseline)
show_prototype(clock_rate)
show_prototype(touch_controller_family)
show_prototype(has_pixel_touch_threshold_adjustment)
show_prototype(has_sensor_assignment)
show_prototype(has_interference_metric)
show_prototype(has_sense_frequency_control)
show_prototype(has_firmware_noise_mitigation)
show_prototype(has_two_byte_report_rate)
show_prototype(has_one_byte_report_rate)
show_prototype(has_relaxation_control)
show_prototype(curve_compensation_mode)
show_prototype(has_iir_filter)
show_prototype(has_cmn_removal)
show_prototype(has_cmn_maximum)
show_prototype(has_touch_hysteresis)
show_prototype(has_edge_compensation)
show_prototype(has_per_frequency_noise_control)
show_prototype(has_signal_clarity)
show_prototype(number_of_sensing_frequencies)

show_store_prototype(no_relax)
show_store_prototype(no_scan)
show_store_prototype(bursts_per_cluster)
show_store_prototype(saturation_cap)
show_store_prototype(pixel_touch_threshold)
show_store_prototype(rx_feedback_cap)
show_store_prototype(low_ref_cap)
show_store_prototype(low_ref_feedback_cap)
show_store_prototype(low_ref_polarity)
show_store_prototype(high_ref_cap)
show_store_prototype(high_ref_feedback_cap)
show_store_prototype(high_ref_polarity)
show_store_prototype(cbc_cap)
show_store_prototype(cbc_polarity)
show_store_prototype(cbc_tx_carrier_selection)
show_store_prototype(integration_duration)
show_store_prototype(reset_duration)
show_store_prototype(noise_sensing_bursts_per_image)
show_store_prototype(slow_relaxation_rate)
show_store_prototype(fast_relaxation_rate)
show_store_prototype(rxs_on_xaxis)
show_store_prototype(curve_comp_on_txs)
show_prototype(sensor_rx_assignment)
show_prototype(sensor_tx_assignment)
show_prototype(burst_count)
show_prototype(disable)
show_prototype(filter_bandwidth)
show_prototype(stretch_duration)
show_store_prototype(disable_noise_mitigation)
show_store_prototype(freq_shift_noise_threshold)
show_store_prototype(medium_noise_threshold)
show_store_prototype(high_noise_threshold)
show_store_prototype(noise_density)
show_store_prototype(frame_count)
show_store_prototype(iir_filter_coef)
show_store_prototype(quiet_threshold)
show_store_prototype(cmn_filter_disable)
show_store_prototype(cmn_filter_max)
show_store_prototype(touch_hysteresis)
show_store_prototype(rx_low_edge_comp)
show_store_prototype(rx_high_edge_comp)
show_store_prototype(tx_low_edge_comp)
show_store_prototype(tx_high_edge_comp)
show_store_prototype(axis1_comp)
show_store_prototype(axis2_comp)
show_prototype(noise_control_1)
show_prototype(noise_control_2)
show_prototype(noise_control_3)
show_store_prototype(no_signal_clarity)
show_store_prototype(cbc_cap_0d)
show_store_prototype(cbc_polarity_0d)
show_store_prototype(cbc_tx_carrier_selection_0d)
/* tddi f54 test reporting + */
show_store_prototype(tddi_full_raw)
show_store_prototype(tddi_ee_short)
show_store_prototype(tddi_amp_open)
/* tddi f54 test reporting - */
show_prototype(self_test)

static ssize_t synaptics_rmi4_f54_data_read(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct attribute *attrs[] = {
	attrify(num_of_mapped_tx),
	attrify(num_of_mapped_rx),
	attrify(report_size),
	attrify(status),
	attrify(do_preparation),
	attrify(force_cal),
	attrify(get_report),
	attrify(resume_touch),
	attrify(do_afe_calibration),
	attrify(report_type),
	attrify(fifoindex),
	attrify(no_auto_cal),
	attrify(read_report),
	attrify(num_of_rx_electrodes),
	attrify(num_of_tx_electrodes),
	attrify(has_image16),
	attrify(has_image8),
	attrify(has_baseline),
	attrify(clock_rate),
	attrify(touch_controller_family),
	attrify(has_pixel_touch_threshold_adjustment),
	attrify(has_sensor_assignment),
	attrify(has_interference_metric),
	attrify(has_sense_frequency_control),
	attrify(has_firmware_noise_mitigation),
	attrify(has_two_byte_report_rate),
	attrify(has_one_byte_report_rate),
	attrify(has_relaxation_control),
	attrify(curve_compensation_mode),
	attrify(has_iir_filter),
	attrify(has_cmn_removal),
	attrify(has_cmn_maximum),
	attrify(has_touch_hysteresis),
	attrify(has_edge_compensation),
	attrify(has_per_frequency_noise_control),
	attrify(has_signal_clarity),
	attrify(number_of_sensing_frequencies),
	/* tddi f54 test reporting + */
	attrify(tddi_full_raw),
	attrify(tddi_ee_short),
	attrify(tddi_amp_open),
	/* tddi f54 test reporting - */
	attrify(self_test),
	NULL,
};

static struct attribute_group attr_group = GROUP(attrs);

static struct attribute *attrs_reg_0[] = {
	attrify(no_relax),
	attrify(no_scan),
	NULL,
};

static struct attribute *attrs_reg_1[] = {
	attrify(bursts_per_cluster),
	NULL,
};

static struct attribute *attrs_reg_2[] = {
	attrify(saturation_cap),
	NULL,
};

static struct attribute *attrs_reg_3[] = {
	attrify(pixel_touch_threshold),
	NULL,
};

static struct attribute *attrs_reg_4__6[] = {
	attrify(rx_feedback_cap),
	attrify(low_ref_cap),
	attrify(low_ref_feedback_cap),
	attrify(low_ref_polarity),
	attrify(high_ref_cap),
	attrify(high_ref_feedback_cap),
	attrify(high_ref_polarity),
	NULL,
};

static struct attribute *attrs_reg_7[] = {
	attrify(cbc_cap),
	attrify(cbc_polarity),
	attrify(cbc_tx_carrier_selection),
	NULL,
};

static struct attribute *attrs_reg_8__9[] = {
	attrify(integration_duration),
	attrify(reset_duration),
	NULL,
};

static struct attribute *attrs_reg_10[] = {
	attrify(noise_sensing_bursts_per_image),
	NULL,
};

static struct attribute *attrs_reg_11[] = {
	NULL,
};

static struct attribute *attrs_reg_12__13[] = {
	attrify(slow_relaxation_rate),
	attrify(fast_relaxation_rate),
	NULL,
};

static struct attribute *attrs_reg_14__16[] = {
	attrify(rxs_on_xaxis),
	attrify(curve_comp_on_txs),
	attrify(sensor_rx_assignment),
	attrify(sensor_tx_assignment),
	NULL,
};

static struct attribute *attrs_reg_17__19[] = {
	attrify(burst_count),
	attrify(disable),
	attrify(filter_bandwidth),
	attrify(stretch_duration),
	NULL,
};

static struct attribute *attrs_reg_20[] = {
	attrify(disable_noise_mitigation),
	NULL,
};

static struct attribute *attrs_reg_21[] = {
	attrify(freq_shift_noise_threshold),
	NULL,
};

static struct attribute *attrs_reg_22__26[] = {
	attrify(medium_noise_threshold),
	attrify(high_noise_threshold),
	attrify(noise_density),
	attrify(frame_count),
	NULL,
};

static struct attribute *attrs_reg_27[] = {
	attrify(iir_filter_coef),
	NULL,
};

static struct attribute *attrs_reg_28[] = {
	attrify(quiet_threshold),
	NULL,
};

static struct attribute *attrs_reg_29[] = {
	attrify(cmn_filter_disable),
	NULL,
};

static struct attribute *attrs_reg_30[] = {
	attrify(cmn_filter_max),
	NULL,
};

static struct attribute *attrs_reg_31[] = {
	attrify(touch_hysteresis),
	NULL,
};

static struct attribute *attrs_reg_32__35[] = {
	attrify(rx_low_edge_comp),
	attrify(rx_high_edge_comp),
	attrify(tx_low_edge_comp),
	attrify(tx_high_edge_comp),
	NULL,
};

static struct attribute *attrs_reg_36[] = {
	attrify(axis1_comp),
	NULL,
};

static struct attribute *attrs_reg_37[] = {
	attrify(axis2_comp),
	NULL,
};

static struct attribute *attrs_reg_38__40[] = {
	attrify(noise_control_1),
	attrify(noise_control_2),
	attrify(noise_control_3),
	NULL,
};

static struct attribute *attrs_reg_41[] = {
	attrify(no_signal_clarity),
	NULL,
};

static struct attribute *attrs_reg_57[] = {
	attrify(cbc_cap_0d),
	attrify(cbc_polarity_0d),
	attrify(cbc_tx_carrier_selection_0d),
	NULL,
};

static struct attribute_group attrs_ctrl_regs[] = {
	GROUP(attrs_reg_0),
	GROUP(attrs_reg_1),
	GROUP(attrs_reg_2),
	GROUP(attrs_reg_3),
	GROUP(attrs_reg_4__6),
	GROUP(attrs_reg_7),
	GROUP(attrs_reg_8__9),
	GROUP(attrs_reg_10),
	GROUP(attrs_reg_11),
	GROUP(attrs_reg_12__13),
	GROUP(attrs_reg_14__16),
	GROUP(attrs_reg_17__19),
	GROUP(attrs_reg_20),
	GROUP(attrs_reg_21),
	GROUP(attrs_reg_22__26),
	GROUP(attrs_reg_27),
	GROUP(attrs_reg_28),
	GROUP(attrs_reg_29),
	GROUP(attrs_reg_30),
	GROUP(attrs_reg_31),
	GROUP(attrs_reg_32__35),
	GROUP(attrs_reg_36),
	GROUP(attrs_reg_37),
	GROUP(attrs_reg_38__40),
	GROUP(attrs_reg_41),
	GROUP(attrs_reg_57),
};

static bool attrs_ctrl_regs_exist[ARRAY_SIZE(attrs_ctrl_regs)];

static struct bin_attribute dev_report_data = {
	.attr = {
		.name = "report_data",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = synaptics_rmi4_f54_data_read,
};

static struct synaptics_rmi4_f54_handle *f54;
static struct synaptics_rmi4_f55_handle *f55;


/* tddi f54 test reporting + */
static unsigned char *g_tddi_full_raw_data_output;
static unsigned char *g_tddi_ee_short_data_output;
static unsigned char *g_tddi_amp_open_data_output;

static short *g_tddi_ee_short_data_log_1;  // 20160920
static short *g_tddi_ee_short_data_log_2;  // 20160920

static short *g_tddi_amp_open_data_log_1;  // 20160920
static short *g_tddi_amp_open_data_log_2;  // 20160920
/* tddi f54 test reporting - */

DECLARE_COMPLETION(f54_remove_complete);

static bool is_report_type_valid(enum f54_report_types report_type)
{
	switch (report_type) {
	case F54_8BIT_IMAGE:
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TX_TO_TX_SHORTS:
	case F54_RX_TO_RX_SHORTS_1:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_RX_OPENS_1:
	case F54_TX_OPENS:
	case F54_TX_TO_GND_SHORTS:
	case F54_RX_TO_RX_SHORTS_2:
	case F54_RX_OPENS_2:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_NO_RX_COUPLING:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_TRX_OPENS:
	case F54_TRX_TO_GND_SHORTS:
	case F54_TRX_SHORTS:
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
	case F54_ABS_HYBRID_DELTA_CAP:
	case F54_ABS_HYBRID_RAW_CAP:
	case F54_AMP_FULL_RAW_CAP:
	case F54_AMP_RAW_ADC:
	/* tddi f54 test reporting + */
	case F54_FULL_RAW_CAP_TDDI:
	case F54_NOISE_TDDI:
	case F54_EE_SHORT_TDDI:
	/* tddi f54 test reporting - */
		return true;
		break;
	default:
		f54->report_type = INVALID_REPORT_TYPE;
		f54->report_size = 0;
		return false;
	}
}

static void set_report_size(void)
{
	int retval;
	unsigned char tx = f54->tx_assigned;
	unsigned char rx = f54->rx_assigned;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	switch (f54->report_type) {
	case F54_8BIT_IMAGE:
		f54->report_size = tx * rx;
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_NO_RX_COUPLING:
	case F54_SENSOR_SPEED:
	case F54_AMP_FULL_RAW_CAP:
	case F54_AMP_RAW_ADC:
	/* tddi f54 test reporting + */
	case F54_FULL_RAW_CAP_TDDI:
	case F54_NOISE_TDDI:
	/* tddi f54 test reporting - */
		f54->report_size = 2 * tx * rx;
		break;
	/* tddi f54 test reporting +  */
	case F54_EE_SHORT_TDDI:
		f54->report_size = 2 * 2 * tx * rx;
		break;
	/* tddi f54 test reporting - */
	case F54_HIGH_RESISTANCE:
		f54->report_size = HIGH_RESISTANCE_DATA_SIZE;
		break;
	case F54_TX_TO_TX_SHORTS:
	case F54_TX_OPENS:
	case F54_TX_TO_GND_SHORTS:
		f54->report_size = (tx + 7) / 8;
		break;
	case F54_RX_TO_RX_SHORTS_1:
	case F54_RX_OPENS_1:
		if (rx < tx)
			f54->report_size = 2 * rx * rx;
		else
			f54->report_size = 2 * tx * rx;
		break;
	case F54_FULL_RAW_CAP_MIN_MAX:
		f54->report_size = FULL_RAW_CAP_MIN_MAX_DATA_SIZE;
		break;
	case F54_RX_TO_RX_SHORTS_2:
	case F54_RX_OPENS_2:
		if (rx <= tx)
			f54->report_size = 0;
		else
			f54->report_size = 2 * rx * (rx - tx);
		break;
	case F54_ADC_RANGE:
		if (f54->query.has_signal_clarity) {
			mutex_lock(&f54->control_mutex);
			retval = f54->fn_ptr->read(rmi4_data,
					f54->control.reg_41->address,
					f54->control.reg_41->data,
					sizeof(f54->control.reg_41->data));
			mutex_unlock(&f54->control_mutex);
			if (retval < 0) {
				TP_LOGI("Failed to read control reg_41\n");
				f54->report_size = 0;
				break;
			}
			if (!f54->control.reg_41->no_signal_clarity) {
				if (tx % 4)
					tx += 4 - (tx % 4);
			}
		}
		f54->report_size = 2 * tx * rx;
		break;
	case F54_TRX_OPENS:
	case F54_TRX_TO_GND_SHORTS:
	case F54_TRX_SHORTS:
		f54->report_size = TRX_OPEN_SHORT_DATA_SIZE;
		break;
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
	case F54_ABS_HYBRID_DELTA_CAP:
	case F54_ABS_HYBRID_RAW_CAP:
		f54->report_size = 4 * (tx + rx);
		break;
	default:
		f54->report_size = 0;
	}

	return;
}

static int set_interrupt(bool set)
{
	int retval;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short f01_ctrl_reg;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	intr_mask = rmi4_data->intr_mask;
	f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (!set) {
		retval = f54->fn_ptr->write(rmi4_data,
				f01_ctrl_reg,
				&zero,
				sizeof(zero));
		if (retval < 0)
			return retval;
	}

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			if (set) {
				retval = f54->fn_ptr->write(rmi4_data,
						f01_ctrl_reg,
						&zero,
						sizeof(zero));
				if (retval < 0)
					return retval;
			} else {
				retval = f54->fn_ptr->write(rmi4_data,
						f01_ctrl_reg,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0)
					return retval;
			}
		}
	}

	f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (set) {
		retval = f54->fn_ptr->write(rmi4_data,
				f01_ctrl_reg,
				&f54->intr_mask,
				1);
		if (retval < 0)
			return retval;
	}

	return 0;
}

// Save test data to storge
static int save_test_data(char *file_name, char *data_buf, int iLen)
{
	struct file *pfile = NULL;

	loff_t pos;
	mm_segment_t old_fs;

	TP_LOGI("save test result to: %s\n", file_name);

	if (NULL == pfile)
		pfile = filp_open(file_name, O_CREAT | O_RDWR | O_TRUNC, 0);
	if (IS_ERR(pfile)) {
		TP_LOGE("error occured while opening file %s\n", file_name);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_write(pfile, data_buf, iLen, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}


static int do_preparation(void)
{
	int retval;
	unsigned char value;
	unsigned char command;
	unsigned char zero = 0x00;
	unsigned char device_ctrl;
	unsigned char timeout_count;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;
	//struct f54_control_86 reg_86;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_LOGE("Failed to set no sleep\n");
		return retval;
	}

	device_ctrl |= NO_SLEEP_ON;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_LOGE("Failed to set no sleep\n");
		return retval;
	}
// syna 20161122 remove
/*
	if ((f54->query.has_query13) &&
			(f54->query_13.has_ctrl86)) {
		reg_86.data[0] = f54->control.reg_86->data[0];
		reg_86.dynamic_sense_display_ratio = 1;
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_86->address,
				reg_86.data,
				sizeof(reg_86.data));
		if (retval < 0) {
			TP_LOGE("Failed to set sense display ratio\n");
			return retval;
		}
	}
*/
	if (f54->skip_preparation)
		return 0;

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
	case F54_ABS_HYBRID_DELTA_CAP:
	case F54_ABS_HYBRID_RAW_CAP:
	/* tddi f54 test reporting + */
	case F54_FULL_RAW_CAP_TDDI:
	case F54_NOISE_TDDI:
	case F54_EE_SHORT_TDDI:
	/* tddi f54 test reporting - */
		break;
	case F54_AMP_RAW_ADC:
		if (f54->query_49.has_ctrl188) {
			retval = synaptics_rmi4_reg_read(rmi4_data,
					f54->control.reg_188->address,
					f54->control.reg_188->data,
					sizeof(f54->control.reg_188->data));
			if (retval < 0) {
				TP_LOGE("Failed to read start production test\n");
				return retval;
			}

			f54->control.reg_188->start_production_test = 1;

			retval = synaptics_rmi4_reg_write(rmi4_data,
					f54->control.reg_188->address,
					f54->control.reg_188->data,
					sizeof(f54->control.reg_188->data));
			if (retval < 0) {
				TP_LOGE("Failed to set start production test\n");
				return retval;
			}
		}
		break;


	default:
		mutex_lock(&f54->control_mutex);
		if (f54->query.touch_controller_family == 1) {
			value = 0;
			retval = f54->fn_ptr->write(rmi4_data,
					f54->control.reg_7->address,
					&value,
					sizeof(f54->control.reg_7->data));
			if (retval < 0) {
				TP_LOGE("Failed to disable CBC\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}
		} else if (f54->query.has_ctrl88 == 1) {
			retval = f54->fn_ptr->read(rmi4_data,
					f54->control.reg_88->address,
					f54->control.reg_88->data,
					sizeof(f54->control.reg_88->data));
			if (retval < 0) {
				TP_LOGE("Failed to disable CBC (read ctrl88)\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}
			f54->control.reg_88->cbc_polarity = 0;
			f54->control.reg_88->cbc_tx_carrier_selection = 0;
			retval = f54->fn_ptr->write(rmi4_data,
					f54->control.reg_88->address,
					f54->control.reg_88->data,
					sizeof(f54->control.reg_88->data));
			if (retval < 0) {
				TP_LOGE("Failed to disable CBC (write ctrl88)\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}
		}

		if (f54->query.has_0d_acquisition_control) {
			value = 0;
			retval = f54->fn_ptr->write(rmi4_data,
					f54->control.reg_57->address,
					&value,
					sizeof(f54->control.reg_57->data));
			if (retval < 0) {
				TP_LOGE("Failed to disable 0D CBC\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}
		}

		if ((f54->query.has_query15) &&
				(f54->query_15.has_query25) &&
				(f54->query_25.has_query27) &&
				(f54->query_27.has_query29) &&
				(f54->query_29.has_query30) &&
				(f54->query_30.has_query32) &&
				(f54->query_32.has_query33) &&
				(f54->query_33.has_query36) &&
				(f54->query_36.has_query38) &&
				(f54->query_38.has_ctrl149)) {
			retval = synaptics_rmi4_reg_write(rmi4_data,
					f54->control.reg_149->address,
					&zero,
					sizeof(f54->control.reg_149->data));
			if (retval < 0) {
				TP_LOGE("Failed to disable global CBC\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}
		}

		if (f54->query.has_signal_clarity) {
			retval = f54->fn_ptr->read(rmi4_data,
					f54->control.reg_41->address,
					&value,
					sizeof(f54->control.reg_41->data));
			if (retval < 0) {
				TP_LOGE("Failed to read signal clarity\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}

			value |= 0x01;
			retval = f54->fn_ptr->write(rmi4_data,
					f54->control.reg_41->address,
					&value,
					sizeof(f54->control.reg_41->data));
			if (retval < 0) {
				TP_LOGE("Failed to disable signal clarity\n");
				mutex_unlock(&f54->control_mutex);
				return retval;
			}
		}

		mutex_unlock(&f54->control_mutex);

		command = (unsigned char)COMMAND_FORCE_UPDATE;

		retval = f54->fn_ptr->write(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));
		if (retval < 0) {
			TP_LOGE("Failed to write force update command\n");
			return retval;
		}

		timeout_count = 0;
		do {
			retval = f54->fn_ptr->read(rmi4_data,
					f54->command_base_addr,
					&value,
					sizeof(value));
			if (retval < 0) {
				TP_LOGE("Failed to read command register\n");
				return retval;
			}

			if (value == 0x00)
				break;

			msleep(100);
			timeout_count++;
		} while (timeout_count < FORCE_TIMEOUT_100MS);

		if (timeout_count == FORCE_TIMEOUT_100MS) {
			TP_LOGE("Timed out waiting for force update\n");
			return -ETIMEDOUT;
		}

		command = (unsigned char)COMMAND_FORCE_CAL;

		retval = f54->fn_ptr->write(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));
		if (retval < 0) {
			TP_LOGE("Failed to write force cal command\n");
			return retval;
		}

		timeout_count = 0;
		do {
			retval = f54->fn_ptr->read(rmi4_data,
					f54->command_base_addr,
					&value,
					sizeof(value));
			if (retval < 0) {
				TP_LOGE("Failed to read command register\n");
				return retval;
			}

			if (value == 0x00)
				break;

			msleep(100);
			timeout_count++;
		} while (timeout_count < FORCE_TIMEOUT_100MS);

		if (timeout_count == FORCE_TIMEOUT_100MS) {
			TP_LOGE("Timed out waiting for force cal\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int test_do_afe_calibration(enum f54_afe_cal mode)
{
	int retval;
	unsigned char timeout = CALIBRATION_TIMEOUT_S;
	unsigned char timeout_count = 0;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->control.reg_188->address,
			f54->control.reg_188->data,
			sizeof(f54->control.reg_188->data));
	if (retval < 0) {
		TP_LOGE("Failed to start calibration\n");
		return retval;
	}

	if (mode == F54_AFE_CAL)
		f54->control.reg_188->start_calibration = 1;
	else if (mode == F54_AFE_IS_CAL)
		f54->control.reg_188->start_is_calibration = 1;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->control.reg_188->address,
			f54->control.reg_188->data,
			sizeof(f54->control.reg_188->data));
	if (retval < 0) {
		TP_LOGE("Failed to start calibration\n");
		return retval;
	}

	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->control.reg_188->address,
				f54->control.reg_188->data,
				sizeof(f54->control.reg_188->data));
		if (retval < 0) {
			TP_LOGE("Failed to complete calibration\n");
			return retval;
		}

		if (mode == F54_AFE_CAL) {
			if (!f54->control.reg_188->start_calibration)
				break;
		} else if (mode == F54_AFE_IS_CAL) {
			if (!f54->control.reg_188->start_is_calibration)
				break;
		}

		if (timeout_count == timeout) {
			TP_LOGE("Timed out waiting for calibration completion\n");
			return -EBUSY;
		}

		timeout_count++;
		msleep(1000);
	} while (true);

	/* check CRC */
	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_31.address,
			f54->data_31.data,
			sizeof(f54->data_31.data));
	if (retval < 0) {
		TP_LOGE("Failed to read calibration CRC\n");
		return retval;
	}

	if (mode == F54_AFE_CAL) {
		if (f54->data_31.calibration_crc == 0)
			return 0;
	} else if (mode == F54_AFE_IS_CAL) {
		if (f54->data_31.is_calibration_crc == 0)
			return 0;
	}

	TP_LOGE("Failed to read calibration CRC\n");

	return -EINVAL;
}


#ifdef WATCHDOG_HRTIMER
static void timeout_set_status(struct work_struct *work)
{
	int retval;
	unsigned char command;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	mutex_lock(&f54->status_mutex);
	if (f54->status == STATUS_BUSY) {
		retval = f54->fn_ptr->read(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));
		if (retval < 0) {
			TP_LOGE("Failed to read command register\n");
		} else if (command & COMMAND_GET_REPORT) {
			TP_LOGE("Report type not supported by FW\n");
		} else {
			queue_delayed_work(f54->status_workqueue,
					&f54->status_work,
					0);
			mutex_unlock(&f54->status_mutex);
			return;
		}
		f54->report_type = INVALID_REPORT_TYPE;
		f54->report_size = 0;
	}
	mutex_unlock(&f54->status_mutex);

	return;
}

static enum hrtimer_restart get_report_timeout(struct hrtimer *timer)
{
	schedule_work(&(f54->timeout_work));

	return HRTIMER_NORESTART;
}
#endif

#ifdef RAW_HEX
static void print_raw_hex_report(void)
{
	unsigned int ii;

	TP_LOGI("Report data (raw hex)\n");

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
		for (ii = 0; ii < f54->report_size; ii += 2) {
			TP_LOGI("%03d: 0x%02x%02x\n",
					ii / 2,
					f54->report_data[ii + 1],
					f54->report_data[ii]);
		}
		break;
	default:
		for (ii = 0; ii < f54->report_size; ii++)
			TP_LOGI("%03d: 0x%02x\n", ii, f54->report_data[ii]);
		break;
	}

	return;
}
#endif

#ifdef HUMAN_READABLE
static void print_image_report(void)
{
	unsigned int ii;
	unsigned int jj;
	short *report_data;

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		TP_LOGI("Report data (image)\n");

		report_data = (short *)f54->report_data;

		for (ii = 0; ii < f54->tx_assigned; ii++) {
			for (jj = 0; jj < f54->rx_assigned; jj++) {
				if (*report_data < -64)
					TP_LOGI(".");
				else if (*report_data < 0)
					TP_LOGI("-");
				else if (*report_data > 64)
					TP_LOGI("*");
				else if (*report_data > 0)
					TP_LOGI("+");
				else
					pr_cont("0");

				report_data++;
			}
			TP_LOGI("");
		}
		TP_LOGI("End of report\n");
		break;
	default:
		TP_LOGI("Image not supported for report type %d\n",
				f54->report_type);
	}

	return;
}
#endif

static ssize_t synaptics_rmi4_f54_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->status);
}

static ssize_t synaptics_rmi4_f54_report_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->report_size);
}

static ssize_t synaptics_rmi4_f54_no_auto_cal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->no_auto_cal);
}

static ssize_t synaptics_rmi4_f54_no_auto_cal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting > 1)
		return -EINVAL;

	retval = f54->fn_ptr->read(rmi4_data,
			f54->control_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		TP_LOGE("Failed to read control register\n");
		return retval;
	}

	if ((data & NO_AUTO_CAL_MASK) == setting)
		return count;

	data = (data & ~NO_AUTO_CAL_MASK) | (data & NO_AUTO_CAL_MASK);

	retval = f54->fn_ptr->write(rmi4_data,
			f54->control_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		TP_LOGE("Failed to write control register\n");
		return retval;
	}

	f54->no_auto_cal = (setting == 1);

	return count;
}

static ssize_t synaptics_rmi4_f54_report_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->report_type);
}

static ssize_t synaptics_rmi4_f54_report_type_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (!is_report_type_valid((enum f54_report_types)setting)) {
		TP_LOGE("Report type not supported by driver\n");
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);

	if (f54->status != STATUS_BUSY) {
		f54->report_type = (enum f54_report_types)setting;
		data = (unsigned char)setting;
		retval = f54->fn_ptr->write(rmi4_data,
				f54->data_base_addr,
				&data,
				sizeof(data));
		mutex_unlock(&f54->status_mutex);
		if (retval < 0) {
			TP_LOGE("Failed to write data register\n");
			return retval;
		}
		return count;
	} else {
		TP_LOGE("Previous get report still ongoing\n");
		mutex_unlock(&f54->status_mutex);
		return -EINVAL;
	}
}

static ssize_t synaptics_rmi4_f54_fifoindex_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned char data[2];
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = f54->fn_ptr->read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			data,
			sizeof(data));
	if (retval < 0) {
		TP_LOGE("Failed to read data registers\n");
		return retval;
	}

	batohs(&f54->fifoindex, data);

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->fifoindex);
}
static ssize_t synaptics_rmi4_f54_fifoindex_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data[2];
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	f54->fifoindex = setting;

	hstoba(data, (unsigned short)setting);

	retval = f54->fn_ptr->write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			data,
			sizeof(data));
	if (retval < 0) {
		TP_LOGE("Failed to write data registers\n");
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f54_read_report_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int ii;
	unsigned int jj;
	int cnt;
	int count = 0;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	char *report_data_8;
	short *report_data_16;
	int *report_data_32;
	unsigned short *report_data_u16;
	unsigned int *report_data_u32;

	switch (f54->report_type) {
	case F54_8BIT_IMAGE:
		report_data_8 = (char *)f54->report_data;
		for (ii = 0; ii < f54->report_size; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%03d: %d\n",
					ii, *report_data_8);
			report_data_8++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_AMP_RAW_ADC:
		report_data_u16 = (unsigned short *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n",
				tx_num, rx_num);
		buf += cnt;
		count += cnt;

		for (ii = 0; ii < tx_num; ii++) {
			for (jj = 0; jj < (rx_num - 1); jj++) {
				cnt = snprintf(buf, PAGE_SIZE - count, "%-4d ",
						*report_data_u16);
				report_data_u16++;
				buf += cnt;
				count += cnt;
			}
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d\n",
					*report_data_u16);
			report_data_u16++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_NO_RX_COUPLING:
	case F54_SENSOR_SPEED:
	case F54_AMP_FULL_RAW_CAP:
	/* tddi f54 test reporting + */
	case F54_NOISE_TDDI:
	/* tddi f54 test reporting - */
		report_data_16 = (short *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n\n",
				tx_num, rx_num);
		buf += cnt;
		count += cnt;

		for (ii = 0; ii < tx_num; ii++) {
			for (jj = 0; jj < (rx_num - 1); jj++) {
				cnt = snprintf(buf, PAGE_SIZE - count, "%-5d ",
						*report_data_16);
				report_data_16++;
				buf += cnt;
				count += cnt;
			}
			cnt = snprintf(buf, PAGE_SIZE - count, "%-5d\n",
					*report_data_16);
			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		break;
	/* tddi f54 test reporting + */
	case F54_FULL_RAW_CAP_TDDI:
		report_data_u16 = (unsigned short *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n\n",
				tx_num, rx_num);
		buf += cnt;
		count += cnt;

		for (ii = 0; ii < tx_num; ii++) {
			for (jj = 0; jj < (rx_num - 1); jj++) {
				cnt = snprintf(buf, PAGE_SIZE - count, "%-5d ",
						*report_data_u16);
				report_data_u16++;
				buf += cnt;
				count += cnt;
			}
			cnt = snprintf(buf, PAGE_SIZE - count, "%-5d\n",
					*report_data_u16);
			report_data_u16++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_EE_SHORT_TDDI:
		report_data_u16 = (unsigned short *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n\n",
				tx_num, rx_num);
		buf += cnt;
		count += cnt;

		for (ii = 0; ii < tx_num; ii++) {
			for (jj = 0; jj < (rx_num - 1); jj++) {
				cnt = snprintf(buf, PAGE_SIZE - count, "%-4d ",
						*report_data_u16);
				report_data_u16++;
				buf += cnt;
				count += cnt;
			}
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d\n",
					*report_data_u16);
			report_data_u16++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			for (jj = 0; jj < (rx_num - 1); jj++) {
				cnt = snprintf(buf, PAGE_SIZE - count, "%-4d ",
						*report_data_u16);
				report_data_u16++;
				buf += cnt;
				count += cnt;
			}
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d\n",
					*report_data_u16);
			report_data_u16++;
			buf += cnt;
			count += cnt;
		}
		break;
	/* tddi f54 test reporting - */
	case F54_HIGH_RESISTANCE:
	case F54_FULL_RAW_CAP_MIN_MAX:
		report_data_16 = (short *)f54->report_data;
		for (ii = 0; ii < f54->report_size; ii += 2) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%03d: %d\n",
					ii / 2, *report_data_16);
			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_ABS_RAW_CAP:
		report_data_u32 = (unsigned int *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "rx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5u",
					*report_data_u32);
			report_data_u32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "tx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5u",
					*report_data_u32);
			report_data_u32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
		break;
	case F54_ABS_DELTA_CAP:
	case F54_ABS_HYBRID_DELTA_CAP:
	case F54_ABS_HYBRID_RAW_CAP:
		report_data_32 = (int *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "rx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5d",
					*report_data_32);
			report_data_32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "tx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5d",
					*report_data_32);
			report_data_32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
		break;
	default:
		for (ii = 0; ii < f54->report_size; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%03d: 0x%02x\n",
					ii, f54->report_data[ii]);
			buf += cnt;
			count += cnt;
		}
	}

	snprintf(buf, PAGE_SIZE - count, "\n");
	count++;

	return count;
}

static ssize_t synaptics_rmi4_f54_read_report_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char timeout = GET_REPORT_TIMEOUT_S * 10;
	unsigned char timeout_count;
	const char cmd[] = {'1', 0};
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_f54_report_type_store(dev, attr, buf, count);
	if (retval < 0)
		goto exit;

	retval = synaptics_rmi4_f54_do_preparation_store(dev, attr, cmd, 1);
	if (retval < 0)
		goto exit;

	retval = synaptics_rmi4_f54_get_report_store(dev, attr, cmd, 1);
	if (retval < 0)
		goto exit;

	timeout_count = 0;
	do {
		if (f54->status != STATUS_BUSY)
			break;
		msleep(100);
		timeout_count++;
	} while (timeout_count < timeout);

	if ((f54->status != STATUS_IDLE) || (f54->report_size == 0)) {
		TP_LOGE("Failed to read report\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	rmi4_data->reset_device(rmi4_data);

	return retval;
}

static ssize_t synaptics_rmi4_f54_do_preparation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	mutex_lock(&f54->status_mutex);

	if (f54->status != STATUS_IDLE) {
		if (f54->status != STATUS_BUSY) {
			TP_LOGE("Invalid status (%d)\n", f54->status);
		} else {
			TP_LOGE("Previous get report still ongoing\n");
		}
		mutex_unlock(&f54->status_mutex);
		return -EBUSY;
	}

	mutex_unlock(&f54->status_mutex);

	retval = do_preparation();
	if (retval < 0) {
		TP_LOGE("Failed to do preparation\n");
		return retval;
	}

	return count;
}

/* syna 20161122 + */
#ifdef F54_POLLING_GET_REPORT
static ssize_t test_sysfs_get_report_polling(void)
{
	int retval = 0;
	unsigned char report_index[2];
	unsigned char value;
	unsigned char timeout_count;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			TP_LOGE("Failed to read command register\n");
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < 20);

	if (timeout_count == 20) {
		TP_LOGE("Timed out waiting for command completion\n");
		retval = -ETIMEDOUT;
		return retval;
	}

	set_report_size();
	if (f54->report_size == 0) {
		TP_LOGE("%s: Report data size = 0\n", __func__);
		retval = -EIO;
		f54->status = STATUS_ERROR;
		return retval;
	}

	if (f54->data_buffer_size < f54->report_size) {
		if (f54->data_buffer_size)
			kfree(f54->report_data);
		f54->report_data = kzalloc(f54->report_size, GFP_KERNEL);
		if (!f54->report_data) {
			TP_LOGE("%s: Failed to alloc mem for data buffer\n", __func__);
			f54->data_buffer_size = 0;
			retval = -EIO;
			f54->status = STATUS_ERROR;
			return retval;
		}
		f54->data_buffer_size = f54->report_size;
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		TP_LOGE("%s: Failed to write report data index\n", __func__);
		retval = -EIO;
		f54->status = STATUS_ERROR;
		return retval;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);

	if (retval < 0) {
		TP_LOGE("%s: Failed to read report data\n", __func__);
		retval = -EIO;
		f54->status = STATUS_ERROR;
		return retval;
	}

	f54->status = STATUS_IDLE;
	return retval;
}
#endif
/* syna 20161122 - */

static ssize_t synaptics_rmi4_f54_get_report_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char command;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	command = (unsigned char)COMMAND_GET_REPORT;

	if (!is_report_type_valid(f54->report_type)) {
		TP_LOGE("Invalid report type\n");
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);

	if (f54->status != STATUS_IDLE) {
		if (f54->status != STATUS_BUSY) {
			TP_LOGE("Invalid status (%d)\n", f54->status);
		} else {
			TP_LOGE("Previous get report still ongoing\n");
		}
		mutex_unlock(&f54->status_mutex);
		return -EBUSY;
	}

	set_interrupt(true);

	f54->status = STATUS_BUSY;

	retval = f54->fn_ptr->write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	mutex_unlock(&f54->status_mutex);
	if (retval < 0) {
		TP_LOGE("Failed to write get report command\n");
		return retval;
	}

/* syna 20161122 + */
#ifdef F54_POLLING_GET_REPORT

	retval = test_sysfs_get_report_polling();
	if (retval < 0) {
		TP_LOGE("Failed to get report image by polling\n");
		return retval;
		//goto exit;
	}

#else

#ifdef WATCHDOG_HRTIMER
	hrtimer_start(&f54->watchdog,
			ktime_set(WATCHDOG_TIMEOUT_S, 0),
			HRTIMER_MODE_REL);
#endif

#endif
/* syna 20161122 - */
	return count;
}


static ssize_t synaptics_rmi4_f54_do_afe_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (!f54->query_49.has_ctrl188) {
		TP_LOGE("F54_ANALOG_Ctrl188 not found\n");
		return -EINVAL;
	}

	if (setting == 0 || setting == 1)
		retval = test_do_afe_calibration((enum f54_afe_cal)setting);
	else
		return -EINVAL;

	if (retval)
		return retval;
	else
		return count;
}

static ssize_t synaptics_rmi4_f54_force_cal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char command;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	command = (unsigned char)COMMAND_FORCE_CAL;

	if (f54->status == STATUS_BUSY)
		return -EBUSY;

	retval = f54->fn_ptr->write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		TP_LOGE("Failed to write force cal command\n");
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f54_resume_touch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char device_ctrl;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_LOGE("Failed to restore no sleep setting\n");
		return retval;
	}

	device_ctrl = device_ctrl & ~NO_SLEEP_ON;
	device_ctrl |= rmi4_data->no_sleep_setting;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_LOGE("Failed to restore no sleep setting\n");
		return retval;
	}

	if ((f54->query.has_query13) &&
			(f54->query_13.has_ctrl86)) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_86->address,
				f54->control.reg_86->data,
				sizeof(f54->control.reg_86->data));
		if (retval < 0) {
			TP_LOGE("Failed to restore sense display ratio\n");
			return retval;
		}
	}

	set_interrupt(false);

	if (f54->skip_preparation)
		return count;

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
	case F54_ABS_HYBRID_DELTA_CAP:
	case F54_ABS_HYBRID_RAW_CAP:
		break;
	case F54_AMP_RAW_ADC:
		if (f54->query_49.has_ctrl188) {
			retval = synaptics_rmi4_reg_read(rmi4_data,
					f54->control.reg_188->address,
					f54->control.reg_188->data,
					sizeof(f54->control.reg_188->data));
			if (retval < 0) {
				TP_LOGE("Failed to read start production test\n");
				return retval;
			}
			f54->control.reg_188->start_production_test = 0;
			retval = synaptics_rmi4_reg_write(rmi4_data,
					f54->control.reg_188->address,
					f54->control.reg_188->data,
					sizeof(f54->control.reg_188->data));
			if (retval < 0) {
				TP_LOGE("Failed to set start production test\n");
				return retval;
			}
		}
		break;
	default:
		rmi4_data->reset_device(rmi4_data);
	}

	return count;
}

static ssize_t synaptics_rmi4_f54_num_of_mapped_rx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->rx_assigned);
}

static ssize_t synaptics_rmi4_f54_num_of_mapped_tx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->tx_assigned);
}

simple_show_func_unsigned(query, num_of_rx_electrodes)
simple_show_func_unsigned(query, num_of_tx_electrodes)
simple_show_func_unsigned(query, has_image16)
simple_show_func_unsigned(query, has_image8)
simple_show_func_unsigned(query, has_baseline)
simple_show_func_unsigned(query, clock_rate)
simple_show_func_unsigned(query, touch_controller_family)
simple_show_func_unsigned(query, has_pixel_touch_threshold_adjustment)
simple_show_func_unsigned(query, has_sensor_assignment)
simple_show_func_unsigned(query, has_interference_metric)
simple_show_func_unsigned(query, has_sense_frequency_control)
simple_show_func_unsigned(query, has_firmware_noise_mitigation)
simple_show_func_unsigned(query, has_two_byte_report_rate)
simple_show_func_unsigned(query, has_one_byte_report_rate)
simple_show_func_unsigned(query, has_relaxation_control)
simple_show_func_unsigned(query, curve_compensation_mode)
simple_show_func_unsigned(query, has_iir_filter)
simple_show_func_unsigned(query, has_cmn_removal)
simple_show_func_unsigned(query, has_cmn_maximum)
simple_show_func_unsigned(query, has_touch_hysteresis)
simple_show_func_unsigned(query, has_edge_compensation)
simple_show_func_unsigned(query, has_per_frequency_noise_control)
simple_show_func_unsigned(query, has_signal_clarity)
simple_show_func_unsigned(query, number_of_sensing_frequencies)

show_store_func_unsigned(control, reg_0, no_relax)
show_store_func_unsigned(control, reg_0, no_scan)
show_store_func_unsigned(control, reg_1, bursts_per_cluster)
show_store_func_unsigned(control, reg_2, saturation_cap)
show_store_func_unsigned(control, reg_3, pixel_touch_threshold)
show_store_func_unsigned(control, reg_4__6, rx_feedback_cap)
show_store_func_unsigned(control, reg_4__6, low_ref_cap)
show_store_func_unsigned(control, reg_4__6, low_ref_feedback_cap)
show_store_func_unsigned(control, reg_4__6, low_ref_polarity)
show_store_func_unsigned(control, reg_4__6, high_ref_cap)
show_store_func_unsigned(control, reg_4__6, high_ref_feedback_cap)
show_store_func_unsigned(control, reg_4__6, high_ref_polarity)
show_store_func_unsigned(control, reg_7, cbc_cap)
show_store_func_unsigned(control, reg_7, cbc_polarity)
show_store_func_unsigned(control, reg_7, cbc_tx_carrier_selection)
show_store_func_unsigned(control, reg_8__9, integration_duration)
show_store_func_unsigned(control, reg_8__9, reset_duration)
show_store_func_unsigned(control, reg_10, noise_sensing_bursts_per_image)
show_store_func_unsigned(control, reg_12__13, slow_relaxation_rate)
show_store_func_unsigned(control, reg_12__13, fast_relaxation_rate)
show_store_func_unsigned(control, reg_14, rxs_on_xaxis)
show_store_func_unsigned(control, reg_14, curve_comp_on_txs)
show_store_func_unsigned(control, reg_20, disable_noise_mitigation)
show_store_func_unsigned(control, reg_21, freq_shift_noise_threshold)
show_store_func_unsigned(control, reg_22__26, medium_noise_threshold)
show_store_func_unsigned(control, reg_22__26, high_noise_threshold)
show_store_func_unsigned(control, reg_22__26, noise_density)
show_store_func_unsigned(control, reg_22__26, frame_count)
show_store_func_unsigned(control, reg_27, iir_filter_coef)
show_store_func_unsigned(control, reg_28, quiet_threshold)
show_store_func_unsigned(control, reg_29, cmn_filter_disable)
show_store_func_unsigned(control, reg_30, cmn_filter_max)
show_store_func_unsigned(control, reg_31, touch_hysteresis)
show_store_func_unsigned(control, reg_32__35, rx_low_edge_comp)
show_store_func_unsigned(control, reg_32__35, rx_high_edge_comp)
show_store_func_unsigned(control, reg_32__35, tx_low_edge_comp)
show_store_func_unsigned(control, reg_32__35, tx_high_edge_comp)
show_store_func_unsigned(control, reg_41, no_signal_clarity)
show_store_func_unsigned(control, reg_57, cbc_cap_0d)
show_store_func_unsigned(control, reg_57, cbc_polarity_0d)
show_store_func_unsigned(control, reg_57, cbc_tx_carrier_selection_0d)

show_replicated_func_unsigned(control, reg_15, sensor_rx_assignment)
show_replicated_func_unsigned(control, reg_16, sensor_tx_assignment)
show_replicated_func_unsigned(control, reg_17, disable)
show_replicated_func_unsigned(control, reg_17, filter_bandwidth)
show_replicated_func_unsigned(control, reg_19, stretch_duration)
show_replicated_func_unsigned(control, reg_38, noise_control_1)
show_replicated_func_unsigned(control, reg_39, noise_control_2)
show_replicated_func_unsigned(control, reg_40, noise_control_3)

show_store_replicated_func_unsigned(control, reg_36, axis1_comp)
show_store_replicated_func_unsigned(control, reg_37, axis2_comp)

/* tddi f54 test reporting + */
static int test_do_command(unsigned char command)
{
	int retval;
	unsigned char value;
	unsigned char timeout_count;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		TP_LOGE("Failed to write command\n");
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			TP_LOGE("Failed to read command register\n");
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < 20);

	if (timeout_count == 20) {
		TP_LOGE("Timed out waiting for command completion\n");
		retval = -ETIMEDOUT;
		return retval;
	}

	return 0;
}

static ssize_t test_sysfs_read_report(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count,
		bool do_preparation, bool do_reset)
{
	int retval = count;
	unsigned char timeout = GET_REPORT_TIMEOUT_S * 10;
	unsigned char timeout_count;
	const char cmd[] = {'1', 0};
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_f54_report_type_store(dev, attr, buf, count);
	if (retval < 0)
		goto exit;

	retval = synaptics_rmi4_f54_get_report_store(dev, attr, cmd, 1);
	if (retval < 0)
		goto exit;

	timeout_count = 0;
	do {
		if (f54->status != STATUS_BUSY)
			break;
		msleep(100);
		timeout_count++;
	} while (timeout_count < timeout);

	if ((f54->status != STATUS_IDLE) || (f54->report_size == 0)) {

		TP_LOGE("Failed to read report\n");
		retval = -EINVAL;
		goto exit;
	}

exit:
	if (do_reset)
		rmi4_data->reset_device(rmi4_data);

	return retval;
}

static short find_median(short* pdata, int num)
{
	int i,j;
	short temp;
	short *value;
	short median;

	value = (short *)kzalloc( num * sizeof(short), GFP_KERNEL);

	for(i=0; i < num; i++)
		*(value+i) = *(pdata+i);

	//sorting
	for ( i=1; i <= num-1; i++)
	{
		for ( j=1; j <= num-i; j++)
		{
			if (*(value+j-1) <= *(value+j))
			{
			   temp = *(value+j-1);
			   *(value+j-1)= *(value+j);
			   *(value+j) = temp;
			}
			else
				continue ;
		}
	}

	//calculation of median
	if ( num % 2 == 0)
		median = ( *(value+(num/2 -1)) + *(value+(num/2)) )/2;
	else
		median = *(value+(num/2));

	if(value)
		kfree(value);

	return median;
}

static int tddi_ee_short_normalize_data(signed short * image)
{
	int retval = 0;
	int i, j;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	unsigned char left_size = f54->left_mux_size;
	unsigned char right_size = f54->right_mux_size;
	signed short *report_data_16;
	signed short *left_median = NULL;
	signed short *right_median = NULL;
	signed short *left_column_buf = NULL;
	signed short *right_column_buf = NULL;
	signed int temp;

	if (!image) {
		TP_LOGE("Fail. image is null\n");
		retval = -EINVAL;
		goto exit;
	}

	// allocate the buffer for the median calculation in left/right half
	right_median = (unsigned short *) kzalloc(rx_num * sizeof(short), GFP_KERNEL);
	if (!right_median) {
		TP_LOGE("Failed to alloc mem for right_median\n");
		retval = -ENOMEM;
		goto exit;
	}

	left_median = (unsigned short *) kzalloc(rx_num * sizeof(short), GFP_KERNEL);
	if (!left_median) {
		TP_LOGE("Failed to alloc mem for left_median\n");
		retval = -ENOMEM;
		goto exit;
	}

	right_column_buf = (unsigned short *) kzalloc(right_size * rx_num * sizeof(short), GFP_KERNEL);
	if (!right_column_buf ) {
		TP_LOGE("Failed to alloc mem for left_column_buf\n");
		retval = -ENOMEM;
		goto exit;
	}

	left_column_buf = (unsigned short *) kzalloc(left_size * rx_num * sizeof(short), GFP_KERNEL);
	if (!left_column_buf ) {
		TP_LOGE("Failed to alloc mem for left_column_buf\n");
		retval = -ENOMEM;
		goto exit;
	}

	// divide the input image into left/right parts
	report_data_16 = image;
	for (i = 0; i < rx_num; i++)
	{
		for (j = 0; j < right_size; j++) {
			right_column_buf[i * right_size + j] =
					report_data_16[j * rx_num + i];
		}
	}

	report_data_16 = image + right_size * rx_num;
	for (i = 0; i < rx_num; i++) {
		for (j = 0; j < left_size; j++) {
			left_column_buf[i * left_size + j] =
					report_data_16[j * rx_num + i];
		}
	}

	// find the median in every column
	for (i = 0; i < rx_num; i++) {
		right_median[i] = find_median(right_column_buf + i * right_size, right_size);
		left_median[i] = find_median(left_column_buf + i * left_size, left_size);
	}

	// walk through the image of all data
	// and calculate the ratio based on the median
	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {

			if (i < right_size) {
				temp = (unsigned int) image[i * rx_num + j];
				temp = temp * 100 / right_median[j];
			} else {
				temp = (unsigned int) image[i * rx_num + j];
				temp = temp * 100 / left_median[j];
			}

			image[i * rx_num + j] = temp;
		}
	}

exit:
	kfree(right_median);
	kfree(left_median);
	kfree(right_column_buf);
	kfree(left_column_buf);
	return retval;
}

static ssize_t synaptics_rmi4_f54_tddi_ee_short_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	int i, j = 0, offset;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	signed short *tddi_rt95_part_one = NULL;
	signed short *tddi_rt95_part_two = NULL;
	unsigned int buffer_size = tx_num * rx_num * 2;
	unsigned long setting;

	signed short min = 0;
	signed short max = 0;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	//retval = count;

	if (setting != 1)
		return -EINVAL;

	// allocate the g_tddi_ee_short_data_output
	if (g_tddi_ee_short_data_output)
		kfree(g_tddi_ee_short_data_output);

	g_tddi_ee_short_data_output = kzalloc(tx_num * rx_num, GFP_KERNEL);
	if (!g_tddi_ee_short_data_output) {
		TP_LOGE("Failed to alloc mem for g_tddi_ee_short_data_output\n");
		return -ENOMEM;
	}

	// 20160920 +
	if (g_tddi_ee_short_data_log_1)
		kfree(g_tddi_ee_short_data_log_1);
	g_tddi_ee_short_data_log_1 = kzalloc(tx_num * rx_num * 2, GFP_KERNEL);
	if (!g_tddi_ee_short_data_log_1) {
		TP_LOGE("Failed to alloc mem for g_tddi_ee_short_data_log_1\n");
		return -ENOMEM;
	}

	if (g_tddi_ee_short_data_log_2)
		kfree(g_tddi_ee_short_data_log_2);
	g_tddi_ee_short_data_log_2 = kzalloc(tx_num * rx_num * 2, GFP_KERNEL);  // 20160920
	if (!g_tddi_ee_short_data_log_2) {
		TP_LOGE("Failed to alloc mem for g_tddi_ee_short_data_log_2\n");
		return -ENOMEM;
	}
	// 20160920 -

	// allocate the buffer
	tddi_rt95_part_one = kzalloc(buffer_size, GFP_KERNEL);
	if (!tddi_rt95_part_one) {
		TP_LOGE("Failed to alloc mem for tddi_rt95_part_one\n");
		retval = -ENOMEM;
		goto exit;
	}

	tddi_rt95_part_two = kzalloc(buffer_size, GFP_KERNEL);
	if (!tddi_rt95_part_two) {
		TP_LOGE("Failed to alloc mem for td43xx_rt95_part_two\n");
		retval = -ENOMEM;
		goto exit;
	}
	// get report image 95
	retval = test_sysfs_read_report(dev, attr, "95", count,
				false, false);
	if (retval < 0) {
		TP_LOGE("Failed to read report 95. exit\n");
		retval = -EIO;
		goto exit;
	}
	// grep the upper half as part 1 image
	// the criteria is lower than TEST_LIMIT_PART1 ( fail, if > TEST_LIMIT_PART1 )
	min = max = (signed short)(f54->report_data[0]) | (signed short)(f54->report_data[1] << 8);

	for (i = 0, offset = 0; i < tx_num * rx_num; i++) {
		tddi_rt95_part_one[i] = (signed short)(f54->report_data[offset]) |
				((signed short)(f54->report_data[offset + 1]) << 8);

		min = (min < tddi_rt95_part_one[i])? min : tddi_rt95_part_one[i];
		max = (max > tddi_rt95_part_one[i])? max : tddi_rt95_part_one[i];

		g_tddi_ee_short_data_log_1[i] = (short)tddi_rt95_part_one[i];   // 20160920

		if (tddi_rt95_part_one[i] > EE_SHORT_TEST_LIMIT_PART1) {
			TP_LOGE("fail at (tx%-2d, rx%-2d) in part 1 image, data = %-4d\n",
					i, j, tddi_rt95_part_one[i]);

			tddi_rt95_part_one[i] = 1; // 1: fail
		}
		else {
			tddi_rt95_part_one[i] = 0;
		}

		offset += 2;
	}

	TP_LOGI("image part 1 data range (max, min) = (%-4d, %-4d)\n",
			max, min);

	// grep the lower half as part 2 image
	// the criteria is over than TEST_LIMIT_PART2 ( fail, if < TEST_LIMIT_PART2 )
	for (i = 0, offset = buffer_size; i < tx_num * rx_num; i++) {
		tddi_rt95_part_two[i] = (signed short)(f54->report_data[offset]) |
								((signed short)(f54->report_data[offset + 1]) << 8);
		offset += 2;
	}

	tddi_ee_short_normalize_data(tddi_rt95_part_two);

	min = max = (signed short)tddi_rt95_part_two[0];

	for (i = 0; i < tx_num * rx_num; i++) {

		min = (min < tddi_rt95_part_two[i])? min : tddi_rt95_part_two[i];
		max = (max > tddi_rt95_part_two[i])? max : tddi_rt95_part_two[i];

		g_tddi_ee_short_data_log_2[i] = (short)tddi_rt95_part_two[i];   // 20160920

		if (tddi_rt95_part_two[i] < EE_SHORT_TEST_LIMIT_PART2) {
			TP_LOGE("fail at (tx%-2d, rx%-2d) in part 2 image, data = %-4d\n",
					i, j, tddi_rt95_part_two[i]);

			tddi_rt95_part_two[i] = 1; // 1: fail
		}
		else {
			tddi_rt95_part_two[i] = 0;
		}
	}

	TP_LOGI("image part 2 data range (max, min) = (%-4d, %-4d)\n",
			max, min);

	// filling out the g_tddi_ee_short_data_output
	// 1: fail / 0 : pass
	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			g_tddi_ee_short_data_output[i * rx_num + j] =
				(unsigned char)(tddi_rt95_part_one[i * rx_num + j]) || tddi_rt95_part_two[i * rx_num + j];
		}
	}

	retval = count;

exit:
	kfree(tddi_rt95_part_one);
	kfree(tddi_rt95_part_two);

//	rmi4_data->reset_device(rmi4_data);

	return retval;
}

static ssize_t synaptics_rmi4_f54_tddi_ee_short_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, j;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	bool result = 1;

	// 20160920 +
	int cnt;
	int count = 0;

	short min = 0, max = 0;
	// 20160920 -

	if (!g_tddi_ee_short_data_output)
		return -EINVAL;

	// 20160920 +
	if (!g_tddi_ee_short_data_log_1 || !g_tddi_ee_short_data_log_2)
		return -EINVAL;

 	cnt = snprintf(buf, PAGE_SIZE - count, "phase 1 limit %-3d (< failure)\nphase 2 limit %-3d (> failure)\n",
 					EE_SHORT_TEST_LIMIT_PART1, EE_SHORT_TEST_LIMIT_PART2);
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "\nphase 1 data \n");
	buf += cnt;
	count += cnt;

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
 			cnt = snprintf(buf, PAGE_SIZE - count, "%-3d ", g_tddi_ee_short_data_log_1[i*rx_num + j]);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}

	min = max = g_tddi_ee_short_data_log_2[0];

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			min = (min < g_tddi_ee_short_data_log_2[i*rx_num + j])? min : g_tddi_ee_short_data_log_2[i*rx_num + j];
			max = (max > g_tddi_ee_short_data_log_2[i*rx_num + j])? max : g_tddi_ee_short_data_log_2[i*rx_num + j];
		}
	}
	cnt = snprintf(buf, PAGE_SIZE - count, "\nphase 2 range (max, min) = (%-4d, %-4d)\n", max, min);
	buf += cnt;
	count += cnt;
	// 20160920 -

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			if (g_tddi_ee_short_data_output[i * rx_num + j] != 0) {
				result &= 0; // 0: fail
				// 20160920 +
				cnt = snprintf(buf, PAGE_SIZE - count, "fail on (tx%-2d, rx%-2d)\n", i, j);
				buf += cnt;
				count += cnt;
				// 20160920 -
			}
		}
	}

	// 20160920 +
	kfree(g_tddi_ee_short_data_log_1);
	g_tddi_ee_short_data_log_1 = NULL;

	kfree(g_tddi_ee_short_data_log_2);
	g_tddi_ee_short_data_log_2 = NULL;
	// 20160920 -

	kfree(g_tddi_ee_short_data_output);
	g_tddi_ee_short_data_output = NULL;

	// 20160920 +
	cnt = snprintf(buf, PAGE_SIZE - count, "\n%s\n", (result == 1) ? "PASS" : "FAIL");
	buf += cnt;
	count += cnt;

	return count;
	//return snprintf(buf, PAGE_SIZE, "%s\n", (result == 1) ? "PASS" : "FAIL");
	// 20160920 -
}

static ssize_t synaptics_rmi4_f54_tddi_full_raw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	unsigned int full_raw_report_size = tx_num * rx_num * 2;
	unsigned long setting;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	//retval = count;

	if (setting != 1)
		return -EINVAL;

	// allocate the g_tddi_full_raw_data_output
	if (g_tddi_full_raw_data_output)
		kfree(g_tddi_full_raw_data_output);

	g_tddi_full_raw_data_output = kzalloc(full_raw_report_size, GFP_KERNEL);
	if (!g_tddi_full_raw_data_output) {
		TP_LOGE("Failed to alloc mem for g_tddi_full_raw_data_output\n");

		return -ENOMEM;
	}

	// get the report image 92
	retval = test_sysfs_read_report(dev, attr, "92", count,
				false, false);
	if (retval < 0) {
		TP_LOGE("Failed to read report 92. exit\n");

		return -EIO;
	}

	secure_memcpy(g_tddi_full_raw_data_output, full_raw_report_size,
		f54->report_data, f54->report_size, f54->report_size);

	retval = count;

//	rmi4_data->reset_device(rmi4_data);

	return retval;
}

static ssize_t synaptics_rmi4_f54_tddi_full_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int i;
	unsigned int j;
	int cnt;
	int count = 0;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	unsigned short *report_data_16;

	unsigned short min = 0, max = 0;

	if (!g_tddi_full_raw_data_output)
		return -EINVAL;

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n",
			tx_num, rx_num);
	buf += cnt;
	count += cnt;

	report_data_16 = (unsigned short *)g_tddi_full_raw_data_output;

	min = max = *report_data_16;

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
 			cnt = snprintf(buf, PAGE_SIZE - count, "%-5d ", *report_data_16);

			min = (min < *report_data_16)? min : *report_data_16;
			max = (max > *report_data_16)? max : *report_data_16;

			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}

	TP_LOGI("data range (max, min) = (%-4d, %-4d)\n", max, min);

	kfree(g_tddi_full_raw_data_output);
	g_tddi_full_raw_data_output = NULL;

	snprintf(buf, PAGE_SIZE - count, "\n");
	count++;

	return count;
}


static int tddi_amp_open_data_testing(signed short * p_image)
{
	int retval = 0;
	int i, j;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	unsigned char left_size = f54->left_mux_size;
	unsigned char right_size = f54->right_mux_size;
	signed short *p_data_16;
	signed short *p_left_median = NULL;
	signed short *p_right_median = NULL;
	signed short *p_left_column_buf = NULL;
	signed short *p_right_column_buf = NULL;
	signed int temp;

	unsigned short min_ph1 = 0, max_ph1 = 0;
	signed int min_ph2 = 0, max_ph2 = 0;

	if (!p_image) {
		TP_LOGE("Fail. p_image is null\n");
		retval = -EINVAL;
		goto exit;
	}

	// allocate the buffer for the median calculation in left/right half
	p_right_median = (signed short *) kzalloc(rx_num * sizeof(short), GFP_KERNEL);
	if (!p_right_median) {
		TP_LOGE("Failed to alloc mem for right_median\n");
		retval = -ENOMEM;
		goto exit;
	}

	p_left_median = (signed short *) kzalloc(rx_num * sizeof(short), GFP_KERNEL);
	if (!p_left_median) {
		TP_LOGE("Failed to alloc mem for left_median\n");
		retval = -ENOMEM;
		goto exit;
	}

	p_right_column_buf = (signed short *) kzalloc(right_size * rx_num * sizeof(short), GFP_KERNEL);
	if (!p_right_column_buf ) {
		TP_LOGE("Failed to alloc mem for left_column_buf\n");
		retval = -ENOMEM;
		goto exit;
	}

	p_left_column_buf = (signed short *) kzalloc(left_size * rx_num * sizeof(short), GFP_KERNEL);
	if (!p_left_column_buf ) {
		TP_LOGE("Failed to alloc mem for left_column_buf\n");
		retval = -ENOMEM;
		goto exit;
	}

	if (f54->swap_sensor_side) {

		// first row is left side data
		p_data_16 = p_image;
		for (i = 0; i < rx_num; i++) {
			for (j = 0; j < left_size; j++) {
				p_left_column_buf[i * left_size + j] = p_data_16[j * rx_num + i];
			}
		}
		// right side data
		p_data_16 = p_image + left_size * rx_num;
		for (i = 0; i < rx_num; i++) {
			for (j = 0; j < right_size; j++) {
				p_right_column_buf[i * right_size + j] = p_data_16[j * rx_num + i];
			}
		}

		// calculate the median value
		for (i = 0; i < rx_num; i++) {
			p_left_median[i] = find_median(p_left_column_buf + i * left_size, left_size);
			p_right_median[i] = find_median(p_right_column_buf + i * right_size, right_size);
		}

		// data testing
		min_ph1 = max_ph1 = p_image[0];
		min_ph2 = max_ph2 = (p_image[0] * 100)/p_left_median[0];

		for (i = 0; i < tx_num; i++) {
			for (j = 0; j < rx_num; j++) {
				if (i < left_size) {
					temp = (signed int) p_image[i * rx_num + j];
					temp = temp * 100 / p_left_median[j];
				} else {
					temp = (signed int) p_image[i * rx_num + j];
					temp = temp * 100 / p_right_median[j];
				}

				// phase 1 comparison
				// the delta value should be within the ph1 lower and upper limit
				if ((p_image[i * rx_num + j] < AMP_OPEN_TEST_LIMIT_PHASE1_LOWER) ||
					(p_image[i * rx_num + j] > AMP_OPEN_TEST_LIMIT_PHASE1_UPPER)) {

					g_tddi_amp_open_data_output[i * rx_num + j] =  1; // fail

					TP_LOGE("phase 1 failed at (tx%-2d, rx%-2d), data = %d\n",
							i, j, p_image[i * rx_num + j]);
				}
				// phase 2 comparison
				// the ratio should be within the ph2 lower and upper limit
				else if ((temp < AMP_OPEN_TEST_LIMIT_PHASE2_LOWER) ||
					(temp > AMP_OPEN_TEST_LIMIT_PHASE2_UPPER)) {

					g_tddi_amp_open_data_output[i * rx_num + j] =  1; // fail

					TP_LOGE("phase 2 failed at (tx%-2d, rx%-2d), data = %d\n",
							i, j, temp);
				}
				else {
					g_tddi_amp_open_data_output[i * rx_num + j] =  0; // pass
				}

				min_ph1 = (min_ph1 < p_image[i * rx_num + j])? min_ph1 : p_image[i * rx_num + j];
				max_ph1 = (max_ph1 > p_image[i * rx_num + j])? max_ph1 : p_image[i * rx_num + j];

				min_ph2 = (min_ph2 < temp)? min_ph2 : temp;
				max_ph2 = (max_ph2 > temp)? max_ph2 : temp;

				// 20160920 +
				g_tddi_amp_open_data_log_1[i * rx_num + j] = p_image[i * rx_num + j];
				g_tddi_amp_open_data_log_2[i * rx_num + j] = (short)temp;
				// 20160920	-
			}
		}

	}
	else {

		// first row is right side data
		p_data_16 = p_image;
		for (i = 0; i < rx_num; i++) {
			for (j = 0; j < right_size; j++) {
				p_right_column_buf[i * right_size + j] = p_data_16[j * rx_num + i];
			}
		}
		// left side data
		p_data_16 = p_image + right_size * rx_num;
		for (i = 0; i < rx_num; i++) {
			for (j = 0; j < left_size; j++) {
				p_left_column_buf[i * left_size + j] = p_data_16[j * rx_num + i];
			}
		}

		// calculate the median value
		for (i = 0; i < rx_num; i++) {
			p_right_median[i] = find_median(p_right_column_buf + i * right_size, right_size);
			p_left_median[i] = find_median(p_left_column_buf + i * left_size, left_size);
		}


		// data testing
		min_ph1 = max_ph1 = p_image[0];
		min_ph2 = max_ph2 = (p_image[0] * 100)/p_right_median[0];

		for (i = 0; i < tx_num; i++) {
			for (j = 0; j < rx_num; j++) {
				if (i < right_size) {
					temp = (signed int) p_image[i * rx_num + j];
					temp = temp * 100 / p_right_median[j];
				} else {
					temp = (signed int) p_image[i * rx_num + j];
					temp = temp * 100 / p_left_median[j];
				}

				// phase 1 comparison
				// the delta value should be within the ph1 lower and upper limit
				if ((p_image[i * rx_num + j] < AMP_OPEN_TEST_LIMIT_PHASE1_LOWER) ||
					(p_image[i * rx_num + j] > AMP_OPEN_TEST_LIMIT_PHASE1_UPPER)) {

					g_tddi_amp_open_data_output[i * rx_num + j] =  1; // fail

					TP_LOGE("phase 1 failed at (tx%-2d, rx%-2d), data = %d\n",
							i, j, p_image[i * rx_num + j]);
				}
				// phase 2 comparison
				// the ratio should be within the ph2 lower and upper limit
				else if ((temp < AMP_OPEN_TEST_LIMIT_PHASE2_LOWER) ||
					(temp > AMP_OPEN_TEST_LIMIT_PHASE2_UPPER)) {

					g_tddi_amp_open_data_output[i * rx_num + j] =  1; // fail

					TP_LOGE("phase 2 failed at (tx%-2d, rx%-2d), data = %d\n",
							i, j, temp);
				}
				else {
					g_tddi_amp_open_data_output[i * rx_num + j] =  0; // pass
				}

				min_ph1 = (min_ph1 < p_image[i * rx_num + j])? min_ph1 : p_image[i * rx_num + j];
				max_ph1 = (max_ph1 > p_image[i * rx_num + j])? max_ph1 : p_image[i * rx_num + j];

				min_ph2 = (min_ph2 < temp)? min_ph2 : temp;
				max_ph2 = (max_ph2 > temp)? max_ph2 : temp;

				// 20160920 +
				g_tddi_amp_open_data_log_1[i * rx_num + j] = p_image[i * rx_num + j];
				g_tddi_amp_open_data_log_2[i * rx_num + j] = (short)temp;
				// 20160920	-
			}
		}

	}

	TP_LOGI("phase 1 data range (max, min) = (%-4d, %-4d)\n",
			max_ph1, min_ph1);

	TP_LOGI("phase 2 data range (max, min) = (%-4d, %-4d)\n",
			max_ph2, min_ph2);

exit:
	kfree(p_right_median);
	kfree(p_left_median);
	kfree(p_right_column_buf);
	kfree(p_left_column_buf);

	return retval;
}

static ssize_t synaptics_rmi4_f54_tddi_amp_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	int i, j, k;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;
	unsigned long setting;
	unsigned char original_data_f54_ctrl99[3] = {0x00, 0x00, 0x00};
	struct f54_control control = f54->control;
	unsigned char *p_report_data_8 = NULL;
	signed short  *p_rt92_delta_image = NULL;
	signed short  *p_rt92_image_1 = NULL;
	signed short  *p_rt92_image_2 = NULL;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	//retval = count;

	if (setting != 1)
		return -EINVAL;

	// allocate the g_tddi_amp_open_data_output
	if (g_tddi_amp_open_data_output)
		kfree(g_tddi_amp_open_data_output);
	g_tddi_amp_open_data_output = kzalloc(tx_num * rx_num, GFP_KERNEL);
	if (!g_tddi_amp_open_data_output) {
		TP_LOGE("Failed to alloc mem for g_tddi_amp_open_data_output\n");
		return -ENOMEM;
	}

	// 20160920 +
	if (g_tddi_amp_open_data_log_1)
		kfree(g_tddi_amp_open_data_log_1);
	g_tddi_amp_open_data_log_1 = kzalloc(tx_num * rx_num * 2, GFP_KERNEL);
	if (!g_tddi_amp_open_data_log_1) {
		TP_LOGE("Failed to alloc mem for g_tddi_amp_open_data_log_1\n");
		return -ENOMEM;
	}
	if (g_tddi_amp_open_data_log_2)
		kfree(g_tddi_amp_open_data_log_2);
	g_tddi_amp_open_data_log_2 = kzalloc(tx_num * rx_num * 2, GFP_KERNEL);
	if (!g_tddi_amp_open_data_log_2) {
		TP_LOGE("Failed to alloc mem for g_tddi_amp_open_data_log_2\n");
		return -ENOMEM;
	}
	// 20160920 -

	// allocate the buffer
	p_report_data_8 = kzalloc(tx_num * rx_num * 2, GFP_KERNEL);
	if (!p_report_data_8) {
		TP_LOGE("Failed to alloc mem for p_report_data_8\n");

		retval = -ENOMEM;
		goto exit;
	}

	p_rt92_delta_image = kzalloc(tx_num * rx_num * sizeof(signed short), GFP_KERNEL);
	if (!p_rt92_delta_image) {
		TP_LOGE("Failed to alloc mem for p_rt92_delta_image\n");

		retval = -ENOMEM;
		goto exit;
	}

	p_rt92_image_1 = kzalloc(tx_num * rx_num * sizeof(signed short), GFP_KERNEL);
	if (!p_rt92_image_1) {
		TP_LOGE("Failed to alloc mem for p_rt92_image_1\n");

		retval = -ENOMEM;
		goto exit;
	}

	p_rt92_image_2 = kzalloc(tx_num * rx_num * sizeof(signed short), GFP_KERNEL);
	if (!p_rt92_image_2) {
		TP_LOGE("Failed to alloc mem for p_rt92_image_2\n");

		retval = -ENOMEM;
		goto exit;
	}

	// keep the original integration duration
	if (f54->query.touch_controller_family != 2) {
		// TODO : support touch controller family = 0 and 1
		TP_LOGE("not support touch controller family = 0 or 1 \n");

		retval = -EINVAL;
		goto exit;
	}
	// touch controller family = 2
	retval = synaptics_rmi4_reg_read(rmi4_data,
			control.reg_99->address,
			original_data_f54_ctrl99,
			sizeof(original_data_f54_ctrl99));
	if (retval < 0) {
		TP_LOGE("Failed to read integration duration\n");

		retval = -EIO;
		goto exit;
	}

	/* step 1 */
	/* set the in_iter_duration_1 setting */
	/* and read the first rt92 image */
	control.reg_99->integration_duration_lsb = AMP_OPEN_INT_DUR_ONE;
	control.reg_99->integration_duration_msb = (AMP_OPEN_INT_DUR_ONE >> 8) & 0xff;
	control.reg_99->reset_duration = original_data_f54_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		TP_LOGE("Failed to write the integration duration to f54_ctrl_99 in step 1\n");
		retval = -EIO;
		goto exit;
	}
	retval = test_do_command(COMMAND_FORCE_UPDATE);
	if (retval < 0) {
		TP_LOGE("Failed to do force update in step 1\n");
		retval = -EIO;
		goto exit;
	}
	// grep the report image 92
	retval = test_sysfs_read_report(dev, attr, "92", count,
				false, false);
	if (retval < 0) {
		TP_LOGE("Failed to read report 92 in step 1. exit\n");
		retval = -EIO;
		goto exit;
	}

	secure_memcpy(p_report_data_8, tx_num * rx_num * 2,
		f54->report_data, f54->report_size, f54->report_size);

	// normalize the rt92 image with 16-bit
	k = 0;
	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			p_rt92_image_1[i * rx_num + j] =
				(signed short)(p_report_data_8[k] & 0xff ) | (signed short)(p_report_data_8[k + 1] << 8);

			k += 2;
		}
	}

	memset(p_report_data_8, 0x00, tx_num * rx_num * 2);

	/* step 2 */
	/* set the in_iter_duration_2 setting */
	/* and read the second rt92 image */
	control.reg_99->integration_duration_lsb = AMP_OPEN_INT_DUR_TWO;
	control.reg_99->integration_duration_msb = (AMP_OPEN_INT_DUR_TWO >> 8) & 0xff;
	control.reg_99->reset_duration = original_data_f54_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		TP_LOGE("Failed to write the integration duration to f54_ctrl_99 in step 2\n");
		retval = -EIO;
		goto exit;
	}
	retval = test_do_command(COMMAND_FORCE_UPDATE);
	if (retval < 0) {
		TP_LOGE("Failed to do force update in step 2\n");
		retval = -EIO;
		goto exit;
	}
	// grep the report image 92
	retval = test_sysfs_read_report(dev, attr, "92", count,
				false, false);
	if (retval < 0) {
		TP_LOGE("Failed to read report 92 in step 2. exit\n");
		retval = -EIO;
		goto exit;
	}

	secure_memcpy(p_report_data_8, tx_num * rx_num * 2,
		f54->report_data, f54->report_size, f54->report_size);

	// normalize the rt92 image with 16-bit
	k = 0;
	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			p_rt92_image_2[i * rx_num + j] =
				(signed short)(p_report_data_8[k] & 0xff ) | (signed short)(p_report_data_8[k + 1] << 8);

			k += 2;
		}
	}

	/* restore the original settings */
	control.reg_99->integration_duration_lsb = original_data_f54_ctrl99[0];
	control.reg_99->integration_duration_msb = original_data_f54_ctrl99[1];
	control.reg_99->reset_duration = original_data_f54_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		TP_LOGE("Failed to write the integration duration to f54_ctrl_99 in restore phase\n");
		retval = -EIO;
		goto exit;
	}
	retval = test_do_command(COMMAND_FORCE_UPDATE);
	if (retval < 0) {
		TP_LOGE("Failed to do force update in restore phase\n");
		retval = -EIO;
		goto exit;
	}

	/* step 3 */
	/* generate the delta image, td43xx_rt92_delta_image */
	/* unit is femtofarad (fF) */
	for (i = 0; i < tx_num * rx_num; i++) {
		p_rt92_delta_image[i] = p_rt92_image_1[i] - p_rt92_image_2[i];
	}

	/* step 4 */
	/* data testing */
	tddi_amp_open_data_testing(p_rt92_delta_image);

	retval = count;

exit:
	// release resource
	kfree(p_rt92_image_1);
	kfree(p_rt92_image_2);
	kfree(p_rt92_delta_image);
	kfree(p_report_data_8);

//	rmi4_data->reset_device(rmi4_data);

	return count;
}

static ssize_t synaptics_rmi4_f54_tddi_amp_open_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, j;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	bool result = 1;

	// 20160920 +
	int cnt;
	int count = 0;

	short min = 0, max = 0;
	// 20160920 -

	if (!g_tddi_amp_open_data_output)
		return -EINVAL;

	// 20160920 +
	if (!g_tddi_amp_open_data_log_1 || !g_tddi_amp_open_data_log_2)
		return -EINVAL;

 	cnt = snprintf(buf, PAGE_SIZE - count, "phase 1 limit %-3d - %-3d\nphase 2 limit %-3d - %-3d\n",
 					AMP_OPEN_TEST_LIMIT_PHASE1_LOWER, AMP_OPEN_TEST_LIMIT_PHASE1_UPPER,
 					AMP_OPEN_TEST_LIMIT_PHASE2_LOWER, AMP_OPEN_TEST_LIMIT_PHASE2_UPPER);
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "\nphase 1 data \n");
	buf += cnt;
	count += cnt;
	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
 			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d ", g_tddi_amp_open_data_log_1[i*rx_num + j]);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}

	min = max = g_tddi_amp_open_data_log_2[0];

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			min = (min < g_tddi_amp_open_data_log_2[i*rx_num + j])? min : g_tddi_amp_open_data_log_2[i*rx_num + j];
			max = (max > g_tddi_amp_open_data_log_2[i*rx_num + j])? max : g_tddi_amp_open_data_log_2[i*rx_num + j];
		}
	}

	cnt = snprintf(buf, PAGE_SIZE - count, "\nphase 2 range (max, min) = (%-4d, %-4d)\n", max, min);
	buf += cnt;
	count += cnt;
	// 20160920 -

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			if (g_tddi_amp_open_data_output[i * rx_num + j] != 0) {
				result &= 0; // 0: fail
				// 20160920 +
				cnt = snprintf(buf, PAGE_SIZE - count, "fail on (tx%-2d, rx%-2d)\n", i, j);
				buf += cnt;
				count += cnt;
				// 20160920 -
			}
		}
	}

	// 20160920 +
	kfree(g_tddi_amp_open_data_log_1);
	g_tddi_amp_open_data_log_1 = NULL;

	kfree(g_tddi_amp_open_data_log_2);
	g_tddi_amp_open_data_log_2 = NULL;
	// 20160920 -

	kfree(g_tddi_amp_open_data_output);
	g_tddi_amp_open_data_output = NULL;

	// 20160920 +
	cnt = snprintf(buf, PAGE_SIZE - count, "\n%s\n", (result == 1) ? "PASS" : "FAIL");
	buf += cnt;
	count += cnt;

	return count;
	//return snprintf(buf, PAGE_SIZE, "%s\n", (result == 1) ? "PASS" : "FAIL");
	// 20160920 -
}


/* tddi f54 test reporting - */

static ssize_t synaptics_rmi4_f54_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int test_fail = 0;
	char result_file_name[128] = {0};
	char *testdata = NULL;
	char *temp = NULL;
	char line1[40] = "******** tddi_amp_open ********\n";
	char line2[40] = "******** tddi_ee_short ********\n";
	ssize_t ret;
	size_t next;

	TP_LOGI("** enter **\n");

	memset(result_file_name, 0, sizeof(result_file_name));
	sprintf(result_file_name, "%s%s",
			TEST_DEFAULT_SAVE_PATH,
			TEST_DEFAULT_RESULT_FILE_NAME);

	testdata = vmalloc(TEST_DATA_SIZE);
	if (testdata == NULL) {
		TP_LOGE("testdata vmalloc failed\n");
		goto exit;
	}
	temp = vmalloc(TEST_TEMP_DATA_SIZE);
	if (temp == NULL) {
		TP_LOGE("temp vmalloc failed\n");
		goto exit2;
	}

	memcpy(testdata, line1, strlen(line1));
	next = strlen(line1);

	/* Step 1 : Open test */
	memset(temp, 0, TEST_TEMP_DATA_SIZE);
	ret = synaptics_rmi4_f54_tddi_amp_open_store(dev, attr, "1", 1);
	if (ret < 0) {
		TP_LOGE("tddi_amp_open store error");
		goto exit3;
	}

	ret = synaptics_rmi4_f54_tddi_amp_open_show(dev, attr, temp);
	if (ret < 0) {
		TP_LOGE("tddi_amp_open show error");
		goto exit3;
	}

	TP_LOGI("Open Test result: %s\n", temp + strlen(temp) - 5);

	if (strncmp(temp + strlen(temp) - 5, "PASS", 4) != 0) {
		test_fail++;
	}

	memcpy(testdata + next, temp, strlen(temp));
	next += strlen(temp);

	memcpy(testdata + next, line2, strlen(line2));
	next += strlen(line2);

	/* Step 2 : Short test */
	memset(temp, 0, TEST_TEMP_DATA_SIZE);
	ret = synaptics_rmi4_f54_tddi_ee_short_store(dev, attr, "1", 1);
	if (ret < 0) {
		TP_LOGE("tddi_ee_short store error");
		goto exit3;
	}

	ret = synaptics_rmi4_f54_tddi_ee_short_show(dev, attr, temp);
	if (ret < 0) {
		TP_LOGE("tddi_ee_short show error");
		goto exit3;
	}

	TP_LOGI("Short Test result: %s\n", temp + strlen(temp) - 5);

	if (strncmp(temp + strlen(temp) - 5, "PASS", 4) != 0) {
		test_fail++;
	}

	memcpy(testdata + next, temp, strlen(temp));

	TP_LOGI("** Self test result = %s **\n", (test_fail == 0) ? "pass" : "fail");

	save_test_data(result_file_name, testdata, strlen(testdata));

exit3:
	vfree(temp);
exit2:
	vfree(testdata);
exit:
	return snprintf(buf, PAGE_SIZE, "%s\n", (test_fail == 0) ? "pass" : "fail");

	// TODO: SP phase always return pass. Remove after SP phase
//	return snprintf(buf, PAGE_SIZE, "%s\n", "pass");
}

static ssize_t synaptics_rmi4_f54_burst_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	int size = 0;
	unsigned char ii;
	unsigned char *temp;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	mutex_lock(&f54->control_mutex);

	retval = f54->fn_ptr->read(rmi4_data,
			f54->control.reg_17->address,
			(unsigned char *)f54->control.reg_17->data,
			f54->control.reg_17->length);
	if (retval < 0) {
		TP_LOGI("Failed to read control reg_17\n");
	}

	retval = f54->fn_ptr->read(rmi4_data,
			f54->control.reg_18->address,
			(unsigned char *)f54->control.reg_18->data,
			f54->control.reg_18->length);
	if (retval < 0) {
		TP_LOGI("Failed to read control reg_18\n");
	}

	mutex_unlock(&f54->control_mutex);

	temp = buf;

	for (ii = 0; ii < f54->control.reg_17->length; ii++) {
		retval = snprintf(temp, PAGE_SIZE - size, "%u ", (1 << 8) *
			f54->control.reg_17->data[ii].burst_count_b8__10 +
			f54->control.reg_18->data[ii].burst_count_b0__7);
		if (retval < 0) {
			TP_LOGE("Faild to write output\n");
			return retval;
		}
		size += retval;
		temp += retval;
	}

	retval = snprintf(temp, PAGE_SIZE - size, "\n");
	if (retval < 0) {
		TP_LOGE("Faild to write null terminator\n");
		return retval;
	}

	return size + retval;
}

static ssize_t synaptics_rmi4_f54_data_read(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	mutex_lock(&f54->data_mutex);

	if (count < f54->report_size) {
		TP_LOGE("Report type %d data size (%d) too large\n",
				f54->report_type, f54->report_size);
		mutex_unlock(&f54->data_mutex);
		return -EINVAL;
	}

	if (f54->report_data) {
		memcpy(buf, f54->report_data, f54->report_size);
		mutex_unlock(&f54->data_mutex);
		return f54->report_size;
	} else {
		TP_LOGE("Report type %d data not available\n",
				f54->report_type);
		mutex_unlock(&f54->data_mutex);
		return -EINVAL;
	}
}

static void synaptics_rmi4_f54_status_work(struct work_struct *work)
{
	int retval;
	unsigned char report_index[2];
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	if (f54->status != STATUS_BUSY)
		return;

	set_report_size();
	if (f54->report_size == 0) {
		TP_LOGE("Report data size = 0\n");
		retval = -EINVAL;
		goto error_exit;
	}

	if (f54->data_buffer_size < f54->report_size) {
		mutex_lock(&f54->data_mutex);
		if (f54->data_buffer_size)
			kfree(f54->report_data);
		f54->report_data = kzalloc(f54->report_size, GFP_KERNEL);
		if (!f54->report_data) {
			TP_LOGE("Failed to alloc mem for data buffer\n");
			f54->data_buffer_size = 0;
			mutex_unlock(&f54->data_mutex);
			retval = -ENOMEM;
			goto error_exit;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = f54->fn_ptr->write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		TP_LOGE("Failed to write report data index\n");
		retval = -EINVAL;
		goto error_exit;
	}

	retval = f54->fn_ptr->read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
		TP_LOGE("Failed to read report data\n");
		retval = -EINVAL;
		goto error_exit;
	}

	retval = STATUS_IDLE;

#ifdef RAW_HEX
	print_raw_hex_report();
#endif

#ifdef HUMAN_READABLE
	print_image_report();
#endif

error_exit:
	mutex_lock(&f54->status_mutex);
	f54->status = retval;
	mutex_unlock(&f54->status_mutex);

	return;
}

static void remove_sysfs(void)
{
	int reg_num;

	sysfs_remove_bin_file(f54->attr_dir, &dev_report_data);

	sysfs_remove_group(f54->attr_dir, &attr_group);

	for (reg_num = 0; reg_num < ARRAY_SIZE(attrs_ctrl_regs); reg_num++)
		sysfs_remove_group(f54->attr_dir, &attrs_ctrl_regs[reg_num]);

	kobject_put(f54->attr_dir);

	return;
}

static int synaptics_rmi4_f54_set_sysfs(void)
{
	int retval;
	int reg_num;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	f54->attr_dir = kobject_create_and_add("f54",
			&rmi4_data->input_dev->dev.kobj);
	if (!f54->attr_dir) {
		TP_LOGE("Failed to create sysfs directory\n");
		goto exit_1;
	}

	retval = sysfs_create_bin_file(f54->attr_dir, &dev_report_data);
	if (retval < 0) {
		TP_LOGE("Failed to create sysfs bin file\n");
		goto exit_2;
	}

	retval = sysfs_create_group(f54->attr_dir, &attr_group);
	if (retval < 0) {
		TP_LOGE("Failed to create sysfs attributes\n");
		goto exit_3;
	}

	for (reg_num = 0; reg_num < ARRAY_SIZE(attrs_ctrl_regs); reg_num++) {
		if (attrs_ctrl_regs_exist[reg_num]) {
			retval = sysfs_create_group(f54->attr_dir,
					&attrs_ctrl_regs[reg_num]);
			if (retval < 0) {
				TP_LOGE("Failed to create sysfs attributes\n");
				goto exit_4;
			}
		}
	}

	return 0;

exit_4:
	sysfs_remove_group(f54->attr_dir, &attr_group);

	for (reg_num--; reg_num >= 0; reg_num--)
		sysfs_remove_group(f54->attr_dir, &attrs_ctrl_regs[reg_num]);

exit_3:
	sysfs_remove_bin_file(f54->attr_dir, &dev_report_data);

exit_2:
	kobject_put(f54->attr_dir);

exit_1:
	return -ENODEV;
}

static void free_control_mem(void)
{
	struct f54_control control = f54->control;

	kfree(control.reg_0);
	kfree(control.reg_1);
	kfree(control.reg_2);
	kfree(control.reg_3);
	kfree(control.reg_4__6);
	kfree(control.reg_7);
	kfree(control.reg_8__9);
	kfree(control.reg_10);
	kfree(control.reg_11);
	kfree(control.reg_12__13);
	kfree(control.reg_14);
	kfree(control.reg_15);
	kfree(control.reg_16);
	kfree(control.reg_17);
	kfree(control.reg_18);
	kfree(control.reg_19);
	kfree(control.reg_20);
	kfree(control.reg_21);
	kfree(control.reg_22__26);
	kfree(control.reg_27);
	kfree(control.reg_28);
	kfree(control.reg_29);
	kfree(control.reg_30);
	kfree(control.reg_31);
	kfree(control.reg_32__35);
	kfree(control.reg_36);
	kfree(control.reg_37);
	kfree(control.reg_38);
	kfree(control.reg_39);
	kfree(control.reg_40);
	kfree(control.reg_41);
	kfree(control.reg_57);
	kfree(control.reg_86);
	kfree(control.reg_88);
	/* tddi f54 test reporting + */
	kfree(control.reg_91);
	kfree(control.reg_96);
	kfree(control.reg_99);
	/* tddi f54 test reporting - */
	kfree(control.reg_110);
	kfree(control.reg_149);
	kfree(control.reg_188);

	return;
}

static void test_set_data(void)
{
	unsigned short reg_addr;

	reg_addr = f54->data_base_addr + DATA_REPORT_DATA_OFFSET + 1;

	/* data 4 */
	if (f54->query.has_sense_frequency_control)
		reg_addr++;

	/* data 5 reserved */

	/* data 6 */
	if (f54->query.has_interference_metric)
		reg_addr += 2;

	/* data 7 */
	if (f54->query.has_one_byte_report_rate |
			f54->query.has_two_byte_report_rate)
		reg_addr++;
	if (f54->query.has_two_byte_report_rate)
		reg_addr++;

	/* data 8 */
	if (f54->query.has_variance_metric)
		reg_addr += 2;

	/* data 9 */
	if (f54->query.has_multi_metric_state_machine)
		reg_addr += 2;

	/* data 10 */
	if (f54->query.has_multi_metric_state_machine |
			f54->query.has_noise_state)
		reg_addr++;

	/* data 11 */
	if (f54->query.has_status)
		reg_addr++;

	/* data 12 */
	if (f54->query.has_slew_metric)
		reg_addr += 2;

	/* data 13 */
	if (f54->query.has_multi_metric_state_machine)
		reg_addr += 2;

	/* data 14 */
	if (f54->query_13.has_cidim)
		reg_addr++;

	/* data 15 */
	if (f54->query_13.has_rail_im)
		reg_addr++;

	/* data 16 */
	if (f54->query_13.has_noise_mitigation_enhancement)
		reg_addr++;

	/* data 17 */
	if (f54->query_16.has_data17)
		reg_addr++;

	/* data 18 */
	if (f54->query_21.has_query24_data18)
		reg_addr++;

	/* data 19 */
	if (f54->query_21.has_data19)
		reg_addr++;

	/* data_20 */
	if (f54->query_25.has_ctrl109)
		reg_addr++;

	/* data 21 */
	if (f54->query_27.has_data21)
		reg_addr++;

	/* data 22 */
	if (f54->query_27.has_data22)
		reg_addr++;

	/* data 23 */
	if (f54->query_29.has_data23)
		reg_addr++;

	/* data 24 */
	if (f54->query_32.has_data24)
		reg_addr++;

	/* data 25 */
	if (f54->query_35.has_data25)
		reg_addr++;

	/* data 26 */
	if (f54->query_35.has_data26)
		reg_addr++;

	/* data 27 */
	if (f54->query_46.has_data27)
		reg_addr++;

	/* data 28 */
	if (f54->query_46.has_data28)
		reg_addr++;

	/* data 29 30 reserved */

	/* data 31 */
	if (f54->query_49.has_data31) {
		f54->data_31.address = reg_addr;
		reg_addr++;
	}

	return;
}

static void synaptics_rmi4_f54_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (!f54)
		return;

	if (f54->intr_mask & intr_mask) {
		queue_delayed_work(f54->status_workqueue,
				&f54->status_work,
				msecs_to_jiffies(STATUS_WORK_INTERVAL));
	}

	return;
}

static int synaptics_rmi4_f54_set_ctrl(void)
{
	int retval;
	unsigned char length;
	unsigned char reg_num = 0;
	unsigned char num_of_sensing_freqs;
	unsigned short reg_addr = f54->control_base_addr;
	struct f54_control *control = &f54->control;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	num_of_sensing_freqs = f54->query.number_of_sensing_frequencies;

	/* control 0 */
	attrs_ctrl_regs_exist[reg_num] = true;
	control->reg_0 = kzalloc(sizeof(*(control->reg_0)),
			GFP_KERNEL);
	if (!control->reg_0)
		goto exit_no_mem;
	control->reg_0->address = reg_addr;
	reg_addr += sizeof(control->reg_0->data);
	reg_num++;

	/* control 1 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1)) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_1 = kzalloc(sizeof(*(control->reg_1)),
				GFP_KERNEL);
		if (!control->reg_1)
			goto exit_no_mem;
		control->reg_1->address = reg_addr;
		reg_addr += sizeof(control->reg_1->data);
	}
	reg_num++;

	/* control 2 */
	attrs_ctrl_regs_exist[reg_num] = true;
	control->reg_2 = kzalloc(sizeof(*(control->reg_2)),
			GFP_KERNEL);
	if (!control->reg_2)
		goto exit_no_mem;
	control->reg_2->address = reg_addr;
	reg_addr += sizeof(control->reg_2->data);
	reg_num++;

	/* control 3 */
	if (f54->query.has_pixel_touch_threshold_adjustment == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_3 = kzalloc(sizeof(*(control->reg_3)),
				GFP_KERNEL);
		if (!control->reg_3)
			goto exit_no_mem;
		control->reg_3->address = reg_addr;
		reg_addr += sizeof(control->reg_3->data);
	}
	reg_num++;

	/* controls 4 5 6 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1)) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_4__6 = kzalloc(sizeof(*(control->reg_4__6)),
				GFP_KERNEL);
		if (!control->reg_4__6)
			goto exit_no_mem;
		control->reg_4__6->address = reg_addr;
		reg_addr += sizeof(control->reg_4__6->data);
	}
	reg_num++;

	/* control 7 */
	if (f54->query.touch_controller_family == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_7 = kzalloc(sizeof(*(control->reg_7)),
				GFP_KERNEL);
		if (!control->reg_7)
			goto exit_no_mem;
		control->reg_7->address = reg_addr;
		reg_addr += sizeof(control->reg_7->data);
	}
	reg_num++;

	/* controls 8 9 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1)) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_8__9 = kzalloc(sizeof(*(control->reg_8__9)),
				GFP_KERNEL);
		if (!control->reg_8__9)
			goto exit_no_mem;
		control->reg_8__9->address = reg_addr;
		reg_addr += sizeof(control->reg_8__9->data);
	}
	reg_num++;

	/* control 10 */
	if (f54->query.has_interference_metric == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_10 = kzalloc(sizeof(*(control->reg_10)),
				GFP_KERNEL);
		if (!control->reg_10)
			goto exit_no_mem;
		control->reg_10->address = reg_addr;
		reg_addr += sizeof(control->reg_10->data);
	}
	reg_num++;

	/* control 11 */
	if (f54->query.has_ctrl11 == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_11 = kzalloc(sizeof(*(control->reg_11)),
				GFP_KERNEL);
		if (!control->reg_11)
			goto exit_no_mem;
		control->reg_11->address = reg_addr;
		reg_addr += sizeof(control->reg_11->data);
	}
	reg_num++;

	/* controls 12 13 */
	if (f54->query.has_relaxation_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_12__13 = kzalloc(sizeof(*(control->reg_12__13)),
				GFP_KERNEL);
		if (!control->reg_12__13)
			goto exit_no_mem;
		control->reg_12__13->address = reg_addr;
		reg_addr += sizeof(control->reg_12__13->data);
	}
	reg_num++;

	/* controls 14 15 16 */
	if (f54->query.has_sensor_assignment == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;

		control->reg_14 = kzalloc(sizeof(*(control->reg_14)),
				GFP_KERNEL);
		if (!control->reg_14)
			goto exit_no_mem;
		control->reg_14->address = reg_addr;
		reg_addr += sizeof(control->reg_14->data);

		control->reg_15 = kzalloc(sizeof(*(control->reg_15)),
				GFP_KERNEL);
		if (!control->reg_15)
			goto exit_no_mem;
		control->reg_15->length = f54->query.num_of_rx_electrodes;
		control->reg_15->data = kzalloc(control->reg_15->length *
				sizeof(*(control->reg_15->data)), GFP_KERNEL);
		if (!control->reg_15->data)
			goto exit_no_mem;
		control->reg_15->address = reg_addr;
		reg_addr += control->reg_15->length;

		control->reg_16 = kzalloc(sizeof(*(control->reg_16)),
				GFP_KERNEL);
		if (!control->reg_16)
			goto exit_no_mem;
		control->reg_16->length = f54->query.num_of_tx_electrodes;
		control->reg_16->data = kzalloc(control->reg_16->length *
				sizeof(*(control->reg_16->data)), GFP_KERNEL);
		if (!control->reg_16->data)
			goto exit_no_mem;
		control->reg_16->address = reg_addr;
		reg_addr += control->reg_16->length;
	}
	reg_num++;

	/* controls 17 18 19 */
	if (f54->query.has_sense_frequency_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;

		length = num_of_sensing_freqs;

		control->reg_17 = kzalloc(sizeof(*(control->reg_17)),
				GFP_KERNEL);
		if (!control->reg_17)
			goto exit_no_mem;
		control->reg_17->length = length;
		control->reg_17->data = kzalloc(length *
				sizeof(*(control->reg_17->data)), GFP_KERNEL);
		if (!control->reg_17->data)
			goto exit_no_mem;
		control->reg_17->address = reg_addr;
		reg_addr += length;

		control->reg_18 = kzalloc(sizeof(*(control->reg_18)),
				GFP_KERNEL);
		if (!control->reg_18)
			goto exit_no_mem;
		control->reg_18->length = length;
		control->reg_18->data = kzalloc(length *
				sizeof(*(control->reg_18->data)), GFP_KERNEL);
		if (!control->reg_18->data)
			goto exit_no_mem;
		control->reg_18->address = reg_addr;
		reg_addr += length;

		control->reg_19 = kzalloc(sizeof(*(control->reg_19)),
				GFP_KERNEL);
		if (!control->reg_19)
			goto exit_no_mem;
		control->reg_19->length = length;
		control->reg_19->data = kzalloc(length *
				sizeof(*(control->reg_19->data)), GFP_KERNEL);
		if (!control->reg_19->data)
			goto exit_no_mem;
		control->reg_19->address = reg_addr;
		reg_addr += length;
	}
	reg_num++;

	/* control 20 */
	attrs_ctrl_regs_exist[reg_num] = true;
	control->reg_20 = kzalloc(sizeof(*(control->reg_20)),
			GFP_KERNEL);
	if (!control->reg_20)
		goto exit_no_mem;
	control->reg_20->address = reg_addr;
	reg_addr += sizeof(control->reg_20->data);
	reg_num++;

	/* control 21 */
	if (f54->query.has_sense_frequency_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_21 = kzalloc(sizeof(*(control->reg_21)),
				GFP_KERNEL);
		if (!control->reg_21)
			goto exit_no_mem;
		control->reg_21->address = reg_addr;
		reg_addr += sizeof(control->reg_21->data);
	}
	reg_num++;

	/* controls 22 23 24 25 26 */
	if (f54->query.has_firmware_noise_mitigation == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_22__26 = kzalloc(sizeof(*(control->reg_22__26)),
				GFP_KERNEL);
		if (!control->reg_22__26)
			goto exit_no_mem;
		control->reg_22__26->address = reg_addr;
		reg_addr += sizeof(control->reg_22__26->data);
	}
	reg_num++;

	/* control 27 */
	if (f54->query.has_iir_filter == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_27 = kzalloc(sizeof(*(control->reg_27)),
				GFP_KERNEL);
		if (!control->reg_27)
			goto exit_no_mem;
		control->reg_27->address = reg_addr;
		reg_addr += sizeof(control->reg_27->data);
	}
	reg_num++;

	/* control 28 */
	if (f54->query.has_firmware_noise_mitigation == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_28 = kzalloc(sizeof(*(control->reg_28)),
				GFP_KERNEL);
		if (!control->reg_28)
			goto exit_no_mem;
		control->reg_28->address = reg_addr;
		reg_addr += sizeof(control->reg_28->data);
	}
	reg_num++;

	/* control 29 */
	if (f54->query.has_cmn_removal == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_29 = kzalloc(sizeof(*(control->reg_29)),
				GFP_KERNEL);
		if (!control->reg_29)
			goto exit_no_mem;
		control->reg_29->address = reg_addr;
		reg_addr += sizeof(control->reg_29->data);
	}
	reg_num++;

	/* control 30 */
	if (f54->query.has_cmn_maximum == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_30 = kzalloc(sizeof(*(control->reg_30)),
				GFP_KERNEL);
		if (!control->reg_30)
			goto exit_no_mem;
		control->reg_30->address = reg_addr;
		reg_addr += sizeof(control->reg_30->data);
	}
	reg_num++;

	/* control 31 */
	if (f54->query.has_touch_hysteresis == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_31 = kzalloc(sizeof(*(control->reg_31)),
				GFP_KERNEL);
		if (!control->reg_31)
			goto exit_no_mem;
		control->reg_31->address = reg_addr;
		reg_addr += sizeof(control->reg_31->data);
	}
	reg_num++;

	/* controls 32 33 34 35 */
	if (f54->query.has_edge_compensation == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_32__35 = kzalloc(sizeof(*(control->reg_32__35)),
				GFP_KERNEL);
		if (!control->reg_32__35)
			goto exit_no_mem;
		control->reg_32__35->address = reg_addr;
		reg_addr += sizeof(control->reg_32__35->data);
	}
	reg_num++;

	/* control 36 */
	if ((f54->query.curve_compensation_mode == 1) ||
			(f54->query.curve_compensation_mode == 2)) {
		attrs_ctrl_regs_exist[reg_num] = true;

		if (f54->query.curve_compensation_mode == 1) {
			length = max(f54->query.num_of_rx_electrodes,
					f54->query.num_of_tx_electrodes);
		} else if (f54->query.curve_compensation_mode == 2) {
			length = f54->query.num_of_rx_electrodes;
		}

		control->reg_36 = kzalloc(sizeof(*(control->reg_36)),
				GFP_KERNEL);
		if (!control->reg_36)
			goto exit_no_mem;
		control->reg_36->length = length;
		control->reg_36->data = kzalloc(length *
				sizeof(*(control->reg_36->data)), GFP_KERNEL);
		if (!control->reg_36->data)
			goto exit_no_mem;
		control->reg_36->address = reg_addr;
		reg_addr += length;
	}
	reg_num++;

	/* control 37 */
	if (f54->query.curve_compensation_mode == 2) {
		attrs_ctrl_regs_exist[reg_num] = true;

		control->reg_37 = kzalloc(sizeof(*(control->reg_37)),
				GFP_KERNEL);
		if (!control->reg_37)
			goto exit_no_mem;
		control->reg_37->length = f54->query.num_of_tx_electrodes;
		control->reg_37->data = kzalloc(control->reg_37->length *
				sizeof(*(control->reg_37->data)), GFP_KERNEL);
		if (!control->reg_37->data)
			goto exit_no_mem;

		control->reg_37->address = reg_addr;
		reg_addr += control->reg_37->length;
	}
	reg_num++;

	/* controls 38 39 40 */
	if (f54->query.has_per_frequency_noise_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;

		control->reg_38 = kzalloc(sizeof(*(control->reg_38)),
				GFP_KERNEL);
		if (!control->reg_38)
			goto exit_no_mem;
		control->reg_38->length = num_of_sensing_freqs;
		control->reg_38->data = kzalloc(control->reg_38->length *
				sizeof(*(control->reg_38->data)), GFP_KERNEL);
		if (!control->reg_38->data)
			goto exit_no_mem;
		control->reg_38->address = reg_addr;
		reg_addr += control->reg_38->length;

		control->reg_39 = kzalloc(sizeof(*(control->reg_39)),
				GFP_KERNEL);
		if (!control->reg_39)
			goto exit_no_mem;
		control->reg_39->length = num_of_sensing_freqs;
		control->reg_39->data = kzalloc(control->reg_39->length *
				sizeof(*(control->reg_39->data)), GFP_KERNEL);
		if (!control->reg_39->data)
			goto exit_no_mem;
		control->reg_39->address = reg_addr;
		reg_addr += control->reg_39->length;

		control->reg_40 = kzalloc(sizeof(*(control->reg_40)),
				GFP_KERNEL);
		if (!control->reg_40)
			goto exit_no_mem;
		control->reg_40->length = num_of_sensing_freqs;
		control->reg_40->data = kzalloc(control->reg_40->length *
				sizeof(*(control->reg_40->data)), GFP_KERNEL);
		if (!control->reg_40->data)
			goto exit_no_mem;
		control->reg_40->address = reg_addr;
		reg_addr += control->reg_40->length;
	}
	reg_num++;

	/* control 41 */
	if (f54->query.has_signal_clarity == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_41 = kzalloc(sizeof(*(control->reg_41)),
				GFP_KERNEL);
		if (!control->reg_41)
			goto exit_no_mem;
		control->reg_41->address = reg_addr;
		reg_addr += sizeof(control->reg_41->data);
	}
	reg_num++;

	/* control 42 */
	if (f54->query.has_variance_metric == 1)
		reg_addr += CONTROL_42_SIZE;

	/* controls 43 44 45 46 47 48 49 50 51 52 53 54 */
	if (f54->query.has_multi_metric_state_machine == 1)
		reg_addr += CONTROL_43_54_SIZE;

	/* controls 55 56 */
	if (f54->query.has_0d_relaxation_control == 1)
		reg_addr += CONTROL_55_56_SIZE;

	/* control 57 */
	if (f54->query.has_0d_acquisition_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_57 = kzalloc(sizeof(*(control->reg_57)),
				GFP_KERNEL);
		if (!control->reg_57)
			goto exit_no_mem;
		control->reg_57->address = reg_addr;
		reg_addr += sizeof(control->reg_57->data);
	}
	reg_num++;

	/* control 58 */
	if (f54->query.has_0d_acquisition_control == 1)
		reg_addr += CONTROL_58_SIZE;

	/* control 59 */
	if (f54->query.has_h_blank == 1)
		reg_addr += CONTROL_59_SIZE;

	/* controls 60 61 62 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_60_62_SIZE;

	/* control 63 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1) ||
			(f54->query.has_slew_metric == 1) ||
			(f54->query.has_slew_option == 1) ||
			(f54->query.has_noise_mitigation2 == 1))
		reg_addr += CONTROL_63_SIZE;

	/* controls 64 65 66 67 */
	if (f54->query.has_h_blank == 1)
		reg_addr += CONTROL_64_67_SIZE * 7;
	else if ((f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_64_67_SIZE;

	/* controls 68 69 70 71 72 73 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_68_73_SIZE;

	/* control 74 */
	if (f54->query.has_slew_metric == 1)
		reg_addr += CONTROL_74_SIZE;

	/* control 75 */
	if (f54->query.has_enhanced_stretch == 1)
		reg_addr += CONTROL_75_SIZE * num_of_sensing_freqs;

	/* control 76 */
	if (f54->query.has_startup_fast_relaxation == 1)
		reg_addr += CONTROL_76_SIZE;

	/* controls 77 78 */
	if (f54->query.has_esd_control == 1)
		reg_addr += CONTROL_77_78_SIZE;

	/* controls 79 80 81 82 83 */
	if (f54->query.has_noise_mitigation2 == 1)
		reg_addr += CONTROL_79_83_SIZE;

	/* controls 84 85 */
	if (f54->query.has_energy_ratio_relaxation == 1)
		reg_addr += CONTROL_84_85_SIZE;

	/* control 86 */
	if (f54->query_13.has_ctrl86 == 1) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_86 = kzalloc(sizeof(*(control->reg_86)),
				GFP_KERNEL);
		if (!control->reg_86)
			goto exit_no_mem;
		control->reg_86->address = reg_addr;
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->control.reg_86->address,
				f54->control.reg_86->data,
				sizeof(f54->control.reg_86->data));
		if (retval < 0) {
			TP_LOGE("Failed to read sense display ratio\n");
			return retval;
		}
		reg_addr += CONTROL_86_SIZE;
	}
	//reg_num++;

	/* control 87 */
	if (f54->query_13.has_ctrl87)
		reg_addr += CONTROL_87_SIZE;

	/* control 88 */
	if (f54->query.has_ctrl88 == 1) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_88 = kzalloc(sizeof(*(control->reg_88)),
				GFP_KERNEL);
		if (!control->reg_88)
			goto exit_no_mem;
		control->reg_88->address = reg_addr;
		reg_addr += sizeof(control->reg_88->data);
	}
	//reg_num++;

	/* control 89 */
	if (f54->query_13.has_cidim ||
			f54->query_13.has_noise_mitigation_enhancement ||
			f54->query_13.has_rail_im)
		reg_addr += CONTROL_89_SIZE;

	/* control 90 */
	if (f54->query_15.has_ctrl90)
		reg_addr += CONTROL_90_SIZE;

	/* control 91 */
	if (f54->query_21.has_ctrl91 == 1) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		/* tddi f54 test reporting + */
		control->reg_91 = kzalloc(sizeof(*(control->reg_91)),
				GFP_KERNEL);
		if (!control->reg_91)
			goto exit_no_mem;
		control->reg_91->address = reg_addr;
		/* tddi f54 test reporting - */
		reg_addr += CONTROL_91_SIZE;
	}
	//reg_num++;

	/* control 92 */
	if (f54->query_16.has_ctrl92)
		reg_addr += CONTROL_92_SIZE;

	/* control 93 */
	if (f54->query_16.has_ctrl93)
		reg_addr += CONTROL_93_SIZE;

	/* control 94 */
	if (f54->query_16.has_ctrl94_query18)
		reg_addr += CONTROL_94_SIZE;

	/* control 95 */
	if (f54->query_16.has_ctrl95_query19)
		reg_addr += CONTROL_95_SIZE;

	/* control 96 */
	if (f54->query_21.has_ctrl96 == 1) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		/* tddi f54 test reporting + */
		control->reg_96 = kzalloc(sizeof(*(control->reg_96)),
				GFP_KERNEL);
		if (!control->reg_96)
			goto exit_no_mem;
		control->reg_96->address = reg_addr;
		/* tddi f54 test reporting - */
		reg_addr += CONTROL_96_SIZE;
	}
	//reg_num++;

	/* control 97 */
	if (f54->query_21.has_ctrl97)
		reg_addr += CONTROL_97_SIZE;

	/* control 98 */
	if (f54->query_21.has_ctrl98)
		reg_addr += CONTROL_98_SIZE;

	/* control 99 */
	if (f54->query.touch_controller_family == 2) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		/* tddi f54 test reporting +  */
		control->reg_99 = kzalloc(sizeof(*(control->reg_99)),
				GFP_KERNEL);
		if (!control->reg_99)
			goto exit_no_mem;
		control->reg_99->address = reg_addr;
		/* tddi f54 test reporting - */
		reg_addr += CONTROL_99_SIZE;
	}
	//reg_num++;

	/* control 100 */
	if (f54->query_16.has_ctrl100)
		reg_addr += CONTROL_100_SIZE;

	/* control 101 */
	if (f54->query_22.has_ctrl101)
		reg_addr += CONTROL_101_SIZE;


	/* control 102 */
	if (f54->query_23.has_ctrl102)
		reg_addr += CONTROL_102_SIZE;

	/* control 103 */
	if (f54->query_22.has_ctrl103_query26) {
		f54->skip_preparation = true;
		reg_addr += CONTROL_103_SIZE;
	}

	/* control 104 */
	if (f54->query_22.has_ctrl104)
		reg_addr += CONTROL_104_SIZE;

	/* control 105 */
	if (f54->query_22.has_ctrl105)
		reg_addr += CONTROL_105_SIZE;

	/* control 106 */
	if (f54->query_25.has_ctrl106)
		reg_addr += CONTROL_106_SIZE;

	/* control 107 */
	if (f54->query_25.has_ctrl107)
		reg_addr += CONTROL_107_SIZE;

	/* control 108 */
	if (f54->query_25.has_ctrl108)
		reg_addr += CONTROL_108_SIZE;

	/* control 109 */
	if (f54->query_25.has_ctrl109)
		reg_addr += CONTROL_109_SIZE;

	/* control 110 */
	if (f54->query_27.has_ctrl110) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_110 = kzalloc(sizeof(*(control->reg_110)),
				GFP_KERNEL);
		if (!control->reg_110)
			goto exit_no_mem;
		control->reg_110->address = reg_addr;
		reg_addr += CONTROL_110_SIZE;
	}
	//reg_num++;

	/* control 111 */
	if (f54->query_27.has_ctrl111)
		reg_addr += CONTROL_111_SIZE;

	/* control 112 */
	if (f54->query_27.has_ctrl112)
		reg_addr += CONTROL_112_SIZE;

	/* control 113 */
	if (f54->query_27.has_ctrl113)
		reg_addr += CONTROL_113_SIZE;

	/* control 114 */
	if (f54->query_27.has_ctrl114)
		reg_addr += CONTROL_114_SIZE;

	/* control 115 */
	if (f54->query_29.has_ctrl115)
		reg_addr += CONTROL_115_SIZE;

	/* control 116 */
	if (f54->query_29.has_ctrl116)
		reg_addr += CONTROL_116_SIZE;

	/* control 117 */
	if (f54->query_29.has_ctrl117)
		reg_addr += CONTROL_117_SIZE;

	/* control 118 */
	if (f54->query_30.has_ctrl118)
		reg_addr += CONTROL_118_SIZE;

	/* control 119 */
	if (f54->query_30.has_ctrl119)
		reg_addr += CONTROL_119_SIZE;

	/* control 120 */
	if (f54->query_30.has_ctrl120)
		reg_addr += CONTROL_120_SIZE;

	/* control 121 */
	if (f54->query_30.has_ctrl121)
		reg_addr += CONTROL_121_SIZE;

	/* control 122 */
	if (f54->query_30.has_ctrl122_query31)
		reg_addr += CONTROL_122_SIZE;

	/* control 123 */
	if (f54->query_30.has_ctrl123)
		reg_addr += CONTROL_123_SIZE;

	/* control 124 */
	if (f54->query_30.has_ctrl124)
		reg_addr += CONTROL_124_SIZE;

	/* control 125 */
	if (f54->query_32.has_ctrl125)
		reg_addr += CONTROL_125_SIZE;

	/* control 126 */
	if (f54->query_32.has_ctrl126)
		reg_addr += CONTROL_126_SIZE;

	/* control 127 */
	if (f54->query_32.has_ctrl127)
		reg_addr += CONTROL_127_SIZE;

	/* control 128 */
	if (f54->query_33.has_ctrl128)
		reg_addr += CONTROL_128_SIZE;

	/* control 129 */
	if (f54->query_33.has_ctrl129)
		reg_addr += CONTROL_129_SIZE;

	/* control 130 */
	if (f54->query_33.has_ctrl130)
		reg_addr += CONTROL_130_SIZE;

	/* control 131 */
	if (f54->query_33.has_ctrl131)
		reg_addr += CONTROL_131_SIZE;

	/* control 132 */
	if (f54->query_33.has_ctrl132)
		reg_addr += CONTROL_132_SIZE;

	/* control 133 */
	if (f54->query_33.has_ctrl133)
		reg_addr += CONTROL_133_SIZE;

	/* control 134 */
	if (f54->query_33.has_ctrl134)
		reg_addr += CONTROL_134_SIZE;

	/* control 135 */
	if (f54->query_35.has_ctrl135)
		reg_addr += CONTROL_135_SIZE;

	/* control 136 */
	if (f54->query_35.has_ctrl136)
		reg_addr += CONTROL_136_SIZE;

	/* control 137 */
	if (f54->query_35.has_ctrl137)
		reg_addr += CONTROL_137_SIZE;

	/* control 138 */
	if (f54->query_35.has_ctrl138)
		reg_addr += CONTROL_138_SIZE;

	/* control 139 */
	if (f54->query_35.has_ctrl139)
		reg_addr += CONTROL_139_SIZE;

	/* control 140 */
	if (f54->query_35.has_ctrl140)
		reg_addr += CONTROL_140_SIZE;

	/* control 141 */
	if (f54->query_36.has_ctrl141)
		reg_addr += CONTROL_141_SIZE;

	/* control 142 */
	if (f54->query_36.has_ctrl142)
		reg_addr += CONTROL_142_SIZE;

	/* control 143 */
	if (f54->query_36.has_ctrl143)
		reg_addr += CONTROL_143_SIZE;

	/* control 144 */
	if (f54->query_36.has_ctrl144)
		reg_addr += CONTROL_144_SIZE;

	/* control 145 */
	if (f54->query_36.has_ctrl145)
		reg_addr += CONTROL_145_SIZE;

	/* control 146 */
	if (f54->query_36.has_ctrl146)
		reg_addr += CONTROL_146_SIZE;

	/* control 147 */
	if (f54->query_38.has_ctrl147)
		reg_addr += CONTROL_147_SIZE;

	/* control 148 */
	if (f54->query_38.has_ctrl148)
		reg_addr += CONTROL_148_SIZE;

	/* control 149 */
	if (f54->query_38.has_ctrl149) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_149 = kzalloc(sizeof(*(control->reg_149)),
				GFP_KERNEL);
		if (!control->reg_149)
			goto exit_no_mem;
		control->reg_149->address = reg_addr;
		reg_addr += CONTROL_149_SIZE;
	}
	//reg_num++;

	/* control 150 */
	if (f54->query_38.has_ctrl150)
		reg_addr += CONTROL_150_SIZE;

	/* control 151 */
	if (f54->query_38.has_ctrl151)
		reg_addr += CONTROL_151_SIZE;

	/* control 152 */
	if (f54->query_38.has_ctrl152)
		reg_addr += CONTROL_152_SIZE;

	/* control 153 */
	if (f54->query_38.has_ctrl153)
		reg_addr += CONTROL_153_SIZE;

	/* control 154 */
	if (f54->query_39.has_ctrl154)
		reg_addr += CONTROL_154_SIZE;

	/* control 155 */
	if (f54->query_39.has_ctrl155)
		reg_addr += CONTROL_155_SIZE;

	/* control 156 */
	if (f54->query_39.has_ctrl156)
		reg_addr += CONTROL_156_SIZE;

	/* controls 157 158 */
	if (f54->query_39.has_ctrl157_ctrl158)
		reg_addr += CONTROL_157_158_SIZE;

	/* controls 159 to 162 reserved */

	/* control 163 */
	if (f54->query_40.has_ctrl163_query41)
		reg_addr += CONTROL_163_SIZE;

	/* control 164 reserved */

	/* control 165 */
	if (f54->query_40.has_ctrl165_query42)
		reg_addr += CONTROL_165_SIZE;

	/* control 166 */
	if (f54->query_40.has_ctrl166)
		reg_addr += CONTROL_166_SIZE;

	/* control 167 */
	if (f54->query_40.has_ctrl167)
		reg_addr += CONTROL_167_SIZE;

	/* control 168 */
	if (f54->query_40.has_ctrl168)
		reg_addr += CONTROL_168_SIZE;

	/* control 169 */
	if (f54->query_40.has_ctrl169)
		reg_addr += CONTROL_169_SIZE;

	/* control 170 reserved */

	/* control 171 */
	if (f54->query_43.has_ctrl171)
		reg_addr += CONTROL_171_SIZE;

	/* control 172 */
	if (f54->query_43.has_ctrl172_query44_query45)
		reg_addr += CONTROL_172_SIZE;

	/* control 173 */
	if (f54->query_43.has_ctrl173)
		reg_addr += CONTROL_173_SIZE;

	/* control 174 */
	if (f54->query_43.has_ctrl174)
		reg_addr += CONTROL_174_SIZE;

	/* control 175 */
	if (f54->query_43.has_ctrl175)
		reg_addr += CONTROL_175_SIZE;

	/* control 176 */
	if (f54->query_46.has_ctrl176)
		reg_addr += CONTROL_176_SIZE;

	/* controls 177 178 */
	if (f54->query_46.has_ctrl177_ctrl178)
		reg_addr += CONTROL_177_178_SIZE;

	/* control 179 */
	if (f54->query_46.has_ctrl179)
		reg_addr += CONTROL_179_SIZE;

	/* controls 180 to 181 reserved */

	/* control 182 */
	if (f54->query_47.has_ctrl182)
		reg_addr += CONTROL_182_SIZE;

	/* control 183 */
	if (f54->query_47.has_ctrl183)
		reg_addr += CONTROL_183_SIZE;

	/* control 184 reserved */

	/* control 185 */
	if (f54->query_47.has_ctrl185)
		reg_addr += CONTROL_185_SIZE;

	/* control 186 */
	if (f54->query_47.has_ctrl186)
		reg_addr += CONTROL_186_SIZE;

	/* control 187 */
	if (f54->query_47.has_ctrl187)
		reg_addr += CONTROL_187_SIZE;

	/* control 188 */
	if (f54->query_49.has_ctrl188) {
		//attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_188 = kzalloc(sizeof(*(control->reg_188)),
				GFP_KERNEL);
		if (!control->reg_188)
			goto exit_no_mem;
		control->reg_188->address = reg_addr;
		reg_addr += CONTROL_188_SIZE;
	}
	//reg_num++;

	return 0;

exit_no_mem:
	TP_LOGE("Failed to alloc mem for control registers\n");

	return -ENOMEM;
}

static int test_set_queries(void)
{
	int retval;
	unsigned char offset;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->query_base_addr,
			f54->query.data,
			sizeof(f54->query.data));
	if (retval < 0)
		return retval;

	offset = sizeof(f54->query.data);

	/* query 12 */
	if (f54->query.has_sense_frequency_control == 0)
		offset -= 1;

	/* query 13 */
	if (f54->query.has_query13) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_13.data,
				sizeof(f54->query_13.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 14 */
	if (f54->query_13.has_ctrl87)
		offset += 1;

	/* query 15 */
	if (f54->query.has_query15) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_15.data,
				sizeof(f54->query_15.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 16 */
	if (f54->query_15.has_query16) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_16.data,
				sizeof(f54->query_16.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 17 */
	if (f54->query_16.has_query17)
		offset += 1;

	/* query 18 */
	if (f54->query_16.has_ctrl94_query18)
		offset += 1;

	/* query 19 */
	if (f54->query_16.has_ctrl95_query19)
		offset += 1;

	/* query 20 */
	if (f54->query_15.has_query20)
		offset += 1;

	/* query 21 */
	if (f54->query_15.has_query21) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_21.data,
				sizeof(f54->query_21.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 22 */
	if (f54->query_15.has_query22) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_22.data,
				sizeof(f54->query_22.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 23 */
	if (f54->query_22.has_query23) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_23.data,
				sizeof(f54->query_23.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 24 */
	if (f54->query_21.has_query24_data18)
		offset += 1;

	/* query 25 */
	if (f54->query_15.has_query25) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_25.data,
				sizeof(f54->query_25.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 26 */
	if (f54->query_22.has_ctrl103_query26)
		offset += 1;

	/* query 27 */
	if (f54->query_25.has_query27) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_27.data,
				sizeof(f54->query_27.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 28 */
	if (f54->query_22.has_query28)
		offset += 1;

	/* query 29 */
	if (f54->query_27.has_query29) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_29.data,
				sizeof(f54->query_29.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 30 */
	if (f54->query_29.has_query30) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_30.data,
				sizeof(f54->query_30.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 31 */
	if (f54->query_30.has_ctrl122_query31)
		offset += 1;

	/* query 32 */
	if (f54->query_30.has_query32) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_32.data,
				sizeof(f54->query_32.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 33 */
	if (f54->query_32.has_query33) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_33.data,
				sizeof(f54->query_33.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 34 */
	if (f54->query_32.has_query34)
		offset += 1;

	/* query 35 */
	if (f54->query_32.has_query35) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_35.data,
				sizeof(f54->query_35.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 36 */
	if (f54->query_33.has_query36) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_36.data,
				sizeof(f54->query_36.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 37 */
	if (f54->query_36.has_query37)
		offset += 1;

	/* query 38 */
	if (f54->query_36.has_query38) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_38.data,
				sizeof(f54->query_38.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 39 */
	if (f54->query_38.has_query39) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_39.data,
				sizeof(f54->query_39.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 40 */
	if (f54->query_39.has_query40) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_40.data,
				sizeof(f54->query_40.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 41 */
	if (f54->query_40.has_ctrl163_query41)
		offset += 1;

	/* query 42 */
	if (f54->query_40.has_ctrl165_query42)
		offset += 1;

	/* query 43 */
	if (f54->query_40.has_query43) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_43.data,
				sizeof(f54->query_43.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	if (f54->query_43.has_ctrl172_query44_query45)
		offset += 2;

	/* query 46 */
	if (f54->query_43.has_query46) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_46.data,
				sizeof(f54->query_46.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 47 */
	if (f54->query_46.has_query47) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_47.data,
				sizeof(f54->query_47.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 48 reserved */

	/* query 49 */
	if (f54->query_47.has_query49) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_49.data,
				sizeof(f54->query_49.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 50 */
	if (f54->query_49.has_query50) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_50.data,
				sizeof(f54->query_50.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 51 */
	if (f54->query_50.has_query51) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_51.data,
				sizeof(f54->query_51.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* tddi f54 test reporting +  */

	/* query 52 reserved */

	/* queries 53 54 */
	if (f54->query_51.has_query53_query54_ctrl198)
		offset += 2;

	/* query 55 */
	if (f54->query_51.has_query55) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_55.data,
				sizeof(f54->query_55.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 56 */
	if (f54->query_55.has_query56)
		offset += 1;

	/* query 57 */
	if (f54->query_55.has_query57) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_57.data,
				sizeof(f54->query_57.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 58 */
	if (f54->query_57.has_query58) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_58.data,
				sizeof(f54->query_58.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 59 */
	if (f54->query_58.has_query59)
		offset += 1;

	/* queries 60 */
	if (f54->query_58.has_query60)
		offset += 1;

	/* queries 61 */
	if (f54->query_58.has_query61) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_61.data,
				sizeof(f54->query_61.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 62 63 */
	if (f54->query_61.has_ctrl215_query62_query63)
		offset += 2;

	/* queries 64 */
	if (f54->query_61.has_query64) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_64.data,
				sizeof(f54->query_64.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 65 */
	if (f54->query_64.has_query65) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_65.data,
				sizeof(f54->query_65.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 66 */
	if (f54->query_65.has_query66_ctrl231)
		offset += 1;

	/* queries 67 */
	if (f54->query_65.has_query67) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_67.data,
				sizeof(f54->query_67.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 68 */
	if (f54->query_67.has_query68) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_68.data,
				sizeof(f54->query_68.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 69 */
	if (f54->query_68.has_query69) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_69.data,
				sizeof(f54->query_69.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* tddi f54 test reporting -  */

	return 0;
}

static void synaptics_rmi4_f54_set_regs(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count,
		unsigned char page)
{
	unsigned char ii;
	unsigned char intr_offset;

	f54->query_base_addr = fd->query_base_addr | (page << 8);
	f54->control_base_addr = fd->ctrl_base_addr | (page << 8);
	f54->data_base_addr = fd->data_base_addr | (page << 8);
	f54->command_base_addr = fd->cmd_base_addr | (page << 8);

	f54->intr_reg_num = (intr_count + 7) / 8;
	if (f54->intr_reg_num != 0)
		f54->intr_reg_num -= 1;

	f54->intr_mask = 0;
	intr_offset = intr_count % 8;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++) {
		f54->intr_mask |= 1 << ii;
	}

	return;
}

static int test_f55_set_controls(void)
{
	unsigned char offset = 0;

	/* controls 0 1 2 */
	if (f55->query.has_sensor_assignment)
		offset += 3;

	/* control 3 */
	if (f55->query.has_edge_compensation)
		offset++;

	/* control 4 */
	if (f55->query.curve_compensation_mode == 0x1 ||
			f55->query.curve_compensation_mode == 0x2)
		offset++;

	/* control 5 */
	if (f55->query.curve_compensation_mode == 0x2)
		offset++;

	/* control 6 */
	if (f55->query.has_ctrl6)
		offset++;

	/* control 7 */
	if (f55->query.has_alternate_transmitter_assignment)
		offset++;

	/* control 8 */
	if (f55->query_3.has_ctrl8)
		offset++;

	/* control 9 */
	if (f55->query_3.has_ctrl9)
		offset++;

	/* control 10 */
	if (f55->query_5.has_corner_compensation)
		offset++;

	/* control 11 */
	if (f55->query.curve_compensation_mode == 0x3)
		offset++;

	/* control 12 */
	if (f55->query_5.has_ctrl12)
		offset++;

	/* control 13 */
	if (f55->query_5.has_ctrl13)
		offset++;

	/* control 14 */
	if (f55->query_5.has_ctrl14)
		offset++;

	/* control 15 */
	if (f55->query_5.has_basis_function)
		offset++;

	/* control 16 */
	if (f55->query_17.has_ctrl16)
		offset++;

	/* control 17 */
	if (f55->query_17.has_ctrl17)
		offset++;

	/* controls 18 19 */
	if (f55->query_17.has_ctrl18_ctrl19)
		offset += 2;

	/* control 20 */
	if (f55->query_17.has_ctrl20)
		offset++;

	/* control 21 */
	if (f55->query_17.has_ctrl21)
		offset++;

	/* control 22 */
	if (f55->query_17.has_ctrl22)
		offset++;

	/* control 23 */
	if (f55->query_18.has_ctrl23)
		offset++;

	/* control 24 */
	if (f55->query_18.has_ctrl24)
		offset++;

	/* control 25 */
	if (f55->query_18.has_ctrl25)
		offset++;

	/* control 26 */
	if (f55->query_18.has_ctrl26)
		offset++;

	/* control 27 */
	if (f55->query_18.has_ctrl27_query20)
		offset++;

	/* control 28 */
	if (f55->query_18.has_ctrl28_query21)
		offset++;

	/* control 29 */
	if (f55->query_22.has_ctrl29)
		offset++;

	/* control 30 */
	if (f55->query_22.has_ctrl30)
		offset++;

	/* control 31 */
	if (f55->query_22.has_ctrl31)
		offset++;

	/* control 32 */
	if (f55->query_22.has_ctrl32)
		offset++;

	/* controls 33 34 35 36 reserved */

	/* control 37 */
	if (f55->query_28.has_ctrl37)
		offset++;

	/* control 38 */
	if (f55->query_30.has_ctrl38)
		offset++;

	/* control 39 */
	if (f55->query_30.has_ctrl39)
		offset++;

	/* control 40 */
	if (f55->query_30.has_ctrl40)
		offset++;

	/* control 41 */
	if (f55->query_30.has_ctrl41)
		offset++;

	/* control 42 */
	if (f55->query_30.has_ctrl42)
		offset++;

	/* controls 43 44 */
	if (f55->query_30.has_ctrl43_ctrl44) {
		f55->afe_mux_offset = offset;
		offset += 2;
	}

	/* controls 45 46 */
	if (f55->query_33.has_ctrl45_ctrl46) {
		f55->has_force = true;
		f55->force_tx_offset = offset;
		f55->force_rx_offset = offset + 1;
		offset += 2;
	}

	return 0;
}

static int test_f55_set_queries(void)
{
	int retval;
	unsigned char offset;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f55->query_base_addr,
			f55->query.data,
			sizeof(f55->query.data));
	if (retval < 0)
		return retval;

	offset = sizeof(f55->query.data);

	/* query 3 */
	if (f55->query.has_single_layer_multi_touch) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_3.data,
				sizeof(f55->query_3.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 4 */
	if (f55->query_3.has_ctrl9)
		offset += 1;

	/* query 5 */
	if (f55->query.has_query5) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_5.data,
				sizeof(f55->query_5.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 6 7 */
	if (f55->query.curve_compensation_mode == 0x3)
		offset += 2;

	/* query 8 */
	if (f55->query_3.has_ctrl8)
		offset += 1;

	/* query 9 */
	if (f55->query_3.has_query9)
		offset += 1;

	/* queries 10 11 12 13 14 15 16 */
	if (f55->query_5.has_basis_function)
		offset += 7;

	/* query 17 */
	if (f55->query_5.has_query17) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_17.data,
				sizeof(f55->query_17.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 18 */
	if (f55->query_17.has_query18) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_18.data,
				sizeof(f55->query_18.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 19 */
	if (f55->query_18.has_query19)
		offset += 1;

	/* query 20 */
	if (f55->query_18.has_ctrl27_query20)
		offset += 1;

	/* query 21 */
	if (f55->query_18.has_ctrl28_query21)
		offset += 1;

	/* query 22 */
	if (f55->query_18.has_query22) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_22.data,
				sizeof(f55->query_22.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 23 */
	if (f55->query_22.has_query23) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_23.data,
				sizeof(f55->query_23.data));
		if (retval < 0)
			return retval;
		offset += 1;

		f55->amp_sensor = f55->query_23.amp_sensor_enabled;
		f55->size_of_column2mux = f55->query_23.size_of_column2mux;
	}

	/* queries 24 25 26 27 reserved */

	/* query 28 */
	if (f55->query_22.has_query28) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_28.data,
				sizeof(f55->query_28.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 29 */
	if (f55->query_28.has_query29)
		offset += 1;

	/* query 30 */
	if (f55->query_28.has_query30) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_30.data,
				sizeof(f55->query_30.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* queries 31 32 */
	if (f55->query_30.has_query31_query32)
		offset += 2;

	/* query 33 */
	if (f55->query_30.has_query33) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->query_base_addr + offset,
				f55->query_33.data,
				sizeof(f55->query_33.data));
		if (retval < 0)
			return retval;
		offset += 1;

		f55->extended_amp = f55->query_33.has_extended_amp_pad;
	}

	return 0;
}

static void synaptics_rmi5_f55_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char rx_electrodes;
	unsigned char tx_electrodes;
	struct f55_control_43 ctrl_43;

	retval = test_f55_set_queries();
	if (retval < 0) {
		TP_LOGE("Failed to read F55 query registers\n");
		return;
	}

	if (!f55->query.has_sensor_assignment)
		return;

	retval = test_f55_set_controls();
	if (retval < 0) {
		TP_LOGE("Failed to set up F55 control registers\n");
		return;
	}

	tx_electrodes = f55->query.num_of_tx_electrodes;
	rx_electrodes = f55->query.num_of_rx_electrodes;

	f55->tx_assignment = kzalloc(tx_electrodes, GFP_KERNEL);
	f55->rx_assignment = kzalloc(rx_electrodes, GFP_KERNEL);

	retval = f54->fn_ptr->read(rmi4_data,
			f55->control_base_addr + SENSOR_TX_MAPPING_OFFSET,
			f55->tx_assignment,
			tx_electrodes);
	if (retval < 0) {
		TP_LOGE("Failed to read F55 tx assignment\n");
		return;
	}

	retval = f54->fn_ptr->read(rmi4_data,
			f55->control_base_addr + SENSOR_RX_MAPPING_OFFSET,
			f55->rx_assignment,
			rx_electrodes);
	if (retval < 0) {
		TP_LOGE("Failed to read F55 rx assignment\n");
		return;
	}

	f54->tx_assigned = 0;
	for (ii = 0; ii < tx_electrodes; ii++) {
		if (f55->tx_assignment[ii] != 0xff)
			f54->tx_assigned++;
	}

	f54->rx_assigned = 0;
	for (ii = 0; ii < rx_electrodes; ii++) {
		if (f55->rx_assignment[ii] != 0xff)
			f54->rx_assigned++;
	}

	if (f55->amp_sensor) {
		f54->tx_assigned = f55->size_of_column2mux;
		f54->rx_assigned /= 2;
	}

	if (f55->extended_amp) {
		retval = f54->fn_ptr->read(rmi4_data,
				f55->control_base_addr + f55->afe_mux_offset,
				ctrl_43.data,
				sizeof(ctrl_43.data));
		if (retval < 0) {
			TP_LOGE("Failed to read F55 AFE mux sizes\n");
			return;
		}

		f54->tx_assigned = ctrl_43.afe_l_mux_size +
				ctrl_43.afe_r_mux_size;
		/* tddi f54 test reporting +  */
		f54->swap_sensor_side = ctrl_43.swap_sensor_side;  // leon
		f54->left_mux_size = ctrl_43.afe_l_mux_size;
		f54->right_mux_size = ctrl_43.afe_r_mux_size;
		/* tddi f54 test reporting -  */
	}

	/* force mapping */
	if (f55->has_force) {
		f55->force_tx_assignment = kzalloc(tx_electrodes, GFP_KERNEL);
		f55->force_rx_assignment = kzalloc(rx_electrodes, GFP_KERNEL);

		retval = f54->fn_ptr->read(rmi4_data,
				f55->control_base_addr + f55->force_tx_offset,
				f55->force_tx_assignment,
				tx_electrodes);
		if (retval < 0) {
			TP_LOGE("Failed to read F55 force tx assignment\n");
			return;
		}

		retval = f54->fn_ptr->read(rmi4_data,
				f55->control_base_addr + f55->force_rx_offset,
				f55->force_rx_assignment,
				rx_electrodes);
		if (retval < 0) {
			TP_LOGE("Failed to read F55 force rx assignment\n");
			return;
		}

		for (ii = 0; ii < tx_electrodes; ii++) {
			if (f55->force_tx_assignment[ii] != 0xff)
				f54->tx_assigned++;
		}

		for (ii = 0; ii < rx_electrodes; ii++) {
			if (f55->force_rx_assignment[ii] != 0xff)
				f54->rx_assigned++;
		}
	}

	return;
}

static void synaptics_rmi4_f55_set_regs(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned char page)
{
	f55 = kzalloc(sizeof(*f55), GFP_KERNEL);
	if (!f55) {
		TP_LOGE("Failed to alloc mem for f55\n");
		return;
	}

	f55->query_base_addr = fd->query_base_addr | (page << 8);
	f55->control_base_addr = fd->ctrl_base_addr | (page << 8);
	f55->data_base_addr = fd->data_base_addr | (page << 8);
	f55->command_base_addr = fd->cmd_base_addr | (page << 8);

	return;
}

static int synaptics_rmi4_f54_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned short ii;
	unsigned char page;
	unsigned char intr_count = 0;
	bool f54found = false;
	bool f55found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;

	if (f54) {
		TP_LOGI("Handle already exists\n");

		return 0;
	}

	f54 = kzalloc(sizeof(*f54), GFP_KERNEL);
	if (!f54) {
		TP_LOGE("Failed to alloc mem for f54\n");
		retval = -ENOMEM;
		goto exit;
	}

	f54->fn_ptr = kzalloc(sizeof(*(f54->fn_ptr)), GFP_KERNEL);
	if (!f54->fn_ptr) {
		TP_LOGE("Failed to alloc mem for fn_ptr\n");
		retval = -ENOMEM;
		goto exit_free_f54;
	}

	f54->rmi4_data = rmi4_data;
	f54->fn_ptr->read = rmi4_data->i2c_read;
	f54->fn_ptr->write = rmi4_data->i2c_write;
	f54->fn_ptr->enable = rmi4_data->irq_enable;

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		for (ii = PDT_START; ii > PDT_END; ii -= PDT_ENTRY_SIZE) {
			ii |= (page << 8);

			retval = f54->fn_ptr->read(rmi4_data,
					ii,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				goto exit_free_mem;

			if (!rmi_fd.fn_number)
				break;

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F54:
				synaptics_rmi4_f54_set_regs(rmi4_data,
						&rmi_fd, intr_count, page);
				f54found = true;
				break;
			case SYNAPTICS_RMI4_F55:
				synaptics_rmi4_f55_set_regs(rmi4_data,
						&rmi_fd, page);
				f55found = true;
				break;
			default:
				break;
			}

			if (f54found && f55found)
				goto pdt_done;

			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
		}
	}

	if (!f54found) {
		retval = -ENODEV;
		goto exit_free_mem;
	}

pdt_done:
	retval = test_set_queries();
	if (retval < 0) {
		TP_LOGE("Failed to read f54 query registers\n");
		goto exit_free_mem;
	}

	f54->rx_assigned = f54->query.num_of_rx_electrodes;
	f54->tx_assigned = f54->query.num_of_tx_electrodes;

	retval = synaptics_rmi4_f54_set_ctrl();
	if (retval < 0) {
		TP_LOGE("Failed to set up f54 control registers\n");
		goto exit_free_control;
	}

	test_set_data();

	if (f55found)
		synaptics_rmi5_f55_init(rmi4_data);

	mutex_init(&f54->status_mutex);
	mutex_init(&f54->data_mutex);
	mutex_init(&f54->control_mutex);

	retval = synaptics_rmi4_f54_set_sysfs();
	if (retval < 0) {
		TP_LOGE("Failed to create sysfs entries\n");
		goto exit_sysfs;
	}

	f54->status_workqueue =
			create_singlethread_workqueue("f54_status_workqueue");
	INIT_DELAYED_WORK(&f54->status_work,
			synaptics_rmi4_f54_status_work);

#ifdef WATCHDOG_HRTIMER
	/* Watchdog timer to catch unanswered get report commands */
	hrtimer_init(&f54->watchdog, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	f54->watchdog.function = get_report_timeout;

	/* Work function to do actual cleaning up */
	INIT_WORK(&f54->timeout_work, timeout_set_status);
#endif

	f54->status = STATUS_IDLE;

	return 0;

exit_sysfs:
	if (f55) {
		kfree(f55->tx_assignment);
		kfree(f55->rx_assignment);
		kfree(f55->force_tx_assignment);
		kfree(f55->force_rx_assignment);
	}

exit_free_control:
	free_control_mem();

exit_free_mem:
	kfree(f55);
	f55 = NULL;
	kfree(f54->fn_ptr);

exit_free_f54:
	kfree(f54);
	f54 = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_f54_remove(struct synaptics_rmi4_data *rmi4_data)
{
	if (!f54)
		goto exit;

#ifdef WATCHDOG_HRTIMER
	hrtimer_cancel(&f54->watchdog);
#endif

	cancel_delayed_work_sync(&f54->status_work);
	flush_workqueue(f54->status_workqueue);
	destroy_workqueue(f54->status_workqueue);

	remove_sysfs();

	if (f55) {
		kfree(f55->tx_assignment);
		kfree(f55->rx_assignment);
		kfree(f55->force_tx_assignment);
		kfree(f55->force_rx_assignment);
	}

	free_control_mem();

	kfree(f55);
	f55 = NULL;

	if (f54->data_buffer_size)
		kfree(f54->report_data);

	kfree(f54->fn_ptr);
	kfree(f54);
	f54 = NULL;

exit:
	complete(&f54_remove_complete);

	return;
}

static void synaptics_rmi4_f54_reset(struct synaptics_rmi4_data *rmi4_data)
{
	if (!f54)
		return;

	synaptics_rmi4_f54_remove(rmi4_data);
	synaptics_rmi4_f54_init(rmi4_data);

	return;
}

static struct synaptics_rmi4_exp_fn f54_module = {
	.fn_type = RMI_F54,
	.init = synaptics_rmi4_f54_init,
	.remove = synaptics_rmi4_f54_remove,
	.reset = synaptics_rmi4_f54_reset,
	.reinit = NULL,
	.early_suspend = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = synaptics_rmi4_f54_attn,
};

static int __init rmi4_f54_module_init(void)
{
	synaptics_rmi4_new_function(&f54_module, true);

	return 0;
}

static void __exit rmi4_f54_module_exit(void)
{
	synaptics_rmi4_new_function(&f54_module, false);

	wait_for_completion(&f54_remove_complete);

	return;
}

module_init(rmi4_f54_module_init);
module_exit(rmi4_f54_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX Test Reporting Module");
MODULE_LICENSE("GPL v2");
