/*
 * Copyright (C) 2023 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __PAD_DEBUG_H
#define __PAD_DEBUG_H

/* Debug event types */
#define PAD_TYPE_INT_DOWN     0
#define PAD_TYPE_INT_UP       1
#define PAD_TYPE_TIMER        2
#define PAD_TYPE_RAW_DATA     3
#define PAD_TYPE_REJECT1      4
#define PAD_TYPE_REJECT2      5
#define PAD_TYPE_FIST_LATENCY 6

#define PAD_DEBUG_CODE    /* Enable all debug functions */

extern int pad_debug_no_response(int enable);
extern void pad_debug_set_time(int time);
extern void pad_em_log_output(int raw_x, int raw_y, int cal_x, int cal_y, int p, int down);
extern void pad_em_log_store(int raw_x, int raw_y, int cal_x, int cal_y, int p, int down);
extern void pad_enable_em_log(int enable);

extern int pad_em_debounce_time;
extern int pad_em_debounce_time0;
extern int pad_em_debounce_time1;
extern int pad_em_asamp;
extern int pad_em_auto_time_interval;
extern int pad_em_sample_cnt;
extern int pad_em_pressure_threshold;
extern int pad_em_spl_num;
extern int pad_em_log;
extern struct mutex pad_em_mutex;

/* Struct to store event data for debugging */
struct pad_em_data {
	int down_status;
	unsigned long time_stamp;
	int x_position;
	int y_position;
	int pressure;
	int finger_num;
	int pre_finger_num;
	int pre_x[10];
	int pre_y[10];
	int pre_pressure[10];
};

extern int pad_debug_time;
extern long pad_debug_last_2_int_time[2];
extern int pad_down_status;

#endif /* __PAD_DEBUG_H */