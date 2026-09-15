/*
 * aw9523b.h   aw9523b martix key
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 *  Author: <shiqiang@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */


#ifndef _AW9523B_H_
#define _AW9523B_H_

#define AW9523B_ID 0x23
#define AW9523B_KEY_PORT_MAX (0x10) /* 16 */
#define AW9523B_LED_MAX_NUMS (0x10) /* 16 */
#define AW9523B_INT_MASK (0xFFFF)

#define AWINIC_DEBUG		1

#ifdef AWINIC_DEBUG
#define AW_DEBUG(fmt, args...)	pr_info(fmt, ##args)
#else
#define AW_DEBUG(fmt, ...)
#endif

struct aw9523b_key;
struct aw9523b_led;
struct aw9523b_gpio;

struct aw9523b {
	struct i2c_client *client;
	struct device *dev;
	int irq_gpio;
	int irq_num;
	int rst_gpio;
	struct regulator *power_supply;
	unsigned char chipid;
	bool matrix_key_enable;
	bool single_key_enable;
	bool led_feature_enable;
	bool gpio_feature_enable;
	bool screen_state;
	struct aw9523b_key *key_data;
	struct aw9523b_led *led_data;
	struct aw9523b_gpio *gpio_data;

	//msx-add FB-suspend
    //struct notifier_block fb_notif;
	struct notifier_block disp_notifier;
	//end
};

typedef struct {
	char name[10];
	int key_code;
	int key_val;
} KEY_STATE;

unsigned int aw9523b_separate_key_data[AW9523B_KEY_PORT_MAX] = {
/*      0    1    2    3 */
	1,   2,   3,   4,
	5,   6,   7,   8,
	9,   10,  11,  12,
	13,  14,  15,  16
};


struct aw9523b_key {
	unsigned int key_mask;
	
	//kcm added by phf, 20250123
	unsigned int port_dir_config;
	unsigned int old_key_row_status;	//output port
	unsigned int old_key_col_status;	// input port
	
	unsigned int input_port_nums;
	unsigned int output_port_nums;
	unsigned int input_port_mask;
	unsigned int output_port_mask;
	unsigned int new_input_state;
	unsigned int old_input_state;
	unsigned int *new_output_state;
	unsigned int *old_output_state;
	unsigned int *def_output_state;
	bool wake_up_enable;
	struct input_dev *input;

//kcm added removed phf, 20250123
//	unsigned int debounce_delay;
//	struct delayed_work int_work;
	
	struct hrtimer key_timer;
	struct work_struct key_work;
	KEY_STATE *keymap;
	int keymap_len;
	struct aw9523b *priv;
};

struct aw9523b_single_led {
	struct led_classdev cdev;
	struct work_struct brightness_work;
	unsigned int idx;
	unsigned int cur_brightness;
	unsigned int max_brightness;
	struct aw9523b *priv;
};

struct aw9523b_led {
	unsigned int led_mask;
	int imax;
	int led_nums;
	struct aw9523b_single_led *single_led_data;
};

enum aw9523b_gpio_dir {
	AW9523B_GPIO_INPUT = 0,
	AW9523B_GPIO_OUTPUT = 1,
};

enum aw9523b_gpio_val {
	AW9523B_GPIO_HIGH = 1,
	AW9523B_GPIO_LOW = 0,
};

enum aw9523b_gpio_output_mode {
	AW9523B_OPEN_DRAIN_OUTPUT = 0,
	AW9523B_PUSH_PULL_OUTPUT = 1,
};

struct aw9523b_singel_gpio {
	unsigned int gpio_idx;
	enum aw9523b_gpio_dir gpio_direction;
	enum aw9523b_gpio_val state;
	struct aw9523b *priv;
};


struct aw9523b_gpio {
	unsigned int gpio_mask;
	unsigned int gpio_num;
	enum aw9523b_gpio_output_mode output_mode;
	struct aw9523b_singel_gpio *single_gpio_data;
};
#endif
