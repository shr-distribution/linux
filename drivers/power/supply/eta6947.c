/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros*/
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
//#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>
#include <linux/irq.h>

#include <linux/errno.h>
#include <linux/interrupt.h>

#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

//#include <mt-plat/upmu_common.h>
#include "charger_class.h"
#include "eta6947.h"
//#include "mtk_charger_intf.h"

static DEFINE_MUTEX(eta6937_i2c_access);
static DEFINE_MUTEX(eta6937_access_lock);


#define ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
#ifdef ETA6937_TIMER_DEBUG 
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kthread.h>

wait_queue_head_t  eta6937_otg_wait_que;
struct hrtimer eta6937_otg_kthread_timer;
int otg_set_tmr_flag =1;
int eta6937_otg_status_flag =0;

#if (defined(CONFIG_KTE_DRV_P30_8183_DOCKING) && defined(CONFIG_KTE_DRV_P30_8183_OTG)) || defined(CONFIG_KTE_DRV_P30_8183_CHRDC)
static struct pinctrl *otgvbus;
static struct pinctrl_state *otg_high;
static struct pinctrl_state *otg_low;
static struct pinctrl_state *dc_en_high;
static struct pinctrl_state *dc_en_low;
void xiaohua_kte_drv_otg_on(void);
void xiaohua_kte_drv_otg_off(void);
#endif

extern void eta6937_set_tmr_rst(unsigned int val);

void _wake_up_eta6937_otg(void)
{
	otg_set_tmr_flag =1;
	wake_up(&eta6937_otg_wait_que);
}

enum hrtimer_restart otg_rst_timer_func(struct hrtimer *timer)
{
	_wake_up_eta6937_otg();
	return HRTIMER_NORESTART;
}

void eta6937_otg_init_timer(void)
{
	ktime_t ktime = ktime_set(10, 0);
	hrtimer_init(&eta6937_otg_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	eta6937_otg_kthread_timer.function = otg_rst_timer_func;
	hrtimer_start(&eta6937_otg_kthread_timer, ktime, HRTIMER_MODE_REL);
}

void eta6937_otg_start_timer(void)
{

	ktime_t ktime = ktime_set(10, 0);
	hrtimer_start(&eta6937_otg_kthread_timer, ktime, HRTIMER_MODE_REL);
}

int eta6937_otg_routine_thread(void *arg)
{
	while (1) {
		wait_event(eta6937_otg_wait_que, (otg_set_tmr_flag == 1));
		otg_set_tmr_flag =0;
		if(eta6937_otg_status_flag ==1)
		{
			eta6937_set_tmr_rst(1);
			eta6937_otg_start_timer();		
		}
	}
	
	return 0;
}
#endif

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/


/*eta6937 VIINDPM*/
const u32 eta6937_VINDPM[]={
	4200000,4280000,4360000,4440000,
	4520000,4600000,4680000,4760000,
	4840000,4920000,5000000,5080000,
	5160000,5240000,5320000,5400000,
	5480000,5560000,5640000,5720000,
	5800000,5880000,5960000,6040000,
	6120000,6200000,6280000,6360000,
	6440000,6520000,6600000,6680000,
	6760000,6840000,6920000,7000000,
	7080000,7160000,7240000,7320000,
	7400000,7480000,7560000,7640000,
	7720000,7800000,7880000,7960000,
	8040000,8120000,8200000,8280000,
	8360000,8440000,8520000,8600000,
	8680000,8760000,8840000,8920000,
	9000000,9080000,9160000,9240000,
	9320000,9400000,9480000,9560000,
	9640000,9720000,9800000,9880000,
	9960000,10040000,10120000,10200000,
	10280000,10360000,10440000,10520000,
	10600000,10680000,10760000,10840000,
	10920000,11000000,11080000,11160000,
	11240000,11320000,11400000,11480000,
	11560000,11640000,11720000,11800000,
	11880000,11960000,12040000,12120000,
	12200000,12280000,12360000,12440000,
	12520000,12600000,12680000,12760000,
	12840000,12920000,13000000,13080000,
	13160000,13240000,13320000,13400000,
	13480000,13560000,13640000,13720000,
	13800000,13880000,13960000,14040000,
	14120000,14200000,14280000,14360000,
};

/*eta6937 REG06 VREG[5:0]*/
const u32 eta6937_VBAT_CV_VTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000,4000000, 4020000, 4040000, 
	4060000,4080000, 4100000, 4120000,
	4140000,4160000, 4180000, 4200000, 
	4220000,4240000, 4260000, 4280000,
	4300000,4320000, 4340000, 4360000,
	4380000,4400000, 4420000, 4440000
};

/*eta6937 REG04 ICHG[6:0]*/
const u32 eta6937_CS_VTH[] = {
	55000, 65000, 75000, 85000,
	95000, 105000, 115000, 125000,
	135000, 145000, 155000, 165000,
	175000, 185000, 195000, 205000,
	215000, 225000, 235000
};

/*eta6937 REG00 IINLIM[5:0]*/
const u32 eta6937_INPUT_CS_VTH[] = {
	10000, 50000, 80000,200000
};


#ifdef CONFIG_OF
#else

#define eta6937_SLAVE_ADDR_WRITE   0xD4
#define eta6937_SLAVE_ADDR_Read    0xD5

#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define eta6937_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define eta6937_BUSNUM 0
#endif

#endif

struct eta6937_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	//enum charger_type chg_type;
	u32 intr_gpio;
	u32 en_gpio;
	atomic_t is_chip_en;
	int irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *psc_chg_en_low;
	struct pinctrl_state *psc_chg_en_high;
} *g_eta6937_info;;

static const struct charger_properties eta6937_chg_props = {
	.alias_name = "eta6937-user",
};

static unsigned int g_input_current;
DEFINE_MUTEX(g_input_current_mutex);
static struct i2c_client *new_client;
static const struct i2c_device_id eta6937_i2c_id[] = { {"eta6937", 0}, {} };

static int eta6937_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	if (val < array_size)
		return parameter[val];

		pr_info("Can't find the parameter\n");
		return parameter[0];

}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	unsigned int i;

	pr_debug_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	printk("NO register value match\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				pr_debug_ratelimited("zzf_%d<=%d, i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}

		printk("Can't find closest level\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}

		printk("Can't find closest level\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}


/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char eta6937_reg[eta6937_REG_NUM] = { 0 };

int g_eta6937_hw_exist;

#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int eta6937_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&eta6937_i2c_access);

	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		new_client->ext_flag = 0;

		mutex_unlock(&eta6937_i2c_access);
		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	new_client->ext_flag = 0;

	mutex_unlock(&eta6937_i2c_access);
	return 1;
}

unsigned int eta6937_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&eta6937_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {

		new_client->ext_flag = 0;
		mutex_unlock(&eta6937_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&eta6937_i2c_access);
	return 1;
}
#else
unsigned int eta6937_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&eta6937_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			printk("skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&eta6937_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int eta6937_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&eta6937_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			printk("skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&eta6937_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int eta6937_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char eta6937_reg = 0;
	unsigned int ret = 0;

	ret = eta6937_read_byte(RegNum, &eta6937_reg);

	printk("[eta6937_read_interface] Reg[%x]=0x%x\n", RegNum, eta6937_reg);

	eta6937_reg &= (MASK << SHIFT);
	*val = (eta6937_reg >> SHIFT);

	printk("[eta6937_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int eta6937_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char eta6937_reg = 0;
	unsigned char eta6937_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&eta6937_access_lock);

	ret = eta6937_read_byte(RegNum, &eta6937_reg);

	eta6937_reg_ori = eta6937_reg;
	eta6937_reg &= ~(MASK << SHIFT);
	eta6937_reg |= (val << SHIFT);

	ret = eta6937_write_byte(RegNum, eta6937_reg);

	mutex_unlock(&eta6937_access_lock);
	printk("[eta6937_config_interface] write Reg[%x]=0x%x from 0x%x\n", RegNum,
		    eta6937_reg, eta6937_reg_ori);

	return ret;
}

/* write one register directly */
unsigned int eta6937_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = eta6937_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void eta6937_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	printk("eta6937_set_tmr_rst enter\n");

	ret = eta6937_config_interface((unsigned char) (eta6937_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int eta6937_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void eta6937_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
}

unsigned int eta6937_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

static int eta6937_get_charging_status(struct charger_device *chg_dev,
				bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;

	ret_val = eta6937_get_chip_status();

	if (ret_val == 0x2)
		*is_done = true;
	else
		*is_done = false;
	pr_info("eta6937_get_charging_status:ret_val = 0x%x,*is_done = %d\n", ret_val, *is_done);
	return status;
}

unsigned int eta6937_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void eta6937_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void eta6937_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void eta6937_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void eta6937_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

void eta6937_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
}

void eta6937_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void eta6937_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void eta6937_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void eta6937_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

unsigned int eta6937_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void eta6937_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
	 eta6937_set_i_safe(ISAFE);
	 eta6937_set_v_safe(VSAFE);   
	    
}

void eta6937_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;
	ret = eta6937_config_interface((unsigned char) (eta6937_CON4),
				       (unsigned char) (val%8),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
	ret = eta6937_config_interface((unsigned char) (eta6937_CON5),
				       (unsigned char) (val/8),
				       (unsigned char) (CON5_I_CHR_MASK),
				       (unsigned char) (CON5_I_CHR_SHIFT)
	    );	
}

void eta6937_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

void eta6937_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int eta6937_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int eta6937_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface((unsigned char) (eta6937_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}

void eta6937_set_vsp(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_VSP_MASK),
				       (unsigned char) (CON5_VSP_SHIFT)
	    );
}

/* CON6 */

void eta6937_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void eta6937_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VSAFE_MASK),
				       (unsigned char) (CON6_VSAFE_SHIFT)
	    );
}

/* CON7 */

void eta6937_en_ilim2(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_ILIM2_MASK),
				       (unsigned char) (CON7_EN_ILIM2_SHIFT)
	    );
}

void eta6937_set_iin_limit2(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6937_config_interface((unsigned char) (eta6937_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_IIN_LIMIT_2_MASK),
				       (unsigned char) (CON7_IIN_LIMIT_2_SHIFT)
	    );
}
/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
static void eta6937_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6937_read_interface(0x03, &val, 0xFF, 0x0);
	
	if (val == 0x51 || val == 0x52 || val == 0x53 || val == 0x54 || val == 0x37)
		g_eta6937_hw_exist = 1;
	else
		g_eta6937_hw_exist = 0;

	printk("[eta6937_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n",
		 g_eta6937_hw_exist, val);
}

static int eta6937_enable_charging(struct charger_device *chg_dev, bool en)
{
//	int ret;
	int status = 0;
//	struct eta6937_info *info = g_eta6937_info;
	
	if (en) {
		eta6937_set_ce(0);
		eta6937_set_hz_mode(0);
		eta6937_set_opa_mode(0);
		eta6937_set_i_safe(ISAFE);
	 	eta6937_set_v_safe(VSAFE); 
		eta6937_set_ce(0);
		eta6937_set_te(1);
		//eta6937_set_iterm(2);//150ma iterm
		eta6937_set_tmr_rst(1);
		eta6937_set_vsp(2);
		eta6937_en_ilim2(1);
		eta6937_set_iin_limit2(5);
//		ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_low);
//		if (ret){
//			printk("mycat Error pinctrl_select_state low");
//		}
//		else{
//			printk("mycat Ok pinctrl_select_state low");
//		}
		printk("eta6937 calm enable charging\n");
	} else {
		eta6937_set_ce(1);
		eta6937_set_hz_mode(1);
		//disable charging: psc_chg_en_high
//		ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_high);
//		if (ret){
//			printk("mycat Error pinctrl_select_state high");
//		}
//		else{
//			printk("mycat Ok pinctrl_select_state high");
//		}
		printk("eta6937 calm disable charging\n");
	}

	return status;
}

static int eta6937_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	u32 status = 0;
	u32 array_size;
	u8 reg_value;
	/* Get current level */
	array_size = ARRAY_SIZE(eta6937_CS_VTH);
	eta6937_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*(u32 *) ichg = charging_value_to_parameter(eta6937_CS_VTH, array_size, reg_value);

	return status;
}

static int eta6937_set_current(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = 0;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;
printk("liml_bat eta6937_set_current=%d\n",current_value);
	current_value /= 10;
	if (current_value <= 35000) {
		eta6937_set_io_level(1);
	} else {
		eta6937_set_io_level(0);
		array_size = ARRAY_SIZE(eta6937_CS_VTH);
		set_chr_current = bmt_find_closest_level(eta6937_CS_VTH, array_size, current_value);

	printk("charging_set_current  set_chr_current=%d\n", set_chr_current);

		register_value = charging_parameter_to_value(eta6937_CS_VTH, array_size, set_chr_current);
	printk("charging_set_current  register_value=%d\n", register_value);
		eta6937_set_iocharge(register_value);
	}
	return status;
}
static int eta6937_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	int ret = 0;

	*aicr = g_input_current;

	return ret;
}

static int eta6937_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = 0;
	u32 set_chr_current = 0;
	u32 array_size;
	u32 register_value;

	printk("liml_bat eta6937_set_input_current=%d\n",current_value);
	mutex_lock(&g_input_current_mutex);
	current_value /= 10;
	if (current_value > 80000) {
		register_value = 0x3;
	} else {
		array_size = ARRAY_SIZE(eta6937_INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(eta6937_INPUT_CS_VTH, array_size, current_value);
		register_value = charging_parameter_to_value(eta6937_INPUT_CS_VTH, array_size, set_chr_current);
	}
	g_input_current = set_chr_current;
	eta6937_set_input_charging_current(register_value);
	
	mutex_unlock(&g_input_current_mutex);
	return status;
}

static int eta6937_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	u32 status = 0;
	u32 register_value;
	u32 array_size;
	u32 set_cv_voltage;
	
	//cv voltage +0.04V, kcm modify by phf, 20211124
	cv += 40000;
	
	array_size = ARRAY_SIZE(eta6937_VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(eta6937_VBAT_CV_VTH, array_size, cv);
	register_value = charging_parameter_to_value(eta6937_VBAT_CV_VTH, ARRAY_SIZE(eta6937_VBAT_CV_VTH), set_cv_voltage);
	printk("eta6937_set_cv_voltage  register_value=%d,set_cv_voltage=%d\n", register_value,set_cv_voltage);
	eta6937_set_oreg(register_value);

	return status;
}



static int eta6937_is_charging_enabled (struct charger_device *chg_dev, bool *en)
{
	int ret=0;
	unsigned char val=0;
	printk("%s\n", __func__);
	ret = eta6937_read_interface((unsigned char) (eta6937_CON1),
				       //(unsigned char) (&val),
					   (&val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT));
	if (ret < 0) {
		*en = false;
		return ret;
	}
	*en = (val == 0 ? false : true);
	return ret ;

}

static int eta6937_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	printk("%s\n", __func__);
	ret = gpio_get_value(g_eta6937_info->en_gpio);
	if(ret){
		*en = 0;	
	}else{
		*en = 1;
	}
	if ((*en && !atomic_read(&g_eta6937_info->is_chip_en)) ||
		(!*en && atomic_read(&g_eta6937_info->is_chip_en)))
		printk("%s: en not sync(%d, %d)\n", __func__, *en,
			atomic_read(&g_eta6937_info->is_chip_en));
	return 0;
}

static int eta6937_enable_chip(struct charger_device *chg_dev, bool en)
{
	bool is_chip_en = false;
	printk("%s\n", __func__);
	
	is_chip_en = gpio_get_value(g_eta6937_info->en_gpio);
	if (en && !is_chip_en) {
		gpio_set_value(g_eta6937_info->en_gpio, 0);
		printk("%s: set gpio high\n", __func__);
	} else if (!en && is_chip_en) {
		gpio_set_value(g_eta6937_info->en_gpio, 1);
		printk("%s: set gpio low\n", __func__);
	}

	/* Wait for chip's enable/disable */
	mdelay(1);
	atomic_set(&g_eta6937_info->is_chip_en, en);
	return 0;
}

#if (defined(CONFIG_KTE_DRV_P30_8183_DOCKING) && defined(CONFIG_KTE_DRV_P30_8183_OTG)) || defined(CONFIG_KTE_DRV_P30_8183_CHRDC)
void xiaohua_kte_drv_otg_on(void)
{
	pinctrl_select_state(otgvbus, otg_high);
	pinctrl_select_state(otgvbus, dc_en_high);
	printk("---xiaohua otg_dc xiaohua_kte_drv_otg_on\n");
	printk("%s otg = %d dc_en = %d\n", __func__, otg_high, dc_en_low);		
}
void xiaohua_kte_drv_otg_off(void)
{
	pinctrl_select_state(otgvbus, otg_low);
	pinctrl_select_state(otgvbus, dc_en_low);
	printk("---xiaohua otg_dc xiaohua_kte_drv_otg_off\n");
	printk("%s otg = %d dc_en = %d\n", __func__, otg_low, dc_en_low);	
}
#endif

#if defined(CONFIG_KTE_DRV_P30_8183_DOCKING)
extern volatile int cur_docking_status;
#endif

#if 0	
static int eta6937_enable_otg(struct charger_device *chg_dev, bool en)
{
#ifdef ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
	eta6937_otg_status_flag = en;
	if(en == 1)
		eta6937_otg_start_timer();
#endif
	#if (defined(CONFIG_KTE_DRV_P30_8183_DOCKING) && defined(CONFIG_KTE_DRV_P30_8183_OTG)) || defined(CONFIG_KTE_DRV_P30_8183_CHRDC)
	
	#if defined(CONFIG_KTE_DRV_P30_8183_DOCKING)
	if (cur_docking_status){
		return 0;
	}
	#endif
	
	if(en)
		xiaohua_kte_drv_otg_on();
	else
		xiaohua_kte_drv_otg_off();
	#elif defined(CONFIG_KTE_DRV_P30_8183_DOCKING)
		if (cur_docking_status){
			eta6937_set_otg_en(0);
			printk("---xiaohua cur_docking_status vbus off\n");
			return 0;
		}
		printk("---xiaohua cur_docking_status vbus en=%d\n",en);
		eta6937_set_otg_en(en);
	#else
		eta6937_set_otg_en(en);
	#endif
	return 0;
}

static int eta6937_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	printk("%s: event = %d\n", __func__, event);
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}


static int eta6937_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	int ret = 0;
	unsigned char val;
	
	if (chg_dev == NULL)
		return -EINVAL;
		
	ret = eta6937_read_interface((unsigned char) (eta6937_CON0),
				(&val), (unsigned char) (CON0_STAT_MASK),(unsigned char) (CON0_STAT_SHIFT));
				
	printk("%s: val = %d\n", __func__, val);
	switch (val)
	 {
	case 2:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case 1:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int eta6937_dump_register(struct charger_device *chg_dev)
{
	//int i = 0;
	/*
	for (i = 0; i <= eta6937_REG_NUM; i++) {
		eta6937_read_byte(i, &eta6937_reg[i]);
		printk("[0x%x]=0x%x ", i, eta6937_reg[i]);
	}
	*/
	eta6937_set_tmr_rst(1);
	return 0;
}

static int eta6937_parse_dt(struct eta6937_info *info, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	pr_info("%s\n", __func__);

	if (!np) {
		printk("%s: no of node\n", __func__);
		return -ENODEV;
	}


	if (of_property_read_string(np, "charger_name", &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_warn("%s: no charger name\n", __func__);
	}

#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "rt,en_gpio", 0);
	if (ret < 0){
		printk("%s: get en_gpio failed\n", __func__);
		return ret;
	}
	info->en_gpio = ret;
#else
	ret = of_property_read_u32(np, "rt,en_gpio_num", &info->en_gpio);
	if (ret < 0){
		printk("%s: get en_gpio_num failed\n", __func__);
		return ret;
	}
#endif /* !CONFIG_MTK_GPIO || CONFIG_MTK_GPIOLIB_STAND */
	
	ret = devm_gpio_request_one(info->dev, info->en_gpio, GPIOF_DIR_OUT,"eta6937_en_gpio");
	if (ret < 0) {
		printk("%s: en gpio request fail\n", __func__);
		return ret;
	}
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "rt,intr_gpio", 0);
	if (ret < 0){
		printk("%s: get intr_gpio failed\n", __func__);
		return ret;
	}
	info->intr_gpio = ret;
#else
	ret = of_property_read_u32(np, "rt,intr_gpio_num", &info->intr_gpio);
	if (ret < 0){
		printk("%s: get intr_gpio_num failed\n", __func__);
		return ret;
	}
#endif

	return 0;
}
static int eta6937_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 550000;
	return 0;
}


static int eta6937_kick_wdt(struct charger_device *chg_dev)
{
	int ret = 0;
	ret = eta6937_config_interface(eta6937_CON0,CON0_TMR_RST_MASK,CON0_TMR_RST_MASK, CON0_TMR_RST_SHIFT);
	
	if (ret < 0)
			printk("%s: enable wdt failed\n",__func__);
	return ret;
	printk("%s: enable wdt success!!\n",__func__);
	return ret;
}

static int eta6937_get_mivr(struct charger_device *chg_dev, u32 *uv)
{
	u32 status = 0;
	u32 array_size;
	u8 reg_value;
	u8 reg_value1;
	u8 reg_value2;
	/* Get current level */
	array_size = ARRAY_SIZE(eta6937_VINDPM);
	eta6937_read_interface(eta6937_CON7, &reg_value1, CON7_VINDPM_MASK, CON7_VINDPM_SHIFT);	/* high bits */
	eta6937_read_interface(eta6937_CON5, &reg_value2, CON5_VSP_MASK, CON5_VSP_SHIFT);	/* low bits */
	reg_value=(reg_value1<<3|reg_value2);
	*uv = charging_value_to_parameter(eta6937_VINDPM, array_size, reg_value);
	printk("eta6937_get_mivr  set_mivr=%d\n", uv);

	return status;
}

static int eta6937_set_mivr(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = 0;
	u32 set_mivr;
	u32 array_size;
	u32 reg_value;
	u32 ret = 0;
	array_size = ARRAY_SIZE(eta6937_VINDPM);
	set_mivr = bmt_find_closest_level(eta6937_VINDPM, array_size, current_value);
	reg_value = charging_parameter_to_value(eta6937_VINDPM, array_size, set_mivr);
	ret = eta6937_config_interface(eta6937_CON7, reg_value/8, CON7_VINDPM_MASK, CON7_VINDPM_SHIFT);//high bits 
	ret = eta6937_config_interface(eta6937_CON5, reg_value%8, CON5_VSP_MASK, CON5_VSP_SHIFT);//low bits
	printk("eta6937_set_mivr  set_mivr=%d\n", set_mivr);

	return status;
}
static struct charger_ops eta6937_chg_ops = {

	/* Normal charging */
	.enable = eta6937_enable_charging,
	.is_enabled = eta6937_is_charging_enabled,
	.is_chip_enabled = eta6937_is_chip_enabled,
	//.enable_safety_timer = eta6937_enable_safety_timer,
	.enable_chip = eta6937_enable_chip,
	.dump_registers = eta6937_dump_register,
	.is_charging_done = eta6937_get_charging_status,
	.get_charging_current = eta6937_get_current,
	.set_charging_current = eta6937_set_current,
	.get_min_charging_current = eta6937_get_min_ichg,
	.set_constant_voltage = eta6937_set_cv_voltage,
	.kick_wdt = eta6937_kick_wdt,
	.get_mivr = eta6937_get_mivr,
	.set_mivr = eta6937_set_mivr,
	//.get_mivr_state = rt9465_get_mivr_state,
	.set_input_current = eta6937_set_input_current,
	.get_input_current = eta6937_get_input_current,
	/* OTG */
	//.enable_otg = eta6937_enable_otg,
	//.event = eta6937_do_event,
};


static int eta6937_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct eta6937_info *info = NULL;

	printk("[eta6937_driver_probe]\n");
	if(client->addr != 0x6a)
		client->addr = 0x6a;
	info = devm_kzalloc(&client->dev, sizeof(struct eta6937_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;


	new_client = client;
	info->dev = &client->dev;

	/* --------------------- */
	eta6937_hw_component_detect();
	if(g_eta6937_hw_exist == 0)
	{
		printk("eta6937_driver_probe fail,can't find eta6937\n");
		return -1;
	}

	ret = eta6937_parse_dt(info, &client->dev);
	if (ret < 0)
		return ret;
	atomic_set(&info->is_chip_en, 0);

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&client->dev, info, &eta6937_chg_ops, &eta6937_chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		printk("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	/* eta6937_hw_init(); //move to charging_hw_xxx.c */

//	info->psy = power_supply_get_by_name("charger");
//	if (!info->psy) {
//		printk("%s: get power supply failed\n", __func__);
//		return -EINVAL;
//	}
	eta6937_set_iterm(0x01);
	eta6937_dump_register(info->chg_dev);
	g_eta6937_info = info;
#ifdef ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
	init_waitqueue_head(&eta6937_otg_wait_que);
	eta6937_otg_init_timer();
	kthread_run(eta6937_otg_routine_thread, 0, "eta6937_otg_thread");
#endif

	printk("eta6937_driver_probe done,ok\n");
	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_eta6937;
static ssize_t show_eta6937_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[show_eta6937_access] 0x%x\n", g_reg_value_eta6937);
	return sprintf(buf, "%u\n", g_reg_value_eta6937);
}

static ssize_t store_eta6937_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	printk("[store_eta6937_access]\n");

	if (buf != NULL && size != 0) {

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			printk(
			    "[store_eta6937_access] write eta6937 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = eta6937_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = eta6937_read_interface(reg_address, &g_reg_value_eta6937, 0xFF, 0x0);
			printk(
			    "[store_eta6937_access] read eta6937 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_eta6937);
			printk(
			    "[store_eta6937_access] Please use \"cat eta6937_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(eta6937_access, 0664, show_eta6937_access, store_eta6937_access);	/* 664 */

static int eta6937_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	printk("******** eta6937_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_eta6937_access);

	return 0;
}

struct platform_device eta6937_user_space_device = {
	.name = "eta6937-user",
	.id = -1,
};

static struct platform_driver eta6937_user_space_driver = {
	.probe = eta6937_user_space_probe,
	.driver = {
		   .name = "eta6937-user",
		   },
};

#ifdef CONFIG_OF
static const struct of_device_id eta6937_of_match[] = {
	{.compatible = "halo,eta6937"},
	{},
};
#else
static struct i2c_board_info i2c_eta6937 __initdata = {
	I2C_BOARD_INFO("eta6937", (eta6937_SLAVE_ADDR_WRITE >> 1))
};
#endif

static void eta6937_shutdown(struct i2c_client *client)
{
	printk("[%s] driver shutdown\n", __func__);
	eta6937_set_otg_en(0x0);
}

static struct i2c_driver eta6937_driver = {
	.driver = {
		   .name = "eta6937",
#ifdef CONFIG_OF
		   .of_match_table = eta6937_of_match,
#endif
		   },
	.probe = eta6937_driver_probe,
	.shutdown = eta6937_shutdown,
	.id_table = eta6937_i2c_id,
};

static int __init eta6937_init(void)
{
	int ret = 0;

	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	printk("[eta6937_init] init start with i2c DTS");
#else
	printk("[eta6937_init] init start. ch=%d\n", eta6937_BUSNUM);
	i2c_register_board_info(eta6937_BUSNUM, &i2c_eta6937, 1);
#endif
	if (i2c_add_driver(&eta6937_driver) != 0) {
		printk(
			    "[eta6937_init] failed to register eta6937 i2c driver.\n");
	} else {
		printk(
			    "[eta6937_init] Success to register eta6937 i2c driver.\n");
	}

	/* eta6937 user space access interface */
	ret = platform_device_register(&eta6937_user_space_device);
	if (ret) {
		printk("****[eta6937_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&eta6937_user_space_driver);
	if (ret) {
		printk("****[eta6937_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit eta6937_exit(void)
{
	i2c_del_driver(&eta6937_driver);
}
module_init(eta6937_init);
module_exit(eta6937_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C eta6937 Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
