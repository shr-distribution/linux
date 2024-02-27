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

#if 1//def CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include "upmu_common.h"
#include "psc5415A.h"
#include "mtk_charger_intf.h"
#include <mt-plat/mtk_battery.h>

const unsigned int VBAT_CVTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000,
	4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000,
	4220000, 4240000, 4260000, 4280000,
	4300000, 4320000, 4340000, 4360000,
	4380000, 4400000, 4420000, 4440000
};

const unsigned int CSTH[] = {
	550000,   650000,	750000,  850000,
	950000,   1050000,	1150000, 1250000,
	1350000,  1450000,  1550000, 1650000,
	1750000,  1850000,  1950000, 2050000,
	2150000,  2250000,  2350000, 2450000,
	2150000,  2650000,  2750000, 2850000,
	2950000,  3050000,  3050000, 3050000,
    3050000,  3050000,  3050000, 3050000
};

/*psc5415A REG00 IINLIM[5:0]*/
const unsigned int INPUT_CSTH[] = {
	300000, 500000, 800000, 1200000,
    1500000,2000000,3000000,5000000,   
};

/* psc5415A REG0A BOOST_LIM[2:0], mA */
const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150,
};

#if 1//def CONFIG_OF
#else
#define psc5415A_SLAVE_ADDR_WRITE 0xD4
#define psc5415A_SLAVE_ADDR_Read	 0xD5
#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define psc5415A_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define psc5415A_BUSNUM 0
#endif
#endif
//extern bool charging_current_limilt_flag;

struct psc5415A_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
//	CHARGER_TYPE chg_type;
	int irq;
};

struct pinctrl *switchchargctrl = NULL;
struct pinctrl_state *switch_charging_en_h = NULL;
struct pinctrl_state *switch_charging_en_l = NULL;

static struct i2c_client *new_client;
static const struct i2c_device_id psc5415A_i2c_id[] = { {"psc5415A", 0}, {} };

unsigned char psc5415A_reg[PSC5415A_REG_NUM] = { 0 };
static DEFINE_MUTEX(psc5415A_i2c_access);
static DEFINE_MUTEX(psc5415A_access_lock);

#if 1
int psc5415A_charging_gpio_init(struct platform_device *pdev)
{
    int ret = 0;

	switchchargctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(switchchargctrl)) {
		dev_err(&pdev->dev, "Cannot find psc5415A_charging_gpio pinctrl!");
		ret = PTR_ERR(switchchargctrl);
	}

	switch_charging_en_h = pinctrl_lookup_state(switchchargctrl, "switch_charging_en1");
	if (IS_ERR(switch_charging_en_h)) {
		ret = PTR_ERR(switch_charging_en_h);
		printk("%s : pinctrl err, psc5415A_charging_en_h\n", __func__);
	}

	switch_charging_en_l = pinctrl_lookup_state(switchchargctrl, "switch_charging_en0");
	if (IS_ERR(switch_charging_en_l)) {
		ret = PTR_ERR(switch_charging_en_l);
		printk("%s : pinctrl err, psc5415A_charging_en_l\n", __func__);
	}

	return ret;

}


int psc5415A_disable_charging(int cmd)                                                                                                                  
{
 //   printk("%s: LINE=%d, disable=%d\n", __func__, __LINE__, cmd);
    switch (cmd)
    {
    case 0 :
        pinctrl_select_state(switchchargctrl, switch_charging_en_l);
        break;
    case 1 :
        pinctrl_select_state(switchchargctrl, switch_charging_en_h);
        break;
        //    printk("%s: LINE=%d\n", __func__, __LINE__);    
        //    //        chargin_hw_init_done = KAL_TRUE;
        //        
    }
    return 0;

}
#endif
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

	pr_info("NO register value match\n");
	/* TODO: ASSERT(0);	// not find the value */
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
		pr_info("Can't find closest level22-- %d\n",pList[0]);
		return pList[0];
		/* return 000; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		pr_info("Can't find closest level33-- %d\n",pList[0]);
		return pList[number - 1];
		/* return 000; */
	}
}

//static int psc5415A_enable_otg(struct charger_device *chg_dev, bool en)
//{
//	int ret = 0;
	
//    psc5415A_set_opa_mode(en);
//	return ret;
//}
static int psc5415A_read_byte(u8 reg_addr, u8 *rd_buf, int rd_len)
{
	int ret = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg[2];
	u8 *w_buf = NULL;
	u8 *r_buf = NULL;

	memset(msg, 0, 2 * sizeof(struct i2c_msg));

	w_buf = kzalloc(1, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;
	r_buf = kzalloc(rd_len, GFP_KERNEL);
	if (r_buf == NULL)
		return -1;

	*w_buf = reg_addr;

	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = w_buf;

	msg[1].addr = new_client->addr;
	msg[1].flags = 1;
	msg[1].len = rd_len;
	msg[1].buf = r_buf;

	ret = i2c_transfer(adap, msg, 2);

	memcpy(rd_buf, r_buf, rd_len);

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}

int psc5415A_write_byte(unsigned char reg_num, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg;
	u8 *w_buf = NULL;

	memset(&msg, 0, sizeof(struct i2c_msg));

	w_buf = kzalloc(wr_len, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;

	w_buf[0] = reg_num;
	memcpy(w_buf + 1, wr_buf, wr_len);

	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = w_buf;

	ret = i2c_transfer(adap, &msg, 1);

	kfree(w_buf);
	return ret;
}

unsigned int psc5415A_read_interface(unsigned char reg_num, unsigned char *val, unsigned char MASK,
				unsigned char SHIFT)
{
	unsigned char psc5415A_reg = 0;
	unsigned int ret = 0;

	ret = psc5415A_read_byte(reg_num, &psc5415A_reg, 1);
//	pr_debug_ratelimited("[psc5415A_read_interface] Reg[%x]=0x%x\n", reg_num, psc5415A_reg);
	psc5415A_reg &= (MASK << SHIFT);
	*val = (psc5415A_reg >> SHIFT);
//	pr_debug_ratelimited("[psc5415A_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int psc5415A_config_interface(unsigned char reg_num, unsigned char val, unsigned char MASK,
					unsigned char SHIFT)
{
	unsigned char psc5415A_reg = 0;
	unsigned char psc5415A_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&psc5415A_access_lock);
	ret = psc5415A_read_byte(reg_num, &psc5415A_reg, 1);
	psc5415A_reg_ori = psc5415A_reg;
	psc5415A_reg &= ~(MASK << SHIFT);
	psc5415A_reg |= (val << SHIFT);
	if (reg_num == PSC5415A_CON4)
		psc5415A_reg &= ~(1 << CON4_RESET_SHIFT);

	ret = psc5415A_write_byte(reg_num, &psc5415A_reg, 2);
	mutex_unlock(&psc5415A_access_lock);
	pr_debug_ratelimited("[psc5415A_config_interface] write Reg[%x]=0x%x from 0x%x,mask = 0x%x,shift = 0x%x,\n", reg_num,psc5415A_reg, psc5415A_reg_ori,MASK,SHIFT);
	/* Check */
	/* psc5415A_read_byte(reg_num, &psc5415A_reg, 1); */
	/* printk("[psc5415A_config_interface] Check Reg[%x]=0x%x\n", reg_num, psc5415A_reg); */

	return ret;
}

/* write one register directly */
unsigned int psc5415A_reg_config_interface(unsigned char reg_num, unsigned char val)
{
	unsigned char psc5415A_reg = val;

	return psc5415A_write_byte(reg_num, &psc5415A_reg, 2);
}

void psc5415A_set_tmr_rst(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_TMR_RST_MASK),
				(unsigned char)(CON0_TMR_RST_SHIFT)
				);
}

unsigned int psc5415A_get_otg_status(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_OTG_MASK),
				(unsigned char)(CON0_OTG_SHIFT)
				);
	return val;
}

void psc5415A_set_en_stat(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_EN_STAT_MASK),
				(unsigned char)(CON0_EN_STAT_SHIFT)
				);
}

unsigned int psc5415A_get_chip_status(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_STAT_MASK),
				(unsigned char)(CON0_STAT_SHIFT)
				);
	return val;
}

unsigned int psc5415A_get_boost_status(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_BOOST_MASK),
				(unsigned char)(CON0_BOOST_SHIFT)
				);
	return val;

}

unsigned int psc5415A_get_fault_status(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_FAULT_MASK),
				(unsigned char)(CON0_FAULT_SHIFT)
				);
	return val;
}

void psc5415A_set_input_charging_current(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON7),
				(unsigned char)(val),
				(unsigned char)(CON7_LIN_LIMIT_MASK),
				(unsigned char)(CON7_LIN_LIMIT_SHIFT)
				);
}

unsigned int psc5415A_get_input_charging_current(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON7),
				(unsigned char *)(&val),
				(unsigned char)(CON7_LIN_LIMIT_MASK),
				(unsigned char)(CON7_LIN_LIMIT_SHIFT)
				);

	return val;
}

void psc5415A_set_v_low(unsigned int val)
{

	psc5415A_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LOW_V_MASK),
				(unsigned char)(CON1_LOW_V_SHIFT)
				);
}

void psc5415A_set_te(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_TE_MASK),
				(unsigned char)(CON1_TE_SHIFT)
				);
}

void psc5415A_set_ce(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_CE_MASK),
				(unsigned char)(CON1_CE_SHIFT)
				);
}

void psc5415A_set_hz_mode(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_HZ_MODE_MASK),
				(unsigned char)(CON1_HZ_MODE_SHIFT)
				);
}

void psc5415A_set_opa_mode(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_OPA_MODE_MASK),
				(unsigned char)(CON1_OPA_MODE_SHIFT)
				);
}

void psc5415A_set_oreg(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OREG_MASK),
				(unsigned char)(CON2_OREG_SHIFT)
				);
}
void psc5415A_set_otg_pl(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_PL_MASK),
				(unsigned char)(CON2_OTG_PL_SHIFT)
				);
}

void psc5415A_set_otg_en(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_EN_MASK),
				(unsigned char)(CON2_OTG_EN_SHIFT)
				);
}

unsigned int psc5415A_get_vender_code(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_VENDER_CODE_MASK),
				(unsigned char)(CON3_VENDER_CODE_SHIFT)
				);
	return val;
}
unsigned int psc5415A_get_pn(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_PIN_MASK),
				(unsigned char)(CON3_PIN_SHIFT)
				);
	return val;
}

unsigned int psc5415A_get_revision(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_REVISION_MASK),
				(unsigned char)(CON3_REVISION_SHIFT)
				);
	return val;
}

void psc5415A_set_reset(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_RESET_MASK),
				(unsigned char)(CON4_RESET_SHIFT)
				);
}

void psc5415A_set_iocharge(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON4),
				(unsigned char)(val%8),
				(unsigned char)(CON4_I_CHR_MASK),
				(unsigned char)(CON4_I_CHR_SHIFT)
				);

	psc5415A_config_interface((unsigned char)(PSC5415A_CON5),
	(unsigned char)(val/8),
	(unsigned char)(CON5_I_CHR_MASK),
	(unsigned char)(CON5_I_CHR_SHIFT)
	);
}

void psc5415A_set_iterm(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_TERM_MASK),
				(unsigned char)(CON4_I_TERM_SHIFT)
				);
}

void psc5415A_set_dis_vreg(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_DIS_VREG_MASK),
				(unsigned char)(CON5_DIS_VREG_SHIFT)
				);
}

void psc5415A_set_io_level(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_IO_LEVEL_MASK),
				(unsigned char)(CON5_IO_LEVEL_SHIFT)
				);
}

unsigned int psc5415A_get_sp_status(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_SP_STATUS_MASK),
				(unsigned char)(CON5_SP_STATUS_SHIFT)
				);
	return val;
}

unsigned int psc5415A_get_en_level(void)
{
	unsigned char val = 0;

	psc5415A_read_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_EN_LEVEL_MASK),
				(unsigned char)(CON5_EN_LEVEL_SHIFT)
				);
	return val;
}

void psc5415A_set_vsp(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_VSP_MASK),
				(unsigned char)(CON5_VSP_SHIFT)
				);
}

void psc5415A_set_i_safe(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_ISAFE_MASK),
				(unsigned char)(CON6_ISAFE_SHIFT)
				);
}

void psc5415A_set_v_safe(unsigned int val)
{
	psc5415A_config_interface((unsigned char)(PSC5415A_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_VSAFE_MASK),
				(unsigned char)(CON6_VSAFE_SHIFT)
				);
}

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
			psc5415A_set_tmr_rst(1);
			eta6937_otg_start_timer();		
		}
	}
	
	return 0;
}
#endif


static int psc5415A_dump_register(struct charger_device *chg_dev)
{
	int i;

  //  printk("zhanglz %s\n",__func__);
	for (i = 0; i < PSC5415A_REG_NUM; i++) {
		psc5415A_read_byte(i, &psc5415A_reg[i], 1);
	//	pr_debug("[0x%x]=0x%x ", i, psc5415A_reg[i]);
		printk("psc5415A yqf[0x%x]=0x%x \n", i, psc5415A_reg[i]);
	}
//	pr_debug("\n");

	return 0;
}
static int psc5415A_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
#ifdef ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
	eta6937_otg_status_flag = en;
	if(en == 1)
		eta6937_otg_start_timer();
#endif
    psc5415A_set_opa_mode(en);
	return ret;
}

static int psc5415A_parse_dt(struct psc5415A_info *info, struct device *dev)
{
	struct device_node *np = dev->of_node;

	pr_info("%s\n", __func__);

	if (!np) {
		pr_err("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name", &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_warn("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name", &(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "psc5415A";
		pr_warn("%s: no alias name\n", __func__);
	}

	return 0;
}

static int psc5415A_do_event(struct charger_device *chg_dev, unsigned int event, unsigned int args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_info("%s: event = %d\n", __func__, event);

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

static int psc5415A_enable_charging(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;
    pr_debug("yqf %s  en %d\n",__func__,en);

	if (en) {
		psc5415A_set_ce(0);
		psc5415A_set_hz_mode(0);
		psc5415A_set_opa_mode(0);
        psc5415A_disable_charging(0);
		psc5415A_set_tmr_rst(1);
		psc5415A_reg_config_interface(0x06, 0x5a);
		psc5415A_reg_config_interface(0x00, 0xC0);
		psc5415A_reg_config_interface(0x06, 0x5a);
	} else {
		psc5415A_set_ce(1);
		psc5415A_set_hz_mode(1);
    //    psc5415A_disable_charging(1);
	}

	return status;
}

static int psc5415A_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;
	//printk("yqf psc5415A_set_cv_voltage %d\n",cv);
	/*static kal_int16 pre_register_value; */
	array_size = ARRAY_SIZE(VBAT_CVTH);
	/*pre_register_value = -1; */
	set_cv_voltage = bmt_find_closest_level(VBAT_CVTH, array_size, cv);

	register_value = charging_parameter_to_value(VBAT_CVTH, array_size, set_cv_voltage);
	pr_info("yqf charging_set_cv_voltage register_value=0x%x %d %d\n",
	 register_value, cv, set_cv_voltage);
	psc5415A_set_oreg(register_value);

	return status;
}

static int psc5415A_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	int status = 0;
	unsigned int array_size;
	unsigned char reg_value,regvalue;

	array_size = ARRAY_SIZE(CSTH);
	psc5415A_read_interface(PSC5415A_CON4, &regvalue, CON4_I_CHR_MASK, CON4_I_CHR_SHIFT);
	reg_value = regvalue;
	psc5415A_read_interface(PSC5415A_CON5, &regvalue, CON5_I_CHR_MASK, CON5_I_CHR_SHIFT);
	reg_value = reg_value + ((regvalue & 0x03) * 8);
	*ichg = charging_value_to_parameter(CSTH, array_size, reg_value);

	return status;
}

static int psc5415A_set_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	// yqf add
	//if(current_value >= 850000)
	//{
	//	if (charging_current_limilt_flag)
	//        current_value = 950000;//950000
//		else
//			current_value = 1250000;
//	}
   // printk("yqf psc5415A_set_current %d\n",current_value);
	//current_value = 1450000;
	if (current_value <= 35000) {
		psc5415A_set_io_level(1);
	} else {
		psc5415A_set_io_level(0);
		array_size = ARRAY_SIZE(CSTH);
		set_chr_current = bmt_find_closest_level(CSTH, array_size, current_value);
		register_value = charging_parameter_to_value(CSTH, array_size, set_chr_current);
		pr_info("yqf psc5415A_set_current register_value 0x%x\n",register_value);
	//	if (current_value == 950000)
	//		psc5415A_set_iocharge(0x07);
	//	else
	        //psc5415A_reg_config_interface(0x06, 0x5a);	/* ISAFE = 2150mA, VSAFE = 4.40V */
		    psc5415A_set_iocharge(register_value);
	       
	}

	return status;
}

static int psc5415A_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(INPUT_CSTH);
	register_value = psc5415A_get_input_charging_current();
	*aicr = charging_parameter_to_value(INPUT_CSTH, array_size, register_value);

	return status;
}

static int psc5415A_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	//printk("yqf psc5415A_set_input_current %d\n",current_value);

	psc5415A_reg_config_interface(0x07, 0x08);

	array_size = ARRAY_SIZE(INPUT_CSTH);
	set_chr_current = bmt_find_closest_level(INPUT_CSTH, array_size, current_value);
	register_value = charging_parameter_to_value(INPUT_CSTH, array_size, set_chr_current);
	psc5415A_set_input_charging_current(register_value);

	return status;
}

static int psc5415A_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;
   // u32 ret_vbat;
	unsigned int ret_uisoc;
	ret_val = psc5415A_get_chip_status();
//	ret_vbat = battery_get_bat_voltage();
	ret_uisoc = battery_get_uisoc();
	pr_debug("yqf psc5415A_get_charging_status ret_val = %d,ret_uisoc=%d \n",ret_val,ret_uisoc);
// if (ret_val == 0x2)
//	if (ret_vbat >= 4350)
	if ((ret_val == 0x2) && (ret_uisoc == 100))
		*is_done = true;
	else
		*is_done = false;
	return status;
}

static int psc5415A_reset_watch_dog_timer(struct charger_device *chg_dev)
{
    pr_debug("yqf %s  line %d\n",__func__,__LINE__);
	psc5415A_set_tmr_rst(1);
	psc5415A_reg_config_interface(0x06, 0x5a);
	psc5415A_set_tmr_rst(1);
	psc5415A_reg_config_interface(0x06, 0x5a);
	return 0;
}

static struct charger_ops psc5415A_chg_ops = {

	/* Normal charging */
	.dump_registers = psc5415A_dump_register,
	.enable = psc5415A_enable_charging,
    //.is_enable = psc5415A_charger_is_enabled,
	.get_charging_current = psc5415A_get_current,
	.set_charging_current = psc5415A_set_current,
	.get_input_current = psc5415A_get_input_current,
	.set_input_current = psc5415A_set_input_current,
	/*.get_constant_voltage = psc5415A_get_battery_voreg,*/
	.set_constant_voltage = psc5415A_set_cv_voltage,
	.kick_wdt = psc5415A_reset_watch_dog_timer,
	.is_charging_done = psc5415A_get_charging_status,
    .enable_otg = psc5415A_enable_otg,
	.event = psc5415A_do_event,
};

static int psc5415A_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct psc5415A_info *info = NULL;

//    unsigned char regdata = 0;
    unsigned int register_value = 0;
//	printk("[psc5415A_driver_probe]\n");
	info = devm_kzalloc(&client->dev, sizeof(struct psc5415A_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	new_client = client;
	info->dev = &client->dev;
	ret = psc5415A_parse_dt(info, &client->dev);
//	pr_info("psc5415A_driver_probe2 \n");

    // psc5415A_switch_dev = &client->dev;
    // psc5415A_gpio_get_info();
	if (ret < 0)
		return ret;

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&client->dev, info, &psc5415A_chg_ops, &info->chg_props);
//	pr_info("psc5415A_driver_probe3 \n");

	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

/*	ret = psc5415A_get_vender_code();

	if (ret != 2) {
		pr_err("%s: get vendor id failed\n", __func__);
		return -ENODEV;
	}
*/
	/* psc5415A_hw_init(); //move to charging_hw_xxx.c */
	info->psy = power_supply_get_by_name("charger");

	if (!info->psy) {
		pr_err("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

#if 1//defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	psc5415A_reg_config_interface(0x06, 0x5A);	/* ISAFE = 1350mA, VSAFE = 4.40V */
	psc5415A_set_i_safe(0x50);
	psc5415A_set_v_safe(0x0A);
	psc5415A_reg_config_interface(0x07, 0x08);	/* ENLIMIT2  */
  //  psc5415A_read_byte(0x06, &regdata, 1);
  //  printk("zhanglz regdata[0] = %d\n",regdata);
	//psc5415A_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
//	psc5415A_reg_config_interface(0x02, 0xB4);
	psc5415A_reg_config_interface(0x01, 0xF8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	psc5415A_reg_config_interface(0x05, 0x06);// 0x04

	psc5415A_reg_config_interface(0x04, 0x02);	/* termination current 150mA */
#endif

	psc5415A_dump_register(info->chg_dev);
    register_value = psc5415A_get_input_charging_current();
	//printk("[zhanglz psc5415A_driver_probe sucess] register_value  = %x\n",register_value);
#ifdef ETA6937_TIMER_DEBUG //add by esky_liml_2018_02_23
	init_waitqueue_head(&eta6937_otg_wait_que);
	eta6937_otg_init_timer();
	kthread_run(eta6937_otg_routine_thread, 0, "eta6937_otg_thread");
#endif

	return 0;
}

#if 1
static int psc5415A_charging_en_probe(struct platform_device *pdev)
{

  // printk("psc5415A_charging_en_probe \n");
	psc5415A_charging_gpio_init(pdev);

	return 0;
}
static int psc5415A_charging_en_remove(struct platform_device *pdev)
{

	return 0;
	
}
#endif

#if 1//def CONFIG_OF
static const struct of_device_id psc5415A_of_match[] = {
	{.compatible = "mediatek,sw_charger"},
	{},
};
#else
static struct i2c_board_info i2c_psc5415A __initdata = {
	I2C_BOARD_INFO("psc5415A", (psc5415A_SLAVE_ADDR_WRITE >> 1))
};
#endif


#if 1
static const struct of_device_id psc5415A_charging_en_of_ids[] = {
	{.compatible = "mediatek,switch_charging_en",},
	{}
};

static struct platform_driver psc5415A_charging_en_driver = {
	.probe = psc5415A_charging_en_probe,
	.remove = psc5415A_charging_en_remove,
	.driver = {
		   .name = "switch_charging_en_driver",
		   .owner = THIS_MODULE,
#if 1//def CONFIG_OF
		   .of_match_table = psc5415A_charging_en_of_ids,
#endif
		   }
};
#endif

static struct i2c_driver psc5415A_driver = {
	.driver = {
		.name = "psc5415A",
#if 1//def CONFIG_OF
		.of_match_table = psc5415A_of_match,
#endif
		},
	.probe = psc5415A_driver_probe,
	.id_table = psc5415A_i2c_id,
};

static int __init psc5415A_init(void)
{
    int ret = 0;
	//pr_info("yqf psc5415A_init \n");
	if (i2c_add_driver(&psc5415A_driver) != 0)
		pr_info("Failed to register psc5415A i2c driver.\n");
	else
		pr_info("Success to register psc5415A i2c driver.\n");

	
    ret = platform_driver_register(&psc5415A_charging_en_driver);
    if (ret) {
        pr_notice("****[fan5405_init] Unable to register charging_en driver (%d)\n", ret);
        return ret;
    }	

	return 0;
}

static void __exit psc5415A_exit(void)
{
	i2c_del_driver(&psc5415A_driver);
	platform_driver_unregister(&psc5415A_charging_en_driver);
}

module_init(psc5415A_init);
module_exit(psc5415A_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C psc5415A Driver");
MODULE_AUTHOR("Henry Chen<henryc.chen@mediatek.com>");
