// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>

#include <linux/regulator/driver.h>
//#include "bq2589x_reg.h"
#include "charger_class.h"
#include "mtk_charger.h"
#include "eta6965-charger.h"

#define PFX "eta6965_charger"
#if 0//0//1-debugging
#define LOG_INF(fmt, args...)        pr_err(PFX "[%s %d] " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...)        pr_err(PFX "[error][%s %d] " fmt, __func__, __LINE__, ##args)
#define LOG_DBG(fmt, args...)        pr_err(PFX "[%s %d] " fmt, __func__, __LINE__, ##args)
#else
#define LOG_INF(fmt, args...)
#define LOG_ERR(fmt, args...)        pr_err(PFX "[error][%s %d] " fmt, __func__, __LINE__, ##args)
#define LOG_DBG(fmt, args...)        
#endif

static int eta6965_load_status = 0;
/**********************************************************
 *
 *   [I2C Slave Setting]
 *
 *********************************************************/

#define GETARRAYNUM(array) (ARRAY_SIZE(array))

/*eta6965 REG06 VREG[5:0]*/
const unsigned int VBAT_CV_VTH[] = {
	3848000, 3880000, 3912000, 3944000,
	3976000, 4008000, 4040000, 4072000,
	4104000, 4136000, 4168000, 4200000,
	4232000, 4264000, 4296000, 4328000,
	4360000, 4392000, 4424000, 4456000,
	4488000, 4520000, 4552000, 4584000,
	4616000

};

/*ETA6965 REG02 ICHG[5:0]*/
const unsigned int CS_VTH[] = {
	0, 6000, 12000, 18000, 24000,
	30000, 36000, 42000, 48000, 54000,
	60000, 66000, 72000, 78000, 84000,
	90000, 96000, 102000, 108000, 114000,
	120000, 126000, 132000, 138000, 144000,
	150000, 156000, 162000, 168000, 174000,
	180000, 186000, 192000, 198000, 204000,
	210000, 216000, 222000, 228000, 234000,
	240000, 246000, 252000, 258000, 264000,
	270000, 276000, 282000, 288000, 294000,
	300000, 306000, 312000, 318000, 324000
};

/*ETA6965 REG00 IINDPM[4:0]*/
const unsigned int INPUT_CS_VTH[] = {
	10000, 20000, 30000, 40000,
	50000, 60000, 70000, 80000,
	90000, 100000, 110000, 120000,
	130000, 140000, 150000, 160000,
	170000, 180000, 190000, 200000,
	210000, 220000, 230000, 250000,
	260000, 270000, 280000, 290000,
	300000, 310000, 320000
};


const unsigned int VCDT_HV_VTH[] = {
	4200000, 4250000, 4300000, 4350000,
	4400000, 4450000, 4500000, 4550000,
	4600000, 6000000, 6500000, 7000000,
	7500000, 8500000, 9500000, 10500000

};


const unsigned int VINDPM_REG[] = {
	3900, 4000, 4100, 4200, 4300, 4400,
	4500, 4600, 4700, 4800, 4900, 5000,
	5100, 5200, 5300, 5400, 5500, 5600,
	5700, 5800, 5900, 6000, 6100, 6200,
	6300, 6400
};

/* BQ25601 REG0A BOOST_LIM[2:0], mA */
const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 1200
};

//REG03 ITERM[3:0]  iterm closest threshold table  //agenew add
const unsigned int ITERM_CTT[] = {  /* uA */
	 60000, 120000, 180000, 240000, 300000, 360000, 420000, 480000, 540000, 600000, 
	660000, 720000, 780000, 840000, 900000, 960000
};

enum eta6965_chg_type {
	ETA6965_CHG_TYPE_NO_INFO = 0x0,
	ETA6965_CHG_TYPE_SDP = 0x01,
	ETA6965_CHG_TYPE_CDP = 0x02,
	ETA6965_CHG_TYPE_DCP = 0x03,
	ETA6965_CHG_TYPE_UNKNOWN_TA = 0x05,
	ETA6965_CHG_TYPE_NON_STANDARD_TA = 0x06,
	ETA6965_CHG_TYPE_OTG = 0x07,
};
struct eta6965_info {
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
	struct regulator_dev *otg_rdev;
	int psy_usb_type;
	int chg_en;
	struct pinctrl *pinctrl;
	struct pinctrl_state *psc_chg_en_low;
	struct pinctrl_state *psc_chg_en_high;
	bool attach;
	int vbat0_flag;

	u32 ieoc;
	u32 vrechg;
} *g_eta6965_info;

static const struct charger_properties eta6965_chg_props = {
	.alias_name = "eta6965-user",
};

DEFINE_MUTEX(g_input_current_mutex);
static struct i2c_client *new_client;
static const struct i2c_device_id eta6965_i2c_id[] = { {"eta6965", 0}, {} };

static int eta6965_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id);

unsigned int charging_value_to_parameter(const unsigned int
		*parameter, const unsigned int array_size,
		const unsigned int val)
{
	if (val < array_size)
		return parameter[val];

	LOG_INF("Can't find the parameter\n");
	return parameter[0];

}

unsigned int charging_parameter_to_value(const unsigned int
		*parameter, const unsigned int array_size,
		const unsigned int val)
{
	unsigned int i;

	LOG_DBG("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	LOG_INF("NO register value match\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList,
		unsigned int number,
		unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0;
		     i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				LOG_DBG("zzf_%d<=%d, i=%d\n",
					pList[i], level, i);
				return pList[i];
			}
		}

		LOG_INF("Can't find closest level\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		/* max value in the first element */
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}

		LOG_INF("Can't find closest level\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}


/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/
unsigned char eta6965_reg[eta6965_REG_NUM] = { 0 };

static DEFINE_MUTEX(eta6965_i2c_access);
static DEFINE_MUTEX(eta6965_access_lock);

int g_eta6965_hw_exist;

/**********************************************************
 *
 *   [I2C Function For Read/Write eta6965]
 *
 *********************************************************/
#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int eta6965_read_byte(unsigned char cmd,
			       unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&eta6965_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) |
	 * I2C_WR_FLAG;
	 */
	new_client->ext_flag =
		((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG |
		I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;
		mutex_unlock(&eta6965_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;
	mutex_unlock(&eta6965_i2c_access);

	return 1;
}

unsigned int eta6965_write_byte(unsigned char cmd,
				unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&eta6965_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) |
			       I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&eta6965_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&eta6965_i2c_access);
	return 1;
}
#else
unsigned int eta6965_read_byte(unsigned char cmd,
			       unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&eta6965_i2c_access);

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
			LOG_INF("skipping non-existent adapter %s\n",
				new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&eta6965_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int eta6965_write_byte(unsigned char cmd,
				unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&eta6965_i2c_access);

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
			LOG_INF("skipping non-existent adapter %s\n",
				new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&eta6965_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif
/**********************************************************
 *
 *   [Read / Write Function]
 *
 *********************************************************/
unsigned int eta6965_read_interface(unsigned char RegNum,
				    unsigned char *val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char eta6965_reg = 0;
	unsigned int ret = 0;

	ret = eta6965_read_byte(RegNum, &eta6965_reg);

	LOG_DBG("[%s] Reg[%x]=0x%x\n", __func__,
			     RegNum, eta6965_reg);

	eta6965_reg &= (MASK << SHIFT);
	*val = (eta6965_reg >> SHIFT);

	LOG_DBG("[%s] val=0x%x\n", __func__, *val);

	return ret;
}

unsigned int eta6965_config_interface(unsigned char RegNum,
				      unsigned char val, unsigned char MASK,
				      unsigned char SHIFT)
{
	unsigned char eta6965_reg = 0;
	unsigned char eta6965_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&eta6965_access_lock);
	ret = eta6965_read_byte(RegNum, &eta6965_reg);

	eta6965_reg_ori = eta6965_reg;
	eta6965_reg &= ~(MASK << SHIFT);
	eta6965_reg |= (val << SHIFT);

	ret = eta6965_write_byte(RegNum, eta6965_reg);
	mutex_unlock(&eta6965_access_lock);
	LOG_DBG("[%s] write Reg[%x]=0x%x from 0x%x\n", __func__,
			     RegNum,
			     eta6965_reg, eta6965_reg_ori);

	/* Check */
	/* eta6965_read_byte(RegNum, &eta6965_reg); */
	/* LOG_INF("[%s] Check Reg[%x]=0x%x\n", __func__,*/
	/* RegNum, eta6965_reg); */

	return ret;
}

/* write one register directly */
unsigned int eta6965_reg_config_interface(unsigned char RegNum,
		unsigned char val)
{
	unsigned int ret = 0;

	ret = eta6965_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
/* CON0---------------------------------------------------- */
unsigned int  eta6965_get_en_hiz(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
	
	ret = eta6965_read_interface((unsigned char) (eta6965_CON0),
				       (&val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
				      );
	LOG_DBG("%s,val=%d\n",__func__,val);
	return val;
}
void eta6965_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
				      );
}

void eta6965_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
				      );
}
unsigned int   eta6965_get_iinlim(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
	
	ret = eta6965_read_interface((unsigned char) (eta6965_CON0),
				       (&val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
				      );
	LOG_DBG("%s,val=%d\n",__func__,val);
	return val;
}

void eta6965_set_stat_ctrl(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON0),
				   (unsigned char) (val),
				   (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
				   (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT)
				   );
}

/* CON1---------------------------------------------------- */

void eta6965_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON11),
				       (unsigned char) (val),
				       (unsigned char) (CON11_REG_RST_MASK),
				       (unsigned char) (CON11_REG_RST_SHIFT)
				      );
}

void eta6965_set_pfm(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_PFM_MASK),
				       (unsigned char) (CON1_PFM_SHIFT)
				      );
}

void eta6965_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT)
				      );
}

void eta6965_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT)
				      );
}

unsigned int eta6965_get_otg_config(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON1),
				     (&val),
				     (unsigned char) (CON1_OTG_CONFIG_MASK),
				     (unsigned char) (CON1_OTG_CONFIG_SHIFT)
				    );
	return val;
}

void eta6965_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
				      );
}

unsigned int  eta6965_get_chg_config(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
	
	ret = eta6965_read_interface((unsigned char) (eta6965_CON1),
				       (&val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
				      );
	LOG_DBG("%s,val=%d\n",__func__,val);
	return val;
}

void eta6965_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT)
				      );
}

void eta6965_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_MIN_VBAT_SEL_MASK),
				       (unsigned char) (CON1_MIN_VBAT_SEL_SHIFT)
				      );
}



/* CON2---------------------------------------------------- */
void eta6965_set_rdson(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_Q1_FULLON_MASK),
				       (unsigned char) (CON2_Q1_FULLON_SHIFT)
				      );
}

void eta6965_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_LIM_MASK),
				       (unsigned char) (CON2_BOOST_LIM_SHIFT)
				      );
}

void eta6965_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK),
				       (unsigned char) (CON2_ICHG_SHIFT)
				      );
}

#ifdef FIXME //this function does not exist on eta6965
void eta6965_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_20PCT_MASK),
				       (unsigned char) (CON2_FORCE_20PCT_SHIFT)
				      );
}
#endif
/* CON3---------------------------------------------------- */

void eta6965_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT)
				      );
}

void eta6965_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK),
				       (unsigned char) (CON3_ITERM_SHIFT)
				      );
}

static int __eta6965_set_iterm_via_table(u32 current_value)
{
	unsigned int array_size;
	unsigned int found_value;
	unsigned int register_value;

	LOG_INF("agenew[%s %d] set iterm = %d mA \n", __func__, __LINE__, current_value/1000);
	array_size     = GETARRAYNUM(ITERM_CTT);
	found_value    = bmt_find_closest_level(ITERM_CTT, array_size, current_value);
	register_value = charging_parameter_to_value(ITERM_CTT, array_size, found_value); //return index value

	LOG_INF("agenew[%s %d] set iterm reg value = 0x%x \n", __func__, __LINE__, register_value);
	eta6965_set_iterm(register_value);

	return 0;
}
static int eta6965_get_ieoc(struct charger_device *dev, u32 *uA)
{
	int ret = 0;
#if 1//def FIXME
	unsigned char val = 0;

	eta6965_read_interface(eta6965_CON3, &val, CON3_ITERM_MASK,
			       CON3_ITERM_SHIFT);
	ret = (int)val;
	*uA = ITERM_CTT[val];
	LOG_INF("agenew[%s %d] get ieoc = %d uA \n", __func__, __LINE__, *uA);
#endif
	return ret;
}
static int eta6965_set_ieoc(struct charger_device *chgdev, u32 uA)
{
    int ret = 0; 
	LOG_INF("agenew[%s %d] set ieoc = %d uA \n", __func__, __LINE__, uA);

    ret = __eta6965_set_iterm_via_table(uA); 

    return ret; 
}

/* CON4---------------------------------------------------- */

void eta6965_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VREG_MASK),
				       (unsigned char) (CON4_VREG_SHIFT)
				      );
}

void eta6965_set_topoff_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_TOPOFF_TIMER_MASK),
				       (unsigned char) (CON4_TOPOFF_TIMER_SHIFT)
				      );

}


void eta6965_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT)
				      );
}

/* CON5---------------------------------------------------- */

void eta6965_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT)
				      );
}



void eta6965_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT)
				      );
}

void eta6965_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT)
				      );
}

void eta6965_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT)
				      );
}

/* CON6---------------------------------------------------- */

void eta6965_set_treg(unsigned int val)
{
#if 1//#ifdef FIXME
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
				      );
#endif
}

void eta6965_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_MASK),
				       (unsigned char) (CON6_VINDPM_SHIFT)
				      );
}


void eta6965_set_ovp(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_OVP_MASK),
				       (unsigned char) (CON6_OVP_SHIFT)
				      );

}

void eta6965_set_boostv(unsigned int val)
{

	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
				      );
}



/* CON7---------------------------------------------------- */

void eta6965_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON7),
					(unsigned char) (val),
					(unsigned char) (CON7_TMR2X_EN_MASK),
					(unsigned char) (CON7_TMR2X_EN_SHIFT)
					);
}

#if 1 //agenew 
unsigned int eta6965_get_batfet_disable(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON7),
				(&val),
				(unsigned char) (CON7_BATFET_Disable_MASK),
				(unsigned char) (CON7_BATFET_Disable_SHIFT)
				);
	return val;
}
#endif

void eta6965_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON7),
				(unsigned char) (val),
				(unsigned char) (CON7_BATFET_Disable_MASK),
				(unsigned char) (CON7_BATFET_Disable_SHIFT)
				);
}


void eta6965_set_batfet_delay(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_DLY_MASK),
				       (unsigned char) (CON7_BATFET_DLY_SHIFT)
				      );
}

void eta6965_set_batfet_reset_enable(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON7),
				(unsigned char) (val),
				(unsigned char) (CON7_BATFET_RST_EN_MASK),
				(unsigned char) (CON7_BATFET_RST_EN_SHIFT)
				);
}

void eta6965_set_vdpm_bat(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_VDPM_BAT_TRACK_MASK),
				       (unsigned char) (CON7_VDPM_BAT_TRACK_SHIFT)
				      );
}

/* CON8---------------------------------------------------- */

unsigned int eta6965_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON8),
				     (&val), (unsigned char) (0xFF),
				     (unsigned char) (0x0)
				    );
	return val;
}

unsigned int eta6965_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT)
				    );
	return val;
}

unsigned int eta6965_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT)
				    );
	return val;
}

unsigned int eta6965_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT)
				    );
	return val;
}

unsigned int eta6965_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON8),
				     (&val),
				     (unsigned char) (CON8_PG_STAT_MASK),
				     (unsigned char) (CON8_PG_STAT_SHIFT)
				    );
	return val;
}

/*CON10----------------------------------------------------------*/

void eta6965_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = eta6965_config_interface((unsigned char) (eta6965_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_INT_MASK_MASK),
				       (unsigned char) (CON10_INT_MASK_SHIFT)
				      );
}

unsigned int eta6965_get_vbus_gd(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON10),
				     (&val),
				     (unsigned char) (CON10_VBUS_GD_MASK),
				     (unsigned char) (CON10_VBUS_GD_SHIFT)
				    );
	return val;
}

/*CON11----------------------------------------------------------*/
unsigned int eta6965_get_pin(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface((unsigned char) (eta6965_CON11),
				     (&val),
				     (unsigned char) (CON11_PN_MASK),
				     (unsigned char) (CON11_PN_SHIFT)
				    );
	return val;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int eta6965_dump_register(struct charger_device *chg_dev)
{

	unsigned char i = 0;
	unsigned int ret = 0;

	LOG_INF("[eta6965] ");
	for (i = 0; i < eta6965_REG_NUM; i++) {
		ret = eta6965_read_byte(i, &eta6965_reg[i]);
		if (ret == 0) {
			LOG_INF("[eta6965] i2c transfor error\n");
			return 1;
		}
		printk("[0x%x]=0x%x ", i, eta6965_reg[i]);
	}
	printk("\n");
	return 0;
}


/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static void eta6965_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = eta6965_read_interface(0x0B, &val, 0xFF, 0x0);

	if (val == 0)
		g_eta6965_hw_exist = 0;
	else
		g_eta6965_hw_exist = 1;

	LOG_INF("[%s] exist=%d, Reg[0x0B]=0x%x\n", __func__,
		g_eta6965_hw_exist, val);
}

static int eta6965_is_enabled(struct charger_device *chg_dev, bool *en)
{
	if(eta6965_get_chg_config())
		*en = true;
	else
		*en = false;
	
	return 0;
}

static int eta6965_enable_charging(struct charger_device *chg_dev,
				   bool en)
{
	int status = 0;
	int ret = 0;
	struct eta6965_info * info = g_eta6965_info;
	LOG_INF("enable state : %d\n", en);
	if (en) {
		/* eta6965_config_interface(eta6965_CON3, 0x1, 0x1, 4); */
		/* enable charging */
		eta6965_set_en_hiz(0x0);
		eta6965_set_chg_config(en);
		ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_low);
		if (ret){
			LOG_ERR("mycat Error pinctrl_select_state low");
		}
		else{
			LOG_INF("mycat Ok pinctrl_select_state low");
		}
	} else {
		/* eta6965_config_interface(eta6965_CON3, 0x0, 0x1, 4); */
		/* enable charging */
		eta6965_set_chg_config(en);
		LOG_INF("[charging_enable] under test mode: disable charging\n");
		ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_high);
		if (ret){
			LOG_ERR("mycat Error pinctrl_select_state high");
		}
		else{
			LOG_INF("mycat Ok pinctrl_select_state high");
		}
		/*eta6965_set_en_hiz(0x1);*/
	}

	return status;
}

static int eta6965_get_current(struct charger_device *chgdev, u32 *uA)
{
	int ret;
	unsigned char val = 0;
#if 1//#ifdef FIXME
	/* Get current level */
	ret = eta6965_read_interface(eta6965_CON2, &val, CON2_ICHG_MASK,
			       CON2_ICHG_SHIFT);
	if (ret < 0)
		return ret;
	*uA = CS_VTH[val]*10;
#endif
	return 0;
}

static int eta6965_set_current(struct charger_device *chg_dev,
			       u32 current_value)
{
	unsigned int status = true;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	LOG_INF("&&&& charge_current_value = %d\n", current_value);
	current_value /= 10;
	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size,
			  current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size,
			 set_chr_current);
	//LOG_INF("&&&& charge_register_value = %d\n",register_value);
	LOG_INF("&&&& %s register_value = %d\n", __func__,
		register_value);
	eta6965_set_ichg(register_value);

	return status;
}

static int eta6965_get_input_current(struct charger_device *chg_dev,
				     u32 *aicr)
{
	int ret = 0;
#if 1//#ifdef FIXME
	unsigned char val = 0;

	eta6965_read_interface(eta6965_CON0, &val, CON0_IINLIM_MASK,
			       CON0_IINLIM_SHIFT);
	ret = (int)val;
	*aicr = INPUT_CS_VTH[val]*10;
	LOG_INF("&&&& %s input_current = %d\n", __func__,*aicr);
#endif
	return ret;
}
static int  eta6965_get_ibus_adc(struct charger_device *chg_dev,
				     u32 *ibus_adc)
{	
	unsigned char val = 0;
	int ret = 0;

	val = eta6965_get_iinlim();

	ret = (int)val;
	*ibus_adc = INPUT_CS_VTH[val]*10;//ua
	LOG_INF("&&&& %s ibus_adc = %d\n", __func__,*ibus_adc);
	return ret;
}

static int eta6965_set_input_current(struct charger_device *chg_dev,
				     u32 current_value)
{
	unsigned int status = true;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	current_value /= 10;
	LOG_INF("&&&& current_value = %d\n", current_value);
	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size,
			  current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size,
			 set_chr_current);
	LOG_INF("&&&& %s register_value = %d\n", __func__,
		register_value);
	eta6965_set_iinlim(register_value);

	return status;
}

static int eta6965_set_cv_voltage(struct charger_device *chg_dev,
				  u32 cv)
{
	unsigned int status = true;
	unsigned int array_size;
	unsigned int set_cv_voltage;
	unsigned short register_value;

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size,
			 set_cv_voltage);
	eta6965_set_vreg(register_value);
	LOG_INF("&&&& cv reg value = %d  cv:%d\n", register_value, cv);

	return status;
}

static int eta6965_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	unsigned int status = true;

	LOG_INF("charging_reset_watch_dog_timer\n");

	eta6965_set_wdt_rst(0x1);	/* Kick watchdog */
	eta6965_set_watchdog(0x3);	/* WDT 160s */

	return status;
}


static int eta6965_set_vindpm_voltage(struct charger_device *chg_dev,
				      u32 vindpm)
{
	int status = 0;
	unsigned int array_size;

	vindpm /= 1000;
	array_size = ARRAY_SIZE(VINDPM_REG);
	vindpm = bmt_find_closest_level(VINDPM_REG, array_size, vindpm);
	vindpm = charging_parameter_to_value(VINDPM_REG, array_size, vindpm);

	LOG_INF("%s vindpm =%d\r\n", __func__, vindpm);

	//	charging_set_vindpm(vindpm);
	/*eta6965_set_en_hiz(en);*/

	return status;
}

static int eta6965_get_charging_status(struct charger_device *chg_dev,
				       bool *is_done)
{
	unsigned int status = true;
	unsigned int ret_val;

	ret_val = eta6965_get_chrg_stat();

	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int eta6965_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

	LOG_INF("%s en = %d\n", __func__, en);
	if (en) {
		eta6965_set_chg_config(0);
		eta6965_set_otg_config(1);
		eta6965_set_watchdog(0x3);	/* WDT 160s */
	} else {
		eta6965_set_otg_config(0);
		eta6965_set_chg_config(1);
	}
	return ret;
}

static int eta6965_set_boost_current_limit(struct charger_device
		*chg_dev, u32 uA)
{
	int ret = 0;
	u32 array_size = 0;
	u32 boost_ilimit = 0;
	u8 boost_reg = 0;

	uA /= 1000;
	array_size = ARRAY_SIZE(BOOST_CURRENT_LIMIT);
	boost_ilimit = bmt_find_closest_level(BOOST_CURRENT_LIMIT, array_size,
					      uA);
	boost_reg = charging_parameter_to_value(BOOST_CURRENT_LIMIT,
						array_size, boost_ilimit);
	eta6965_set_boost_lim(boost_reg);

	return ret;
}

static int eta6965_enable_safetytimer(struct charger_device *chg_dev,
				      bool en)
{
	int status = 0;

	if (en)
		eta6965_set_en_timer(0x1);
	else
		eta6965_set_en_timer(0x0);
	return status;
}

static int eta6965_get_is_safetytimer_enable(struct charger_device
		*chg_dev, bool *en)
{
	unsigned char val = 0;

	eta6965_read_interface(eta6965_CON5, &val, CON5_EN_TIMER_MASK,
			       CON5_EN_TIMER_SHIFT);
	*en = (bool)val;
	return val;
}

#if 0 //agenew 
int eta6965_get_is_power_path_enable(struct charger_device *dev, bool *en)
{
    *en = (0 == eta6965_get_en_hiz())?1:0;

	LOG_INF("agenew[%s %d] en:%d \n", __func__, __LINE__, *en);
	
	return 0;
}
static int eta6965_enable_power_path(struct charger_device *chg_dev,
				      bool en)
{
	int ret = 0;

	LOG_INF("agenew[%s %d] en:%d \n", __func__, __LINE__, en);
	if (en) {
		eta6965_set_en_hiz(0);
	} else {
		LOG_INF("agenew[%s %d]notes: OTG NOK and battery discharge to Isys!!! if in doubt, please consult the FAE \n", __func__, __LINE__);
		eta6965_set_en_hiz(1);
	}
	return ret;
}
#endif

static unsigned int charging_hw_init(void)
{
	unsigned int status = 0;
	unsigned int temp = 0;

	eta6965_set_en_hiz(0x0);
	eta6965_set_vindpm(0x7);	/* VIN DPM check 4.6V */
	eta6965_set_vindpm(0x3);	/* VIN DPM check 4.6V */
	eta6965_set_wdt_rst(0x1);	/* Kick watchdog */
	eta6965_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	eta6965_set_iprechg(0x8);	/* Precharge current 540mA */

	eta6965_set_iterm(0x1);	/* Termination current 60mA */
	__eta6965_set_iterm_via_table(g_eta6965_info->ieoc);

	eta6965_set_vreg(0x11);	/* VREG 4.4V */
	eta6965_set_pfm(0x0);//enable pfm
	eta6965_set_rdson(0x0);     /*close rdson*/
	eta6965_set_batlowv(0x1);	/* BATLOWV 3.0V */

	temp = (240000 == g_eta6965_info->vrechg) ? 1 : 0;
	eta6965_set_vrechg(temp);	/* VRECHG   0 - 120 mV,  1 - 240 mV */

	eta6965_set_en_term(0x1);	/* Enable termination */
	eta6965_set_watchdog(0x0);	/* WDT disable */
	eta6965_set_en_timer(0x0);	/* Enable charge timer */
	eta6965_set_int_mask(0x0);	/* Disable fault interrupt */
	eta6965_set_boost_lim(0x0);
	eta6965_set_vdpm_bat(0x03);
	eta6965_set_stat_ctrl(0x3);//25ab61f//BUG #73288 BUG #72320
//	eta6965_set_ovp(0x03);//12v input(14v ovp)
	eta6965_set_ovp(0x02);//9v input(10.5 ovp)
	LOG_INF("%s: hw_init down!\n", __func__);
	return status;
}

static int eta6965_parse_dt(struct eta6965_info *info,
			    struct device *dev)
{
	struct device_node *np = dev->of_node;
	//int eta6965_en_pin = 0;
	int ret = 0;
	LOG_INF("%s\n", __func__);
	if (!np) {
		LOG_ERR("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name",
				    &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		LOG_ERR("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name",
				    &(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "eta6965";
		LOG_ERR("%s: no alias name\n", __func__);
	}
	
	
		info->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(info->pinctrl)) {
		ret = PTR_ERR(info->pinctrl);
		LOG_ERR("mycat Cannot find eta6965 info->pinctrl! ret=%d\n", ret);
	}

	info->psc_chg_en_high = pinctrl_lookup_state(info->pinctrl,"psc_chg_en_high");
	if (IS_ERR(info->psc_chg_en_high))
	{
		ret = PTR_ERR(info->psc_chg_en_high);
		LOG_ERR("mycat info->psc_chg_en_high ret = %d\n",ret);
	}

	info->psc_chg_en_low = pinctrl_lookup_state(info->pinctrl,"psc_chg_en_low");
	if (IS_ERR(info->psc_chg_en_low))
	{
		ret = PTR_ERR(info->psc_chg_en_low);
		LOG_ERR("mycat info->psc_chg_en_low ret = %d\n",ret);
	}
	
	
	/*
	 * eta6965_en_pin = of_get_named_gpio(np,"gpio_eta6965_en",0);
	 * if(eta6965_en_pin < 0){
	 * LOG_INF("%s: no eta6965_en_pin\n", __func__);
	 * return -ENODATA;
	 * }
	 * gpio_request(eta6965_en_pin,"eta6965_en_pin");
	 * gpio_direction_output(eta6965_en_pin,0);
	 * gpio_set_value(eta6965_en_pin,0);
	 */
	/*
	 * if (of_property_read_string(np, "eint_name", &info->eint_name) < 0) {
	 * info->eint_name = "chr_stat";
	 * pr_debug("%s: no eint name\n", __func__);
	 * }
	 */
	return 0;
}

static int eta6965_do_event(struct charger_device *chg_dev, u32 event,
			    u32 args)
{
	unsigned char val;
	
	struct eta6965_info *info = dev_get_drvdata(&chg_dev->dev);

	if (!info->psy) {
		LOG_ERR("%s: cannot get psy\n", __func__);
		return -ENODEV;
	}
		
	val = eta6965_get_chrg_stat();
				
	LOG_DBG("%s: val = %d,event=%d\n", __func__, val,event);
	
	switch (event) {
	case EVENT_FULL:
	case EVENT_RECHARGE:
	case EVENT_DISCHARGE:
		power_supply_changed(info->psy);
		break;
	default:
		break;
	}
	return 0;
}

static int eta6965_enable_vbus(struct regulator_dev *rdev)
{
	eta6965_set_chg_config(0);
	eta6965_set_otg_config(1);

	return 0;
}

static int eta6965_disable_vbus(struct regulator_dev *rdev)
{
	eta6965_set_otg_config(0);
	eta6965_set_chg_config(1);

	return 0;
}
static int eta6965_set_ta_current_pattern(struct charger_device *chg_dev, bool is_increase)
{
    int ret = 0;
	struct eta6965_info * info = g_eta6965_info;
	
    ret = pinctrl_select_state(info->pinctrl, info->psc_chg_en_low);
    msleep(85);
	
	LOG_INF("eta6965_set_ta_current_pattern PE protocol handshake, Sending current pulse signal !\n");
	eta6965_dump_register(info->chg_dev);
 	if (is_increase) {
		eta6965_set_ichg(9);  /* 64mA */
		eta6965_set_iinlim(0);
		msleep(85);
	
		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		LOG_DBG("mtk_ta_increase() on 1");
		msleep(85);

		eta6965_set_ichg(9);/* 64mA */
		eta6965_set_iinlim(0);
		LOG_DBG("mtk_ta_increase() off 1");
		msleep(85);

		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		LOG_DBG("mtk_ta_increase() on 2");
		msleep(85);

		eta6965_set_ichg(9);/* 64mA */
		eta6965_set_iinlim(0);
		LOG_DBG("mtk_ta_increase() off 2");
		msleep(85);

		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		LOG_DBG("mtk_ta_increase() on 3");
		msleep(281);

		eta6965_set_ichg(9);/* 64mA */
		eta6965_set_iinlim(0);
		msleep(85);

		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		LOG_DBG("mtk_ta_increase() on 4");
		msleep(281);

		eta6965_set_ichg(9);/* 64mA */
		eta6965_set_iinlim(0);
		msleep(85);

		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		LOG_DBG("mtk_ta_increase() on 5");
		msleep(281);

		eta6965_set_ichg(9);/* 64mA */
		eta6965_set_iinlim(0);
		msleep(85);

		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		LOG_DBG("mtk_ta_increase() on 6");
		msleep(485);

		eta6965_set_ichg(9);/* 64mA */
		eta6965_set_iinlim(0);
		msleep(50);
		LOG_DBG("mtk_ta_increase() end\n");

		eta6965_set_ichg(9);/* 512mA */
		eta6965_set_iinlim(4);
		msleep(200);
	} else {
		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);
		msleep(85);

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
		LOG_DBG("mtk_ta_decrease() on 1");
		msleep(281);

		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);
		LOG_DBG("mtk_ta_decrease() off 1");
		msleep(85);

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
		LOG_DBG("mtk_ta_decrease() on 2");
		msleep(281);

		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);
		LOG_DBG("mtk_ta_decrease() off 2");
		msleep(85);

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
		LOG_DBG("mtk_ta_decrease() on 3");
		msleep(281);

		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);
		LOG_DBG("mtk_ta_decrease() off 3");
		msleep(85);

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
		LOG_DBG("mtk_ta_decrease() on 4");
		msleep(85);

		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);

		msleep(85);

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
		LOG_DBG("mtk_ta_decrease() on 5");
		msleep(85);

		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);

		msleep(85);

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
		LOG_DBG("mtk_ta_decrease() on 6");
		msleep(485);

		//bq25890_set_ichg(0x0);	/* 64mA */
		eta6965_set_ichg(1);
		eta6965_set_iinlim(0);
		//eta6965_set_chg_config(0);

		msleep(50);

		LOG_DBG("mtk_ta_decrease() end\n");

		//bq25890_set_ichg(0x8);	/* 512mA */
		eta6965_set_ichg(9);
		eta6965_set_iinlim(4);
		//eta6965_set_chg_config(3);
	}
	msleep(3000);
	return ret;
}
static int eta6965_is_enabled_vbus(struct regulator_dev *rdev)
{
	return eta6965_get_otg_config();
}

static const struct regulator_ops eta6965_vbus_ops = {
	.enable = eta6965_enable_vbus,
	.disable = eta6965_disable_vbus,
	.is_enabled = eta6965_is_enabled_vbus,
};

static const struct regulator_desc eta6965_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &eta6965_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static struct charger_ops eta6965_chg_ops = {
#ifdef FIXME
	.enable_hz = eta6965_enable_hz,
#endif

	/* Normal charging */
	.dump_registers = eta6965_dump_register,
	/* enable/disable charger (to battery)*/
	.enable = eta6965_enable_charging,
	.is_enabled =eta6965_is_enabled,

	/* enable/disable chip */
	//int (*enable_chip)(struct charger_device *dev, bool en);
	//int (*is_chip_enabled)(struct charger_device *dev, bool *en);

	.get_charging_current = eta6965_get_current,
	.set_charging_current = eta6965_set_current,
	.get_input_current = eta6965_get_input_current,
	.set_input_current = eta6965_set_input_current,
	.get_ibus_adc =  eta6965_get_ibus_adc,
	/*.get_constant_voltage = eta6965_get_battery_voreg,*/
	.set_constant_voltage = eta6965_set_cv_voltage,

    /* set termination current, End of charge */
    .get_eoc_current = eta6965_get_ieoc,
    .set_eoc_current = eta6965_set_ieoc,

	.kick_wdt = eta6965_reset_watch_dog_timer,
	.set_mivr = eta6965_set_vindpm_voltage,
	.is_charging_done = eta6965_get_charging_status,

	/* PE+/PE+2.0 protocol handshake, Sending current pulse signal */
    .send_ta_current_pattern = eta6965_set_ta_current_pattern,//v
    //int (*send_ta20_current_pattern)(struct charger_device *dev, u32 uV);
    //int (*reset_ta)(struct charger_device *dev);
    //int (*enable_cable_drop_comp)(struct charger_device *dev, bool en);

	/* Safety timer */
	.enable_safety_timer = eta6965_enable_safetytimer,
	.is_safety_timer_enabled = eta6965_get_is_safetytimer_enable,


	/* Power path */
//	.enable_powerpath = eta6965_enable_power_path,
//	.is_powerpath_enabled = eta6965_get_is_power_path_enable,


	/* OTG */
	.enable_otg = eta6965_enable_otg,
	.set_boost_current_limit = eta6965_set_boost_current_limit,
	.event = eta6965_do_event,
	//int (*get_ibus_adc)(struct charger_device *dev, u32 *ibus);
	//int (*get_ibat_adc)(struct charger_device *dev, u32 *ibat);  //charger_dev_get_ibat()
};

//agenew add 
static int eta6965_chg_get_drvdata_by_i2c_dts(struct eta6965_info * info)
{
	struct device_node *np = new_client->dev.of_node;

	pr_err("agenew[%s %d] entry\n", __func__, __LINE__);
	if (IS_ERR_OR_NULL(np))
	    return -ENODEV;

	if (of_property_read_u32(np, "ieoc", &info->ieoc) < 0)
		pr_err("agenew[%s %d] no ieoc\n", __func__, __LINE__);

	if (of_property_read_u32(np, "vrechg", &info->vrechg) < 0)
		pr_err("agenew[%s %d] no vrechg\n", __func__, __LINE__);

	return info ? 0 : -ENODEV;
}

static int eta6965_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct eta6965_info *info = NULL;
	
	info = g_eta6965_info;
	new_client = client;

	eta6965_chg_get_drvdata_by_i2c_dts(info);
	eta6965_hw_component_detect();
	charging_hw_init();

	eta6965_dump_register(info->chg_dev);

	eta6965_load_status = 1;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id eta6965_of_match[] = {
	{.compatible = "mediatek,eta6965"},
	{},
};
#else
static struct i2c_board_info i2c_eta6965 __initdata = {
	I2C_BOARD_INFO("eta6965", (eta6965_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver eta6965_driver = {
	.driver = {
		.name = "eta6965",
#ifdef CONFIG_OF
		.of_match_table = eta6965_of_match,
#endif
	},
	.probe = eta6965_i2c_probe,
	.id_table = eta6965_i2c_id,
};

/**********************************************************
 *
 *   [platform_driver API]
 *
 *********************************************************/
unsigned char g_reg_value_eta6965;
static ssize_t eta6965_access_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	LOG_INF("[%s] 0x%x\n", __func__, g_reg_value_eta6965);
	return sprintf(buf, "%u\n", g_reg_value_eta6965);
}

static ssize_t eta6965_access_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	LOG_INF("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		LOG_INF("[%s] buf is %s and size is %zu\n", __func__, buf,
			size);

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16,
				(unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16,
				(unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);
			LOG_INF(
			"[%s] write eta6965 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, reg_value);
			ret = eta6965_config_interface(reg_address,
				reg_value, 0xFF, 0x0);
		} else {
			ret = eta6965_read_interface(reg_address,
					     &g_reg_value_eta6965, 0xFF, 0x0);
			LOG_INF(
			"[%s] read eta6965 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, g_reg_value_eta6965);
			LOG_INF(
			"[%s] use \"cat eta6965_access\" to get value\n",
			__func__);
		}
	}
	return size;
}

static DEVICE_ATTR_RW(eta6965_access);

#if 1
static int eta6965_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct eta6965_info *chg_data = power_supply_get_drvdata(psy);
	int ret = 0;
	unsigned char chg_stat = 0;
	enum eta6965_chg_type port_stat;

	LOG_INF("%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE://4
		val->intval = g_eta6965_info->attach;
		//val->intval = eta6965_get_vbus_gd();
		//val->intval = 1;
		LOG_DBG("%s: prop = %d, vbus=%d,eta6965_get_vbus_gd()=%d,g_eta6965_info->attach=%d \n", __func__, psp,val->intval,eta6965_get_vbus_gd(),g_eta6965_info->attach);
		break;
	case POWER_SUPPLY_PROP_STATUS://0
	
		chg_stat = eta6965_get_chrg_stat();
		
		LOG_DBG("%s: prop = %d,chg_stat=%d,enum::%d,%d,%d,%d\n", __func__, psp,chg_stat,eta6965_CHG_STATUS_READY,eta6965_CHG_STATUS_PROGRESS,eta6965_CHG_STATUS_DONE,eta6965_CHG_STATUS_NOT_CHARGING);

		switch (chg_stat) {              //REG08 CHRG_STAT[1 0]
		case eta6965_CHG_STATUS_READY:   //01 - Pre-charge (< VBATLOW)
		case eta6965_CHG_STATUS_PROGRESS://10 - Fast Charging
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case eta6965_CHG_STATUS_DONE:    //11 - Charge Termination
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case eta6965_CHG_STATUS_NOT_CHARGING:   //00 - Not Charging  //Connected, not charging
			//val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;//BUG #75444 battery_charging_enabled    
			if (1 == eta6965_get_en_hiz()) {
			    val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			break;
		default:
			ret = -ENODATA;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_TYPE://63
		port_stat = eta6965_get_vbus_stat();
		LOG_INF("agenew [%s %d]: port_stat:%d \n", __func__, __LINE__, port_stat);

		switch (port_stat) {
			case ETA6965_CHG_TYPE_NO_INFO:
				LOG_DBG("%s: no information###***\n", __func__);
				//chg_data->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
				//chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;				
				break;
			case ETA6965_CHG_TYPE_UNKNOWN_TA:
				chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
				chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
				break;
			case ETA6965_CHG_TYPE_SDP:
				chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
				chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
				break;
			case ETA6965_CHG_TYPE_CDP:
#if 1	
				chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
				chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
#else
				chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
				chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
#endif
				break;
			case ETA6965_CHG_TYPE_DCP:
				chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
				chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
				break;
			case ETA6965_CHG_TYPE_NON_STANDARD_TA:
				chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
				chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
				break;
			case ETA6965_CHG_TYPE_OTG:
				LOG_DBG("%s: entry otg \n", __func__);
				break;
			default:
				ret = -ENODATA;
				break;
		}

		val->intval = chg_data->psy_desc.type;
		
		LOG_DBG("%s ,val->intval=%d,ret=%d,port_stat###=%x\n", __func__,val->intval,ret,port_stat);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE://64
	
		val->intval = chg_data->psy_usb_type;
		LOG_DBG("%s ,val->intval=%d \n", __func__,val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX://16
		if (chg_data->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX://8
		if (chg_data->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY://43
		val->intval = g_eta6965_info->vbat0_flag;
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static int eta6965_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	int ret;

	LOG_INF("%s: prop = %d,val->intval=%d\n", __func__, psp,val->intval);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE://4
		g_eta6965_info->attach = val->intval;
		LOG_DBG("%s: prop = %d,g_eta6965_info->attach=%d\n", __func__, psp,g_eta6965_info->attach);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY://43
		g_eta6965_info->vbat0_flag = val->intval;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int eta6965_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_usb_type eta6965_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property eta6965_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
};

static const struct power_supply_desc eta6965_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= eta6965_charger_properties,
	.num_properties		= ARRAY_SIZE(eta6965_charger_properties),
	.get_property		= eta6965_charger_get_property,
	.set_property		= eta6965_charger_set_property,
	.property_is_writeable	= eta6965_charger_property_is_writeable,
	.usb_types		= eta6965_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(eta6965_charger_usb_types),
};

static char *eta6965_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};
#endif

static int eta6965_platform_probe(struct platform_device *pdev)
{
	int ret_device_file = 0;
	
	int ret = 0;

	struct eta6965_info *info = NULL;
	//struct power_supply_desc *charger_desc;
	struct power_supply_config charger_cfg = {};
	struct regulator_config config = { };

	LOG_DBG("[eta6965_platform_probe]\n");
	
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	
	g_eta6965_info = info;
	g_eta6965_info->attach = 0;
	
	ret = eta6965_parse_dt(info, &pdev->dev);
	if (ret < 0)
		return ret;
	
	
	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	LOG_DBG("[eta6965_init] init start with i2c DTS");
#else
	LOG_DBG("[eta6965_init] init start. ch=%d\n", eta6965_BUSNUM);
	i2c_register_board_info(eta6965_BUSNUM, &i2c_eta6965, 1);
#endif
	if (i2c_add_driver(&eta6965_driver) != 0) {
		LOG_ERR(
			    "[eta6965_init] failed to register eta6965 i2c driver.\n");
	} else {
		LOG_DBG(
			    "[eta6965_init] Success to register eta6965 i2c driver.\n");
	}

	if (eta6965_load_status == 0) {
		LOG_ERR("[fan]Failed to register eta6965 i2c driver.\n");
		i2c_del_driver(&eta6965_driver);
		return -1;
	}
	
	

	//info = g_eta6965_info;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);
	
	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&pdev->dev, info, &eta6965_chg_ops, &eta6965_chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		LOG_ERR("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}
	
	#if 1
	/* power supply register */
	memcpy(&info->psy_desc,
		&eta6965_charger_desc, sizeof(info->psy_desc));
	info->psy_desc.name = dev_name(&pdev->dev);
	LOG_INF("agenew %s: devm_power_supply_register name:%s \n", __func__, dev_name(&pdev->dev));//get_by_phandle "charger" OR get_by_name this?

	charger_cfg.drv_data = info;
	charger_cfg.of_node = pdev->dev.of_node;
	charger_cfg.supplied_to = eta6965_charger_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(eta6965_charger_supplied_to);
	info->psy = devm_power_supply_register(&pdev->dev,
					&info->psy_desc, &charger_cfg);
	if (IS_ERR(info->psy)) {
		LOG_ERR("%s: Failed to register power supply: %d\n", __func__,ret);
		ret = PTR_ERR(info->psy);
		return ret;
	}
	#endif
	
	/* otg regulator */
	config.dev = &pdev->dev;
	config.driver_data = info;
	info->otg_rdev = devm_regulator_register(&pdev->dev,
						&eta6965_otg_rdesc, &config);
	if (IS_ERR(info->otg_rdev)) {
		ret = PTR_ERR(info->otg_rdev);
		LOG_ERR("%s: register otg regulator failed (%d)\n", __func__, ret);
		return ret;
	}
	
	LOG_INF("******** %s!! ********\n", __func__);

	ret_device_file = device_create_file(&(pdev->dev),
					     &dev_attr_eta6965_access);

	return 0;
}

static int eta6965_platform_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id mt_ofid_table[] = {
	{ .compatible = "mediatek,eta6965_chg_driver", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt_ofid_table);

static const struct platform_device_id mt_id_table[] = {
	{ "eta6965_chg_driver", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, mt_id_table);

static struct platform_driver eta6965_chg_driver = {
	.driver = {
		   .name = "eta6965_chg_driver",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mt_ofid_table),
	 },
	.probe = eta6965_platform_probe,
	.remove = eta6965_platform_remove,
	.id_table = mt_id_table,
};

static int __init eta6965_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&eta6965_chg_driver);
	if (ret) {
		LOG_ERR("****[%s] Unable to register driver (%d)\n", __func__,
			ret);
		return ret;
	}

	return 0;
}

static void __exit eta6965_exit(void)
{
	i2c_del_driver(&eta6965_driver);
}
module_init(eta6965_init);
module_exit(eta6965_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C eta6965 Driver");
MODULE_AUTHOR("author mediatek");
