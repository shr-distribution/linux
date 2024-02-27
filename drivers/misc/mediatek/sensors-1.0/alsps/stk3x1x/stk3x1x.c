/*
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pm_wakeup.h>
#include <hwmsensor.h>
#include <linux/of_gpio.h>
#include "cust_alsps.h"
#include "alsps.h"
#include "stk3x1x.h"

#define DRIVER_VERSION          "3.9.1"
#define STK_PS_POLLING_LOG
//#define STK_PS_DEBUG
#define STK_TUNE0
// #define STK_TUNE1
#define CALI_EVERY_TIME
#define STK_ALS_FIR
// #define STK_IRS
//#define STK_CHK_REG
// #define STK_GES
#define MTK_AUTO_DETECT_ALSPS

/******************************************************************************
 * configuration
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define stk3x1x_DEV_NAME     "stk3x1x"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#if 0
#define APS_FUN(f)               printk(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)    printk(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)    
#else
#define APS_FUN(f)               do {} while (0)
#define APS_ERR(fmt, args...)    do {} while (0)
#define APS_LOG(fmt, args...)    do {} while (0)
#define APS_DBG(fmt, args...)    do {} while (0) 
#endif

/******************************************************************************
 * extern functions
*******************************************************************************/
#ifdef STK_TUNE0
	#define STK_MAX_MIN_DIFF	200
	#define STK_LT_N_CT	55 // 100
	#define STK_HT_N_CT	65 // 150
#endif /* #ifdef STK_TUNE0 */

#ifdef STK_TUNE1
	#define STK_FIN_THD			(2000)
	#define STK_ALS_NEAR			(40)
	#define STK_ALS_FAR			(50)
	#define STK_PS_HIGHER_THDH	(STK_FIN_THD - 400)
	#define STK_PS_HIGHER_THDL	(STK_FIN_THD - 500)	

	#define STK_PS_CT_LIMIT			STK_FIN_THD
	#define PS_NOISE					(35)	
	#define STK_CT_UPDATE_DIFF		(40)	

	#define STK_PS_VAL_BUF			(20)
	#define STK_INT_NEAR_SAMPLE_NO	(10)
	#define STK_TUNE1_INVALID_PS_LIMIT	(5)
#endif	// #ifdef STK_TUNE1

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2		
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		748

#define STK_IRS_IT_REDUCE				2
#define STK_ALS_READ_IRS_IT_REDUCE	5
#define STK_ALS_THRESHOLD			30

#define STK3310SA_PID		0x17
#define STK3311SA_PID		0x1E
#define STK3311WV_PID		0x1D
#define STK3311X_PID			0x12
#define STK33119_PID			0x11

/*----------------------------------------------------------------------------*/
static struct i2c_client *stk3x1x_i2c_client = NULL;
static int distance_flag = 1;	/* hw default away after enable. */
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id stk3x1x_i2c_id[] = {{stk3x1x_DEV_NAME,0},{}};
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int stk3x1x_i2c_remove(struct i2c_client *client);
static int stk3x1x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_suspend(struct device *dev);
static int stk3x1x_i2c_resume(struct device *dev);
extern void app_get_als_ps_sensor_name(char *als_ps_name);

#ifndef C_I2C_FIFO_SIZE
	#define C_I2C_FIFO_SIZE     8
#endif	
static DEFINE_MUTEX(STK3X1X_i2c_mutex);
static int	stk3x1x_init_flag = -1;	// 0<==>OK -1 <==> fail
static int	stk3x1x_calibration_flag = 0;
static int  stk3x1x_local_init(void);
static int  stk3x1x_local_uninit(void);
// static struct sensor_init_info stk3x1x_init_info = {
static struct alsps_init_info stk3x1x_init_info = {
		.name = "stk3x1x",
		.init = stk3x1x_local_init,
		.uninit = stk3x1x_local_uninit,
};
//struct platform_device *alspsPltFmDev;		
#if 0
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;
/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}
#endif
/*----------------------------------------------------------------------------*/
typedef enum {
    STK_TRC_ALS_DATA= 0x0001,
    STK_TRC_PS_DATA = 0x0002,
    STK_TRC_EINT    = 0x0004,
    STK_TRC_IOCTL   = 0x0008,
    STK_TRC_I2C     = 0x0010,
    STK_TRC_CVT_ALS = 0x0020,
    STK_TRC_CVT_PS  = 0x0040,
    STK_TRC_DEBUG   = 0x8000,
} STK_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
    STK_BIT_ALS    = 1,
    STK_BIT_PS     = 2,
} STK_BIT;
/*----------------------------------------------------------------------------*/
struct stk3x1x_i2c_addr {    
/*define a series of i2c slave address*/
    u8  state;      	/* enable/disable state */
    u8  psctrl;     	/* PS control */
    u8  alsctrl;    	/* ALS control */
    u8  ledctrl;   		/* LED control */
    u8  intmode;    	/* INT mode */
    u8  wait;     		/* wait time */
    u8  thdh1_ps;   	/* PS INT threshold high 1 */
	u8	thdh2_ps;		/* PS INT threshold high 2 */
    u8  thdl1_ps;   	/* PS INT threshold low 1 */
	u8  thdl2_ps;   	/* PS INT threshold low 2 */
    u8  thdh1_als;   	/* ALS INT threshold high 1 */
	u8	thdh2_als;		/* ALS INT threshold high 2 */
    u8  thdl1_als;   	/* ALS INT threshold low 1 */
	u8  thdl2_als;   	/* ALS INT threshold low 2 */	
	u8  flag;			/* int flag */
	u8  data1_ps;		/* ps data1 */
	u8  data2_ps;		/* ps data2 */
	u8  data1_als;		/* als data1 */
	u8  data2_als;		/* als data2 */
	u8  data1_offset;	/* offset data1 */
	u8  data2_offset;	/* offset data2 */
	u8  data1_ir;		/* ir data1 */
	u8  data2_ir;		/* ir data2 */
	u8  soft_reset;		/* software reset */
};
/*----------------------------------------------------------------------------*/
#ifdef STK_ALS_FIR
	#define STK_FIR_LEN	8
	#define MAX_FIR_LEN 32
struct data_filter {
    u16 raw[MAX_FIR_LEN];
    int sum;
    int num;
    int idx;
};
#endif

#ifdef STK_GES
union stk_ges_operation{
	uint8_t ops[4];
	struct {
		uint8_t rw_len_retry;
		uint8_t reg;
		uint8_t reg_value_retry_crit;
		uint8_t sleep_10ns;
	}action;
};

union stk_ges_operation stk_ges_op[10] =
{
	{.ops={0xc1, 0x24, 0, 0}},
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}}
};
#endif

#ifdef STK_TUNE1
enum ps_int_state
{
	STK_PSINT_NORM = 0,
	STK_PSINT_FIN_DET,
	STK_PSINT_CT_DET
};
#endif

struct stk3x1x_priv {
    struct alsps_hw  hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;

    /*i2c address group*/
    struct stk3x1x_i2c_addr  addr;
    
    /*misc*/
    atomic_t    trace;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
	atomic_t    init_done;
	struct      device_node *irq_node;
	int         irq;

    /*data*/
    u16         als;
    u16         ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

	atomic_t	state_val;
	atomic_t 	psctrl_val;
	atomic_t 	alsctrl_val;
	u8 			wait_val;
	u8		 	ledctrl_val;
	u8		 	int_val;
	
    atomic_t    ps_high_thd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_low_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
	atomic_t	recv_reg;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
	bool first_boot;
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;	
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	uint16_t stk_max_min_diff;
	uint16_t stk_lt_n_ct;
	uint16_t stk_ht_n_ct;
	uint16_t crosstalk;
#ifdef CALI_EVERY_TIME
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif  
	uint16_t boot_cali;

	ktime_t ps_tune0_delay;	
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;	
#endif	
#ifdef STK_ALS_FIR
	struct data_filter      fir;
	atomic_t                firlength;		
#endif
	uint16_t ir_code;
	uint16_t als_correct_factor;	
	u16 als_last;
#ifdef STK_GES		
	struct input_dev *ges_input_dev;
	int ges_enabled;
	bool re_enable_ges;	
	int re_enable_ges2;	
	atomic_t gesture2;	
#endif	
	bool re_enable_ps;
	bool re_enable_als;
	u16 ps_cali;		
	
	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
	uint32_t als_code_last;	
	uint16_t als_data_index;
	uint8_t ps_distance_last;	
#ifdef STK_TUNE1	
	enum ps_int_state stk_int_handle_state;
	int invalid_ps_cnt;
#endif	
	uint8_t	p_1x_r_bd_with_co;	
	uint8_t	p_19_r_bc;	
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops stk3x1x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk3x1x_i2c_suspend, stk3x1x_i2c_resume)
};
#endif

static struct i2c_driver stk3x1x_i2c_driver = {	
	.probe      = stk3x1x_i2c_probe,
	.remove     = stk3x1x_i2c_remove,
	.detect     = stk3x1x_i2c_detect,
	.id_table   = stk3x1x_i2c_id,
	.driver = {
		.name           = stk3x1x_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm   = &stk3x1x_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

static struct stk3x1x_priv *stk3x1x_obj = NULL;
static int stk3x1x_get_ps_value(struct stk3x1x_priv *obj, u16 ps);
static int stk3x1x_get_ps_value_only(struct stk3x1x_priv *obj, u16 ps);
static int stk3x1x_get_als_value(struct stk3x1x_priv *obj, u16 als);
static int stk3x1x_read_als(struct i2c_client *client, u16 *data);
static int stk3x1x_read_ps(struct i2c_client *client, u16 *data);
static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj, int32_t als_it_reduce);
#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3x1x_priv *obj);
#endif
#ifdef STK_CHK_REG	
static int stk3x1x_validate_n_handle(struct i2c_client *client);
#endif
static int stk3x1x_init_client(struct i2c_client *client);
static struct wakeup_source mps_lock;
#ifdef STK_GES		
static uint32_t stk3x1x_get_ges_value(struct stk3x1x_priv *obj, unsigned int *ges0, unsigned int *ges1, unsigned int *ges2);
static int32_t stk3x1x_enable_ges(struct i2c_client *client, int enable, int mode);
#endif
static DEFINE_MUTEX(run_cali_mutex);
/*----------------------------------------------------------------------------*/
int stk3x1x_get_addr(struct alsps_hw hw, struct stk3x1x_i2c_addr *addr)
{
	if(!addr)
	{
		return -EFAULT;
	}
	addr->state   = STK_STATE_REG; 
	addr->psctrl   = STK_PSCTRL_REG;         
	addr->alsctrl  = STK_ALSCTRL_REG;
	addr->ledctrl  = STK_LEDCTRL_REG;
	addr->intmode    = STK_INT_REG;
	addr->wait    = STK_WAIT_REG;
	addr->thdh1_ps    = STK_THDH1_PS_REG;
	addr->thdh2_ps    = STK_THDH2_PS_REG;
	addr->thdl1_ps = STK_THDL1_PS_REG;
	addr->thdl2_ps = STK_THDL2_PS_REG;
	addr->thdh1_als    = STK_THDH1_ALS_REG;
	addr->thdh2_als    = STK_THDH2_ALS_REG;
	addr->thdl1_als = STK_THDL1_ALS_REG ;
	addr->thdl2_als = STK_THDL2_ALS_REG;
	addr->flag = STK_FLAG_REG;	
	addr->data1_ps = STK_DATA1_PS_REG;
	addr->data2_ps = STK_DATA2_PS_REG;
	addr->data1_als = STK_DATA1_ALS_REG;	
	addr->data2_als = STK_DATA2_ALS_REG;	
	addr->data1_offset = STK_DATA1_OFFSET_REG;
	addr->data2_offset = STK_DATA2_OFFSET_REG;
	addr->data1_ir = STK_DATA1_IR_REG;	
	addr->data2_ir = STK_DATA2_IR_REG;		
	addr->soft_reset = STK_SW_RESET_REG;	
	
	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr; 
	struct i2c_msg msgs[2] = 
	{
		{
			.addr = client->addr,	 
			.flags = 0,
			.len = 1,				 
			.buf= &beg
		},
		{
			.addr = client->addr,	 
			.flags = I2C_M_RD,
			.len = len, 			 
			.buf = data,
		}
	};

	mutex_lock(&STK3X1X_i2c_mutex);
	if (!client)
	{
		mutex_unlock(&STK3X1X_i2c_mutex);	
		return -EINVAL;
	}	
	else if (len > C_I2C_FIFO_SIZE) 
	{		 
	    mutex_unlock(&STK3X1X_i2c_mutex);
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	mutex_unlock(&STK3X1X_i2c_mutex);	
	if (err != 2) 
	{
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else 
	{
		err = 0;/*no error*/
	}
	return err;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_get_timing(void)
{
	return 200;
}

/*----------------------------------------------------------------------------*/
int stk3x1x_master_recv(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0, retry = 0;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);

	while(retry++ < max_try)
	{
		ret = stk3x1x_hwmsen_read_block(client, addr, buf, count);
		if(ret == 0)
            break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			APS_LOG("(recv) %d/%d\n", retry-1, max_try); 

		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_master_send(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	int ret = 0, retry = 0;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);

	while(retry++ < max_try)
	{
		ret = hwmsen_write_block(client, addr, buf, count);
		if (ret == 0)
		    break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			APS_LOG("(send) %d/%d\n", retry-1, max_try);
		}
	}
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_otp_read_byte_data(struct i2c_client *client, unsigned char command)
{
	int ret;
	int value;
	u8 data;

	data = 0x2;
    ret = stk3x1x_master_send(client, 0x0, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write 0x0 = %d\n", ret);
		return -EFAULT;
	} 
	
	data = command;
    ret = stk3x1x_master_send(client, 0x90, &data, 1);
	if (ret < 0)	
	{
		APS_ERR("write 0x90 = %d\n", ret);
		return -EFAULT;
	} 

	data = 0x82;
    ret = stk3x1x_master_send(client, 0x92, &data, 1);
	if (ret < 0)	
	{
		APS_ERR("write 0x0 = %d\n", ret);
		return -EFAULT;
	} 
	usleep_range(2000, 4000);

	ret = stk3x1x_master_recv(client, 0x91, &data, 1);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	value = data;
	APS_LOG("%s: 0x%x=0x%x\n", __func__, command, value);
	
	data = 0x0;
    ret = stk3x1x_master_send(client, 0x0, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write 0x0 = %d\n", ret);
		return -EFAULT;
	}
	
	return value;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_led(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    
    ret = stk3x1x_master_send(client, obj->addr.ledctrl, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write led = %d\n", ret);
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
void stk_als_ir_get_corr(struct stk3x1x_priv *obj, u32 als_data)
{
	int32_t als_comperator;	
	
	if(obj->ir_code)
	{
		obj->als_correct_factor = 1000;
		if(als_data < STK_IRC_MAX_ALS_CODE && als_data > STK_IRC_MIN_ALS_CODE && 
			obj->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = als_data * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(obj->ir_code > als_comperator)
				obj->als_correct_factor = STK_IRC_ALS_CORREC;
		}
		APS_LOG("%s: als=%d, ir=%d, als_correct_factor=%d", __func__, als_data, obj->ir_code, obj->als_correct_factor);
		obj->ir_code = 0;
	}		
	return;
}

/*----------------------------------------------------------------------------*/
#ifdef STK_IRS
int stk_als_ir_skip_als(struct stk3x1x_priv *obj)
{
	int ret;
	u8 buf[2];
	
	if(obj->als_data_index < 60000)
		obj->als_data_index++;
	else
		obj->als_data_index = 0;
		
	if(	obj->als_data_index % 10 == 1)
	{
		ret = stk3x1x_master_recv(obj->client, obj->addr.data1_als, buf, 0x02);
		if(ret < 0)
		{
			APS_DBG("error: %d\n", ret);
			return -EFAULT;
		}
		return 1;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk_als_ir_run(struct stk3x1x_priv *obj)
{
	u32 ir_data;	
	
	if(	obj->als_data_index % 10 == 0)
	{
		if(obj->ps_distance_last != 0 && obj->ir_code == 0)
		{
			ir_data =  stk3x1x_get_ir_value(obj, STK_IRS_IT_REDUCE);			
			if(ir_data > 0)
				obj->ir_code = ir_data;
		}
		return ir_data;
	}	
	return 0;
}
#endif	/* #ifdef STK_IRS */
/*----------------------------------------------------------------------------*/

#define STK_ALS_THRESHOLD	30
int stk3x1x_read_als(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];
	u32 als_data;
	int32_t ir_data;
#ifdef STK_IRS	
	const int ir_enlarge = 1 << (STK_ALS_READ_IRS_IT_REDUCE - STK_IRS_IT_REDUCE);	
#endif	
#ifdef STK_ALS_FIR
	int idx;   
	int firlen = atomic_read(&obj->firlength);   	
#endif
	if(NULL == client)
	{
		return -EINVAL;
	}	
	
#ifdef STK_IRS
	ret = stk_als_ir_skip_als(obj);
	if(ret == 1)
		return 0;
#endif	
	
	ret = stk3x1x_master_recv(client, obj->addr.data1_als, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	als_data = (buf[0] << 8) | (buf[1]);
	
	if(obj->p_1x_r_bd_with_co == 0x07 || obj->p_19_r_bc == 0x03)
	{
		als_data = als_data * 16 / 10;
		if(als_data > 65535)
			als_data = 65535;
	}
	
	if(obj->p_wv_r_bd_with_co & 0b010)
	{
		if(als_data < STK_ALS_THRESHOLD && obj->als_code_last > 10000)
		{
			ir_data = stk3x1x_get_ir_value(obj, STK_ALS_READ_IRS_IT_REDUCE);
#ifdef STK_IRS				
			if(ir_data > 0)
				obj->ir_code = ir_data * ir_enlarge;
#endif				
			if(ir_data > (STK_ALS_THRESHOLD * 3))
			{
				als_data = obj->als_code_last;
			}
		}
#ifdef STK_IRS			
		else
		{
			obj->ir_code = 0;
		}
#endif		
	}
	obj->als_code_last = als_data;
	
#ifdef STK_ALS_FIR
	if(obj->fir.num < firlen)
	{                
		obj->fir.raw[obj->fir.num] = als_data;
		obj->fir.sum += als_data;
		obj->fir.num++;
		obj->fir.idx++;
	}
	else
	{
		idx = obj->fir.idx % firlen;
		obj->fir.sum -= obj->fir.raw[idx];
		obj->fir.raw[idx] = als_data;
		obj->fir.sum += als_data;
		obj->fir.idx++;
		als_data = (obj->fir.sum / firlen);
	}	
#endif

#ifdef STK_IRS
	stk_als_ir_get_corr(obj, als_data);
	als_data = als_data * obj->als_correct_factor / 1000;
#endif	

	*data = (u16)als_data;
	
#ifdef STK_IRS		
	stk_als_ir_run(obj);
#endif	
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("ALS: 0x%04X\n", (u32)(*data));
	}
	return 0; 
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    ret = stk3x1x_master_send(client, obj->addr.alsctrl, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write als = %d\n", ret);
		return -EFAULT;
	}	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_state(struct i2c_client *client, u8 *data)
{ 
	int ret = 0;
	u8 buf;
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, STK_STATE_REG, &buf, 0x01);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = buf;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_flag(struct i2c_client *client, u8 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf;
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.flag, &buf, 0x01);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = buf;
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS NF flag: 0x%04X\n", (u32)(*data));
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_id(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);	
	int ret = 0;
	u8 buf[2];
	u8 pid_msb;
	int otp25 = 0;
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	obj->p_wv_r_bd_with_co = 0;
	obj->p_1x_r_bd_with_co = 0;	
	obj->p_19_r_bc = 0;	
	
	ret = stk3x1x_master_recv(client, STK_PDT_ID_REG, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	obj->pid = buf[0];
	APS_LOG("%s: PID=0x%x, VID=0x%x\n", __func__, buf[0], buf[1]);
	
	if(obj->pid == STK3310SA_PID || obj->pid == STK3311SA_PID)
		obj->ledctrl_val &= 0x3F;		
	
	if(buf[0] == STK3311WV_PID)
		obj->p_wv_r_bd_with_co |= 0b100;
	else if(buf[0] == STK3311X_PID)
		obj->p_1x_r_bd_with_co |= 0b100;
	else if(buf[0] == STK33119_PID)
		obj->p_19_r_bc |= 0b10;		
	
	if(buf[1] == 0xC3)
	{
		obj->p_wv_r_bd_with_co |= 0b010;
		obj->p_1x_r_bd_with_co |= 0b010;
	}
	else if(buf[1] == 0xC2)
	{
		obj->p_19_r_bc |= 0b01;
	}
		
	ret = stk3x1x_otp_read_byte_data(client, 0x25);
	if(ret < 0)
		return ret;
	otp25 = ret;
	if(otp25 & 0x80)	
		obj->p_wv_r_bd_with_co |= 0b001;
	APS_LOG("%s: p_wv_r_bd_with_co = 0x%x\n", __func__, obj->p_wv_r_bd_with_co);	
	
	if(otp25 & 0x40)
		obj->p_1x_r_bd_with_co |= 0b001;
	APS_LOG("%s: p_1x_r_bd_with_co = 0x%x\n", __func__, obj->p_1x_r_bd_with_co);		
	APS_LOG("%s: p_19_r_bc = 0x%x\n", __func__, obj->p_19_r_bc);			
	
	if(buf[0] == 0)
	{
		APS_ERR( "PID=0x0, please make sure the chip is stk3x1x!\n");
		return -2;			
	}
	
	pid_msb = buf[0] & 0xF0;
	switch(pid_msb)
	{
	case 0x10:
	case 0x20:
	case 0x30:
		return 0;
	default:
		APS_ERR( "invalid PID(%#x)\n", buf[0]);	
		return -1;
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_ps(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];
	
	if(NULL == client)
	{
		APS_ERR("i2c client is NULL\n");
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.data1_ps, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = (buf[0] << 8) | (buf[1]);
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS: 0x%04X\n", (u32)(*data));
	}
	
	return 0;     
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.psctrl, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_wait(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.wait, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write wait = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_int(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.intmode, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write intmode = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_state(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.state, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write state = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_flag(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.flag, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_sw_reset(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf = 0, r_buf = 0;	
	int ret = 0;

	buf = 0x7F;
    ret = stk3x1x_master_send(client, obj->addr.wait, (char*)&buf, sizeof(buf));
	if (ret < 0)
	{
		APS_ERR("i2c write test error = %d\n", ret);
		return -EFAULT;
	} 	
	
    ret = stk3x1x_master_recv(client, obj->addr.wait, &r_buf, 1);
	if (ret < 0)
	{
		APS_ERR("i2c read test error = %d\n", ret);
		return -EFAULT;
	}	
	
	if(buf != r_buf)
	{
        APS_ERR("i2c r/w test error, read-back value is not the same, write=0x%x, read=0x%x\n", buf, r_buf);		
		return -EIO;
	}
	
	buf = 0;
    ret = stk3x1x_master_send(client, obj->addr.soft_reset, (char*)&buf, sizeof(buf));
	if (ret < 0)
	{
		APS_ERR("write software reset error = %d\n", ret);
		return -EFAULT;
	} 
	msleep(13);
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps_high_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	

    ret = stk3x1x_master_send(client, obj->addr.thdh1_ps, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %d\n",  ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdh2_ps, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %d\n", ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps_low_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdl1_ps, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdl2_ps, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_high_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdh1_als, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdh2_als, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_low_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdl1_als, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdl2_als, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
#ifdef STK_PS_DEBUG
static int show_allreg(void)
{
	int ret = 0;
	u8 rbuf[0x22];
	int cnt;	
	
	memset(rbuf, 0, sizeof(rbuf));
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0, &rbuf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 7, &rbuf[7], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 14, &rbuf[14], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 21, &rbuf[21], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 28, &rbuf[28], 4);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, STK_PDT_ID_REG, &rbuf[32], 2);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		APS_LOG("reg[0x%x]=0x%x\n", cnt, rbuf[cnt]);
	}	
	APS_LOG("reg[0x3E]=0x%x\n", rbuf[cnt]);
	APS_LOG("reg[0x3F]=0x%x\n", rbuf[cnt++]);

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static void stk_ps_report(struct stk3x1x_priv *obj, int nf)
{
	if(ps_report_interrupt_data(nf))
		APS_ERR("call ps_report_interrupt_data fail\n");
	
	APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps, nf);					
	obj->ps_distance_last = nf;		
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_als(struct i2c_client *client, int enable)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->state_val);
	int trc = atomic_read(&obj->trace);

	APS_LOG("%s: enable=%d\n", __func__, enable);
	
#ifdef STK_GES		
	if(obj->ges_enabled)
	{
		APS_LOG( "%s: since ges is enabled, ALS is disabled\n", __func__);
		obj->re_enable_als = enable ? true : false;		
		return 0;
	}
#endif		
	cur = old & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)); 
	if(enable)
	{
		cur |= STK_STATE_EN_ALS_MASK;
	}
	else if (old & STK_STATE_EN_PS_MASK)
	{
		cur |= STK_STATE_EN_WAIT_MASK;   
	}
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		return 0;
	}
		
#ifdef STK_IRS		
	if(enable && !(old & STK_STATE_EN_PS_MASK))
	{		
		err =  stk3x1x_get_ir_value(obj, STK_IRS_IT_REDUCE);
		if(err > 0)
			obj->ir_code = err;
	}			
#endif
		
	if(enable && obj->hw.polling_mode_als == 0)
	{
		stk3x1x_write_als_high_thd(client, 0x0);
		stk3x1x_write_als_low_thd(client, 0xFFFF);
	}
	err = stk3x1x_write_state(client, cur);
	if(err < 0)
		return err;
	else
		atomic_set(&obj->state_val, cur);
	
	if(enable)
	{
		
		obj->als_last = 0;
		if(obj->hw.polling_mode_als)
		{
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)*HZ/1000);
		}
		else
		{
			//set_bit(STK_BIT_ALS,  &obj->pending_intr);
			schedule_delayed_work(&obj->eint_work,220*HZ/1000); 
		}
	}
#ifdef STK_IRS
		obj->als_data_index = 0;
#endif	
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("enable als (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_ps_set_thd(struct stk3x1x_priv *obj)
{
	int err;
	
#ifdef STK_TUNE1
	#ifdef CALI_EVERY_TIME
	if( obj->crosstalk > 0 )
	{
		atomic_set(&obj->ps_high_thd_val, obj->ps_high_thd_boot); 
		atomic_set(&obj->ps_low_thd_val, obj->ps_low_thd_boot); 
	}
	else if( obj->ps <= STK_FIN_THD )
	{
		atomic_set(&obj->ps_high_thd_val, obj->ps + obj->stk_ht_n_ct); 
		atomic_set(&obj->ps_low_thd_val, obj->ps + obj->stk_lt_n_ct); 
	}
	else if(obj->boot_cali == 1 && obj->ps_low_thd_boot < STK_PS_HIGHER_THDH )
	{
		atomic_set(&obj->ps_high_thd_val, obj->ps_high_thd_boot); 
		atomic_set(&obj->ps_low_thd_val, obj->ps_low_thd_boot);
	}
	else
	#endif
	{
		atomic_set(&obj->ps_high_thd_val, STK_PS_HIGHER_THDH); 
		atomic_set(&obj->ps_low_thd_val, STK_PS_HIGHER_THDL); 	
	}

	if( atomic_read(&obj->ps_high_thd_val) > STK_PS_HIGHER_THDH)
	{
		atomic_set(&obj->ps_high_thd_val, STK_PS_HIGHER_THDH); 
		atomic_set(&obj->ps_low_thd_val, STK_PS_HIGHER_THDL); 			
	}

	if ((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val)))) 
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;
	}
	if ((err = stk3x1x_write_ps_low_thd(obj->client, 0)))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;
	}
	APS_LOG("%s:in reg, HT=%d, LT=0\n", __func__, atomic_read(&obj->ps_high_thd_val));
#elif defined(STK_TUNE0)
	
	#ifdef CALI_EVERY_TIME
	if( obj->crosstalk > 0)
	{
		atomic_set(&obj->ps_high_thd_val, obj->ps_high_thd_boot); 
		atomic_set(&obj->ps_low_thd_val, obj->ps_low_thd_boot); 
	}
	else if(obj->boot_cali == 1 && obj->ps_low_thd_boot < 1000 )
	{
		atomic_set(&obj->ps_high_thd_val, obj->ps_high_thd_boot); 
		atomic_set(&obj->ps_low_thd_val, obj->ps_low_thd_boot);
	}
	#endif				
	if ((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;
	}
	if ((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;
	}
	APS_LOG("%s:in reg, HT=%d, LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));		
#else
	if ((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;
	}
	if ((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;
	}
	APS_LOG("%s:in reg, HT=%d, LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));			
#endif

	APS_LOG("%s:in driver, HT=%d, LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_ps(struct i2c_client *client, int enable, int validate_reg)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->state_val);
	int trc = atomic_read(&obj->trace);

#ifdef STK_PS_DEBUG
	if(enable)
	{
		APS_LOG("%s: before enable ps, HT=%d, LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val),  atomic_read(&obj->ps_low_thd_val));
		APS_LOG("%s: before enable ps, show all reg\n", __FUNCTION__);
		show_allreg(); //for debug
	}
#endif

	// APS_LOG("%s: enable=%d\n", __FUNCTION__, enable);	
	
	cur = old;		
	cur &= (~(0x45)); 
	if(enable)
	{
		cur |= (STK_STATE_EN_PS_MASK);
		if(!(old & STK_STATE_EN_ALS_MASK))
			cur |= STK_STATE_EN_WAIT_MASK;
	}

	if(0 == (cur ^ old))
	{
		APS_LOG("%s: repeat enable=%d, reg=0x%x\n", __FUNCTION__, enable, cur);
		return 0;
	}
		
#ifdef STK_GES		
	
	if(obj->ges_enabled)
	{
		if(enable)
		{
			APS_LOG( "%s: force disable Ges, mode = %d\n", __func__, obj->ges_enabled);
			obj->re_enable_ges2 = obj->ges_enabled;
			stk3x1x_enable_ges(obj->client, 0, obj->ges_enabled);
			old = atomic_read(&obj->state_val);
		}
		else
		{
			APS_LOG( "%s: ps is disabled\n", __func__);	
			return 0;
		}
	}
#endif	
	
#ifdef STK_CHK_REG	
	if(validate_reg)
	{
		err = stk3x1x_validate_n_handle(obj->client);
		if(err < 0)	
		{
			APS_ERR("stk3x1x_validate_n_handle fail: %d\n", err); 
		}	
	}		
#endif	
#ifdef STK_TUNE0		
	if (!(obj->psi_set) && !enable)
	{
		hrtimer_cancel(&obj->ps_tune0_timer);	
		cancel_work_sync(&obj->stk_ps_tune0_work);
	}	
#endif

	if(obj->first_boot == true)
	{		
		obj->first_boot = false;
	}

	if(enable)
	{
		stk3x1x_enable_ps_set_thd(obj);
		if(1 == obj->hw.polling_mode_ps)
			__pm_stay_awake(&mps_lock);
	}
	else
	{
		if(1 == obj->hw.polling_mode_ps)
				__pm_relax(&mps_lock);
	}
	
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}

	err = stk3x1x_write_state(client, cur);
	if(err < 0)
		return err;
	atomic_set(&obj->state_val, cur);
	
	if(enable)
	{
#ifdef STK_TUNE0
	#ifndef CALI_EVERY_TIME
		if (!(obj->psi_set))
			hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);	
	#else
		obj->psi_set = 0;
		obj->psa = 0;
		obj->psi = 0xFFFF;
		hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);			
	#endif
#endif	
#ifdef STK_TUNE1	
		obj->stk_int_handle_state = STK_PSINT_NORM;
#endif
		if(obj->hw.polling_mode_ps)
		{
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)*HZ/1000);
		}
		else
		{
#ifdef STK_CHK_REG				
			if(!validate_reg)
			{
				stk_ps_report(obj, 1);
			}
			else
#endif			
			{
				msleep(5);
				if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
				{
					APS_ERR("stk3x1x read ps data: %d\n", err);
					return err;
				}
#ifdef STK_TUNE1
			//	err = (obj->ps >= atomic_read(&obj->ps_high_thd_val))?0:1;
				err = 1;
#else					
				err = stk3x1x_get_ps_value_only(obj, obj->ps);
#endif			
				if(err < 0)
				{
					APS_ERR("stk3x1x get ps value: %d\n", err);
					return err;
				}
				else if(stk3x1x_obj->hw.polling_mode_ps == 0)
				{	
					distance_flag = err;
					APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps,
									distance_flag);
					stk_ps_report(obj, distance_flag);
				}
				
			#ifdef STK_PS_DEBUG
				APS_LOG("%s: after stk_ps_report, show all reg\n", __FUNCTION__);
				show_allreg(); //for debug
			#endif
			}
		}
	}
#ifdef STK_GES	
	else
	{
		if(obj->re_enable_ges2)
		{
			cur = obj->re_enable_ges2;
			obj->re_enable_ges2 = 0;
			APS_LOG( "%s: re-enable Ges, mode = %d\n", __func__, cur);
			stk3x1x_enable_ges(obj->client, 1, cur);
		}
	}
#endif	

	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("enable ps  (%d)\n", enable);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef STK_GES
static int32_t stk3x1x_enable_ges(struct i2c_client *client, int enable, int mode)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);	
	u8 reg;	
	int err;
	int org_state_reg = atomic_read(&obj->state_val);
	int org_mode = 0;
	
	APS_LOG("%s: enable=%d\n", __FUNCTION__, enable);		
	
	if(enable == obj->ges_enabled)
		return 0;
	
	if(enable)
	{
		if(org_state_reg & STK_STATE_EN_PS_MASK)
		{
		//	APS_LOG( "%s: force disable PS\n", __func__);
		//	stk3x1x_enable_ps(obj->client, 0, 1);
		//	obj->re_enable_ps = true;
			APS_LOG( "%s: since PS is enabled, Ges is disabled, mode = %d\n", __func__, mode);
			obj->re_enable_ges2 = mode;	
			return 0;
		}
		if(org_state_reg & STK_STATE_EN_ALS_MASK) 
		{
			APS_LOG( "%s: force disable ALS\n", __func__);						
			stk3x1x_enable_als(obj->client, 0);		
			obj->re_enable_als = true;
		}
		
		if((err = stk3x1x_write_wait(client, 0)))
		{
			APS_ERR("write wait error: %d\n", err);
			return err;
		}		
		
		if((err = stk3x1x_write_int(obj->client, 0)))
		{
			APS_ERR("write int mode error: %d\n", err);
			return err;        
		}				
			
		reg = STK_STATE_EN_WAIT_MASK | STK_STATE_EN_PS_MASK; 
		err = stk3x1x_write_state(client, reg);
		if(err < 0)
			return err;
		atomic_set(&obj->state_val, reg);	
		obj->ges_enabled = mode;		
	}
	else
	{
		org_mode = obj->ges_enabled;
		if(org_mode == 2)
		{	
			if(obj->hw.polling_mode_ps == 1)
			{
				hrtimer_cancel(&obj->ps_tune0_timer);					
				cancel_work_sync(&obj->stk_ps_tune0_work);					
			}
			else
			{
#if defined(CONFIG_OF)
		disable_irq(obj->irq); 
#elif ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))
				mt65xx_eint_mask(CUST_EINT_ALS_NUM);    
#else				
				mt_eint_mask(CUST_EINT_ALS_NUM);
#endif						
			}
		}
		
		err = stk3x1x_write_state(client, 0);
		if(err < 0)
			return err;	
		atomic_set(&obj->state_val, 0);				
		if((err = stk3x1x_write_wait(client, obj->wait_val)))
		{
			APS_ERR("write wait error: %d\n", err);
			return err;
		}	
				
		if((err = stk3x1x_write_int(obj->client, obj->int_val)))
		{
			APS_ERR("write int mode error: %d\n", err);
			return err;        
		}				
		obj->ges_enabled = 0;
		if(obj->re_enable_als) 
		{
			APS_LOG( "%s: re-enable ALS\n", __func__);
			stk3x1x_enable_als(obj->client, 1);
			obj->re_enable_als = false;
		}
	}	

	return 0;
}
#endif /* #ifdef STK_GES */

/*----------------------------------------------------------------------------*/
static int stk3x1x_ps_calibration(struct i2c_client *client) 
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);	
	u8 reg;	
	int err = 0;
	int org_state_reg = atomic_read(&obj->state_val);
	bool re_en_ps = false, re_en_als = false;
	int counter = 0;
	uint32_t ps_sum = 0;
	const int PS_CALI_COUNT = 5;
	
#ifdef STK_GES	
	bool re_en_ges = false;
	
	if(obj->ges_enabled)
	{
		stk3x1x_enable_ges(obj->client, 0, 1);
		re_en_ges = true;
	}
#endif	
	if(org_state_reg & STK_STATE_EN_PS_MASK)
	{
		APS_LOG( "%s: force disable PS\n", __func__);
		stk3x1x_enable_ps(obj->client, 0, 1);
		re_en_ps = true;
	}
	
	if(org_state_reg & STK_STATE_EN_ALS_MASK) 
	{
		APS_LOG( "%s: force disable ALS\n", __func__);						
		stk3x1x_enable_als(obj->client, 0);		
		re_en_als = true;
	}
#if defined(CONFIG_OF)
	disable_irq(obj->irq); 
#elif ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))
	mt65xx_eint_mask(CUST_EINT_ALS_NUM);    
#else				
	mt_eint_mask(CUST_EINT_ALS_NUM);
#endif	
	
	reg = STK_STATE_EN_WAIT_MASK | STK_STATE_EN_PS_MASK; 
	err = stk3x1x_write_state(client, reg);
	if(err < 0)
		goto err_out;
	
	while(counter < PS_CALI_COUNT)
	{
		msleep(60);
		if((err = stk3x1x_read_ps(obj->client, &obj->ps)))		
			goto err_out;
//#ifdef STK_PS_POLLING_LOG			
		APS_LOG( "%s: ps=%d\n", __func__, obj->ps);		
//#endif		
		ps_sum += obj->ps;		
		counter++;
	}
	
	obj->ps_cali = (ps_sum / PS_CALI_COUNT);
#ifdef STK_TUNE0
	obj->crosstalk = (ps_sum / PS_CALI_COUNT);
	
	#ifdef STK_TUNE1
	if (obj->crosstalk == 0 || obj->crosstalk > STK_PS_HIGHER_THDL)
	#else
	if (obj->crosstalk == 0)
	#endif	
	{
	#ifdef STK_TUNE1			
		obj->crosstalk = STK_PS_HIGHER_THDL;
	#else
		obj->crosstalk = obj->hw.ps_threshold_low;
	#endif			
		stk3x1x_calibration_flag = 0;
	}else {
		stk3x1x_calibration_flag = 1;	
	}
#endif	
#ifdef CALI_EVERY_TIME	
	obj->ps_high_thd_boot = obj->crosstalk + obj->stk_ht_n_ct;
	obj->ps_low_thd_boot = obj->crosstalk + obj->stk_lt_n_ct;
	// printk( "%s: crosstalk=%d, high thd=%d, low thd=%d\n", __func__, obj->crosstalk, obj->ps_high_thd_boot, obj->ps_low_thd_boot);
#endif

err_out:	
#if defined(CONFIG_OF)
	disable_irq(obj->irq); 
#elif ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))
	mt65xx_eint_mask(CUST_EINT_ALS_NUM);    
#else				
	mt_eint_mask(CUST_EINT_ALS_NUM);
#endif
	
	if(re_en_als) 
	{
		APS_LOG( "%s: re-enable ALS\n", __func__);
		stk3x1x_enable_als(obj->client, 1);
	}
		
	if(re_en_ps)
	{
		APS_LOG( "%s: re-enable PS\n", __func__);
		stk3x1x_enable_ps(obj->client, 1, 1);
	}
#ifdef STK_GES	
	if(re_en_ges)	
		stk3x1x_enable_ges(obj->client, 1, 1);		
#endif		
	return err;
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_check_intr(struct i2c_client *client, u8 *status) 
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err;

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	err = stk3x1x_read_flag(client, status);	
	if (err < 0)
	{
		APS_ERR("WARNING: read flag reg error: %d\n", err);
		return -EFAULT;
	}
	APS_LOG("%s: read status reg: 0x%x\n", __func__, *status);
    
	if(*status & STK_FLG_ALSINT_MASK)
	{
		set_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	else
	{
	   clear_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	
	if(*status & STK_FLG_PSINT_MASK)
	{
		set_bit(STK_BIT_PS,  &obj->pending_intr);
	}
	else
	{
	    clear_bit(STK_BIT_PS, &obj->pending_intr);
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_DEBUG)
	{
		APS_LOG("check intr: 0x%02X => 0x%08lX\n", *status, obj->pending_intr);
	}

	return 0;
}


static int stk3x1x_clear_intr(struct i2c_client *client, u8 status, u8 disable_flag) 
{
    int err = 0;

    status = status | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
    status &= (~disable_flag);
	//APS_LOG(" set flag reg: 0x%x\n", status);
	if((err = stk3x1x_write_flag(client, status)))
		APS_ERR("stk3x1x_write_flag failed, err=%d\n", err);
    return err;
}

/*----------------------------------------------------------------------------*/
#ifdef STK_CHK_REG	
static int stk3x1x_chk_reg_valid(struct stk3x1x_priv *obj) 
{    
	int ret = 0;	
	u8 buf[9];
	
	if(NULL == obj)
	{
		return -EINVAL;
	}	
	memset(buf, 0, sizeof(buf));

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 1, &buf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 8, &buf[7], 2);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}		

	if(buf[0] != atomic_read(&obj->psctrl_val))
	{
		APS_ERR("%s: invalid reg 0x01=0x%2x\n", __func__, buf[0]);
		return 0xFF;
	}
#ifdef STK_IRS	
	if(buf[1] != atomic_read(&obj->alsctrl_val) && buf[1] != (atomic_read(&obj->alsctrl_val) - STK_IRS_IT_REDUCE)
		&& buf[1] != (atomic_read(&obj->alsctrl_val) - STK_ALS_READ_IRS_IT_REDUCE))
#else
	if(buf[1] != atomic_read(&obj->alsctrl_val)	&& buf[1] != (atomic_read(&obj->alsctrl_val) - STK_ALS_READ_IRS_IT_REDUCE))
#endif		
	{
		APS_ERR("%s: invalid reg 0x02=0x%2x\n", __func__, buf[1]);
		return 0xFF;
	}
	if(buf[2] != obj->ledctrl_val)
	{
		APS_ERR("%s: invalid reg 0x03=0x%2x\n", __func__, buf[2]);
		return 0xFF;
	}		
	if(buf[3] != obj->int_val)
	{
		APS_ERR("%s: invalid reg 0x04=0x%2x\n", __func__, buf[3]);
		return 0xFF;
	}
	if(buf[4] != obj->wait_val)
	{
		APS_ERR("%s: invalid reg 0x05=0x%2x\n", __func__, buf[4]);
		return 0xFF;
	}	
	if(buf[5] != (atomic_read(&obj->ps_high_thd_val) & 0xFF00) >> 8)
	{
		APS_ERR("%s: invalid reg 0x06=0x%2x\n", __func__, buf[5]);
		return 0xFF;
	}	
	if(buf[6] != (atomic_read(&obj->ps_high_thd_val) & 0x00FF))
	{
		APS_ERR("%s: invalid reg 0x07=0x%2x\n", __func__, buf[6]);
		return 0xFF;
	}	
	if(buf[7] != (atomic_read(&obj->ps_low_thd_val) & 0xFF00) >> 8)
	{
		APS_ERR("%s: invalid reg 0x08=0x%2x\n", __func__, buf[7]);
		return 0xFF;
	}	
	if(buf[8] != (atomic_read(&obj->ps_low_thd_val) & 0x00FF))
	{
		APS_ERR("%s: invalid reg 0x09=0x%2x\n", __func__, buf[8]);
		return 0xFF;
	}	
		
	return 0;
}

static int stk3x1x_validate_n_handle(struct i2c_client *client) 
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client); 
	int err;
	
	err = stk3x1x_chk_reg_valid(obj);
	if(err < 0)
	{
		APS_ERR("stk3x1x_chk_reg_valid fail: %d\n", err);        
		return err;
	}
	
	if(err == 0xFF)
	{		
		APS_ERR("%s: Re-init chip\n", __func__);
		stk3x1x_init_client(obj->client);		
		//obj->psa = 0;
		//obj->psi = 0xFFFF;		
		if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}
		
		if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}		
		
		return 0xFF;
	}
	return 0;
}
#endif /* #ifdef STK_CHK_REG	*/
/*----------------------------------------------------------------------------*/
static int stk3x1x_ps_val(void)
{
	int mode;
	int32_t word_data, lii;
	u8 buf[4];
	int ret;
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x20, buf, 4);
	if(ret < 0)	
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}
	word_data = (buf[0] << 8) | buf[1];
	word_data += (buf[2] << 8) | buf[3];	
	
	mode = atomic_read(&stk3x1x_obj->psctrl_val) & 0x3F;
	if(mode == 0x30)	
	{
		lii = 100;		
	}
	else if (mode == 0x31)
	{
		lii = 200;		
	}
	else if (mode == 0x32)
	{
		lii = 400;				
	}
	else if (mode == 0x33)
	{
		lii = 800;			
	}
	else
	{
		APS_ERR("%s: unsupported PS_IT(0x%x)\n", __FUNCTION__, mode);
		return -1;
	}
	
	if(word_data > lii)	
	{
		APS_LOG( "%s: word_data=%d, lii=%d\n", __FUNCTION__, word_data, lii);		
		return 0xFFFF;	
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef STK_GES		
static int32_t stk_ges_poll_func(struct stk3x1x_priv *obj)
{
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#ifdef STK_TUNE0	
static int stk_ps_tune_zero_final(struct stk3x1x_priv *obj)
{
	int err;
	
	obj->tune_zero_init_proc = false;
	if((err = stk3x1x_write_int(obj->client, obj->int_val)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
	
	if((err = stk3x1x_write_state(obj->client, atomic_read(&obj->state_val))))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}		
	
	if(obj->data_count == -1)
	{
		APS_LOG("%s: exceed limit\n", __func__);
		hrtimer_cancel(&obj->ps_tune0_timer);	
		return 0;
	}	
	
	obj->psa = obj->ps_stat_data[0];
	obj->psi = obj->ps_stat_data[2];							

#ifndef CALI_EVERY_TIME
	atomic_set(&obj->ps_high_thd_val, obj->ps_stat_data[1] + obj->stk_ht_n_ct); 
	atomic_set(&obj->ps_low_thd_val, obj->ps_stat_data[1] + obj->stk_lt_n_ct); 		
#else						
	obj->ps_high_thd_boot = obj->ps_stat_data[1] + obj->stk_ht_n_ct;
	obj->ps_low_thd_boot = obj->ps_stat_data[1] + obj->stk_lt_n_ct;
	atomic_set(&obj->ps_high_thd_val, obj->ps_high_thd_boot); 
	atomic_set(&obj->ps_low_thd_val, obj->ps_low_thd_boot); 
#endif
//	obj->crosstalk = obj->ps_stat_data[1];

	if((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;        
	}	
	if((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;        
	}
	obj->boot_cali = 1;
	
	APS_LOG("%s: set HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val),  atomic_read(&obj->ps_low_thd_val));		
	hrtimer_cancel(&obj->ps_tune0_timer);					
	return 0;
}
/*----------------------------------------------------------------------------*/
static int32_t stk_tune_zero_get_ps_data(struct stk3x1x_priv *obj)
{
	int err;
	
	err = stk3x1x_ps_val();	
	if(err == 0xFFFF)
	{	
		obj->data_count = -1;
		stk_ps_tune_zero_final(obj);
		return 0;	
	}
	
	if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
	{
		APS_ERR("stk3x1x read ps data: %d\n", err);
		return err;
	}	
	APS_LOG("%s: ps #%d=%d\n", __func__, obj->data_count, obj->ps);
	
	obj->ps_stat_data[1]  +=  obj->ps;			
	if(obj->ps > obj->ps_stat_data[0])
		obj->ps_stat_data[0] = obj->ps;
	if(obj->ps < obj->ps_stat_data[2])
		obj->ps_stat_data[2] = obj->ps;						
	obj->data_count++;	
	
	if(obj->data_count == 5)
	{
		obj->ps_stat_data[1]  /= obj->data_count;			
		stk_ps_tune_zero_final(obj);
	}		
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk_ps_tune_zero_init(struct stk3x1x_priv *obj)
{
	u8 w_state_reg;	
	int err;
	
	obj->psa = 0;
	obj->psi = 0xFFFF;	
	obj->psi_set = 0;	
	obj->tune_zero_init_proc = true;		
	obj->ps_stat_data[0] = 0;
	obj->ps_stat_data[2] = 9999;
	obj->ps_stat_data[1] = 0;
	obj->data_count = 0;
	obj->boot_cali = 0;
	
#ifdef CALI_EVERY_TIME
	obj->ps_high_thd_boot = obj->hw.ps_threshold_high;
	obj->ps_low_thd_boot = obj->hw.ps_threshold_low;
	if(obj->ps_high_thd_boot <= 0)
	{
		obj->ps_high_thd_boot = obj->stk_ht_n_ct*3;
		obj->ps_low_thd_boot = obj->stk_lt_n_ct*3;
	}
#endif
	
	if((err = stk3x1x_write_int(obj->client, 0)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
	
	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);			
	if((err = stk3x1x_write_state(obj->client, w_state_reg)))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}			
	hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);		
	return 0;	
}
/*----------------------------------------------------------------------------*/
static int stk_ps_tune_zero_func_fae(struct stk3x1x_priv *obj)
{
	int32_t word_data;
	u8 flag;
	bool ps_enabled = false;
	u8 buf[2];
	int ret, diff;
	
	ps_enabled = (atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK) ? true : false;	

#ifndef CALI_EVERY_TIME
	if(obj->psi_set || !(ps_enabled))
#else
	if(!(ps_enabled))
#endif
	{
		return 0;
	}	
	
	ret = stk3x1x_read_flag(obj->client, &flag);
	if(ret < 0)
	{
		APS_ERR( "%s: get flag failed, err=0x%x\n", __func__, ret);
		return ret;
	}
	if(!(flag&STK_FLG_PSDR_MASK))
	{
		return 0;
	}
	
	ret = stk3x1x_ps_val();	
	if(ret == 0)
	{
		ret = stk3x1x_master_recv(obj->client, 0x11, buf, 2);
		if(ret < 0)
		{
			APS_ERR( "%s fail, err=0x%x", __func__, ret);
			return ret;	   
		}
		word_data = (buf[0] << 8) | buf[1];
		//APS_LOG("%s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			//APS_ERR( "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		
		if(word_data > obj->psa)
		{
			obj->psa = word_data;
			APS_LOG("%s: update psa: psa=%d,psi=%d\n", __func__, obj->psa, obj->psi);
		}
		if(word_data < obj->psi)
		{
			obj->psi = word_data;	
			APS_LOG("%s: update psi: psa=%d,psi=%d\n", __func__, obj->psa, obj->psi);	
		}	
	}	
	
	diff = obj->psa - obj->psi;
	if(diff > obj->stk_max_min_diff)
	{
		obj->psi_set = obj->psi;
#ifndef STK_TUNE1
		atomic_set(&obj->ps_high_thd_val, obj->psi + obj->stk_ht_n_ct); 
		atomic_set(&obj->ps_low_thd_val, obj->psi + obj->stk_lt_n_ct); 
		
#ifdef CALI_EVERY_TIME
		if( (atomic_read(&obj->ps_high_thd_val) > (obj->ps_high_thd_boot + 500)) && (obj->boot_cali == 1) )
		{
			atomic_set(&obj->ps_high_thd_val, obj->ps_high_thd_boot); 
			atomic_set(&obj->ps_low_thd_val, obj->ps_low_thd_boot); 
			APS_LOG("%s: change THD to boot HT=%d, LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));
		}
		else
		{
			APS_LOG("%s: boot HT=%d, LT=%d, boot_cali=%d\n", __func__, obj->ps_high_thd_boot, obj->ps_low_thd_boot, obj->boot_cali);
		}
#endif

		if((ret = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", ret);
			return ret;        
		}		
		if((ret = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", ret);
			return ret;        
		}	
#ifdef STK_DEBUG_PRINTF				
		APS_LOG("%s: FAE tune0 psa-psi(%d) > DIFF found\n", __func__, diff);
#endif					
		APS_LOG("%s: set HT=%d, LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));
#endif		
		hrtimer_cancel(&obj->ps_tune0_timer);
	}
	
	return 0;
}	
#endif	/*#ifdef STK_TUNE0	*/

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x1x_priv *obj = container_of(work, struct stk3x1x_priv, stk_ps_tune0_work);		
#ifdef STK_GES		
	if(obj->ges_enabled)
		stk_ges_poll_func(obj);
#endif
#ifdef STK_TUNE0	
	if(obj->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(obj);
	else
		stk_ps_tune_zero_func_fae(obj);
#endif	
	return;
}	

/*----------------------------------------------------------------------------*/
#ifdef STK_TUNE0	
static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_priv *obj = container_of(timer, struct stk3x1x_priv, ps_tune0_timer);
	queue_work(obj->stk_ps_tune0_wq, &obj->stk_ps_tune0_work);	
	hrtimer_forward_now(&obj->ps_tune0_timer, obj->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
#endif
/*----------------------------------------------------------------------------*/

#ifdef STK_TUNE1

struct get_ct_struct
{
	int ps_val[STK_PS_VAL_BUF];
	unsigned int ps_val_buffer_cnt;
	int psi;
	int psa;
};

int stk_tune1_check_stable(int val[], int val_len, int criteria, int *imax, int *imin)
{
	int aa;
	int max, min;
	
	max = INT_MIN;
	min = INT_MAX;
	
	for(aa=0;aa<val_len;aa++)
	{
		if(val[aa] > max)                                                                        
			max = val[aa];
		if(val[aa] < min)
			min = val[aa];
	}
	
	if(imax != NULL)
		*imax = max;
	if(imin != NULL)
		*imin = min;
	
	if((max - min) < criteria)
		return 1;
	return 0;
}

static struct get_ct_struct ps_get_ct;

static void stk_tune1_reset_para(void)
{
	int cnt;
	
	ps_get_ct.psi = 0xFFFF;
	ps_get_ct.psa = 0;	
	ps_get_ct.ps_val_buffer_cnt = 0;
	
	for(cnt=0;cnt<STK_PS_VAL_BUF;cnt++)
		ps_get_ct.ps_val[cnt] = -100;
	return;
}

static int stk_tune1_get_ct(uint32_t ps)
{
	int ps_max, ps_min;
	
	ps_get_ct.ps_val[ps_get_ct.ps_val_buffer_cnt] = ps;
	ps_get_ct.ps_val_buffer_cnt++;
	if(ps_get_ct.ps_val_buffer_cnt >= STK_PS_VAL_BUF)
	{
		ps_get_ct.ps_val_buffer_cnt = 0;
	}
	
	if(ps < ps_get_ct.psi)
		ps_get_ct.psi = ps;
	if(ps > ps_get_ct.psa)
		ps_get_ct.psa = ps;
			
	// if(ps_get_ct.psa - ps_get_ct.psi > STK_CT_UPDATE_DIFF)
	// {
		if(stk_tune1_check_stable(ps_get_ct.ps_val, STK_PS_VAL_BUF, PS_NOISE, &ps_max, &ps_min) == 1)
		{
			stk_tune1_reset_para();			
			return ps_min;
		}
	// }
	return -1;
}

static int stk_tune1_ct_det(struct stk3x1x_priv *obj, uint32_t ps)
{
	int ct = 0;
	int err;
	
	err = stk3x1x_ps_val();
	if(err == 0xFFFF)
	{
		if(obj->invalid_ps_cnt++ > STK_TUNE1_INVALID_PS_LIMIT)
		{
			APS_LOG("%s: invalid_ps_cnt > limit\n", __func__);
			return 3;
		}
		return 0;
	}
	
	ct = stk_tune1_get_ct(ps);
	if(ct >= 0)
	{
		if(ct < STK_PS_CT_LIMIT)
		{
			atomic_set(&obj->ps_high_thd_val, ct + obj->stk_ht_n_ct); 
			atomic_set(&obj->ps_low_thd_val, ct + obj->stk_lt_n_ct); 	
			stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val));
			if(obj->hw.polling_mode_ps)
			{
				stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
			}
			else
			{
				stk3x1x_write_ps_low_thd(obj->client, 0);
			}
			// update_thd += STK_THD_F_PS0_CT_OK;
			// printk("%s: HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));
			return 1;
		}
		else
		{
			// update_thd = STK_THD_F_PS0_CT_NG;
			APS_LOG("%s: ct>STK_PS_CT_LIMIT,keep HT/LT\n", __func__);
			return 2;
		}
	}
	return 0;
}

static int stk_tune1_fin_det(struct stk3x1x_priv *obj, uint32_t ps, int nf_flag)
{
	static uint8_t ps_far_re_cali = 0;
	
	if(nf_flag == 0)
	{
		if(ps >= STK_FIN_THD)
		{
			ps_far_re_cali = 1;
			atomic_set(&obj->ps_high_thd_val, STK_PS_HIGHER_THDH); 
			atomic_set(&obj->ps_low_thd_val, STK_PS_HIGHER_THDL); 
			if(obj->hw.polling_mode_ps)
			{
				stk3x1x_write_ps_high_thd(obj->client, STK_PS_HIGHER_THDH);	
				stk3x1x_write_ps_low_thd(obj->client, STK_PS_HIGHER_THDL);
			}
			// printk("%s: fixed HT=%d,LT=%d\n", __func__, STK_PS_HIGHER_THDH, STK_PS_HIGHER_THDL);
			return 1;
		}
	}
	else
	{
		if(ps_far_re_cali)
		{
			ps_far_re_cali = 0;
			if(ps < STK_FIN_THD)
			{
				atomic_set(&obj->ps_high_thd_val, ps + obj->stk_ht_n_ct * 2); 
				atomic_set(&obj->ps_low_thd_val, ps + obj->stk_lt_n_ct * 2); 				
				if(obj->hw.polling_mode_ps)
				{
					stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val));
					stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
				}
				// printk("%s: keep HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));
			}
			else
			{
				APS_LOG("%s: keep HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val), atomic_read(&obj->ps_low_thd_val));
			}
			return 0x10;
		}
	}	
	return 0;
}

static int stk_tune1_ps_int_handle(struct stk3x1x_priv *obj)
{
	int ret;
	int32_t nf_state;
	u16 als_reading = 0;
	
	int ps_thd_h = atomic_read(&obj->ps_high_thd_val); 
	int ps_thd_l = atomic_read(&obj->ps_low_thd_val);
	
	nf_state = (obj->ps >= ps_thd_h) ? 0 : (obj->ps<= ps_thd_l ? 1 : obj->ps_distance_last);
	// printk("%s:handle_state=%d,ps=%d,als=%d,nf_flag=%d,ps_distance_last=%d\n", 
	//	__func__, obj->stk_int_handle_state, obj->ps, als_reading, 
	//	nf_state, obj->ps_distance_last);
		
	switch(obj->stk_int_handle_state)
	{
	case STK_PSINT_NORM:
		stk_ps_report(obj, nf_state);
		// printk("%s:(N) ps input event=%d, ps code=%d\n",__func__, nf_state, obj->ps);
		if(nf_state)
		{
			stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val));
			stk3x1x_write_ps_low_thd(obj->client, 0);
			obj->stk_int_handle_state = STK_PSINT_NORM;
			// printk("%s:(N) HT=%d,LT=0\n", __func__, ps_thd_h);
		}
		else
		{
			stk3x1x_write_ps_high_thd(obj->client, STK_FIN_THD);
			stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
			obj->stk_int_handle_state = STK_PSINT_FIN_DET;
			// printk("%s:(N) HT=%d,LT=%d\n", __func__, STK_FIN_THD, ps_thd_l);				
		}
		break;	
	case STK_PSINT_FIN_DET:
		if(nf_state)
		{
			stk_ps_report(obj, nf_state);
			// printk("%s:(F) ps input event=%d, ps=%d\n",__func__, nf_state, obj->ps);			
			stk3x1x_write_ps_high_thd(obj->client,  atomic_read(&obj->ps_high_thd_val));		
			stk3x1x_write_ps_low_thd(obj->client, 0);
			obj->stk_int_handle_state = STK_PSINT_NORM;							
			// printk("%s:(F) HT=%d,LT=0\n", __func__, ps_thd_h);
		}
		break;
	case STK_PSINT_CT_DET:
		ret = stk_tune1_ct_det(obj, obj->ps);
		if(ret > 0)
		{
			stk3x1x_write_ps_high_thd(obj->client,  atomic_read(&obj->ps_high_thd_val));
			stk3x1x_write_ps_low_thd(obj->client, 0);
			obj->stk_int_handle_state = STK_PSINT_NORM;							
			// printk("%s:(C) HT=%d,LT=0\n", __func__, ps_thd_h);
		}
		
		if(!nf_state)
		{
			stk_ps_report(obj, nf_state);
			// printk("%s:(C) ps input event=%d, ps=%d\n",__func__, nf_state, obj->ps);				
			stk3x1x_write_ps_high_thd(obj->client, STK_FIN_THD);
			stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
			obj->stk_int_handle_state = STK_PSINT_FIN_DET;
			// printk("%s:(C) HT=%d,LT=%d\n", __func__, STK_FIN_THD, ps_thd_l);				
		}		
		break;			
	default:
		// printk(KERN_ERR "%s: invalid stk_int_handle_state %d\n", __func__, obj->stk_int_handle_state);
	}
	
	ret = stk_tune1_fin_det(obj, obj->ps, nf_state);
	if(ret == 1)
	{
		if(obj->stk_int_handle_state == STK_PSINT_FIN_DET)
		{
			obj->stk_int_handle_state = STK_PSINT_NORM;
			stk3x1x_write_ps_high_thd(obj->client, 0xFFFF);
			stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
			// printk("%s: HT=0xffff,LT=%d\n", __func__, atomic_read(&obj->ps_low_thd_val));
		}
	}
	else if(ret == 0x10 && nf_state == 1)
	{
		stk3x1x_write_ps_high_thd(obj->client, 0);
		stk3x1x_write_ps_low_thd(obj->client, 0xffff);
		obj->stk_int_handle_state = STK_PSINT_CT_DET;
		obj->invalid_ps_cnt = 0;
		// printk("%s: HT=0,LT=0xffff\n", __func__);

		stk_tune1_reset_para();
		// printk("%s: reset ps_get_ct\n", __func__);		
	}
	return 0;
}

#endif	// #ifdef STK_TUNE1	


/*----------------------------------------------------------------------------*/
void stk3x1x_eint_func(void)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;
	APS_LOG(" interrupt fuc\n");
	if(!obj)
	{
		return;
	}
	//schedule_work(&obj->eint_work);
	if(obj->hw.polling_mode_ps == 0 || obj->hw.polling_mode_als == 0)
		schedule_delayed_work(&obj->eint_work,0);
	if(atomic_read(&obj->trace) & STK_TRC_EINT)
	{
		APS_LOG("eint: als/ps intrs\n");
	}
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_eint_work(struct work_struct *work)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;
	int err;
	u8 flag_reg, disable_flag = 0;
	
	APS_LOG(" eint work\n");
	
	if((err = stk3x1x_check_intr(obj->client, &flag_reg)))
	{
		APS_ERR("stk3x1x_check_intr fail: %d\n", err);
		goto err_i2c_rw;
	}

    APS_LOG(" &obj->pending_intr =%lx\n",obj->pending_intr);
	
	if(((1<<STK_BIT_PS) &  obj->pending_intr) && (obj->hw.polling_mode_ps == 0))
	{
		APS_LOG("stk ps change\n");
		disable_flag |= STK_FLG_PSINT_MASK;
		
		if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
		{
			APS_ERR("stk3x1x read ps data: %d\n", err);
			goto err_i2c_rw;
		}
		
		distance_flag = (flag_reg & STK_FLG_NF_MASK)? 1 : 0;
		APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps,distance_flag);
		//let up layer to know
		stk_ps_report(obj, distance_flag);	
		
#ifdef STK_PS_DEBUG
		APS_LOG("%s: ps interrupt, show all reg\n", __FUNCTION__);
		show_allreg(); //for debug
#endif					
	}
	if(disable_flag)
	{
		if((err = stk3x1x_clear_intr(obj->client, flag_reg, disable_flag)))
		{
			APS_ERR("fail: %d\n", err);
			goto err_i2c_rw;
		}		
	}
	msleep(1);

#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#endif

	return;
	
err_i2c_rw:	
	msleep(30);
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#endif	
	return;
}

#if defined(CONFIG_OF)
static irqreturn_t stk3x1x_eint_handler(int irq, void *desc)
{
	// printk("----------------stk3x1x_eint_handler-----------------------\n");
	disable_irq_nosync(stk3x1x_obj->irq);
	stk3x1x_eint_func();

	return IRQ_HANDLED;
}
#endif

/*----------------------------------------------------------------------------*/
int stk3x1x_setup_eint(struct i2c_client *client)
{
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;

	u32 ints[2] = {0, 0};

//	alspsPltFmDev = get_alsps_platformdev();
/* gpio setting */
	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	 }
	 pinctrl_select_state(pinctrl, pins_cfg);
/* eint request */
	if (stk3x1x_obj->irq_node) {
#ifndef CONFIG_MTK_EIC
		/*upstream code*/
		ints[0] = of_get_named_gpio(stk3x1x_obj->irq_node, "deb-gpios", 0);
		if (ints[0] < 0) {
			APS_LOG("debounce gpio not found\n");
		} else{
			ret = of_property_read_u32(stk3x1x_obj->irq_node, "debounce", &ints[1]);
			if (ret < 0)
				APS_LOG("debounce time not found\n");
			else
				gpio_set_debounce(ints[0], ints[1]);
			APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		}
#else
		ret = of_property_read_u32_array(stk3x1x_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		if (ret) {
			APS_LOG("of_property_read_u32_array fail, ret = %d\n", ret);
			return ret;
		}
		gpio_set_debounce(ints[0], ints[1]);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
#endif
	//	gpio_request(ints[0], "p-sensor");
	//	pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		stk3x1x_obj->irq = irq_of_parse_and_map(stk3x1x_obj->irq_node, 0);
		APS_LOG("stk3x1x_obj->irq = %d\n", stk3x1x_obj->irq);
		if (!stk3x1x_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(stk3x1x_obj->irq, stk3x1x_eint_handler, IRQ_TYPE_LEVEL_LOW, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq_wake(stk3x1x_obj->irq);// yqf add
		enable_irq(stk3x1x_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
	
    return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_init_client(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err;
	int ps_ctrl;
	//u8 int_status;
	
	if((err = stk3x1x_write_sw_reset(client)))
	{
		APS_ERR("software reset error, err=%d", err);
		return err;
	}
	if((err = stk3x1x_read_id(client)))
	{
		APS_ERR("stk3x1x_read_id error, err=%d", err);
		return err;
	}		
	if(obj->first_boot == true)
	{	
		if(obj->hw.polling_mode_ps == 0 || obj->hw.polling_mode_als == 0)
		{
			if((err = stk3x1x_setup_eint(client)))
			{
				APS_ERR("setup eint error: %d\n", err);
				return err;
			}
		}
	}
	if((err = stk3x1x_write_state(client, atomic_read(&obj->state_val))))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}	
	
	ps_ctrl = atomic_read(&obj->psctrl_val);
	if(obj->hw.polling_mode_ps == 1)
		ps_ctrl &= 0x3F;
	
	if((err = stk3x1x_write_ps(client, ps_ctrl)))
	{
		APS_ERR("write ps error: %d\n", err);
		return err;        
	}
	
	if((err = stk3x1x_write_als(client, atomic_read(&obj->alsctrl_val))))
	{
		APS_ERR("write als error: %d\n", err);
		return err;
	}	
	
	if((err = stk3x1x_write_led(client, obj->ledctrl_val)))
	{
		APS_ERR("write led error: %d\n", err);
		return err;
	}	
	
	if((err = stk3x1x_write_wait(client, obj->wait_val)))
	{
		APS_ERR("write wait error: %d\n", err);
		return err;
	}	
#ifndef STK_TUNE0	
	if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;        
	}
	
	if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;        
	}
#endif	
#ifdef STK_TUNE1
	if(obj->hw.polling_mode_ps == 0)
		obj->int_val = 0x7;
#endif
	if((err = stk3x1x_write_int(client, obj->int_val)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}		
	
#ifdef STK_ALS_FIR
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
#ifdef STK_TUNE0
	if(obj->first_boot == true)
		stk_ps_tune_zero_init(obj);
#endif	
#ifdef STK_GES	
	obj->re_enable_ges = false;	
	obj->re_enable_ges2 = 0;	
	atomic_set(&obj->gesture2, 0);
#endif
	obj->re_enable_ps = false;
	obj->re_enable_als = false;
	obj->als_code_last = 100;
#ifdef STK_TUNE1	
	if(obj->hw.polling_mode_als == 1 && obj->hw.polling_mode_ps == 0)
		obj->hw.polling_mode_als = 0;
#endif	
	obj->ps_distance_last = -1;
	obj->als_code_last = 100;
#ifdef STK_TUNE1	
	obj->stk_int_handle_state = STK_PSINT_NORM;
#endif
	return 0;
}

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t stk3x1x_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	res = scnprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d)\n", 
		atomic_read(&stk3x1x_obj->i2c_retry), atomic_read(&stk3x1x_obj->als_debounce), 
		atomic_read(&stk3x1x_obj->ps_mask), atomic_read(&stk3x1x_obj->ps_high_thd_val),atomic_read(&stk3x1x_obj->ps_low_thd_val), atomic_read(&stk3x1x_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, hthres, lthres, err;
	struct i2c_client *client;
	client = stk3x1x_i2c_client;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(6 == sscanf(buf, "%d %d %d %d %d %d", &retry, &als_deb, &mask, &hthres, &lthres, &ps_deb))
	{ 
		atomic_set(&stk3x1x_obj->i2c_retry, retry);
		atomic_set(&stk3x1x_obj->als_debounce, als_deb);
		atomic_set(&stk3x1x_obj->ps_mask, mask);
		atomic_set(&stk3x1x_obj->ps_high_thd_val, hthres);    
		atomic_set(&stk3x1x_obj->ps_low_thd_val, lthres);        
		atomic_set(&stk3x1x_obj->ps_debounce, ps_deb);

		if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}
		
		if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}
	}
	else
	{
		APS_ERR("invalid content: '%s'\n", buf);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}

	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&stk3x1x_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ir(struct device_driver *ddri, char *buf)
{
    int32_t reading;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
    reading = stk3x1x_get_ir_value(stk3x1x_obj, STK_IRS_IT_REDUCE);
	if(reading < 0)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", reading);

	stk3x1x_obj->ir_code = reading;
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->ir_code);     
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	if(stk3x1x_obj->hw.polling_mode_als == 0)
	{
		if((res = stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als)))
			return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
		else
			return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->als);     
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->als_code_last);    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ps(struct device_driver *ddri, char *buf)
{
	int res;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if((res = stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_reg(struct device_driver *ddri, char *buf)
{
	u8 int_status;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	/*read*/
	stk3x1x_check_intr(stk3x1x_obj->client, &int_status);
	//stk3x1x_clear_intr(stk3x1x_obj->client, int_status, 0x0);
	stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps);
	stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als);
	/*write*/
	stk3x1x_write_als(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->alsctrl_val));
	stk3x1x_write_ps(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->psctrl_val)); 
	stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val));
	stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val));
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	APS_LOG("send(%02X, %02X) = %d\n", addr, cmd, 
	stk3x1x_master_send(stk3x1x_obj->client, (u16)addr, &dat, sizeof(dat)));
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_recv(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->recv_reg));     	
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	u8 dat;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
    stk3x1x_master_recv(stk3x1x_obj->client, (u16)addr, (char*)&dat, sizeof(dat));
	//APS_LOG("recv(%02X) = %d, 0x%02X\n", addr, 
	//stk3x1x_master_recv(stk3x1x_obj->client, (u16)addr, (char*)&dat, sizeof(dat)), dat);
	atomic_set(&stk3x1x_obj->recv_reg, dat);	
	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t stk3x1x_show_allreg(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	u8 rbuf[0x22];
	int cnt;	
	int len = 0;
	
	memset(rbuf, 0, sizeof(rbuf));
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0, &rbuf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 7, &rbuf[7], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 14, &rbuf[14], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 21, &rbuf[21], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 28, &rbuf[28], 4);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, STK_PDT_ID_REG, &rbuf[32], 2);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		APS_LOG("reg[0x%x]=0x%x\n", cnt, rbuf[cnt]);
		len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, rbuf[cnt]);
	}	
	APS_LOG("reg[0x3E]=0x%x\n", rbuf[cnt]);
	APS_LOG("reg[0x3F]=0x%x\n", rbuf[cnt++]);
	len += scnprintf(buf+len, PAGE_SIZE-len, "[0x3E]%2X,[0x3F]%2X\n", rbuf[cnt-1], rbuf[cnt]);	
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	u8 rbuf[25];
	int ret = 0;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d) (%02X) (%02X %02X %02X) (%02X %02X %02X %02X)\n", 
			stk3x1x_obj->hw.i2c_num, stk3x1x_obj->hw.power_id, stk3x1x_obj->hw.power_vol, stk3x1x_obj->addr.flag, 
			stk3x1x_obj->addr.alsctrl, stk3x1x_obj->addr.data1_als, stk3x1x_obj->addr.data2_als, stk3x1x_obj->addr.psctrl, 
			stk3x1x_obj->addr.data1_ps, stk3x1x_obj->addr.data2_ps, stk3x1x_obj->addr.thdh1_ps);
	
	len += scnprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02X %02X %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&stk3x1x_obj->state_val), atomic_read(&stk3x1x_obj->psctrl_val), atomic_read(&stk3x1x_obj->alsctrl_val), 
				stk3x1x_obj->ledctrl_val, stk3x1x_obj->int_val, stk3x1x_obj->wait_val, 
				atomic_read(&stk3x1x_obj->ps_high_thd_val), atomic_read(&stk3x1x_obj->ps_low_thd_val),stk3x1x_obj->enable, stk3x1x_obj->pending_intr);
#ifdef MT6516
	len += scnprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
				CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

	len += scnprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n",	GPIO_ALS_EINT_PIN, 
				mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN), 
				mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
#endif

	len += scnprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&stk3x1x_obj->als_suspend), atomic_read(&stk3x1x_obj->ps_suspend));	
	len += scnprintf(buf+len, PAGE_SIZE-len, "VER.: %s\n", DRIVER_VERSION);
	
	memset(rbuf, 0, sizeof(rbuf));	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0, &rbuf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 7, &rbuf[7], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 14, &rbuf[14], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}		

    len += scnprintf(buf+len, PAGE_SIZE-len, "[PS=%2X] [ALS=%2X] [WAIT=%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n", 
		rbuf[0]&0x01,(rbuf[0]&0x02)>>1,((rbuf[0]&0x04)>>2)*rbuf[5]*6,(rbuf[0]&0x20)>>5,
		(rbuf[0]&0x40)>>6,rbuf[16]&0x01,(rbuf[16]&0x04)>>2,(rbuf[16]&0x10)>>4,(rbuf[16]&0x20)>>5);		
	
	return len;
}
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct stk3x1x_priv *obj, const char* buf, size_t count,
                             u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3x1x_obj->als_level_num; idx++)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", stk3x1x_obj->hw.als_level[idx]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3x1x_obj->als_level, stk3x1x_obj->hw.als_level, sizeof(stk3x1x_obj->als_level));
	}
	else if(stk3x1x_obj->als_level_num != read_int_from_buf(stk3x1x_obj, buf, count, 
			stk3x1x_obj->hw.als_level, stk3x1x_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3x1x_obj->als_value_num; idx++)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", stk3x1x_obj->hw.als_value[idx]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3x1x_obj->als_value, stk3x1x_obj->hw.als_value, sizeof(stk3x1x_obj->als_value));
	}
	else if(stk3x1x_obj->als_value_num != read_int_from_buf(stk3x1x_obj, buf, count, 
			stk3x1x_obj->hw.als_value, stk3x1x_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}

#ifdef STK_TUNE0
static ssize_t stk3x1x_show_cali(struct device_driver *ddri, char *buf)
{
	int32_t word_data;
	u8 r_buf[2];
	int ret;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x20, r_buf, 2);
	if(ret < 0)	
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}
	word_data = (r_buf[0] << 8) | r_buf[1];

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x22, r_buf, 2);
	if(ret < 0)		
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}	
	word_data += (r_buf[0] << 8) | r_buf[1];	

	APS_LOG("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __FUNCTION__, 
		stk3x1x_obj->psi_set, stk3x1x_obj->psa, stk3x1x_obj->psi, word_data);	
#ifdef CALI_EVERY_TIME
	APS_LOG("%s: boot HT=%d, LT=%d\n", __func__, stk3x1x_obj->ps_high_thd_boot, stk3x1x_obj->ps_low_thd_boot);
#endif
	return scnprintf(buf, PAGE_SIZE, "%5d\n", stk3x1x_obj->psi_set);		
	//return 0;
}

static ssize_t stk3x1x_ps_maxdiff_store(struct device_driver *ddri, const char *buf, size_t count)
{
    unsigned long value = 0;
	int ret;
	
	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	stk3x1x_obj->stk_max_min_diff = (int) value;
	return count;
}


static ssize_t stk3x1x_ps_maxdiff_show(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", stk3x1x_obj->stk_max_min_diff);     		
}

static ssize_t stk3x1x_ps_ltnct_store(struct device_driver *ddri, const char *buf, size_t count)
{
    unsigned long value = 0;
	int ret;
	
	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	stk3x1x_obj->stk_lt_n_ct = (int) value;
	return count;
}

static ssize_t stk3x1x_ps_ltnct_show(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", stk3x1x_obj->stk_lt_n_ct);     		
}

static ssize_t stk3x1x_ps_htnct_store(struct device_driver *ddri, const char *buf, size_t count)
{
    unsigned long value = 0;
	int ret;
	
	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	stk3x1x_obj->stk_ht_n_ct = (int) value;
	return count;
}

static ssize_t stk3x1x_ps_htnct_show(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", stk3x1x_obj->stk_ht_n_ct);     		
}

#endif	// #ifdef STK_TUNE0

#ifdef STK_ALS_FIR
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_firlen(struct device_driver *ddri, char *buf)
{
	int len = atomic_read(&stk3x1x_obj->firlength);
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
		
	APS_LOG("%s: len = %2d, idx = %2d\n", __func__, len, stk3x1x_obj->fir.idx);			
	APS_LOG("%s: sum = %5d, ave = %5d\n", __func__, stk3x1x_obj->fir.sum, stk3x1x_obj->fir.sum/len);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", len);		
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_firlen(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;

	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%d", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
		
	if(value > MAX_FIR_LEN)
	{
		APS_ERR("%s: firlen exceed maximum filter length\n", __func__);
	}
	else if (value < 1)
	{
		atomic_set(&stk3x1x_obj->firlength, 1);
		memset(&stk3x1x_obj->fir, 0x00, sizeof(stk3x1x_obj->fir));
	}
	else
	{ 
		atomic_set(&stk3x1x_obj->firlength, value);
		memset(&stk3x1x_obj->fir, 0x00, sizeof(stk3x1x_obj->fir));
	}
	
	return count;
}
#endif /* #ifdef STK_ALS_FIR */

/*---Offset At-------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ps_offset(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
#ifdef STK_TUNE0
	return snprintf(buf, PAGE_SIZE, "%d\n", stk3x1x_obj->crosstalk);     
#else
	return 0;
#endif	
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_set_ps_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret;
	ret = stk3x1x_ps_calibration(stk3x1x_obj->client);
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_pthreshold_calibration(struct device_driver *ddri, char *buf)
{
	// struct stk3x1x_priv *data = i2c_get_clientdata(stk3x1x_obj->client);	
#ifdef CALI_EVERY_TIME
	return snprintf(buf,PAGE_SIZE, "Low threshold = %d , High threshold = %d\n", stk3x1x_obj->ps_low_thd_boot, stk3x1x_obj->ps_high_thd_boot);
#else
	return 0;
#endif	
}

static ssize_t stk3x1x_store_ps_offset(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef STK_TUNE0	
	struct stk3x1x_priv *obj = stk3x1x_obj;
	int ps_cali_val = 0;
	//int threshold[2] = {0};
	sscanf(buf,"%d",&ps_cali_val);
	obj->crosstalk = ps_cali_val;
	#ifdef STK_TUNE1		
	if (obj->crosstalk == 0 || obj->crosstalk > STK_PS_HIGHER_THDL)
	#else
	if (obj->crosstalk == 0)
	#endif	
	{
	#ifdef STK_TUNE1				
		obj->crosstalk = STK_PS_HIGHER_THDL;
	#else
		obj->crosstalk = obj->hw.ps_threshold_low;
	#endif	
		stk3x1x_calibration_flag = 0;
	}else {
		stk3x1x_calibration_flag = 1;
	}
	mutex_lock(&run_cali_mutex);
	#ifdef CALI_EVERY_TIME	
	obj->ps_high_thd_boot = obj->crosstalk + obj->stk_ht_n_ct;
	obj->ps_low_thd_boot = obj->crosstalk + obj->stk_lt_n_ct;
	// printk( "%s: crosstalk=%d, high thd=%d, low thd=%d\n", __func__, obj->crosstalk, obj->ps_high_thd_boot, obj->ps_low_thd_boot);
	#endif	
	
	mutex_unlock(&run_cali_mutex);
#endif	
	return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, stk3x1x_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, stk3x1x_show_ps,    NULL);
static DRIVER_ATTR(ir,      S_IWUSR | S_IRUGO, stk3x1x_show_ir,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, stk3x1x_show_config,stk3x1x_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, stk3x1x_show_alslv, stk3x1x_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, stk3x1x_show_alsval,stk3x1x_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, stk3x1x_show_trace, stk3x1x_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, stk3x1x_show_status,  NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, stk3x1x_show_send,  stk3x1x_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, stk3x1x_show_recv,  stk3x1x_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, stk3x1x_show_reg,   NULL);
static DRIVER_ATTR(allreg,  S_IWUSR | S_IRUGO, stk3x1x_show_allreg,   NULL);
static DRIVER_ATTR(pscalibration, 		S_IWUSR | S_IRUGO, stk3x1x_show_ps_offset, stk3x1x_set_ps_offset);
static DRIVER_ATTR(pthredcalibration, 	S_IWUSR | S_IRUGO, stk3x1x_show_pthreshold_calibration, NULL);
#ifdef STK_TUNE0
static DRIVER_ATTR(cali,    S_IWUSR | S_IRUGO, stk3x1x_show_cali,  NULL);
static DRIVER_ATTR(maxdiff, S_IWUSR | S_IRUGO, stk3x1x_ps_maxdiff_show,  stk3x1x_ps_maxdiff_store);
static DRIVER_ATTR(ltnct,   S_IWUSR | S_IRUGO, stk3x1x_ps_ltnct_show,  stk3x1x_ps_ltnct_store);
static DRIVER_ATTR(htnct,   S_IWUSR | S_IRUGO, stk3x1x_ps_htnct_show,  stk3x1x_ps_htnct_store);
#endif
#ifdef STK_ALS_FIR
static DRIVER_ATTR(firlen,   S_IWUSR | S_IRUGO, stk3x1x_show_firlen,  stk3x1x_store_firlen);
#endif
static DRIVER_ATTR(ps_offset, S_IWUSR | S_IRUGO,NULL, stk3x1x_store_ps_offset);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *stk3x1x_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_ir,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_allreg,
    &driver_attr_reg,
    &driver_attr_pscalibration,
    &driver_attr_pthredcalibration,
#ifdef STK_TUNE0
    &driver_attr_cali,
    &driver_attr_maxdiff,
    &driver_attr_ltnct,
    &driver_attr_htnct,
#endif	
#ifdef STK_ALS_FIR
    &driver_attr_firlen,
#endif
	&driver_attr_ps_offset,	
};

/*----------------------------------------------------------------------------*/
static int stk3x1x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(stk3x1x_attr_list));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, stk3x1x_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", stk3x1x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(ARRAY_SIZE(stk3x1x_attr_list));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, stk3x1x_attr_list[idx]);
	}
	
	return err;
}


#ifdef STK_GES		
static ssize_t stk_ges_poll_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0, ii = 0, jj = 0;
	
	while(stk_ges_op[ii].ops[0] != 0)
	{
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", ii);
		for(jj=0;jj<4;jj++)
			len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", stk_ges_op[ii].ops[jj]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		ii++;
	}
	return len;
}

static ssize_t stk_ges_poll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int32_t ret, i = 0, index = 0;	
	char *token;
	unsigned long value = 0;
		
	while(buf != '\0')
	{
		token = strsep((char **)&buf, " ");
		if((ret = kstrtoul(token, 16, &value)) < 0)
		{
			printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
			return ret;	
		}
		
		if(i == 0)
		{
			if(value >= 10)
			{
				memset(stk_ges_op, 0, sizeof(stk_ges_op));				
				break;
			}
			else
				index = value;
		}
		else
		{
			stk_ges_op[index].ops[i-1] = value;
		}
		i++;
		if(i == 5)
			break;
	}
	if(i != 5)
	{
		printk(KERN_ERR "%s: invalid length(%d)\n", __func__, i);
		memset(&(stk_ges_op[index]), 0, sizeof(union stk_ges_operation));				
	}
	return size;
}
		
static ssize_t stk_ges_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_priv *obj =  dev_get_drvdata(dev);	
    int ret;
	unsigned int gest0 = 0, gest1 = 0, gest2 = 0;
	
	if(!obj->ges_enabled)
		return 0;
	
	ret = stk3x1x_get_ges_value(obj, &gest0, &gest1, &gest2);	
	if(ret < 0)
		return ret;
	else if(ret == 0xFFFF)
		atomic_set(&obj->gesture2, 0);		
		
    return scnprintf(buf, PAGE_SIZE, "%5d,%5d,%5d\n", gest0, gest1, atomic_read(&obj->gesture2));
}

static ssize_t stk_ges_code_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_priv *obj =  dev_get_drvdata(dev);
	uint8_t ges;
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		APS_ERR( "%s:kstrtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	
	if(obj->ges_enabled)
	{
		switch(value)
		{
		case 3:
			//APS_LOG( "%s: ges input event, not detected\n",__func__);			
		case 0:
			return size;
		case 1:
			ges = KEY_PAGEUP;	
			atomic_set(&obj->gesture2, 0);
			APS_LOG( "%s: ges input event >>>\n",__func__);		
			break;
		case 2:
			ges = KEY_PAGEDOWN;	
			atomic_set(&obj->gesture2, 0);
			APS_LOG( "%s: ges input event <<<\n",__func__);		
			break;
		case 32:
			ges = KEY_VOLUMEDOWN;
			APS_LOG( "%s: ges input event near\n",__func__);				
			break;
		case 48:
			ges = KEY_VOLUMEUP;
			APS_LOG( "%s: ges input event far\n",__func__);				
			break;
		default:
			APS_ERR( "%s, invalid value %d\n", __func__, (int)value);
			return -EINVAL;
		}

		input_report_key(obj->ges_input_dev, ges, 1);
		input_report_key(obj->ges_input_dev, ges, 0);
		input_sync(obj->ges_input_dev);
	}
    return size;
}

static ssize_t stk_ges_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_priv *obj =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		APS_ERR( "%s:kstrtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
    APS_LOG( "%s: Enable GES : %d\n", __func__, (int)value);
	
	switch(value)
	{
	case 0:
		if(obj->ges_enabled == 1)
			stk3x1x_enable_ges(obj->client, 0, 1);
		else
			stk3x1x_enable_ges(obj->client, 0, 2);
		break;
	case 1:
		stk3x1x_enable_ges(obj->client, 1, 1);
		break;
	case 2:
		stk3x1x_enable_ges(obj->client, 1, 2);
		break;
	default:
		APS_ERR( "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;	
	}

    return size;
}

static ssize_t stk_ges_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_priv *obj =  dev_get_drvdata(dev);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", obj->ges_enabled);		
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->recv_reg));     	
}

static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr;
	u8 dat;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	stk3x1x_master_recv(stk3x1x_obj->client, (u16)addr, (char*)&dat, sizeof(dat));
	//APS_LOG("recv(%02X) = %d, 0x%02X\n", addr, 
	//stk3x1x_master_recv(stk3x1x_obj->client, (u16)addr, (char*)&dat, sizeof(dat)), dat);
	atomic_set(&stk3x1x_obj->recv_reg, dat);	
	return size;
}

static ssize_t stk_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t stk_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	u8 dat;

	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	APS_LOG("send(%02X, %02X) = %d\n", addr, cmd, 
	stk3x1x_master_send(stk3x1x_obj->client, (u16)addr, &dat, sizeof(dat)));
	
	return size;
}

static struct device_attribute ges_enable_attribute = __ATTR(enable,0664,stk_ges_enable_show,stk_ges_enable_store);
static struct device_attribute ges_code_attribute = __ATTR(code, 0664, stk_ges_code_show, stk_ges_code_store);
static struct device_attribute ges_poll_attribute = __ATTR(poll, 0664, stk_ges_poll_show, stk_ges_poll_store);
static struct device_attribute ges_recv_attribute = __ATTR(recv,0664,stk_recv_show,stk_recv_store);
static struct device_attribute ges_send_attribute = __ATTR(send,0664,stk_send_show, stk_send_store);

static struct attribute *stk_ges_attrs [] =
{
    &ges_enable_attribute.attr,	
    &ges_code_attribute.attr,
    &ges_poll_attribute.attr,	
	&ges_recv_attribute.attr,
	&ges_send_attribute.attr,
    NULL
};

static struct attribute_group stk_ges_attribute_group = 
{
	.name = "driver",	
	.attrs = stk_ges_attrs,
};
#endif	/* #ifdef STK_GES */
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
#ifdef STK_GES		
static uint32_t stk3x1x_get_ges_value(struct stk3x1x_priv *obj, unsigned int *ges0, unsigned int *ges1, unsigned int *ges2)
{
	u8 buf[4];
	int err, retry = 10;
	u8 flag;

	do {
		err = stk3x1x_read_flag(obj->client, &flag);	
		if(err < 0)
			return err;
		if(flag & STK_FLG_PSDR_MASK)
			break;
		//APS_LOG( "%s: ps isnot ready\n", __func__);
		retry--;
		usleep_range(350, 1000);
	} while(retry > 0);

	err = stk3x1x_master_recv(obj->client, obj->addr.data1_ps, buf, 0x02);
	if(err < 0)
	{
		APS_DBG("error: %d\n", err);
		return -EFAULT;
	}

	err = stk3x1x_master_recv(obj->client, 0x24, buf, 0x04);
	if(err < 0)
	{
		APS_DBG("error: %d\n", err);
		return -EFAULT;
	}
	*ges0 = (buf[0]<<8) | buf[1];	
	*ges1 = (buf[2]<<8) | buf[3];	
	//APS_LOG( "%s: ges=%d,%d\n",__func__, *ges0, *ges1);	
	return 0;
}
#endif

static int stk3x1x_get_als_value(struct stk3x1x_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw.als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(atomic_read(&obj->als_deb_on) == 1)
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if (atomic_read(&obj->trace) & STK_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, obj->hw.als_value[idx]);
		}
		
		return obj->hw.als_value[idx];
	}
	else
	{
		if(atomic_read(&obj->trace) & STK_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw.als_value[idx]);    
		}
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_get_ps_value_only(struct stk3x1x_priv *obj, u16 ps)
{
	int mask = atomic_read(&obj->ps_mask);
	int invalid = 0, val;
	int err;
	u8 flag;
	
	err = stk3x1x_read_flag(obj->client, &flag);
	if(err)
		return err;
	val = (flag & STK_FLG_NF_MASK)? 1 : 0;	
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(atomic_read(&obj->ps_deb_on) == 1)
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (atomic_read(&obj->ps_deb_on) == 1)
		{
			invalid = 1;
		}
	}		
	
	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		return val;
		
	}	
	else
	{
		APS_ERR(" ps value is invalid, PS:  %05d => %05d\n", ps, val);
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_get_ps_value(struct stk3x1x_priv *obj, u16 ps)
{
	int mask = atomic_read(&obj->ps_mask);
	int invalid = 0, val;
	int err;
	u8 flag;

	err = stk3x1x_read_flag(obj->client, &flag);
	if(err)
		return err;
	
	val = (flag & STK_FLG_NF_MASK)? 1 : 0;	
#ifdef STK_TUNE1
		stk_tune1_fin_det(obj, ps, val);
		// printk("%s: als=%d,ps=%d,nf_flag=%d\n", __func__, obj->als_code_last, ps,val);
		if(val == 1)
			stk_tune1_ct_det(obj, ps);
#endif
	APS_LOG("%s: read_flag = 0x%x, val = %d\n", __FUNCTION__, flag, val);
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(atomic_read(&obj->ps_deb_on) == 1)
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (atomic_read(&obj->ps_deb_on) == 1)
		{
			invalid = 1;
		}
	}
		
	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		return val;
		
	}	
	else
	{
		APS_ERR(" ps value is invalid, PS:  %05d => %05d\n", ps, val);
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/

static int32_t stk3x1x_set_irs_it_slp(struct stk3x1x_priv *obj, uint16_t *slp_time, int32_t ials_it_reduce)
{
	uint8_t irs_alsctrl;
	int32_t ret;
		
	irs_alsctrl = (atomic_read(&obj->alsctrl_val) & 0x0F) - ials_it_reduce;		
	switch(irs_alsctrl)
	{
	case 2:
		*slp_time = 1;
		break;			
	case 3:
		*slp_time = 2;
		break;	
	case 4:
		*slp_time = 3;
		break;	
	case 5:
		*slp_time = 6;
		break;	
	case 6:
		*slp_time = 12;
		break;
	case 7:
		*slp_time = 24;			
		break;
	case 8:
		*slp_time = 48;			
		break;
	case 9:
		*slp_time = 96;			
		break;
	case 10:
		*slp_time = 192;
		break;		
	default:
		APS_ERR( "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
		ret = -EINVAL;	
		return ret;
	}
	irs_alsctrl |= (atomic_read(&obj->alsctrl_val) & 0xF0);
	ret = i2c_smbus_write_byte_data(obj->client, STK_ALSCTRL_REG, irs_alsctrl);
	if (ret < 0)
	{
		APS_ERR( "%s: write i2c error\n", __func__);
		return ret;		
	}		
	return 0;
}

static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj, int32_t als_it_reduce)
{
    int32_t word_data, ret;
	uint8_t w_reg, retry = 0;	
	uint16_t irs_slp_time = 100;
	// bool re_enable_ps = false;
	u8 flag;
	u8 buf[2];
	
	ret = stk3x1x_set_irs_it_slp(obj, &irs_slp_time, als_it_reduce);
	if(ret < 0)
		goto irs_err_i2c_rw;
		
	w_reg = atomic_read(&obj->state_val) | STK_STATE_EN_IRS_MASK;		
    ret = i2c_smbus_write_byte_data(obj->client, STK_STATE_REG, w_reg);
    if (ret < 0)
	{
		APS_ERR( "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}	
	msleep(irs_slp_time);	
	
	do
	{
		msleep(3);		
		ret = stk3x1x_read_flag(obj->client, &flag);	
		if (ret < 0)
		{
			APS_ERR("WARNING: read flag reg error: %d\n", ret);
			goto irs_err_i2c_rw;
		}	
		retry++;
	}while(retry < 10 && ((flag&STK_FLG_IR_RDY_MASK) == 0));
	
	if(retry == 10)
	{
		APS_ERR( "%s: ir data is not ready for a long time\n", __func__);
		ret = -EINVAL;
		goto irs_err_i2c_rw;
	}

	ret = stk3x1x_clear_intr(obj->client, flag, STK_FLG_IR_RDY_MASK);	
    if (ret < 0)
	{
		APS_ERR( "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}		
	
	ret = stk3x1x_master_recv(obj->client, STK_DATA1_IR_REG, buf, 2);
	if(ret < 0)	
	{
		APS_ERR( "%s fail, ret=0x%x", __func__, ret); 
		goto irs_err_i2c_rw;		
	}
	word_data =  (buf[0] << 8) | buf[1];

	ret = i2c_smbus_write_byte_data(obj->client, STK_ALSCTRL_REG, atomic_read(&obj->alsctrl_val));
	if (ret < 0)
	{
		APS_ERR( "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}
	//if(re_enable_ps)
		//stk3x1x_enable_ps(obj->client, 1, 0);		
	return word_data;

irs_err_i2c_rw:	
	//if(re_enable_ps)
	//	stk3x1x_enable_ps(obj->client, 1, 0);	
	return ret;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
#if 0
static int stk3x1x_open(struct inode *inode, struct file *file)
{
	file->private_data = stk3x1x_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long stk3x1x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result, ps_cali_result;
	int threshold[2];
	
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = stk3x1x_enable_ps(obj->client, 1, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(STK_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = stk3x1x_enable_ps(obj->client, 0, 1)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
	
					goto err_out;
				}
				
				clear_bit(STK_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(STK_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = stk3x1x_get_ps_value(obj, obj->ps);
			if(dat < 0)
			{
				err = dat;
				goto err_out;
			}
#ifdef STK_PS_POLLING_LOG	
			APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps, dat);			
#endif			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;            

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = stk3x1x_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 

					goto err_out;
				}
				set_bit(STK_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = stk3x1x_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 

					goto err_out;
				}
				clear_bit(STK_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(STK_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = stk3x1x_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = stk3x1x_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = stk3x1x_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			dat = atomic_read(&obj->ps_high_thd_val);
			APS_LOG("%s:ps_high_thd_val:%d\n",__func__,dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_PS_THRESHOLD_LOW:
			dat = atomic_read(&obj->ps_low_thd_val);
			APS_LOG("%s:ps_low_thd_val:%d\n",__func__,dat);			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;		
			
		case ALSPS_GET_PS_TEST_RESULT: 
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}

			if(obj->ps > atomic_read(&obj->ps_high_thd_val))
				ps_result = 0;
			else	
				ps_result = 1;
			APS_LOG("ALSPS_GET_PS_TEST_RESULT : ps_result = %d\n",ps_result); 				
			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}			   
			break;
			
		case ALSPS_IOCTL_CLR_CALI: 
			if(copy_from_user(&dat, ptr, sizeof(dat))) 
			{ 
				err = -EFAULT; 
				goto err_out; 
			} 
			obj->ps_cali = 0; 
			atomic_set(&obj->ps_high_thd_val,  obj->hw.ps_threshold_high); 
			atomic_set(&obj->ps_low_thd_val,  obj->hw.ps_threshold_low);  //obj->hw.ps_low_thd_val
			APS_LOG("ALSPS_IOCTL_CLR_CALI : obj->ps_cali:%d high:%d low:%d\n", obj->ps_cali, 
													atomic_read(&obj->ps_high_thd_val), 
													atomic_read(&obj->ps_low_thd_val)); 			
			break;
			
		case ALSPS_IOCTL_GET_CALI: 
			stk3x1x_ps_calibration(obj->client); 
			ps_cali_result = obj->ps_cali; 
			APS_LOG("ALSPS_IOCTL_GET_CALI : ps_cali = %d\n",obj->ps_cali); 
			if(copy_to_user(ptr, &ps_cali_result, sizeof(ps_cali_result))) 
			{ 
				err = -EFAULT; 
				goto err_out; 
			} 
			break; 
			
		case ALSPS_IOCTL_SET_CALI:                 
			//1. libhwm.so calc value store in ps_cali; 
			//2. nvram_daemon update ps_cali in driver 
			if(copy_from_user(&ps_cali_result, ptr, sizeof(ps_cali_result))) 
			{ 
				err = -EFAULT; 
				goto err_out; 
			} 
			obj->ps_cali = ps_cali_result;
#ifdef STK_TUNE0			
			atomic_set(&obj->ps_high_thd_val, obj->ps_cali + obj->stk_ht_n_ct); 
			atomic_set(&obj->ps_low_thd_val,  obj->ps_cali + obj->stk_lt_n_ct); 
#else			
			atomic_set(&obj->ps_high_thd_val, obj->ps_cali + 200); 
			atomic_set(&obj->ps_low_thd_val,  obj->ps_cali + 150); 	
#endif			
			if((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
				goto err_out;       
			if((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
				goto err_out;    
			APS_LOG("ALSPS_IOCTL_SET_CALI : ps_cali_result = %d\n",ps_cali_result); 
			APS_LOG("ALSPS_IOCTL_SET_CALI : obj->ps_cali:%d high:%d low:%d\n", obj->ps_cali, 
													atomic_read(&obj->ps_high_thd_val), 
													atomic_read(&obj->ps_low_thd_val)); 
			break;					

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
			atomic_set(&obj->ps_high_thd_val,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_low_thd_val,  (threshold[1]+obj->ps_cali));//need to confirm

			if((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
			{
				APS_ERR("write high thd error: %ld\n", err);
				goto err_out;        
			}
			
			if((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
			{
				APS_ERR("write low thd error: %ld\n", err);
				goto err_out;       
			}
			
			break;
				
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations stk3x1x_fops = {
	.open = stk3x1x_open,
	.release = stk3x1x_release,
	.unlocked_ioctl = stk3x1x_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice stk3x1x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &stk3x1x_fops,
};
#endif	

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_suspend(struct device *dev) 
{
	APS_FUN();    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_resume(struct device *dev)
{
	APS_FUN();
	return 0;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)
/*----------------------------------------------------------------------------*/
static void stk3x1x_early_suspend(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	int err;
	struct stk3x1x_priv *obj = container_of(h, struct stk3x1x_priv, early_drv);   	
	int old = atomic_read(&obj->state_val);
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
#ifdef STK_CHK_REG		
	err = stk3x1x_validate_n_handle(obj->client);
	if(err < 0)	
	{
		APS_ERR("stk3x1x_validate_n_handle fail: %d\n", err); 
	}
	else if (err == 0xFF)
	{
		if(old & STK_STATE_EN_PS_MASK)
			stk3x1x_enable_ps(obj->client, 1, 0);
	}
#endif /* #ifdef STK_CHK_REG	*/	

#ifdef STK_GES	
	if(obj->ges_enabled == 1)
	{
		obj->re_enable_ges = obj->ges_enabled;		
		stk3x1x_enable_ges(obj->client, 0, 1);	
	}
	else if(obj->ges_enabled == 2)
	{
		obj->re_enable_ges = obj->ges_enabled;	
		stk3x1x_enable_ges(obj->client, 0, 2);	
	}
#endif
	if(old & STK_STATE_EN_ALS_MASK)
	{
		atomic_set(&obj->als_suspend, 1);    
		if((err = stk3x1x_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
		APS_LOG( "%s: Enable ALS : 0\n", __func__);		
	}
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	int err;
	struct stk3x1x_priv *obj = container_of(h, struct stk3x1x_priv, early_drv);         	
	
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
#ifdef STK_CHK_REG		
	err = stk3x1x_validate_n_handle(obj->client);
	if(err < 0)	
	{
		APS_ERR("stk3x1x_validate_n_handle fail: %d\n", err); 
	}
	else if (err == 0xFF)
	{
		if(atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK)
			stk3x1x_enable_ps(obj->client, 1, 0);
	}	
#endif /* #ifdef STK_CHK_REG	*/	
	
#ifdef STK_GES	
		if(obj->re_enable_ges == 1)
		{
			stk3x1x_enable_ges(obj->client, 1, 1);				
			obj->re_enable_ges = 0;		
		}
		else if(obj->re_enable_ges == 2)
		{
			stk3x1x_enable_ges(obj->client, 1, 2);				
			obj->re_enable_ges = 0;				
		}
#endif	
}
#endif

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strlcpy(info->type, stk3x1x_DEV_NAME,sizeof(info->type));
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef STK_GES
static int stk3x1x_set_input_device(struct stk3x1x_priv *obj)
{
	int err;
	
	obj->ges_input_dev = input_allocate_device();
	if (obj->ges_input_dev==NULL)
	{
		APS_ERR( "%s: could not allocate ps device\n", __func__);		
		err = -ENOMEM;
		goto err_ges_input_allocate;
	}
	obj->ges_input_dev->name = "stk_ges";
	obj->ges_input_dev->evbit[0] = BIT_MASK(EV_KEY);
	set_bit(KEY_PAGEUP, obj->ges_input_dev->keybit);
	set_bit(KEY_PAGEDOWN, obj->ges_input_dev->keybit);
	set_bit(KEY_VOLUMEUP, obj->ges_input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, obj->ges_input_dev->keybit);
	
	err = input_register_device(obj->ges_input_dev);	
	if (err<0)
	{
		APS_ERR( "%s: can not register ps input device\n", __func__);	
		return err;
	}
	
	err = sysfs_create_group(&obj->ges_input_dev->dev.kobj, &stk_ges_attribute_group);
	if (err < 0) 
	{
		APS_ERR( "%s:could not create sysfs group for ps\n", __func__);
		return err;
	}	
	input_set_drvdata(obj->ges_input_dev, obj);	
	return 0;

err_ges_input_allocate:
	input_free_device(obj->ges_input_dev);			
	return err;
}
#endif	
	
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int als_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("stk3x1x_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_LIGHT;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
	res = stk3x1x_enable_als(stk3x1x_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}
static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else
    struct stk3x1x_priv *obj = NULL;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_LIGHT;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&stk3x1x_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
	obj = stk3x1x_obj;
	if((err = stk3x1x_read_als(obj->client, &obj->als)))
	{
		err = -1;
	}
	else
	{
		if(obj->als < 3)
		{
			obj->als_last = obj->als;
			*value = stk3x1x_get_als_value(obj, 0);
		}
		else if(abs(obj->als - obj->als_last) >= STK_ALS_CODE_CHANGE_THD)
		{
			obj->als_last = obj->als;
			*value = stk3x1x_get_als_value(obj, obj->als);
		}
		else
		{
			*value = stk3x1x_get_als_value(obj, obj->als_last);
		}
	
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}

static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int ps_enable_nodata(int en)
{
	int res = 0;
	//struct stk3x1x_priv *obj = i2c_get_clientdata(client);
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("stk3x1x_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_PROXIMITY;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
	res = stk3x1x_enable_ps(stk3x1x_obj->client, en,1);
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}	
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}
static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_PROXIMITY;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&stk3x1x_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
    
    if((err = stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps)))
    {
        err = -1;;
    }
    else
    {
        *value = stk3x1x_get_ps_value(stk3x1x_obj, stk3x1x_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	return 0;
}
static int stk3x1x_als_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;

	err = als_enable_nodata(enable_disable ? 1 : 0);
	if (err) {
		APS_ERR("%s:%s failed\n", __func__, enable_disable ? "enable" : "disable");
		return -1;
	}
	err = als_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		APS_ERR("%s set_batch failed\n", __func__);
		return -1;
	}
	return 0;
}
static int stk3x1x_als_factory_get_data(int32_t *data)
{
	int status;

	return als_get_data(data, &status);
}
static int stk3x1x_als_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	struct stk3x1x_priv *obj = stk3x1x_obj;

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return -1;
	}

	err = stk3x1x_read_als(obj->client, &obj->als);
	if (err) {
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
	*data = stk3x1x_obj->als;

	return 0;
}
static int stk3x1x_als_factory_enable_calibration(void)
{
	return 0;
}
static int stk3x1x_als_factory_clear_cali(void)
{
	return 0;
}
static int stk3x1x_als_factory_set_cali(int32_t offset)
{
	return 0;
}
static int stk3x1x_als_factory_get_cali(int32_t *offset)
{
	return 0;
}
static int stk3x1x_ps_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;

	err = ps_enable_nodata(enable_disable ? 1 : 0);
	if (err) {
		APS_ERR("%s:%s failed\n", __func__, enable_disable ? "enable" : "disable");
		return -1;
	}
	err = ps_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		APS_ERR("%s set_batch failed\n", __func__);
		return -1;
	}
	return err;
}
static int stk3x1x_ps_factory_get_data(int32_t *data)
{
	int err = 0, status = 0;

	err = ps_get_data(data, &status);
	if (err < 0)
		return -1;
	return 0;
}
static int stk3x1x_ps_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	struct stk3x1x_priv *obj = stk3x1x_obj;

	err = stk3x1x_read_ps(obj->client, &obj->ps);
	if (err) {
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
	*data = stk3x1x_obj->ps;
	return 0;
}
static int stk3x1x_ps_factory_enable_calibration(void)
{
	return 0;
}
static int stk3x1x_ps_factory_clear_cali(void)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;

	obj->ps_cali = 0;
	return 0;
}
static int stk3x1x_ps_factory_set_cali(int32_t offset)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;

	obj->ps_cali = offset;
	return 0;
}
static int stk3x1x_ps_factory_get_cali(int32_t *offset)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;

	*offset = obj->ps_cali;
	return 0;
}
static int stk3x1x_ps_factory_set_threashold(int32_t threshold[2])
{
	int err = 0;
	struct stk3x1x_priv *obj = stk3x1x_obj;

	APS_LOG("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0], threshold[1]);
	atomic_set(&obj->ps_high_thd_val, (threshold[0] + obj->ps_cali));
	atomic_set(&obj->ps_low_thd_val, (threshold[1] + obj->ps_cali));
	err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val));
	err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
	//err = set_psensor_threshold(obj->client);

	if (err < 0) {
		APS_ERR("set_psensor_threshold fail\n");
		return -1;
	}
	return 0;
}
static int stk3x1x_ps_factory_get_threashold(int32_t threshold[2])
{
	struct stk3x1x_priv *obj = stk3x1x_obj;

	threshold[0] = atomic_read(&obj->ps_high_thd_val) - obj->ps_cali;
	threshold[1] = atomic_read(&obj->ps_low_thd_val) - obj->ps_cali;
	return 0;
}

static struct alsps_factory_fops stk3x1x_factory_fops = {
	.als_enable_sensor = stk3x1x_als_factory_enable_sensor,
	.als_get_data = stk3x1x_als_factory_get_data,
	.als_get_raw_data = stk3x1x_als_factory_get_raw_data,
	.als_enable_calibration = stk3x1x_als_factory_enable_calibration,
	.als_clear_cali = stk3x1x_als_factory_clear_cali,
	.als_set_cali = stk3x1x_als_factory_set_cali,
	.als_get_cali = stk3x1x_als_factory_get_cali,

	.ps_enable_sensor = stk3x1x_ps_factory_enable_sensor,
	.ps_get_data = stk3x1x_ps_factory_get_data,
	.ps_get_raw_data = stk3x1x_ps_factory_get_raw_data,
	.ps_enable_calibration = stk3x1x_ps_factory_enable_calibration,
	.ps_clear_cali = stk3x1x_ps_factory_clear_cali,
	.ps_set_cali = stk3x1x_ps_factory_set_cali,
	.ps_get_cali = stk3x1x_ps_factory_get_cali,
	.ps_set_threashold = stk3x1x_ps_factory_set_threashold,
	.ps_get_threashold = stk3x1x_ps_factory_get_threashold,
};

static struct alsps_factory_public stk3x1x_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &stk3x1x_factory_fops,
};

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stk3x1x_priv *obj = NULL;
	int err = 0;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};

	APS_LOG("%s: driver version: %s\n", __FUNCTION__, DRIVER_VERSION);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	err = get_alsps_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		APS_LOG("get customization info from dts failed\n");
		goto exit_init_failed;
	}
//	memset(obj, 0, sizeof(*obj));
	stk3x1x_obj = obj;
//	obj->hw = hw;
	stk3x1x_get_addr(obj->hw, &obj->addr);

	INIT_DELAYED_WORK(&obj->eint_work, stk3x1x_eint_work);
	// printk("client->addr======%x\n",client->addr);
	//client->addr = 0x48;
	// printk("client->addr======%x\n",client->addr);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->init_done, 0);
	obj->irq_node = client->dev.of_node;
	//obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

	atomic_set(&obj->state_val, 0);
	atomic_set(&obj->psctrl_val, 0x31);
	atomic_set(&obj->alsctrl_val, 0x39);
	obj->ledctrl_val = 0xFF;	
	obj->wait_val = 0xF;
	obj->int_val = 0;
	obj->first_boot = true;			 
	obj->als_correct_factor = 1000;
	atomic_set(&obj->ps_high_thd_val, obj->hw.ps_threshold_high);//  obj->hw.ps_high_thd_val
	atomic_set(&obj->ps_low_thd_val, obj->hw.ps_threshold_low);	
	atomic_set(&obj->recv_reg, 0);  
#ifdef STK_ALS_FIR	
	atomic_set(&obj->firlength, STK_FIR_LEN);	
#endif	

#ifdef STK_TUNE0
	obj->crosstalk = 0;
	obj->stk_max_min_diff = STK_MAX_MIN_DIFF;
	obj->stk_lt_n_ct = STK_LT_N_CT;
	obj->stk_ht_n_ct = STK_HT_N_CT;
	#ifdef CALI_EVERY_TIME
	obj->ps_high_thd_boot = obj->hw.ps_threshold_high;
	obj->ps_low_thd_boot = obj->hw.ps_threshold_low;
	if(obj->ps_high_thd_boot <= 0)
	{
		obj->ps_high_thd_boot = obj->stk_ht_n_ct*3;
		obj->ps_low_thd_boot = obj->stk_lt_n_ct*3;
	}
	#endif	
#endif

#ifdef STK_TUNE1
	obj->hw.polling_mode_ps = 0;
	obj->hw.polling_mode_als = 1;
	APS_LOG("%s: force PS = interrupt and ALS = polling in tune1 mode\n", __FUNCTION__);	
#endif

	if(obj->hw.polling_mode_ps == 0)
	{
		APS_LOG("%s: enable PS interrupt\n", __FUNCTION__);
	}
	obj->int_val |= STK_INT_PS_MODE1;
	
	if(obj->hw.polling_mode_als == 0)
	{
	  obj->int_val |= STK_INT_ALS;		
	  APS_LOG("%s: enable ALS interrupt\n", __FUNCTION__);
	}	

	APS_LOG("%s: state_val=0x%x, psctrl_val=0x%x, alsctrl_val=0x%x, ledctrl_val=0x%x, wait_val=0x%x, int_val=0x%x\n", 
		__FUNCTION__, atomic_read(&obj->state_val), atomic_read(&obj->psctrl_val), atomic_read(&obj->alsctrl_val), 
		obj->ledctrl_val, obj->wait_val, obj->int_val);
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = ARRAY_SIZE(obj->hw.als_level);
	obj->als_value_num = ARRAY_SIZE(obj->hw.als_value);   
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw.als_level));
	memcpy(obj->als_level, obj->hw.als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw.als_value));
	memcpy(obj->als_value, obj->hw.als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	if(atomic_read(&obj->state_val) & STK_STATE_EN_ALS_MASK)
	{
		set_bit(STK_BIT_ALS, &obj->enable);
	}
	
	if(atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK)
	{
		set_bit(STK_BIT_PS, &obj->enable);
	}
	
	stk3x1x_i2c_client = client;

	obj->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&obj->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&obj->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#ifdef STK_TUNE0
	obj->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	obj->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif		
	if((err = stk3x1x_init_client(client)))
	{
		goto exit_init_failed;
	}
	err = alsps_factory_device_register(&stk3x1x_factory_device);
	if(err)
	{
		APS_ERR("stk3x1x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = stk3x1x_create_attr(&(stk3x1x_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
  //  als_ctl.is_use_common_factory =false;	
	if(1 == obj->hw.polling_mode_als)
	{
		als_ctl.is_polling_mode = true;
	}
	else
	{
		als_ctl.is_polling_mode = false;
	}	
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw.is_batch_supported_als;
#else
    als_ctl.is_support_batch = false;
#endif
	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("als_control register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("als_data register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.ps_calibration  = NULL;
//	ps_ctl.ps_calibration  = NULL;
//	ps_ctl.is_use_common_factory = false;
	if(1 == obj->hw.polling_mode_ps)
	{
		ps_ctl.is_polling_mode = true;
		ps_ctl.is_report_input_direct = false;
		wakeup_source_init(&mps_lock, "ps wakelock");		
	}
	else
	{
		ps_ctl.is_polling_mode = false;
		ps_ctl.is_report_input_direct = true;
	}
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw.is_batch_supported_ps;
#else
    ps_ctl.is_support_batch = false;
#endif
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("ps_control register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("ps_data register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
#if 0
	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
	}
	
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
	}
#endif	
#ifdef STK_GES
	err = stk3x1x_set_input_device(obj);
	if(err < 0)
		goto exit_set_input_failed;
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = stk3x1x_early_suspend,
	obj->early_drv.resume   = stk3x1x_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
    stk3x1x_init_flag = 0;
	APS_LOG("%s: OK\n", __FUNCTION__);
//	printk("stk3x1x_i2c_probe======%d,%d\n",atomic_read(&obj->ps_high_thd_val),atomic_read(&obj->ps_low_thd_val));
	app_get_als_ps_sensor_name("stk3x1x");
	return 0;
	
#ifdef STK_GES	
exit_set_input_failed:
	input_unregister_device(obj->ges_input_dev);	
#endif	
exit_sensor_obj_attach_fail:
exit_create_attr_failed:
	//misc_deregister(&stk3x1x_device);
exit_misc_device_register_failed:
exit_init_failed:
   kfree(obj);
exit:
	obj = NULL;   
	stk3x1x_obj = NULL;
	stk3x1x_i2c_client = NULL;
    stk3x1x_init_flag = -1;
	APS_ERR("%s: err = %d\n", __FUNCTION__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_remove(struct i2c_client *client)
{
	int err;	
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);		
	
#ifdef STK_TUNE0
	destroy_workqueue(obj->stk_ps_tune0_wq);	
#endif		
#ifdef STK_GES		
	sysfs_remove_group(&obj->ges_input_dev->dev.kobj, &stk_ges_attribute_group);
	input_unregister_device(obj->ges_input_dev);	
#endif	
	//if((err = stk3x1x_delete_attr(&stk3x1x_i2c_driver.driver)))
	if((err = stk3x1x_delete_attr(&(stk3x1x_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("stk3x1x_delete_attr fail: %d\n", err);
	} 

	alsps_factory_device_deregister(&stk3x1x_factory_device);
	
	stk3x1x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int  stk3x1x_local_uninit(void)
{
	APS_FUN();    
	i2c_del_driver(&stk3x1x_i2c_driver);
	stk3x1x_i2c_client = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/

static int stk3x1x_local_init(void) 
{
//	struct stk3x1x_i2c_addr addr;

	APS_FUN();       
	//stk3x1x_get_addr(hw, &addr);
	if(i2c_add_driver(&stk3x1x_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 

    if(-1 == stk3x1x_init_flag)
    {
		APS_ERR("stk3x1x_local_init fail with stk3x1x_init_flag=%d\n", stk3x1x_init_flag);
        return -1;
    }
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init stk3x1x_init(void)
{
   	alsps_driver_add(&stk3x1x_init_info);// hwmsen_alsps_add(&stk3x1x_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit stk3x1x_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("MingHsien Hsieh");
MODULE_DESCRIPTION("SensorTek stk3x1x proximity and light sensor driver");
MODULE_LICENSE("GPL");
