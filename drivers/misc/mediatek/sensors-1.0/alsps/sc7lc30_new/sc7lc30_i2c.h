
#ifndef __SC7LC30_H__
#define __SC7LC30_H__
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h>
#include <linux/fs.h>   
//#include <linux/wakelock.h> 
#include <linux/sched/clock.h>
#include <asm/io.h>
#include <linux/module.h>


#include "cust_alsps.h"  
#include "alsps.h"
#include <linux/ioctl.h>
#define AUTO_CALI
#define THD_H 900 // 40
#define THD_L 850 // 30
#define MAX_DIFF 40
#define DRIVER_VERSION "sc7lc30_0.9.1_for_android8.1"
#define SC7LC30_I2C_DRIVER_NAME "sc7lc30_driver"
#define SC7LC30_I2C_NAME "sc7lc30"

/*ALSPS REGS*/
#define RPR             "[SC7LC30-ALS/PS]"

#define  SC7LC30_ERR(fmt, arg...)       do {printk(KERN_ERR RPR "ERROR (%s, %d):" fmt, __func__,  __LINE__, ##arg); } while (0)
#define  SC7LC30_WARNING(fmt, arg...)   do {printk(KERN_WARNING RPR "(%s, %d):" fmt, __func__,  __LINE__, ##arg);} while (0)

#define SC7LC30_DEBUG

#ifdef SC7LC30_DEBUG
#define  SC7LC30_FUN(f)              do {printk(KERN_WARNING RPR "(%s, %d)\n", __func__,  __LINE__);} while (0)
#define  SC7LC30_INFO(fmt, arg...)   do {printk(KERN_INFO RPR "INFO (%s, %d):" fmt, __func__,  __LINE__, ##arg);} while (0)
#define  SC7LC30_DBG(fmt, arg...)    do {printk(KERN_DEBUG RPR "DEBUG (%s, %d):" fmt, __func__,  __LINE__, ##arg);} while (0)
#else
#define  SC7LC30_FUN()              do {} while (0)
#define  SC7LC30_INFO(f, a...)      do {} while (0)
#define  SC7LC30_DBG(f, a...)       do {} while (0)
#endif

#define SC7LC30_INTC_CONFIG	         0x00   
#define SC7LC30_PIEN_MASK	         0x40    
#define SC7LC30_PIEN_SHIFT	         (6)    
#define SC7LC30_AIEN_MASK	         0x20   
#define SC7LC30_AIEN_SHIFT	         (5)    
#define SC7LC30_WEN_MASK	         0x10   
#define SC7LC30_WEN_SHIFT	         (4)

#define SC7LC30_MODE_COMMAND	     0x00   
#define SC7LC30_MODE_MASK	         0x07   
#define SC7LC30_MODE_SHIFT	         (0) 


#define SC7LC30_PLS_BOTH_ACTIVE		 0x07  
#define SC7LC30_PLS_ALPS_ACTIVE		 0x03  
#define SC7LC30_PLS_PXY_ACTIVE		 0x05  
#define SC7LC30_PLS_BOTH_DEACTIVE	 0x00  

//SC7LC30 ITIME register
#define SC7LC30_PX_IT_TIME	    	 0x01   
#define SC7LC30_PX_IT_TIME_MASK	     0xf0  
#define SC7LC30_PX_IT_TIME_SHIFT     (4)    	
#define SC7LC30_ALS_IT_TIME_MASK	 0x0f    
#define SC7LC30_ALS_IT_TIME_SHIFT    (0)    	
 
//SC7LC30 GAINALS register
#define SC7LC30_RAN_COMMAND	         0x02   
#define SC7LC30_RAN_MASK		     0x30   
#define SC7LC30_RAN_SHIFT	         (4)    
#define SC7LC30_RAN1_MASK		     0x0c   
#define SC7LC30_RAN1_SHIFT	         (2)    
#define SC7LC30_ALS_RES_MASK	     0x03   
#define SC7LC30_ALS_RES_SHIFT        (0)  

//SC7LC30 GAINPSGS register
#define SC7LC30_PX_CONFIGURE	     0x03 
#define SC7LC30_PX_GAIN_MASK	     0x30 
#define SC7LC30_PX_GAIN_SHIFT	     (4)   
#define SC7LC30_PX_RES_MASK	         0x0f   
#define SC7LC30_PX_RES_SHIFT	     (0)    

//SC7LC30 ALS interrupt filter
#define SC7LC30_AIF_COMMAND	         0x04   
#define SC7LC30_AIF_MASK		     0x0f  
#define SC7LC30_AIF_SHIFT	         (0)    

//SC7LC30 PS interrupt filter
#define SC7LC30_PIF_COMMAND	         0x04   
#define SC7LC30_PIF_MASK		     0xf0   
#define SC7LC30_PIF_SHIFT	         (4)    

//SC7LC30 LEDCTRL register
#define SC7LC30_PX_LED		         0x05  
#define SC7LC30_PX_DRIVE_MASK 	     0xc0  
#define SC7LC30_PX_DRIVE_SHIFT       (6) 
#define SC7LC30_PX_LED_MASK 	     0x3f   
#define SC7LC30_PX_LED_SHIFT         (0)    
  
//SC7LC30 Wait time register
#define SC7LC30_WAIT_TIME	         0x06   

//SC7LC30 ALS Interrupt Threshold register
#define SC7LC30_ALS_LTHH		     0x07    
#define SC7LC30_ALS_LTHH_MASK	     0xff  
#define SC7LC30_ALS_LTHH_SHIFT	     (0)  
#define SC7LC30_ALS_LTHL		     0x08   
#define SC7LC30_ALS_LTHL_MASK	     0xff   
#define SC7LC30_ALS_LTHL_SHIFT	     (0) 

#define SC7LC30_ALS_HTHH		     0x09   
#define SC7LC30_ALS_HTHH_MASK	     0xff  
#define SC7LC30_ALS_HTHH_SHIFT	     (0) 
#define SC7LC30_ALS_HTHL		     0x0a      
#define SC7LC30_ALS_HTHL_MASK	     0xff  
#define SC7LC30_ALS_HTHL_SHIFT	     (0)

//SC7LC30 Proximity Interrupt Threshold register
#define SC7LC30_PX_LTHH			     0x0b    
#define SC7LC30_PX_LTHH_MASK	     0xff  
#define SC7LC30_PX_LTHH_SHIFT	     (0)  

#define SC7LC30_PX_LTHL			     0x0c    
#define SC7LC30_PX_LTHL_MASK	     0xff  
#define SC7LC30_PX_LTHL_SHIFT	     (0) 

#define SC7LC30_PX_HTHH			     0x0d   
#define SC7LC30_PX_HTHH_MASK	     0xff  
#define SC7LC30_PX_HTHH_SHIFT	     (0)  

#define SC7LC30_PX_HTHL			     0x0e   
#define SC7LC30_PX_HTHL_MASK	     0xff   
#define SC7LC30_PX_HTHL_SHIFT	     (0)
#define SC7LC30_PSDATA_READY		0x1f
//SC7LC30 Proximity Data register 
#define	SC7LC30_PX_MSB		         0x20  
#define	SC7LC30_PX_LSB		         0x21  
#define	SC7LC30_PX_LSB_MASK	         0xff   
#define	SC7LC30_PX_MSB_MASK	         0xff   

//SC7LC30 ALS Data register
#define	SC7LC30_ADC_MSB		         0x22 
#define	SC7LC30_ADC_LSB		         0x23  

//SC7LC30 Proximity Offset Data register
#define  SC7LC30_POFFSETH_REG        0x2c   
#define  SC7LC30_POFFSETL_REG        0x2d   

//SC7LC30 STATUS register
#define SC7LC30_OBJ_COMMAND	         0x30   
#define SC7LC30_OBJ_MASK		     0x20   
#define SC7LC30_OBJ_SHIFT	         (5)    

#define SC7LC30_INT_COMMAND	         0x30    
#define SC7LC30_INT_MASK		     0x44  
#define SC7LC30_INT_SHIFT	         (0)  
#define SC7LC30_INT_PMASK		     0x40   
#define SC7LC30_INT_AMASK		     0x04   

//SC7LC30 CONFIG register
#define SC7LC30_CONFIG_REG	         0x31   
#define SC7LC30_CONFIG_BDU_MASK      0x40   
#define SC7LC30_CONFIG_BDU_SHIFT     (6)  

//SC7LC30 COMMAND registers
#define  SC7LC30_COMMAND_REG         0x32   
#define  SC7LC30_COMMAND_INT_H_MASK  0x40  
#define  SC7LC30_COMMAND_INT_H_SHIFT (6)   
#define  SC7LC30_COMMAND_LIR_A_MASK  0x02  
#define  SC7LC30_COMMAND_LIR_A_SHIFT (1)   
#define  SC7LC30_COMMAND_LIR_P_MASK  0x01  
#define  SC7LC30_COMMAND_LIR_P_SHIFT (0)   	
 typedef struct{
	struct alsps_hw  hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
	struct delayed_work  cali_work;
	//atomic_t	trace;
	atomic_t	 i2c_retry;
	//atomic_t	 als_suspend;
	//atomic_t	 ps_mask;		 /*mask ps: always return far away*/
	//atomic_t	 ps_debounce;	 /*debounce time after enabling ps*/
	//atomic_t	 ps_deb_on; 	 /*indicates if the debounce is on*/
	//atomic_t	 ps_deb_end;	 /*the jiffies representing the end of debounce*/
	//atomic_t	 ps_suspend;
	//atomic_t	 init_done;
	struct 	 device_node *irq_node;
	int		 irq;
	//struct sc7lc30_i2c_addr addr;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif   
	/*data*/
	u16		 als;
	u16		 ps;
	//u8 		 _align;
	u16		 als_level_num;
	u16		 als_value_num;
	u32		 als_level[C_CUST_ALS_LEVEL-1];
	u32		 als_value[C_CUST_ALS_LEVEL];
	u8 		it_val ;
	u8 		led_ctrl;
	u8 		ps_gain;
	u8 		als_gain;
	u8 		wait_time;
	u8 		config_val;
	u8 		pers_val;
	u8 		command_val;
	u8		ps_pers;
	u16 	ps_data_max;
	u16 	ps_data_min;
	u16 	h_thd_offset;
	u16 	l_thd_offset;
	u16 	firlength;
	u16		ps_data;
	u16		als_data;
	u16 	als_last;
	u32 	initial_noise;
	u8 	ps_distance_last;
	atomic_t 	state_val;
	atomic_t 	ps_hth;
	atomic_t	 ps_lth;
	atomic_t 	ps_enable_state;
	atomic_t 	als_enable_state;
	atomic_t 	als_tran;
	atomic_t    als_suspend;
	int 	diff;
	bool 	first_enable;
	int 	cali_done;
	int 	cali_fail;
	u8 		renable_als;

}SC7LC30_DATA;
#endif





