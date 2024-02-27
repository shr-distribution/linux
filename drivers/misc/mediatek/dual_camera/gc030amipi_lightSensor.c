/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC030Amipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/wakelock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/kthread.h>

#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#include <uapi/linux/sched/types.h>

// extern void ISP_MCLK1_EN(BOOL En);

#define Sleep(ms) mdelay(ms)
static struct i2c_client *g_pstI2Cclient_gc030a;

static struct pinctrl *camlight_gpio = NULL;
static struct pinctrl_state *camlight_gpio_l = NULL;
static struct pinctrl_state *camlight_gpio_h = NULL;

static DEFINE_SPINLOCK(gc030a_drv_lock);

atomic_t trigger_gc030a_thread_flag = ATOMIC_INIT(0);

#define CAMERA_GC030A_DRVNAME1  "kd_camera_gc030a"

static const struct i2c_device_id CAMERA_GC030A_i2c_id[] = { {CAMERA_GC030A_DRVNAME1, 0}, {} };

DECLARE_WAIT_QUEUE_HEAD(trigger_gc030a_thread_wq);

struct camera_device {
	
	struct input_dev *dev;
	
};

struct camera_device *gc030a_inut_dev = NULL;

#define CLIGHT_DEBUG_ON          		0

#if CLIGHT_DEBUG_ON
/* Log define */
//#define CLIGHT_INFO(fmt, arg...)      	printk("<<-GYR INFO->> "fmt"\n", ##arg)
#define CLIGHT_ERR(fmt, arg...)          	printk("<<-yqf camera light->> "fmt"\n", ##arg)
//#define GYR_FUNC(fmt, arg...)      	printk("<<-GYR FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);
#else
//#define GSE_TAG
//#define CLIGHT_INFO(fmt, args...)	do {} while (0)
#define CLIGHT_ERR(fmt, args...)	do {} while (0)
//#define GYR_FUNC(fmt, arg...)   do {} while (0)
#endif

#define CLIGHT_INFO_NAME  "camlight"
static struct class *clight_info_class=NULL;
static struct device *clight_dev=NULL;
unsigned char clight_value = 0;

int iReadRegI2C_gc030a(unsigned char reg_addr, unsigned char *rd_buf, int rd_len)
{
	int ret = 0;
	struct i2c_adapter *adap = g_pstI2Cclient_gc030a->adapter;
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

	msg[0].addr = g_pstI2Cclient_gc030a->addr >> 1;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = w_buf;

	msg[1].addr = g_pstI2Cclient_gc030a->addr >> 1;
	msg[1].flags = 1;
	msg[1].len = rd_len;
	msg[1].buf = r_buf;

	ret = i2c_transfer(adap, msg, 2);

	memcpy(rd_buf, r_buf, rd_len);

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}
EXPORT_SYMBOL(iReadRegI2C_gc030a);

int iWriteRegI2C_gc030a(unsigned char reg_num, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	struct i2c_adapter *adap = g_pstI2Cclient_gc030a->adapter;
	struct i2c_msg msg;
	u8 *w_buf = NULL;

	memset(&msg, 0, sizeof(struct i2c_msg));

	w_buf = kzalloc(wr_len, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;

	w_buf[0] = reg_num;
	memcpy(w_buf + 1, wr_buf, wr_len);

	msg.addr = g_pstI2Cclient_gc030a->addr >> 1;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = w_buf;

	ret = i2c_transfer(adap, &msg, 1);

	kfree(w_buf);
	return ret;
}
EXPORT_SYMBOL(iWriteRegI2C_gc030a);

unsigned char read_cmos_sensor_gc030a(unsigned char addr)
{
	unsigned char get_byte = 0;
	iReadRegI2C_gc030a(addr,&get_byte, 1);

	return get_byte;

}
void write_cmos_sensor_gc030a(unsigned char addr, unsigned char para)
{
		//char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
		iWriteRegI2C_gc030a(addr, &para, 2);

}

static int trigger_gc030a_thread(void *data)
{
	unsigned char camera_light = 0,camera_light_old = 0;
    struct sched_param param = { .sched_priority = 90 }; // RTPM_PRIO_SCRN_UPDATE 94 4
    sched_setscheduler(current, SCHED_FIFO, &param);

	for(;;)
	{ 
        wait_event_interruptible(trigger_gc030a_thread_wq, atomic_read(&trigger_gc030a_thread_flag));
      //  atomic_set(&trigger_gc030a_thread_flag,0);
        msleep(200);

	    camera_light = read_cmos_sensor_gc030a(0xef);   
        clight_value = camera_light;
	    CLIGHT_ERR("camera_light = 0x%x,camera_light_old = 0x%x\n",camera_light,camera_light_old);	
#if 1
     if((camera_light >= 0) && (camera_light != camera_light_old))
	 {	       
               if(camera_light > camera_light_old)
               {

			         if((camera_light - camera_light_old) > 20)
					 {
					       // First NO Occlusion
						   if(camera_light > 0xfa)
						   {
							        camera_light_old = camera_light; 
					                camera_light = 0;
									input_report_key(gc030a_inut_dev->dev, KEY_RIGHTALT, 1);
									input_sync(gc030a_inut_dev->dev);
									input_report_key(gc030a_inut_dev->dev, KEY_RIGHTALT, 0);
									input_sync(gc030a_inut_dev->dev);
						   	}
						   else // Occlusion
							{	
								    camera_light_old = camera_light; 
								    camera_light = 0;
								    input_report_key(gc030a_inut_dev->dev, KEY_LEFTALT, 1);
								    input_sync(gc030a_inut_dev->dev);
								    input_report_key(gc030a_inut_dev->dev, KEY_LEFTALT, 0);
								    input_sync(gc030a_inut_dev->dev);
								   

							 }
					 }
					 else ;
						
               }
			   else
			   {
			   	       if((camera_light_old - camera_light) > 20)
					   {
		                         // First NO Occlusion
							   if(camera_light > 0xfa)
							   {
							        camera_light_old = camera_light; 
					                camera_light = 0;
									input_report_key(gc030a_inut_dev->dev, KEY_RIGHTALT, 1);
									input_sync(gc030a_inut_dev->dev);
									input_report_key(gc030a_inut_dev->dev, KEY_RIGHTALT, 0);
									input_sync(gc030a_inut_dev->dev);
							   	}
							   else // Occlusion
								{	

									     camera_light_old = camera_light; 
									     camera_light = 0;
										 input_report_key(gc030a_inut_dev->dev, KEY_LEFTALT, 1);
										 input_sync(gc030a_inut_dev->dev);
										 input_report_key(gc030a_inut_dev->dev, KEY_LEFTALT, 0);
										 input_sync(gc030a_inut_dev->dev);					
								   
								 }
					    }
						else ;

			   }
          }
		  else ;
#endif	   
	   // exposure_gc030a_1  = read_cmos_sensor_gc030a(0x03);
	   // exposure_gc030a_2  = read_cmos_sensor_gc030a(0x04)    
       
	}
	
	return 0;
}

// yqf add for get_camera_light
static ssize_t clight_chip_show(struct device *dev,struct device_attribute *attr, char *buf)
{
      return sprintf(buf, "0x%x\n", clight_value);
}

static DEVICE_ATTR(get_camera_light, S_IRUGO, clight_chip_show, NULL);
 
void clight_info_init(void)
{

	clight_info_class = class_create(THIS_MODULE, CLIGHT_INFO_NAME);
	clight_dev = device_create(clight_info_class, NULL, 0, 0,"gc030a");
	if (IS_ERR(clight_dev)) {
		printk(KERN_DEBUG
			   "clight_info_init: device_create "
			   "failed (%ld)\n", PTR_ERR(clight_dev));
		return;
	}
	device_create_file(clight_dev, &dev_attr_get_camera_light);  //add by yqf for show device's name

}

void clight_info_deinit(void)
{

       device_remove_file(clight_dev, &dev_attr_get_camera_light); //add by yqf for show device's name
       device_unregister(clight_dev);
	   
	if(clight_info_class!=NULL)
	       class_destroy(clight_info_class);
	
}

static int CAMERA_GC030A_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* get sensor i2c client */
	spin_lock(&gc030a_drv_lock);
	g_pstI2Cclient_gc030a = client;
	/* set I2C clock rate */
	//g_pstI2Cclient_gc030a->timing = 200;	/* 100k */
	//CLIGHT_ERR("yqf to add CAMERA_GC030A_i2c_probe.\n");
	pinctrl_select_state(camlight_gpio, camlight_gpio_h);
   // g_pstI2Cclient_gc030a->addr = 0x78;
	spin_unlock(&gc030a_drv_lock);
//	sensor_id = ((read_cmos_sensor_gc030a(0xf0) << 8) | read_cmos_sensor_gc030a(0xf1));
	if(!kthread_run(trigger_gc030a_thread, NULL, "trigger_gc030a_thread"))
			CLIGHT_ERR("xxxxxxx kthread_run trigger_gc030a_thread failed!!!!!");
	
	//CLIGHT_ERR("yqf to add CAMERA_GC030A_i2c_probe.end\n");
    clight_info_init();
	return 0;
}
/*******************************************************************************
* CAMERA_HW_i2c_remove
********************************************************************************/
static int CAMERA_GC030A_i2c_remove(struct i2c_client *client)
{
 //   CLIGHT_ERR("CAMERA_GC030A_i2c_remove\n");
    clight_info_deinit();
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id CAMERA_GC030A_i2c_of_ids[] = {
    { .compatible = "mediatek,camera_light", },
	{}
};
#endif

struct i2c_driver CAMERA_GC030A_i2c_driver = {
	.probe = CAMERA_GC030A_i2c_probe,
	.remove = CAMERA_GC030A_i2c_remove,
	.driver = {
		   .name = CAMERA_GC030A_DRVNAME1,
		   .owner = THIS_MODULE,

#ifdef CONFIG_OF
		   .of_match_table = CAMERA_GC030A_i2c_of_ids,
#endif
		   },
	.id_table = CAMERA_GC030A_i2c_id,
};

/*******************************************************************************
* CAMERA_HW_probe camera_device *gc030a_inut_dev
********************************************************************************/
static int CAMERA_GC030A_probe(struct platform_device *pdev)
{
	    int ret = 0;

		gc030a_inut_dev = kzalloc(sizeof(struct camera_device), GFP_KERNEL);
		if (gc030a_inut_dev == NULL) {
			CLIGHT_ERR("alloc gc030a_inut_dev failed!\n");
			return -ENOMEM;;
		}
		CLIGHT_ERR("yqf to add CAMERA_GC030A_probe.\n");
		///  allocate input device 
		gc030a_inut_dev->dev = input_allocate_device();
		if (gc030a_inut_dev->dev == NULL) {
			kfree(gc030a_inut_dev);
			CLIGHT_ERR("input_allocate_device gc030a_inut_dev failed!\n");
			return -ENOMEM;
		}
		__set_bit(EV_KEY, gc030a_inut_dev->dev->evbit);
		__set_bit(KEY_LEFTALT, gc030a_inut_dev->dev->keybit);
		__set_bit(KEY_RIGHTALT, gc030a_inut_dev->dev->keybit);
		gc030a_inut_dev->dev->name = "GC030A_sensor";
		gc030a_inut_dev->dev->id.bustype = BUS_HOST;
		if (input_register_device(gc030a_inut_dev->dev)){
			CLIGHT_ERR("input_register_device CAMERA_GC030A_probe failed.\n");
			input_free_device(gc030a_inut_dev->dev);
			return -1;
		}

	    camlight_gpio = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(camlight_gpio)) {
			ret = PTR_ERR(camlight_gpio);
			pr_err("[ERROR] Cannot find camlight_gpio %d!\n", ret);
			return ret;
		}
		
		camlight_gpio_h = pinctrl_lookup_state(camlight_gpio, "cam_light_high");
		if (IS_ERR(camlight_gpio_h)) {
			ret = PTR_ERR(camlight_gpio_h);
			pr_err( "[ERROR] Cannot find camlight_gpio_h:%d!\n", ret);
			return ret;
		}

		camlight_gpio_l = pinctrl_lookup_state(camlight_gpio, "cam_light_low");
		if (IS_ERR(camlight_gpio_l)) {
			ret = PTR_ERR(camlight_gpio_l);
			pr_err( "[ERROR] Cannot find camlight_gpio_l:%d!\n", ret);
			return ret;
		}	

    if (i2c_add_driver(&CAMERA_GC030A_i2c_driver) != 0) {
		CLIGHT_ERR("unable to add CAMERA_GC030A_i2c_driver.");
		return -1;
	}


  CLIGHT_ERR("yqf CAMERA_GC030A_probe!\n");
  return 0;
}


int camlight_pdn_enable(int cmd)                                                                                                                  
{
   // CLIGHT_ERR("yqf %s: LINE=%d, disable=%d\n", __func__, __LINE__, cmd);
    switch (cmd)
    {
	    case 0 :
			//pinctrl_select_state(camlight_gpio, camlight_mclk_off);
	        pinctrl_select_state(camlight_gpio, camlight_gpio_h);
	        break;
	    case 1 :
			//pinctrl_select_state(camlight_gpio, camlight_mclk_on);
	        pinctrl_select_state(camlight_gpio, camlight_gpio_l);
	        break;      
    }
    return 0;

}


/*******************************************************************************
* CAMERA_HW_remove()
********************************************************************************/
static int CAMERA_GC030A_remove(struct platform_device *pdev)
{
#if 1
    input_unregister_device(gc030a_inut_dev->dev);
#endif
	i2c_del_driver(&CAMERA_GC030A_i2c_driver);
	return 0;
}

/*******************************************************************************
*CAMERA_HW_suspend()
********************************************************************************/
static int CAMERA_GC030A_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

/*******************************************************************************
  * CAMERA_HW_DumpReg_To_Proc()
  * Used to dump some critical sensor register
  ********************************************************************************/
static int CAMERA_GC030A_resume(struct platform_device *pdev)
{
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id CAMERA_GC030A_of_ids[] = {
	{.compatible = "mediatek,mtk6739_cam_light",},
	{}
};
#endif


static struct platform_driver g_stCAMERA_GC030A_Driver = {
		.probe = CAMERA_GC030A_probe,
		.remove = CAMERA_GC030A_remove,
		.suspend = CAMERA_GC030A_suspend,
		.resume = CAMERA_GC030A_resume,
		.driver = {
			   .name = "kd_camera_gc030a",
			   .owner = THIS_MODULE,
#ifdef CONFIG_OF
			   .of_match_table = CAMERA_GC030A_of_ids,
#endif
			   }
	};
  
static int __init CAMERA_GC030A_i2C_init(void)
{
    //CLIGHT_ERR("yqf CAMERA_GC030A_i2C_init star\n");
	if (platform_driver_register(&g_stCAMERA_GC030A_Driver)) {
		CLIGHT_ERR("failed to register CAMERA_GC030A driver\n");
		return -ENODEV;
	}

	return 0;
}

/*=======================================================================
  * CAMERA_HW_i2C_exit()
  *=======================================================================*/
static void __exit CAMERA_GC030A_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAMERA_GC030A_Driver);
}

late_initcall(CAMERA_GC030A_i2C_init);
MODULE_AUTHOR("<yangqingfeng@passion-sz.com>");
MODULE_DESCRIPTION("dual camera driver");
MODULE_LICENSE("GPL");

//module_init(CAMERA_GC030A_i2C_init);
//module_exit(CAMERA_GC030A_i2C_exit);

