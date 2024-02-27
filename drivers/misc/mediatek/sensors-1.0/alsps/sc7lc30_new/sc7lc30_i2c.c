
#include <linux/of.h>
#include <linux/of_address.h>
//#include <linux/of_irq.h>
#include "cust_alsps.h"
#include "alsps.h"
#include "sc7lc30_i2c.h"
#include "core.h"

#include <linux/gpio.h>

   
#if 1

#define APS_TAG                  "[sc7lc30] "
#define APS_FUN(f)               printk(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)    printk(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args) 

#else

#define APS_TAG                  "[sc7lc30]"
#define APS_FUN(f)                do {} while (0)
#define APS_ERR(fmt, args...)     do {} while (0)
#define APS_LOG(fmt, args...)      do {} while (0) 
#define APS_DBG(fmt, args...)      do {} while (0)

#endif



/* Maintain alsps cust info here */
//static struct alsps_hw alsps_cust;
//static struct alsps_hw *hw = &alsps_cust;
//struct platform_device *alspsPltFmDev;




/* logical functions */
static int sc7lc30_set_l_thd(int thd_l);
static int sc7lc30_set_h_thd(int thd_h);
static int sc7lc30_i2c_suspend(struct device *dev);
static int sc7lc30_i2c_resume(struct device *dev);

static int __init           sc7lc30_init(void);
static void __exit          sc7lc30_exit(void);
static int                  sc7lc30_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  sc7lc30_remove(struct i2c_client *client);
//static void                 sc7lc30_power(struct alsps_hw *hw, unsigned int on);
static int                  sc7lc30_als_open_report_data(int open);
static int                  sc7lc30_als_enable_nodata(int en);
static int                  sc7lc30_als_set_delay(u64 delay);
static int                  sc7lc30_als_get_data(int *als_value, int *status);
static int                  sc7lc30_local_init(void);
static int                  sc7lc30_local_remove(void);
static int                  sc7lc30_ps_open_report_data(int open);
static int                  sc7lc30_ps_enable_nodata(int en);
static int                  sc7lc30_ps_set_delay(u64 delay);
static int                  sc7lc30_ps_get_data(int *als_value, int *status);
static struct i2c_client *sc7lc30_i2c_client = NULL; 
static SC7LC30_DATA                 *sc7lc30_pldata = NULL;
static int sc7lc30_enable_ps(struct i2c_client *client,int en);
static int sc7lc30_enable_als(struct i2c_client *client,int en);
static int sc7lc30_read_ps_data(SC7LC30_DATA  * pls);
static int sc7lc30_read_als_data(SC7LC30_DATA  *pls);

static int sc7lc30_init_flag = -1;


/**************************** variable declaration ****************************/


/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id sc7lc30_id[] = {
    { SC7LC30_I2C_NAME, 0 }, 
    { }
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,alsps"},  /* this string should be the same with dts */
    {},
};
#endif
static int sc7lc30_i2c_suspend(struct device *dev) 
{
	int err;
	struct i2c_client *client = to_i2c_client(dev);
	SC7LC30_DATA *obj = i2c_get_clientdata(client);
	

	APS_LOG("sc7lc30_i2c_suspend\n");

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return 0;
	}

	atomic_set(&obj->als_suspend, 1);
	err = sc7lc30_enable_als(obj->client, 0);
	if (err)
		APS_ERR("disable als fail: %d\n", err);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int sc7lc30_i2c_resume(struct device *dev)
{
	int err;
	struct i2c_client *client = to_i2c_client(dev);
    SC7LC30_DATA *obj = i2c_get_clientdata(client);
	
	struct hwm_sensor_data sensor_data;
	memset(&sensor_data, 0, sizeof(sensor_data));
	APS_LOG("sc7lc30_i2c_resume\n");
	if (!obj) {
		APS_ERR("null pointer!!\n");
		return 0;
	}

	atomic_set(&obj->als_suspend, 0);
	
		err = sc7lc30_enable_als(obj->client, 1);
		if (err)
			APS_ERR("enable als fail: %d\n", err);


	return 0;
}

static int sc7lc30_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, SC7LC30_I2C_DRIVER_NAME);
	return 0;
}

/* represent an I2C device driver */
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops sc7lc30_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sc7lc30_i2c_suspend, sc7lc30_i2c_resume)
};
#endif

static struct i2c_driver sc7lc30_driver = {
    .driver = {                     /* device driver model driver */
        .owner = THIS_MODULE,
        .name  = SC7LC30_I2C_DRIVER_NAME,
#ifdef CONFIG_PM_SLEEP
					.pm   = &sc7lc30_pm_ops,
#endif

#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
#endif
    },
    .probe    = sc7lc30_probe,          /* callback for device binding */
    .remove   = sc7lc30_remove,         /* callback for device unbinding */
    .id_table = sc7lc30_id,             /* list of I2C devices supported by this driver */
	.detect     = sc7lc30_i2c_detect,
};

/* MediaTek alsps information */
static struct alsps_init_info sc7lc30_init_info = {
    .name = SC7LC30_I2C_NAME,        /* Alsps driver name */
    .init = sc7lc30_local_init,      /* Initialize alsps driver */
    .uninit = sc7lc30_local_remove,        /* Uninitialize alsps driver */
};

/**
 * @Brief: sc7lc30_register_dump print register value for debug
 *
 * @Param: reg_address regsiter address
 *
 * @return: no return
 */
static int sc7lc30_pls_read_data(unsigned char addr, unsigned char *data)
{
	int ret;
	struct i2c_client *client;
	client = sc7lc30_i2c_client;
	
	 ret = i2c_smbus_read_byte_data(client, addr);
	 if (ret < 0) {
        SC7LC30_ERR( "sc7lc30_pls_read_data : transfer error \n");
    }
    *data = (unsigned char)(ret);
	return ret;
}
static int sc7lc30_pls_write_data(unsigned char addr, unsigned char data)
{
	struct i2c_client *client;
	int ret;
	//u8 buf[2];
	client = sc7lc30_i2c_client;
	ret = i2c_smbus_write_byte_data(client, addr, data);
	 if (ret < 0) {
        SC7LC30_ERR( "sc7lc30_pls_read_data : transfer error \n");
    }
	 return ret;
}
#if 0
static int sc7lc30_open(struct inode *inode, struct file *file)
{
	file->private_data = sc7lc30_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
static int sc7lc30_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
#endif
#if 0
static long sc7lc30_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//	struct i2c_client *client = (struct i2c_client*)file->private_data;
	
	SC7LC30_DATA *obj = sc7lc30_pldata;
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat,val;
	uint32_t enable;
	int ps_result;
	
	
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
				if((val = sc7lc30_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				atomic_set(&obj->ps_enable_state,1);
				
			}
			else
			{
				if((val = sc7lc30_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
	
					goto err_out;
				}
				
				atomic_set(&obj->ps_enable_state,0);
			}
			break;

		case ALSPS_GET_PS_MODE:
			
			enable = atomic_read(&obj->ps_enable_state);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA: 
			
			if((val = sc7lc30_read_ps_data(obj)))
			{
				goto err_out;
			}
			if(val < 80)
				dat = 1;
			else if(val > 120)
				dat = 0;
		
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA: 
			
			
			val = sc7lc30_read_ps_data(obj);
			if(val < 0)
			{
				goto err_out;
			}
			
			dat = obj->ps_data;
			
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
				if((val = sc7lc30_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 

					goto err_out;
				}
				atomic_set(&obj->als_enable_state,1);
			}
			else
			{
				if((val = sc7lc30_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 

					goto err_out;
				}
				atomic_set(&obj->als_enable_state,0);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			
			enable = atomic_read(&obj->als_enable_state);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			
			if((val = sc7lc30_read_als_data(obj)))
			{
				goto err_out;
			}

			dat = val;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA: 
			
			if(( sc7lc30_read_als_data(obj))<0)
			{
				goto err_out;
			}

			dat = obj->als_data;
			printk("======8   dat == %d  \n",dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			
			dat = atomic_read(&obj->ps_hth);
			APS_LOG("%s:ps_high_thd_val:%d\n",__func__,dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_PS_THRESHOLD_LOW:
			
			dat = atomic_read(&obj->ps_hth);
			APS_LOG("%s:ps_low_thd_val:%d\n",__func__,dat);			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;		
			
		case ALSPS_GET_PS_TEST_RESULT: 
				
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
			break;
			
		case ALSPS_IOCTL_GET_CALI: 
			
			break; 
			
		case ALSPS_IOCTL_SET_CALI:                 
			
			break;					

		case ALSPS_SET_PS_THRESHOLD:
			
		
			break;
				
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}



static struct file_operations sc7lc30_fops = {
	.owner = THIS_MODULE,
	.open = sc7lc30_open,
	.release = sc7lc30_release,
	.unlocked_ioctl = sc7lc30_unlocked_ioctl,
};





/*----------------------------------------------------------------------------*/
static struct miscdevice sc7lc30_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &sc7lc30_fops,

};

#endif

/*----------------------------------------------------------------------------*/

static ssize_t sc7lc30_show_enablep(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	ret = atomic_read(&sc7lc30_pldata->ps_enable_state);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_store_enablep(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%d", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	ret = sc7lc30_enable_ps(sc7lc30_pldata->client,value);
	return count;
}
static ssize_t sc7lc30_show_enablel(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	ret = atomic_read(&sc7lc30_pldata->als_enable_state);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_store_enablel(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%d", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	ret = sc7lc30_enable_als(sc7lc30_pldata->client,value);
	return count;
}
static ssize_t sc7lc30_show_ps(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	ret = sc7lc30_read_ps_data(sc7lc30_pldata);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_show_als(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return 0;
	}
	ret = sc7lc30_read_als_data(sc7lc30_pldata);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_store_w_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
		{
			APS_ERR("invalid format: '%s'\n", buf);
			return 0;
		}
	dat = (u8)cmd;
	sc7lc30_pls_write_data(addr,dat);
	return count;
}
static ssize_t sc7lc30_show_r_reg(struct device_driver *ddri, char *buff)
{
	ssize_t ret = 0;
	char i = 0;
	unsigned char val = 0;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	for(i = 0;i <= 0x32;i++){
		sc7lc30_pls_read_data(i,&val);
		ret += sprintf(&buff[ret], "0x%x: 0x%x\n", i, val);
	}	

	return ret;
}
static ssize_t sc7lc30_show_ps_h_thd(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return 0;
	}
	ret = atomic_read(&sc7lc30_pldata->ps_hth);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_show_ps_l_thd(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return 0;
	}
	ret = atomic_read(&sc7lc30_pldata->ps_lth);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_show_h_ofsset(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	ret = sc7lc30_pldata->h_thd_offset;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t sc7lc30_store_h_ofsset(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%d", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	sc7lc30_pldata->l_thd_offset = value;
	atomic_set(&sc7lc30_pldata->ps_hth,sc7lc30_pldata->ps_data_min + value);
	ret = sc7lc30_set_h_thd(atomic_read(&sc7lc30_pldata->ps_hth));
	return count;
}
static ssize_t sc7lc30_show_l_offset(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return 0;
	}
	ret = sc7lc30_pldata->l_thd_offset;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_store_l_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%d", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	sc7lc30_pldata->l_thd_offset = value;
	atomic_set(&sc7lc30_pldata->ps_lth,sc7lc30_pldata->ps_data_min + value);
	ret = sc7lc30_set_h_thd(atomic_read(&sc7lc30_pldata->ps_lth));
	return count;
}
static ssize_t sc7lc30_show_tran(struct device_driver *ddri, char *buf)
{
	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return 0;
	}
	ret = atomic_read(&sc7lc30_pldata->als_tran);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static ssize_t sc7lc30_store_tran(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
//	int ret;
	if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%d", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	atomic_set(&sc7lc30_pldata->als_tran,value);
	return count;
}

static DRIVER_ATTR(enablep,  S_IWUSR | S_IRUGO, sc7lc30_show_enablep,sc7lc30_store_enablep);
static DRIVER_ATTR(enablel,  S_IWUSR | S_IRUGO, sc7lc30_show_enablel,sc7lc30_store_enablel);
static DRIVER_ATTR(ps,  S_IWUSR | S_IRUGO, sc7lc30_show_ps,NULL);
static DRIVER_ATTR(als,  S_IWUSR | S_IRUGO, sc7lc30_show_als,NULL);
static DRIVER_ATTR(tran,  S_IWUSR | S_IRUGO, sc7lc30_show_tran,sc7lc30_store_tran);

static DRIVER_ATTR(w_reg,  S_IWUSR | S_IRUGO,NULL,sc7lc30_store_w_reg);
static DRIVER_ATTR(r_reg,  S_IWUSR | S_IRUGO,sc7lc30_show_r_reg,NULL);
static DRIVER_ATTR(ps_h_thd,  S_IWUSR | S_IRUGO,sc7lc30_show_ps_h_thd,NULL);
static DRIVER_ATTR(ps_l_thd,  S_IWUSR | S_IRUGO,sc7lc30_show_ps_l_thd,NULL);
static DRIVER_ATTR(h_offset,  S_IWUSR | S_IRUGO,sc7lc30_show_h_ofsset,sc7lc30_store_h_ofsset);
static DRIVER_ATTR(l_offset,  S_IWUSR | S_IRUGO,sc7lc30_show_l_offset,sc7lc30_store_l_offset);

static struct driver_attribute *sc7lc30_attr_list[] = {
	&driver_attr_enablep,
	&driver_attr_enablel,
	&driver_attr_ps,
	&driver_attr_als,
	&driver_attr_tran,
	&driver_attr_w_reg,
	&driver_attr_r_reg,
	&driver_attr_ps_h_thd,
	&driver_attr_ps_l_thd,
	&driver_attr_h_offset,
	&driver_attr_l_offset,
};

/*----------------------------------------------------------------------------*/
static int sc7lc30_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(sc7lc30_attr_list)/sizeof(sc7lc30_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, sc7lc30_attr_list[idx])))
        {
            SC7LC30_WARNING("driver_create_file (%s) = %d\n", sc7lc30_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int sc7lc30_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(sc7lc30_attr_list)/sizeof(sc7lc30_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, sc7lc30_attr_list[idx]);
    }

    return err;
}

/*----------------------------------------------------------------------------*/


static int sc7lc30_als_open_report_data(int open)
{
    

    return 0;
}


 static int sc7lc30_enable_als(struct i2c_client *client,int en)
{
	SC7LC30_DATA *pls = sc7lc30_pldata;
	int cur_en;
	int ret;
	unsigned char state_reg;
	cur_en = en;
	ret = sc7lc30_pls_read_data(SC7LC30_INTC_CONFIG,&state_reg);
	if(ret < 0)
	{
		APS_ERR("sc7lc30_pls_read_data err\n");
		return -1;
	}
	if(cur_en)
	{
		state_reg |= 0x03;
		
		ret = sc7lc30_pls_write_data(SC7LC30_INTC_CONFIG,state_reg);
		atomic_set(&pls->als_enable_state,1);
	}
	else
	{
		state_reg &= (~(0x02));
		
		ret = sc7lc30_pls_write_data(SC7LC30_INTC_CONFIG,state_reg);
		atomic_set(&pls->als_enable_state,0);
	}
	SC7LC30_DBG("%s: robert enable = %d err =  %d\n",__func__,cur_en,ret);
	return ret;
}
static int sc7lc30_als_enable_nodata(int en)
{
    SC7LC30_DATA *pls = sc7lc30_pldata;
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("sc7lc30_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_LIGHT;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!pls)
	{
		APS_ERR("sc7lc30_data is null!!\n");
		return -1;
	}
	res = sc7lc30_enable_als(pls->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(res < 0){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	
	return 0;
}

/**
 * @Brief: sc7lc30_als_set_delay Set delay,not used for now.
 *
 * @Param: delay Delay time.
 *
 * @Returns: 0 for success,other for failed.
 */
static int sc7lc30_als_set_delay(u64 delay)
{
    SC7LC30_FUN();
   // msleep(delay/1000/1000);

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


static int sc7lc30_set_l_thd(int thd_l)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_PX_LTHL ,(thd_l & 0x00FF))<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	if(sc7lc30_pls_write_data(SC7LC30_PX_LTHH ,(thd_l & 0xFF00) >> 8)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}
		return ret;
}
static int sc7lc30_set_h_thd(int thd_h)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_PX_HTHL ,(thd_h & 0x00FF))<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	if(sc7lc30_pls_write_data(SC7LC30_PX_HTHH ,(thd_h & 0xFF00) >> 8)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
#ifdef AUTO_CALI
static void sc7lc30_cali_work(struct work_struct *work)
{
	SC7LC30_DATA *pls = sc7lc30_pldata;
	int ret;
	int diff;
	ret = sc7lc30_read_ps_data(pls);
	if(ret < 0)
	{
		SC7LC30_DBG("%s: get ps data fail\n",__func__);
		pls->cali_fail ++;
		goto err_cali_fail;
	}
	if(ret > pls->ps_data_max)
		pls->ps_data_max = ret;
	if(ret < pls->ps_data_min)
		pls->ps_data_min = ret;
	SC7LC30_DBG("%s:PS_DATA_MAX = %d,PS_DATA_MIN = %d\n",__func__,pls->ps_data_max,pls->ps_data_min);
	diff =  pls->ps_data_max - pls->ps_data_min;
	if(diff > pls->diff)
		{
		atomic_set(&pls->ps_hth,pls->ps_data_min + pls->h_thd_offset);
		atomic_set(&pls->ps_lth,pls->ps_data_min + pls->l_thd_offset);
		sc7lc30_set_h_thd(atomic_read(&pls->ps_hth));
		sc7lc30_set_l_thd(atomic_read(&pls->ps_lth));
		pls->cali_done = 1;
	}
	if(1 == pls->cali_done)
	{
		SC7LC30_DBG("%s:ps cali done,pls->ps_hth = %d,pls->ps_lth = %d\n",__func__,atomic_read(&pls->ps_hth),atomic_read(&pls->ps_lth));
		//cancel_delayed_work_sync(&pls->cali_work);
		pls->cali_fail = 0;
		return ;
	}
	else
	{
		schedule_delayed_work(&pls->cali_work, msecs_to_jiffies(200));
	}
	return ;
err_cali_fail:
	if(pls->cali_fail > 30)
	{
		SC7LC30_DBG("%s:cali fail\n",__func__);
		//cancel_delayed_work_sync(&pls->cali_work);
		return ;
	}
return ;
}

static void sc7lc30_cali(struct i2c_client *client)
{
	SC7LC30_DATA *pls = sc7lc30_pldata;
	cancel_delayed_work_sync(&pls->cali_work);
	INIT_DELAYED_WORK(&pls->cali_work, sc7lc30_cali_work);
	schedule_delayed_work(&pls->cali_work, msecs_to_jiffies(200));
}

#endif
static void sc7lc30_ps_int_handle(SC7LC30_DATA  * pls,int state)
{
	unsigned char data[2]={0};
	int ret = -1;
	unsigned char int_state;
	ps_report_interrupt_data(state);
	if(0 == state)
	{
		ret = sc7lc30_set_h_thd(9999);
		ret = sc7lc30_set_l_thd(atomic_read(&pls->ps_lth));
		sc7lc30_pls_read_data(SC7LC30_PX_HTHL,&data[0]);
		sc7lc30_pls_read_data(SC7LC30_PX_HTHH,&data[1]);
		ret = (((data[1]&0xff)<<8) + data[0]); 
		SC7LC30_DBG("%s: h_thd = %d\n",__func__,ret);
	}
	else if(1 == state)
		{
		ret = sc7lc30_set_h_thd(atomic_read(&pls->ps_hth));
		ret = sc7lc30_set_l_thd(1);
		sc7lc30_pls_read_data(SC7LC30_PX_LTHL,&data[0]);
		sc7lc30_pls_read_data(SC7LC30_PX_LTHH,&data[1]);
		ret = (((data[1]&0xff)<<8) + data[0]); 
		SC7LC30_DBG("%s: l_thd = %d\n",__func__,ret);
		}
	ret = sc7lc30_pls_read_data(SC7LC30_INT_COMMAND,&int_state);
	if(ret < 0)
	{
		APS_ERR("sc7lc30_pls_read_data err\n");	
	}
	
	SC7LC30_DBG("%s: state = %d\n",__func__,state);	
	pls->ps_distance_last = state;
}
static void sc7lc30_eint_work(struct work_struct *work)
{
	SC7LC30_DATA *pls = sc7lc30_pldata;
	int ret;
//	unsigned char int_state;
	int n_f_state=-1;
#if 0
	err = sc7lc30_pls_read_data(SC7LC30_INT_COMMAND,&int_state);
	if(err < 0)
	{
		APS_ERR("sc7lc30_pls_read_data err\n");	
	}
	//SC7LC30_DBG("%s: int_state = 0x%x\n",__func__,int_state);
	if(((int_state & 0x40)>>6))
	{
		 if(((int_state & 0x20)>>5))
		 	n_f_state = 0;
		 else if(((int_state & 0x10)>>4))
		 	n_f_state = 1;
		 sc7lc30_ps_int_handle(pls, n_f_state);
	}
	msleep(1);
#endif
	ret = sc7lc30_read_ps_data(pls);
	if(ret > atomic_read(&pls->ps_hth))
		n_f_state = 0;
	else if(ret < atomic_read(&pls->ps_hth))
		n_f_state = 1;
	 sc7lc30_ps_int_handle(pls, n_f_state);
#if defined(CONFIG_OF)
	enable_irq(pls->irq);
#endif
	 
}

static int sc7lc30_enable_ps(struct i2c_client *client,int cur_en)
{
	SC7LC30_DATA *pls = sc7lc30_pldata;
	//int cur_en;
	int err;
	unsigned char state_reg;
	//cur_en = en;
	err = sc7lc30_pls_read_data(SC7LC30_INTC_CONFIG,&state_reg);
	SC7LC30_DBG("%s: start\n",__func__);
	if(err < 0)
	{
		APS_ERR("sc7lc30_pls_read_data err\n");
	}
	if(cur_en)
	{
		state_reg |= 0x05;
		if(! pls->hw.polling_mode_ps)
		{
			state_reg |= 0x40;
		}
		err = sc7lc30_pls_write_data(SC7LC30_INTC_CONFIG,state_reg);
		atomic_set(&pls->ps_enable_state,1);
#ifdef AUTO_CALI
	if(!pls->first_enable)
		sc7lc30_cali(client);
#endif
	}
	else
	{
		state_reg &= (~(0x04));
		if(! pls->hw.polling_mode_ps)
		{
			state_reg &= (~(0x40));
		}
		err = sc7lc30_pls_write_data(SC7LC30_INTC_CONFIG,state_reg);
		atomic_set(&pls->ps_enable_state,0);
	}
	return err;
}
static int sc7lc30_check_psdata_ready(void)
{
	int i;
	unsigned char flag;
	for(i =0;i<5;i++)
	{
		msleep(10);// mdelay(100); yqf modify
		sc7lc30_pls_read_data(SC7LC30_PSDATA_READY,&flag);
		SC7LC30_DBG("%s: sc7lc30_pls_read_data flag = 0x%x\n",__func__,flag);
		if(flag&0x10)
			return 1;
	}
	return -1; 
}
static int sc7lc30_read_ps_data(SC7LC30_DATA  * pls)
{
	unsigned char data[2]={0};
	int ret;
	ret = sc7lc30_check_psdata_ready();
	if(ret)
	{
		sc7lc30_pls_read_data(SC7LC30_PX_LSB,&data[0]);
		sc7lc30_pls_read_data(SC7LC30_PX_MSB,&data[1]);
	}
	else
	{
		SC7LC30_DBG("%s: check psdata err\n",__func__);
		return -1;
	}
	ret = (((data[1]&0xff)<<8) + data[0]); 
	pls->ps_data= ret;
	SC7LC30_DBG("sc7lc30_read_ps_data ret=%d\n",ret);
	return ret;
}

static int sc7lc30_boot_cali(SC7LC30_DATA  * pls)
{
	int ret;
	int i;
	int loop=0;
	int crosstalk;
	if((ret = sc7lc30_enable_ps(pls->client,1)) < 0)
	{
		APS_ERR("%s: sc7lc30_enable_ps fail\n",__func__);
		return -1;
	}
	for(i= 0;i<10;i++)
	{
		msleep(100);
		crosstalk = sc7lc30_read_ps_data(pls);
		if(crosstalk >= 0)
		{
			pls->initial_noise += crosstalk;
			loop++;
		}
	}
	pls->initial_noise = pls->initial_noise/loop;
	atomic_set(&pls->ps_hth,pls->initial_noise + pls->h_thd_offset);
	atomic_set(&pls->ps_lth,pls->initial_noise + pls->l_thd_offset);
	ret = sc7lc30_set_h_thd(atomic_read(&pls->ps_hth));
	if(ret < 0)
	{
		SC7LC30_DBG("%s: set sc7lc30_set_h_thd fail\n",__func__);
		return -1;
	}
	ret = sc7lc30_set_l_thd(atomic_read(&pls->ps_lth));
	if(ret < 0)
	{
		SC7LC30_DBG("%s: set sc7lc30_set_h_thd fail\n",__func__);
		return -1;
	}
	
	SC7LC30_DBG("%s initial_noise = %d,pls->ps_hth=%d,pls->ps_lth = %d\n ",__FUNCTION__,pls->initial_noise,atomic_read(&pls->ps_hth),atomic_read(&pls->ps_lth));
	if((ret = sc7lc30_enable_ps(pls->client,0)) < 0)
	{
		APS_ERR("%s: sc7lc30_enable_ps fail\n",__func__);
		return -1;
	}	
	pls->first_enable = false;
	return 0;
}
static int sc7lc30_ps_open_report_data(int open)
{
    
    return 0;
}

static int sc7lc30_ps_enable_nodata(int en)
{
    SC7LC30_DATA *pls = sc7lc30_pldata;
	int res = 0;
	
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("%s: enable value = %d\n",__func__, en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_PROXIMITY;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!pls)
	{
		APS_ERR("sc7lc30_pldata is null!!\n");
		return -1;
	}
	res = sc7lc30_enable_ps(pls->client, en);
	if(res<0){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}	
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return 0;
}

/**
 * @Brief: sc7lc30_als_set_delay Set delay,not used for now.
 *
 * @Param: Delay time: unit ns.
 *
 * @Returns: 0 for success,other for failed.
 */
static int sc7lc30_ps_set_delay(u64 delay)
{
    SC7LC30_FUN();

    //msleep(delay/1000/1000);
    return 0;
}
static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return sc7lc30_als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int sc7lc30_read_als_data(SC7LC30_DATA  *pls)
{
	unsigned char data[2]={0};
	int ret;
	//ret = sc7lc30_check_alsdata_ready();
	if(1)
	{
		sc7lc30_pls_read_data(SC7LC30_ADC_LSB,&data[0]);
		sc7lc30_pls_read_data(SC7LC30_ADC_MSB,&data[1]);
	}
	else
	{
		SC7LC30_DBG("%s: check psdata err\n",__func__);
		return -1;
	}
	ret = (((data[1]&0xff)<<8) + data[0]); 
//	ret = ret * atomic_read(&pls->als_tran)/1000;
	SC7LC30_DBG("%s:ret = %d",__func__,ret);
	pls->als_data = ret;
	return ret;
}

static int sc7lc30_get_als_value(SC7LC30_DATA  *obj,int als_data)
{
	int idx;
	for(idx = 0; idx < 15; idx++)
	{
		if(als_data < obj->hw.als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= 15)
	{
		APS_ERR("exceed range\n"); 
		idx =14;
	}
		SC7LC30_DBG("%s: als_data = %d,obj->hw.als_value[idx] =%d",__func__,als_data,obj->hw.als_value[idx]);
	return obj->hw.als_value[idx];
}

static int sc7lc30_als_get_data(int *als_value, int *status)
{
    int err = 0;
	int ret;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else
    SC7LC30_DATA *obj = sc7lc30_pldata;
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
        *als_value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!obj)
	{
		APS_ERR("sc7lc30_obj is null!!\n");
		return -1;
	}
	
	if((ret = sc7lc30_read_als_data(obj)) < 0)
	{
		err = -1;
	}
	else
	{
		obj->als_last = obj->als_data;
		SC7LC30_DBG("%s:obj->als_last = %d",__func__,obj->als_last);
		*als_value = sc7lc30_get_als_value(obj,obj->als_last);
		//*als_value = ret;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif

   
    return 0;
}

 int sc7lc30_read_ps_state(SC7LC30_DATA  *pls)
{
	int n_f_state=-1;
	//unsigned char int_state;
	int ret;
	
	ret = sc7lc30_read_ps_data(pls);
	if(ret > atomic_read(&pls->ps_hth))
		n_f_state = 0;
	else if(ret < atomic_read(&pls->ps_hth))
		n_f_state = 1;
	
	 SC7LC30_DBG("robert read n_f_state = 0x%x\n",n_f_state);
	 return n_f_state;
}
static int sc7lc30_ps_get_data(int *ps_value, int *status)
{
    int err=0;
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
        *ps_value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

   
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(!sc7lc30_pldata)
	{
		APS_ERR("sc7lc30_pls is null!!\n");
		return -1;
	}
    
    if((sc7lc30_read_ps_data(sc7lc30_pldata)) < 0)
    {
        err = -1;
    }
    else
    {
        *ps_value = sc7lc30_read_ps_state(sc7lc30_pldata);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
		SC7LC30_DBG("%s: report ps_value = %d\n",__func__,*ps_value);
		err=0;
		
    }
#endif
 return err;
}


static int sc7lc30_local_init(void)
{

    SC7LC30_FUN();

    if (i2c_add_driver(&sc7lc30_driver))
    {
        APS_ERR("add driver error\n");
        SC7LC30_DBG("sc7lc30: sc7lc30_local_init: add driver error\n");
        return -1;
    }
    
    SC7LC30_DBG("sc7lc30 : sc7lc30_local_init\n");

    if (-1 == sc7lc30_init_flag)
    {
        APS_ERR("sc7lc30_local_init fail with sc7lc30_init_flag=%d\n", sc7lc30_init_flag);
        return -1;
    }

    SC7LC30_DBG("sc7lc30 : sc7lc30_local_init Out\n");
    
    return 0;
	
}


static int sc7lc30_local_remove(void)
{

    SC7LC30_FUN();

    i2c_del_driver(&sc7lc30_driver);

    return 0;
}
void sc7lc30_eint_func(void)
{
	SC7LC30_DATA *pls = sc7lc30_pldata;
	APS_LOG(" interrupt fuc\n");
	if(!pls)
	{
			return;
	}
	if(pls->hw.polling_mode_ps == 0 || pls->hw.polling_mode_als == 0)
		schedule_delayed_work(&pls->eint_work,0);
	
}

static irqreturn_t sc7lc30_eint_handler(int irq, void *desc)
{
	SC7LC30_DBG("----------------sc7lc30_eint_handler-----------------------\n");
	
	sc7lc30_eint_func();
	disable_irq_nosync(sc7lc30_pldata->irq);
	return IRQ_HANDLED;
}

static int sc7lc30_setup_eint(struct i2c_client *client)
{
    unsigned int ints[2] = {0, 0};

    int ret = -1;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_cfg;

    //alspsPltFmDev = get_alsps_platformdev();
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        SC7LC30_ERR("Cannot find alsps pinctrl!\n");
        return ret;
    }
    pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
    if (IS_ERR(pins_default)) {
        ret = PTR_ERR(pins_default);
        SC7LC30_ERR("Cannot find alsps pinctrl default!\n");
    }

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        SC7LC30_ERR("Cannot find alsps pinctrl pin_cfg!\n");
        return ret;
    }
    pinctrl_select_state(pinctrl, pins_cfg);
	if (sc7lc30_pldata->irq_node)
	{
#ifndef CONFIG_MTK_EIC
		/*upstream code*/
		ints[0] = of_get_named_gpio(sc7lc30_pldata->irq_node, "deb-gpios", 0);
		if (ints[0] < 0) 
		{
			APS_ERR("debounce gpio not found\n");
		} 
		else
		{
			ret = of_property_read_u32(sc7lc30_pldata->irq_node, "debounce", &ints[1]);
			if (ret < 0)
				APS_ERR("debounce time not found\n");
			else
				gpio_set_debounce(ints[0], ints[1]);
			APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		}
#else
		ret = of_property_read_u32_array(sc7lc30_pldata->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		if (ret) 
		{
			APS_ERR("of_property_read_u32_array fail, ret = %d\n", ret);
			return ret;
		}
		gpio_set_debounce(ints[0], ints[1]);
#endif
		sc7lc30_pldata->irq = irq_of_parse_and_map(sc7lc30_pldata->irq_node, 0);
		APS_LOG("sc7lc30_pldata->irq = %d\n", sc7lc30_pldata->irq);
		if (!sc7lc30_pldata->irq) 
		{
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if(request_irq(sc7lc30_pldata->irq, sc7lc30_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL)) 
	 	{
            SC7LC30_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
		enable_irq_wake(sc7lc30_pldata->irq);
		enable_irq(sc7lc30_pldata->irq);
	} 
	else
	{
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
    return 0;
}

static int sc7lc30_set_reg_mode(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_INTC_CONFIG ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_it_reg(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(0x01 ,reg)<0) 
		{
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;
		}	
	return ret;
}
static int sc7lc30_set_als_gain(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_RAN_COMMAND ,reg)<0) 
		{
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;
		}	
	return ret;
}
static int sc7lc30_set_ps_gain(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_PX_CONFIGURE ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_ps_ledctrl(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_PX_LED ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_pers(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_PIF_COMMAND ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_wait_time(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_WAIT_TIME ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_config_reg(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_CONFIG_REG ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_command_reg(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_COMMAND_REG ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}
static int sc7lc30_set_led_cur_reg(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(0x31 ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}
	return ret;
}
static int sc7lc30_set_pers_reg(u8 reg)
{
	int ret =0;
	if(sc7lc30_pls_write_data(SC7LC30_AIF_COMMAND ,reg)<0) {
		SC7LC30_DBG("%s: I2C Write Config Failed\n", __func__);
		ret = -1;}	
	return ret;
}

static void sc7lc30_pls_reg_dump(void)
{
	unsigned char config,intstatus,alslsb,alsmsb,pslsb,psmsb,alsconfig,psconfig;
	sc7lc30_pls_read_data(SC7LC30_MODE_COMMAND,&config);
	sc7lc30_pls_read_data(SC7LC30_INT_COMMAND,&intstatus);
	sc7lc30_pls_read_data(SC7LC30_ADC_LSB,&alslsb);
	sc7lc30_pls_read_data(SC7LC30_ADC_MSB,&alsmsb);
	sc7lc30_pls_read_data(SC7LC30_PX_LSB,&pslsb);
	sc7lc30_pls_read_data(SC7LC30_PX_MSB,&psmsb);
	sc7lc30_pls_read_data(SC7LC30_RAN_COMMAND,&alsconfig);
	sc7lc30_pls_read_data(SC7LC30_PIF_COMMAND,&psconfig);

	SC7LC30_DBG("%s: config=0x%x,intstatus=0x%x,alslsb=0x%x,alsmsb=0x%x,pslsb=0x%x,psmsb=0x%x,alsconfig=0x%x, \
		psconfig=0x%x", __func__,  config,intstatus,alslsb,alsmsb,pslsb,psmsb,alsconfig,psconfig);
}

/*----------------------------------------------------------------------------*/
static int sc7lc30_ps_init_data(SC7LC30_DATA  * pls)
{
    int ret;

    SC7LC30_FUN();

    if (NULL == pls || NULL == pls->client)
    {
        SC7LC30_ERR(" Parameter error \n");
        return EINVAL;
    }
	pls->it_val = 0x58;// reg 01 0x93
	pls->ps_gain = 0x33;//reg 03
	pls->als_gain = 0x3b;// reg 02
	pls->wait_time = 0x28;// reg 06
	pls->led_ctrl = 0xBF;// reg 05 0xFF
	pls->config_val = 0x00;// reg 31
	pls->pers_val = 0x00; // reg 04
	pls->command_val = 0x43; //reg 32
	pls->first_enable = true;
	pls->h_thd_offset = THD_H;
	pls->l_thd_offset = THD_L;
	pls->ps_pers = 0x10; // reg 04
#ifdef AUTO_CALI
	pls->ps_data_max = 0x00;	
	pls->ps_data_min = 0xFFFF;
	pls->diff = MAX_DIFF;
#endif	
	atomic_set(&pls->als_tran,10000);
	ret = sc7lc30_set_reg_mode(0x57); // reg 00 0x17
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_reg_mode fail\n");
		return ret;
	}
	
	ret = sc7lc30_set_it_reg(pls->it_val);// reg 01
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_it_reg fail\n");
		return ret;
	}
	ret = sc7lc30_set_als_gain(pls->als_gain);// reg 02
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_als_gain fail\n");
		return ret;
	}
	ret = sc7lc30_set_ps_gain(pls->ps_gain);// reg 03
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_ps_gain fail\n");
		return ret;
	}
	ret = sc7lc30_set_ps_ledctrl(pls->led_ctrl);// reg 05
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_ps_ledctrl fail\n");
		return ret;
	}
	ret = sc7lc30_set_pers(pls->pers_val);// reg 04
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_pers fail\n");
		return ret;
	}
	ret = sc7lc30_set_wait_time(pls->wait_time);// reg 06
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_wait_time fail\n");
		return ret;
	}
	ret = sc7lc30_set_config_reg(pls->config_val);// reg 31
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_config_reg fail\n");
		return ret;
	}
	ret = sc7lc30_set_command_reg(pls->command_val);// reg 32
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_command_reg fail\n");
		return ret;
	}
	ret = sc7lc30_set_led_cur_reg(0x00); // reg 31 0x02
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_led_cur_reg fail\n");
		return ret;
	}
	ret = sc7lc30_set_pers_reg(pls->ps_pers);// reg 04
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_pers_reg fail\n");
		return ret;
	}
	ret = sc7lc30_set_h_thd(atomic_read(&pls->ps_hth));// reg 0e 0d
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_h_thd fail\n");
		return ret;
	}
	ret = sc7lc30_set_l_thd(atomic_read(&pls->ps_lth)); // reg 0b 0c
	if(ret < 0)
	{
		SC7LC30_DBG("sc7lc30_set_l_thd fail\n");
		return ret;
	}
	sc7lc30_pls_reg_dump();
   
	return ret;

}
static int sc7lc30_init_client(struct i2c_client *client)
{
	int ret;
	if(sc7lc30_pldata->hw.polling_mode_ps == 0 )
	{
	  SC7LC30_FUN();
	  if((ret = sc7lc30_setup_eint(client)))
	  {
	      SC7LC30_ERR("setup eint error: %d\n", ret);
	      return ret;
	  }
	}
	
	return 0;
}
static int sc7lc30_als_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;
	APS_FUN();
	
	err = sc7lc30_als_enable_nodata(enable_disable ? 1 : 0);
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

static int sc7lc30_als_factory_get_data(int32_t *data)
{
	int status;
	
	APS_FUN();
	return sc7lc30_als_get_data(data, &status);
}

static int sc7lc30_als_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	SC7LC30_DATA *obj = sc7lc30_pldata;

	APS_FUN();

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return -1;
	}

	err = sc7lc30_read_als_data(obj);
	if (err) {
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
	*data = err;

	return 0;
}
static int sc7lc30_als_factory_enable_calibration(void)
{
	APS_FUN();
	return 0;
}
static int sc7lc30_als_factory_clear_cali(void)
{
	APS_FUN();
	return 0;
}
static int sc7lc30_als_factory_set_cali(int32_t offset)
{
	APS_FUN();
	return 0;
}
static int sc7lc30_als_factory_get_cali(int32_t *offset)
{
	APS_FUN();
	return 0;
}
static int sc7lc30_ps_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;
	
	APS_FUN();
	err = sc7lc30_ps_enable_nodata(enable_disable ? 1 : 0);
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

static int sc7lc30_ps_factory_get_data(int32_t *data)
{
	int status = 0;
	SC7LC30_DBG("sc7lc30_ps_factory_get_data start\n");
	return sc7lc30_ps_get_data(data, &status);
}

static int sc7lc30_ps_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	SC7LC30_DATA *obj = sc7lc30_pldata;

	APS_FUN();
	err = sc7lc30_read_ps_data(obj);
	if (err<0) {
		APS_ERR("%s failed \n", __func__);
		return -1;
	}
	*data = err;
	return 0;
}

static int sc7lc30_ps_factory_enable_calibration(void)
{
	APS_FUN();
	return 0;
}

static int sc7lc30_ps_factory_clear_cali(void)
{
	APS_FUN();
	return 0;
}

static int sc7lc30_ps_factory_set_cali(int32_t offset)
{
	APS_FUN();
	return 0;
}

static int sc7lc30_ps_factory_get_cali(int32_t *offset)
{
	APS_FUN();
	return 0;
}

static int sc7lc30_ps_factory_set_threashold(int32_t threshold[2])
{
#if 0
	APS_FUN();
	APS_ERR("%s set threshold high: 0x%x, low: 0x%x, ps_cali=%d\n", __func__, threshold[0], threshold[1], obj->ps_cali);
	atomic_set(&obj->ps_high_thd_val, (threshold[0] + obj->ps_cali));
	atomic_set(&obj->ps_low_thd_val, (threshold[1] + obj->ps_cali));
	err = sc7lc30_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val));
	err = sc7lc30_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val));
	//err = set_psensor_threshold(obj->client);

	if (err < 0) {
		APS_ERR("set_psensor_threshold fail\n");
		return -1;
	}
#endif
	return 0;
}

static int sc7lc30_ps_factory_get_threashold(int32_t threshold[2])
{
#if 0
	struct sc7lc30_priv *obj = sc7lc30_obj;

	APS_FUN();
	threshold[0] = atomic_read(&obj->ps_high_thd_val) - obj->ps_cali;
	threshold[1] = atomic_read(&obj->ps_low_thd_val) - obj->ps_cali;
#endif
	return 0;
}

static struct alsps_factory_fops sc7lc30_factory_fops = {
	.als_enable_sensor = sc7lc30_als_factory_enable_sensor,
	.als_get_data = sc7lc30_als_factory_get_data,
	.als_get_raw_data = sc7lc30_als_factory_get_raw_data,
	.als_enable_calibration = sc7lc30_als_factory_enable_calibration,
	.als_clear_cali = sc7lc30_als_factory_clear_cali,
	.als_set_cali = sc7lc30_als_factory_set_cali,
	.als_get_cali = sc7lc30_als_factory_get_cali,
	.ps_enable_sensor = sc7lc30_ps_factory_enable_sensor,
	.ps_get_data = sc7lc30_ps_factory_get_data,
	.ps_get_raw_data = sc7lc30_ps_factory_get_raw_data,
	.ps_enable_calibration = sc7lc30_ps_factory_enable_calibration,
	.ps_clear_cali = sc7lc30_ps_factory_clear_cali,
	.ps_set_cali = sc7lc30_ps_factory_set_cali,
	.ps_get_cali =sc7lc30_ps_factory_get_cali,
	.ps_set_threashold = sc7lc30_ps_factory_set_threashold,
	.ps_get_threashold = sc7lc30_ps_factory_get_threashold,
};

static struct alsps_factory_public sc7lc30_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &sc7lc30_factory_fops,
};


/******************************************************************************
 * NAME       : sc7lc30_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int sc7lc30_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SC7LC30_DATA            *sc7lc30_data;
	///struct device_node *node = NULL;
	int                     result;
	int err;
	struct als_control_path als_ctl = {0};  //als control function pointer
	struct als_data_path    als_data = {0}; //als report funcation pointer
	struct ps_control_path ps_ctl = {0};    //ps control function pointer
	struct ps_data_path    ps_data = {0};   //ps report funcation pointer
	SC7LC30_WARNING("called sc7lc30_probe for SC7LC30 \n");

	/* check parameter */
	if (NULL == client)
	{
	    SC7LC30_ERR(" Parameter error !!! \n");
	    return EINVAL;
	}

	sc7lc30_data = kzalloc(sizeof(*sc7lc30_data), GFP_KERNEL);
	if (sc7lc30_data == NULL) {
	    result = -ENOMEM;
	    return result;
	}
	//initilize to zero
    memset(sc7lc30_data, 0 , sizeof(*sc7lc30_data));     
	 
	SC7LC30_DBG("<<-SC7L30 INFO->> client->dev.of_node->name = %s\n",client->dev.of_node->name);
	err = get_alsps_dts_func(client->dev.of_node, &sc7lc30_data->hw);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		goto exit_init_failed;
	}

	sc7lc30_data->hw.i2c_num = 0x1;
	sc7lc30_data->hw.i2c_addr[0] = 0x39; 
	client->addr = 0x39;
	sc7lc30_data->client = client;

	i2c_set_clientdata(client, sc7lc30_data);
	sc7lc30_pldata = sc7lc30_data;
	sc7lc30_i2c_client  = client;
	atomic_set(&sc7lc30_data->ps_hth, sc7lc30_data->hw.ps_threshold_high);
	atomic_set(&sc7lc30_data->ps_hth, sc7lc30_data->hw.ps_threshold_low);
	result =  sc7lc30_ps_init_data(sc7lc30_data); /* Initialize data*/
    if (result)
    {
        SC7LC30_ERR("sc7lc30_ps_init_data failed, error = %d\n", result);
        goto err_power_failed;
	}
  	
	sc7lc30_pldata->irq_node= client->dev.of_node;
	sc7lc30_data->als_level_num = sizeof(sc7lc30_data->hw.als_level)/sizeof(sc7lc30_data->hw.als_level[0]);
	sc7lc30_data->als_value_num = sizeof(sc7lc30_data->hw.als_value)/sizeof(sc7lc30_data->hw.als_value[0]); 
	
	INIT_DELAYED_WORK(&sc7lc30_data->eint_work, sc7lc30_eint_work); /* interrupt work quene*/
if(sc7lc30_data->first_enable)
		sc7lc30_boot_cali(sc7lc30_data);
#ifdef AUTO_CALI
	INIT_DELAYED_WORK(&sc7lc30_data->cali_work, sc7lc30_cali_work);
#endif
	 if(sc7lc30_pldata->hw.polling_mode_ps == 0 ){
		if((result = sc7lc30_init_client(client)))  /* interrupt register*/
    {
        goto err_power_failed;
    }
	}
	if((err = alsps_factory_device_register(&sc7lc30_factory_device)))
	{
		APS_ERR("sc7lc30_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;
	if((result = sc7lc30_create_attr(&(sc7lc30_init_info.platform_diver_addr->driver))))
	{
		SC7LC30_WARNING("create attribute err = %d\n", result);
		goto exit_create_attr_failed;
	}
	
	/* als initialize */
	als_ctl.open_report_data = sc7lc30_als_open_report_data;
	als_ctl.enable_nodata    = sc7lc30_als_enable_nodata;
	als_ctl.set_delay        = sc7lc30_als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	if(1 == sc7lc30_data->hw.polling_mode_als)
	{
		als_ctl.is_polling_mode = true;
	}
	else
	{
		als_ctl.is_polling_mode = false;
	}
    als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = sc7lc30_data->hw.is_batch_supported_als;
#else
	als_ctl.is_support_batch = false;
#endif
	result = als_register_control_path(&als_ctl);
	if (result)
	{
		SC7LC30_ERR("als_register_control_path failed, error = %d\n", result);
		goto err_power_failed;
	}
	
	als_data.get_data   = sc7lc30_als_get_data;
	als_data.vender_div = 100;
	
	result = als_register_data_path(&als_data);
	if (result)
	{
		SC7LC30_ERR("als_register_data_path failed, error = %d\n", result);
		goto err_power_failed;
	}
	
	/* ps initialize */
	ps_ctl.open_report_data = sc7lc30_ps_open_report_data;
	ps_ctl.enable_nodata    = sc7lc30_ps_enable_nodata;
	ps_ctl.set_delay        = sc7lc30_ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	
	ps_ctl.is_support_batch = false;
	if(1 == sc7lc30_data->hw.polling_mode_ps)
	{
		ps_ctl.is_polling_mode = true;
		ps_ctl.is_report_input_direct = false;
	}
	else
	{
		ps_ctl.is_polling_mode = false;
		ps_ctl.is_report_input_direct = true;
	}
	result = ps_register_control_path(&ps_ctl);
	if (result)
	{
	  SC7LC30_ERR("ps_register_control_path failed, error = %d\n", result);
	  goto err_power_failed;
	}
	
	ps_data.get_data   = sc7lc30_ps_get_data;
	ps_data.vender_div = 100;
	
	result = ps_register_data_path(&ps_data);
	if (result)
	{
	  SC7LC30_ERR("ps_register_data_path failed, error = %d\n", result);
	  goto err_power_failed;
    }
//add by xiaobin.zhai for esd check
#if 0	
	result = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(result)
	{
		SC7LC30_ERR("register light batch support err = %d\n", result);
	}
	result = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(result)
	{
		SC7LC30_ERR("register proximity batch support err = %d\n", result);
	}
#endif

	sc7lc30_init_flag = 0;

	SC7LC30_WARNING(" sc7lc30_probe for SC7LC30 OK \n");
	
	return (result);
exit_init_failed:

exit_create_attr_failed:
   // misc_deregister(&sc7lc30_device);
exit_misc_device_register_failed:
err_power_failed:
	kfree(sc7lc30_data);

    sc7lc30_init_flag = -1;
	
    return (result);

}

/******************************************************************************
 * NAME       : sc7lc30_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int sc7lc30_remove(struct i2c_client *client)
{
    int err;
    SC7LC30_DATA *sc7lc30_data;

    if (NULL == client)
    {
        SC7LC30_ERR(" Parameter error \n");
        return EINVAL;
    }

    if((err = sc7lc30_delete_attr(&(sc7lc30_init_info.platform_diver_addr->driver))))
    {
        SC7LC30_ERR("sc7lc30_delete_attr fail: %d\n", err);
    }

    sc7lc30_data    = i2c_get_clientdata(client);

    kfree(sc7lc30_data);

    return (0);
}

/******************************************************************************
 * NAME       : sc7lc30_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __init sc7lc30_init(void)
{
    SC7LC30_WARNING("sc7lc30_init\n");

    alsps_driver_add(&sc7lc30_init_info);
    return 0;

}

/******************************************************************************
 * NAME       : sc7lc30_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit sc7lc30_exit(void)
{
    //nothing to do
    return;
}


MODULE_DESCRIPTION("ROHM Ambient Light And Proximity Sensor Driver");
MODULE_LICENSE("GPL");

module_init(sc7lc30_init);
module_exit(sc7lc30_exit);
