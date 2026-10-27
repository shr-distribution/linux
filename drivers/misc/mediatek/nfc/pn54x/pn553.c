/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h> 
#include <linux/regulator/consumer.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#ifdef CONFIG_TINNO_PRODUCT_INFO
#include <dev_info.h>
#endif

#define NFC_DESC            "NXP NQ310"
#define NFC_DRIVER_NAME     "pn553"
#define NFC_FW_VERSION		"11.01.15"


#ifndef I2C_MASK_FLAG
#define I2C_MASK_FLAG   (0x00ff)
#define I2C_DMA_FLAG    (0x2000)
#define I2C_ENEXT_FLAG  (0x0200)
#endif

#define I2C_NFC_SLAVE_7_BIT_ADDR	0X28
#define PN544_DRVNAME		"pn553"


#define MAX_BUFFER_SIZE		512
#define I2C_ID_NAME		"pn553"
#define WAKEUP_SRC_TIMEOUT 2000

#define PN544_MAGIC		0xE9
//#define PN544_SET_PWR		_IOW(PN544_MAGIC, 0x01, unsigned int)
#define PN544_SET_PWR _IOW(PN544_MAGIC, 0x01, long)
/******************************************************************************
 * extern functions
 *******************************************************************************/
#define PN544_DEBUG_EN 0
#if PN544_DEBUG_EN
#define PN544_DEBUG(fmt, args...) do { \
    printk(KERN_DEBUG "[PN544]%s:"fmt"\n", __func__, ##args); \
} while (0)
#else /* #if FTS_DEBUG_EN*/
#define PN544_DEBUG(fmt, args...)
#endif
struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;

	int			irq_gpio;
	struct regulator	*reg;
};

struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
struct pinctrl_state *st_dwn_h = NULL;
struct pinctrl_state *st_dwn_l = NULL;
struct pinctrl_state *st_eint_int = NULL;

/* For DMA */
#ifdef CONFIG_MTK_I2C_EXTENSION
static char *I2CDMAWriteBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAWriteBuf_pa;	/* = NULL; */
static char *I2CDMAReadBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAReadBuf_pa;	/* = NULL; */
#else
static char I2CDMAWriteBuf[MAX_BUFFER_SIZE];
static char I2CDMAReadBuf[MAX_BUFFER_SIZE];
#endif

int nxp_flag = 0;

/*****************************************************************************
 * Function
 *****************************************************************************/

static void pn544_enable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;
	PN544_DEBUG("%s\n", __func__);

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	enable_irq(pn544_dev->client->irq);
	pn544_dev->irq_enabled = true;
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	PN544_DEBUG("%s\n", __func__);


	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		//mt_eint_mask(EINT_NUM);
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev)
{
	struct pn544_dev *pn544_dev = dev;

	PN544_DEBUG("pn544_dev_irq_handler()\n");		

	if (device_may_wakeup(&pn544_dev->client->dev)) {
		pm_wakeup_event(&pn544_dev->client->dev, WAKEUP_SRC_TIMEOUT);
	}
	pn544_disable_irq(pn544_dev);
	nxp_flag = 1;

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static int pn544_platform_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;

	PN544_DEBUG("%s\n", __func__);

	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		PN544_DEBUG("%s: pinctrl_select err\n", __func__);
		ret = -1;
	}

	return ret;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	int ret,i;

	PN544_DEBUG("%s start \n", __func__);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	PN544_DEBUG("pn544 %s : reading %zu bytes.\n", __func__, count);
	PN544_DEBUG(KERN_ERR "michael: gpio_get_value test !!!\n");
	//PN544_DEBUG("gpio_get_value(pn544_dev->irq_gpio) 0 =%d\n", gpio_get_value(pn544_dev->irq_gpio));
	
	mutex_lock(&pn544_dev->read_mutex);
	PN544_DEBUG("gpio_get_value(pn544_dev->irq_gpio) 1 =%d\n", gpio_get_value(pn544_dev->irq_gpio));

	if (!nxp_flag) {
		PN544_DEBUG("pn544 read no event\n");		
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}
		
		PN544_DEBUG("pn544 read wait event\n");		
		pn544_enable_irq(pn544_dev);
		PN544_DEBUG("gpio_get_value(pn544_dev->irq_gpio) 2 =%d\n", gpio_get_value(pn544_dev->irq_gpio));
		ret = wait_event_interruptible(pn544_dev->read_wq, nxp_flag);
		//nxp_flag = 0;

		//pn544_disable_irq(pn544_dev);

		if (ret) 
		{
			PN544_DEBUG("pn544 read wait event error\n");
			goto fail;
		}
		pn544_disable_irq(pn544_dev);
	}

	/* Read data */	
	ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf, count);
	   
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) 
	{
		pr_err("pn544 %s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		pr_err("pn544 %s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	
	if (copy_to_user(buf, I2CDMAReadBuf, ret)) 
	{
		PN544_DEBUG(KERN_DEBUG "%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	PN544_DEBUG("pn544 IFD->PC:");
	for(i = 0; i < ret; i++) 
	{
		PN544_DEBUG(" %02X", I2CDMAReadBuf[i]);
	}
	PN544_DEBUG("\n");
nxp_flag = 0;
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev;
	int ret, i,idx = 0;

	PN544_DEBUG("%s\n", __func__);

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*255)], count)) 
	{
		PN544_DEBUG(KERN_DEBUG "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}
	
	PN544_DEBUG("pn544 %s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf, count);
	
	if (ret != count) 
	{
		pr_err("pn544 %s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	PN544_DEBUG("pn544 PC->IFD:");
	for(i = 0; i < count; i++) 
	{
		PN544_DEBUG(" %02X\n", I2CDMAWriteBuf[i]);
	}
	PN544_DEBUG("\n");

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);
	
	PN544_DEBUG("%s:pn544_dev=%p\n", __func__, pn544_dev);

	filp->private_data = pn544_dev;
	
	PN544_DEBUG("pn544 %s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return ret;
}

//delete by tzf@meitu begin


static long pn544_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				      unsigned long arg)
{
	int ret;
#if PN544_DEBUG_EN
	struct pn544_dev *pn544_dev = filp->private_data;

	PN544_DEBUG("%s:cmd=%d, arg=%ld, pn544_dev=%p\n", __func__, cmd, arg, pn544_dev);
#endif
	switch (cmd) 
	{
		case PN544_SET_PWR:
			if (arg == 2) {
				/* power on with firmware download (requires hw reset) */
				PN544_DEBUG("pn544 %s power on with firmware\n", __func__);

				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_h);
				msleep(10);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				msleep(10);
			} else if (arg == 1) {
				/* power on */
				PN544_DEBUG("pn544 %s power on\n", __func__);

				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
                //jcy add
                //ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_h);
				msleep(10);
			} else  if (arg == 0) {
				/* power off */
				PN544_DEBUG("pn544 %s power off\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(50);
			} else {
				PN544_DEBUG("pn544 %s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}
			break;
		default:
			PN544_DEBUG("pn544 %s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}

	return ret;
}


static const struct file_operations pn544_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn544_dev_read,
	.write = pn544_dev_write,
	.open = pn544_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pn544_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = pn544_dev_unlocked_ioctl,
};

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (I2CDMAWriteBuf) {
		#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
		#endif
		I2CDMAWriteBuf = NULL;
		I2CDMAWriteBuf_pa = 0;
	}

	if (I2CDMAReadBuf) {
		#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#endif
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}
#endif

	pn544_dev = i2c_get_clientdata(client);
	gpio_free(pn544_dev->irq_gpio);
	misc_deregister(&pn544_dev->pn544_device);

	mutex_destroy(&pn544_dev->read_mutex);


	regulator_put(pn544_dev->reg);

	kfree(pn544_dev);
	
	return 0;
}

#ifdef CONFIG_TINNO_PRODUCT_INFO
int get_nfc_info(char *buf, void *arg0)
{
    return sprintf(buf,
          "%s-%s-%s-%s",
          NFC_DESC,
          CONFIG_ARCH_MTK_PROJECT,
          NFC_DRIVER_NAME,
          NFC_FW_VERSION);
}
#endif
#ifdef CONFIG_AGENEW_CHARGER_H231_NFC_CONTROL
#define NFC_ID_GPIO  	 66+329
#endif
static int pn544_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	int ret=0;
	struct pn544_dev *pn544_dev;
	struct device_node *node;
#ifdef CONFIG_AGENEW_CHARGER_H231_NFC_CONTROL		
if(gpio_get_value(NFC_ID_GPIO) == 1)
{
	PN544_DEBUG("%s: NO NFC so return ...\n", __func__);
	return 0;
}
#endif
	PN544_DEBUG("%s: start...\n", __func__);
	PN544_DEBUG("michael: --- pn544_probe ---\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	PN544_DEBUG("%s: step02 is ok\n", __func__);

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	PN544_DEBUG("pn544_dev=%p\n", pn544_dev);

	if (pn544_dev == NULL) 
	{
		dev_err(&client->dev, "pn544 failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	memset(pn544_dev, 0, sizeof(struct pn544_dev));

	PN544_DEBUG("%s: step03 is ok\n", __func__);

#ifdef CONFIG_MTK_I2C_EXTENSION
	client->addr = (client->addr & I2C_MASK_FLAG);
	client->addr = (client->addr | I2C_DMA_FLAG);
	client->addr = (client->addr | I2C_DIRECTION_FLAG);
	client->timing = 400;
#else
#endif
	pn544_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_DRVNAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;
	pn544_dev->pn544_device.parent = &client->dev;

	i2c_set_clientdata(client, pn544_dev);
	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) 
	{
		pr_err("%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}
    
	PN544_DEBUG("%s: step04 is ok\n", __func__);
	
	/* request irq.  the irq is set whenever the chip has data available
	* for reading.  it is cleared when all data has been read.
	*/    
#ifdef CONFIG_MTK_I2C_EXTENSION
#ifdef CONFIG_64BIT
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&pn544_dev->read_mutex);
		gpio_free(platform_data->sysrstb_gpio);
		return ret;
	}
#ifdef CONFIG_64BIT
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&pn544_dev->read_mutex);
		gpio_free(platform_data->sysrstb_gpio);
		return ret;
	}
	pr_debug("%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__,
		 I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);
#else
	memset(I2CDMAWriteBuf, 0x00, sizeof(I2CDMAWriteBuf));
	memset(I2CDMAReadBuf, 0x00, sizeof(I2CDMAReadBuf));
#endif

	PN544_DEBUG("%s: step05 is ok\n", __func__);

	/*  NFC IRQ settings     */	
	node = of_find_compatible_node(NULL, NULL, "mediatek,irq_nfc");
	if (node) {
  	//of_property_read_u32_array(node, "gpio-irq",&(pn544_dev->irq_gpio), 1);               
  	pn544_dev->irq_gpio = of_get_named_gpio(node, "gpio-irq", 0);
  	PN544_DEBUG("pn544_dev->irq_gpio = %d\n", pn544_dev->irq_gpio);
  	ret = gpio_request(pn544_dev->irq_gpio, "gpio-irq");
		if (ret < 0) {
	    pr_err("gpio-irq request failed!");
	    return ret;
		}
		ret = gpio_direction_input(pn544_dev->irq_gpio);
		if (ret < 0) {
	    pr_err("gpio_direction_input failed!");
	    return ret;
		} 
		
		client->irq = irq_of_parse_and_map(node, 0);
		PN544_DEBUG("client->irq = %d\n", client->irq);
	
		ret =
		    request_irq(client->irq, pn544_dev_irq_handler,
				IRQF_TRIGGER_HIGH, "nfc", pn544_dev);

		if (ret) {
			pr_err("%s: EINT IRQ LINE NOT AVAILABLE, ret = %d\n", __func__, ret);
		} else {
			PN544_DEBUG("%s: set EINT finished, client->irq=%d", __func__,
				 client->irq);

			pn544_dev->irq_enabled = true;
			pn544_disable_irq(pn544_dev);
		}
		
		PN544_DEBUG("%s success\n", __func__);

	} else {
		pr_err("%s: can not find NFC eint compatible node\n",
		       __func__);
	}
	device_init_wakeup(&client->dev, true);
	device_set_wakeup_capable(&client->dev, true); 

	i2c_set_clientdata(client, pn544_dev);

#ifdef CONFIG_TINNO_PRODUCT_INFO
    FULL_PRODUCT_DEVICE_CB(ID_NFC, get_nfc_info, NULL);
#endif

	return 0;

#if 0
err_dma_alloc:
	misc_deregister(&pn544_dev->pn544_device);
#endif

err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
#if 0
err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);   
#endif
	return ret;
}

static int nqx_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct pn544_dev *pn544_dev = i2c_get_clientdata(client);
	if (device_may_wakeup(&client->dev) && pn544_dev->irq_enabled)
		enable_irq_wake(client->irq);
	return 0;
}
static int nqx_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
	return 0;
}
static const struct i2c_device_id pn544_id[] = {
	{I2C_ID_NAME, 0},
	{}
};
static const struct dev_pm_ops nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nqx_suspend, nqx_resume)
};

static const struct of_device_id nfc_i2c_of_match[] = {
	{.compatible = "mediatek,nfc"},
	{},
};

static struct i2c_driver pn544_i2c_driver = 
{
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	/* .detect	= pn544_detect, */
	.driver		= {
		.name = "pn544",
		.owner = THIS_MODULE,
		.pm = &nfc_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = nfc_i2c_of_match,
#endif
	},
};


static int pn544_platform_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	PN544_DEBUG("%s\n", __func__);

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}

	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		PN544_DEBUG("%s: pinctrl err, ven_high\n", __func__);
	}

	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		PN544_DEBUG("%s: pinctrl err, ven_low\n", __func__);
	}
	
	pn544_platform_pinctrl_select(gpctrl, st_ven_l);
	msleep(20);
	pn544_platform_pinctrl_select(gpctrl, st_ven_h);
	msleep(20); 

	st_dwn_h = pinctrl_lookup_state(gpctrl, "dwn_high");
	if (IS_ERR(st_dwn_h)) {
		ret = PTR_ERR(st_dwn_h);
		PN544_DEBUG("%s: pinctrl err, dwn_high\n", __func__);
	}


	st_dwn_l = pinctrl_lookup_state(gpctrl, "dwn_low");
	if (IS_ERR(st_dwn_l)) {
		ret = PTR_ERR(st_dwn_l);
		PN544_DEBUG("%s: pinctrl err, dwn_low\n", __func__);
	}

	st_eint_int = pinctrl_lookup_state(gpctrl, "irq_init");
    if (IS_ERR(st_eint_int)) {
    	ret = PTR_ERR(st_eint_int);
    	PN544_DEBUG("%s: pinctrl err, st_eint_int\n", __func__);
    }
    
	pn544_platform_pinctrl_select(gpctrl, st_eint_int);

end:
	return ret;
}

static int pn544_platform_probe(struct platform_device *pdev)
{
	int ret = 0;

	PN544_DEBUG("%s: start &pdev=%p\n", __func__, pdev);
	PN544_DEBUG("michael: --- pn544_platform_probe ---\n");

	/* pinctrl init */
	ret = pn544_platform_pinctrl_init(pdev);

	return ret;
}

static int pn544_platform_remove(struct platform_device *pdev)
{
	PN544_DEBUG("%s: &pdev=%p\n", __func__, pdev);

	return 0;
}

/*  platform driver */
static const struct of_device_id pn544_platform_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};

static struct platform_driver pn544_platform_driver = {
	.probe		= pn544_platform_probe,
	.remove		= pn544_platform_remove,
	.driver		= {
		.name = I2C_ID_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = pn544_platform_of_match,
#endif
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn544_dev_init(void)
{
	PN544_DEBUG("pn544_dev_init\n");
	PN544_DEBUG("michael: 20171208\n");

	platform_driver_register(&pn544_platform_driver);

	i2c_add_driver(&pn544_i2c_driver);

	PN544_DEBUG(" pn544_dev_init success\n");

	return 0;
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	PN544_DEBUG("pn544_dev_exit\n");

	i2c_del_driver(&pn544_i2c_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");

