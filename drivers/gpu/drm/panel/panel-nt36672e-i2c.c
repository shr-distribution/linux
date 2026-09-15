// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include "panel-nt36672e-i2c.h"
#include "panel-nt36672e-cpld-fw.h"


#ifndef CONFIG_FPGA_EARLY_PORTING
#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL	/* for I2C channel 0 */
#define I2C_ID_NAME "nt36672e"
#define TPS_ADDR 0x3E


#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info nt36672e_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id i2c_lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif

/*static struct i2c_client *nt36672e_i2c_client;*/
struct i2c_client *nt36672e_i2c_client;
int nt36672e_read_bytes(unsigned char addr, unsigned char *returnData);
int nt36672e_write_bytes(unsigned char addr, unsigned char value);

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int nt36672e_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int nt36672e_remove(struct i2c_client *client);
int fpga_update(void);
int fpga_send_cmd_recive(unsigned char cmd, unsigned char ops,unsigned int *returnData);
int fpga_send_cmd_only(unsigned char cmd);
int fpga_send_cmd_ops_val(unsigned char cmd,unsigned char ops,unsigned char value);
static int update_firmware(void);
//static int firmware_load(unsigned char *firmeware, int *firmeware_length);
int fpga_send_val_only(unsigned char val);
int fpga_recive_only(void);
int fpga_send_cmd_recive_ops3(unsigned char cmd, unsigned char ops,unsigned int *returnData);
int fpga_send_cmd_ops3_val(unsigned char cmd,unsigned char ops,unsigned char value);

/*****************************************************************************
 * Data Structure
 *****************************************************************************/



//--------------------------------------------------------------------------------------//msx-pangos
PGCDevice pgc_dev;
PGCDevice *dev_cpld = &pgc_dev;
const unsigned int feature_control_val_a_i2c = (0x4b << 18) | DEFAULT_FEATURE_CTL | 1;
unsigned int feature_ctl = feature_control_val_a_i2c;

//--------------------------------------------------------------------------------------//msx-pangos-241016

int refresh_mode = 0;
unsigned char cpld_reg_fw_version = 0;
const unsigned char lastest_fw_version = 0x25;								//每次必须手动修改这个值，这个值由CPLD出固件人员告知
																			//如果需要OTA更新新版固件，这个值必须比旧版固件大
																			//且OTA完成后，CPLD读寄存器读出固件版本的值和这个值一致

void check_need_upgrade(void);
																		
void check_need_upgrade(void)
{

	nt36672e_read_bytes(0x80, &cpld_reg_fw_version);						//read fw version
	printk("fpga read reg fw version is %x\n",cpld_reg_fw_version);

	if (cpld_reg_fw_version < lastest_fw_version)
	{
		printk("cpld_reg_fw_version < lastest_fw_version, need to update fw !\n");
		update_firmware();
	}
	else if (cpld_reg_fw_version >= lastest_fw_version)
	{
		printk("cpld_reg_fw_version >= lastest_fw_version, dont need to update !\n");
	}

}

//--------------------------------------------------------------------------------------//


static const struct i2c_device_id nt36672e_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver nt36672e_iic_driver = {
	.id_table = nt36672e_id,
	.probe = nt36672e_probe,
	.remove = nt36672e_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "nt36672e",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = i2c_lcm_of_match,
#endif
		   },
};

struct nt36672e_i2c_dev {
	struct device *dev;
};

#define FPGA_FW_BIN_PATH "FPGAFW.bin"    //only save in /drivers/base/FPGAFW.bin


static ssize_t
refresh_mode_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	printk("refresh_mode_show enter, refresh_mode value is %d\n",refresh_mode);

	return snprintf(buf, PAGE_SIZE, "%d\n", refresh_mode);  		
}

static ssize_t
vcom_show_registers(struct device *dev, struct device_attribute *attr,
			   char *buf)
{

	unsigned char data = 0;

	nt36672e_read_bytes(0x8A, &data);			//read vcom value
	printk("vcom_show_registers enter, fpga read vcom value is %x\n",data);

	return snprintf(buf, PAGE_SIZE, "%x\n", data);  		

}

static ssize_t
nt36672e_show_registers(struct device *dev, struct device_attribute *attr,
			   char *buf)
{

	unsigned char data = 0;

	nt36672e_read_bytes(0x80, &data);			//read fw version
	printk("nt36672e_show_registers enter, fpga read fw version is %x\n",data);

	return snprintf(buf, PAGE_SIZE, "%x\n", data);  		//hall status，1：open 0：close

}

static ssize_t
nt36672e_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	int ret;
	unsigned int val;
	unsigned char data = 0;
	unsigned int id_code = 0;

	ret = sscanf(buf, "%d", &val);
	printk("nt36672e_store_registers val is %d\n",val);
	if(val ==1)
	{
		nt36672e_write_bytes(0x8C, 0x00);			//test for write---fpga
		refresh_mode = 0;
		printk("nt36672e_store_registers11111 enter\n");
	}
	else if(val == 2)
	{
		nt36672e_write_bytes(0x8C, 0x01);			//test for write---fpga
		refresh_mode = 1;
		printk("nt36672e_store_registers22222 enter\n");
	}
	else if(val ==3)
	{
		nt36672e_write_bytes(0x8C, 0x02);			//test for write---fpga
		refresh_mode = 2;
		printk("nt36672e_store_registers33333 enter\n");
	}
	else if(val ==4)
	{
		nt36672e_write_bytes(0x8C, 0x03);			//test for write---fpga
		refresh_mode = 3;
		printk("nt36672e_store_registers44444 enter\n");
	}

	else if(val ==5)
	{
		printk("nt36672e_store_registers55555 enter\n");
		nt36672e_write_bytes(0x8E, 0x05);			//test for write---fpga
	}

	else if(val == 6)								//read cpld id
 	{
		nt36672e_read_bytes(0x88, &data);			//test for read---fpga
		printk("fpga read cpld id is %x\n",data);
	}

	else if(val == 7)								//read fw version
 	{
		nt36672e_read_bytes(0x80, &data);			//test for read---fpga
		printk("fpga read fw version is %x\n",data);
	}

	else if(val == 8)								//read Idcode
 	{
		id_code = pango_PGC_I2C_ReadIdcode();		//0xA1
		printk("fpga read id_code is %x\n",id_code);

		if((id_code&0xFFFFFF) != 0x422899)			//0x0422899
		{
			printk("Invalid PGC id coda read out: 0x%x!\n", id_code);
		}
		else
    		printk("Read out id code ok: 0x%x!\r\n", id_code);
	}

	else if(val == 9)								//fw update
 	{
		printk("nt36672e_store_registers9999 enter moshaoxi1111\n");
		update_firmware();
		printk("nt36672e_store_registers9999 exit moshaoxi222\n");
	}

	//TEST for read---fpga

	else if(val == 10)								//read waveform version
 	{
		nt36672e_read_bytes(0x84, &data);			//test for read---fpga
		printk("fpga read waveform version is %d\n",data);
	}

	else if(val == 11)								//read display switch
 	{
		nt36672e_read_bytes(0x89, &data);			//test for read---fpga
		printk("fpga read display switch is %d\n",data);
	}

	else if(val == 12)								//read VCOM
 	{
		nt36672e_read_bytes(0x8A, &data);			//test for read---fpga
		printk("fpga read VCOM is %d\n",data);
	}

	else if(val == 13)								//read CLEAN
 	{
		nt36672e_read_bytes(0x8B, &data);			//test for read---fpga
		printk("fpga read CLEAN is %d\n",data);
	}

	else if(val == 14)								//read DISPLAY MODE
 	{
		nt36672e_read_bytes(0x8C, &data);			//test for read---fpga
		printk("fpga read DISPLAY MODE is %d\n",data);
	}

	else if(val == 15)								//read REFRESH
 	{
		nt36672e_read_bytes(0x8D, &data);			//test for read---fpga
		printk("fpga read REFRESH is %d\n",data);
	}

	printk("nt36672e_store_registers exit\n");
	return count;
}

#if 0
unsigned char cpld_firmware_data[0x50000] = { 0 };				//全局数组，保存固件, 固件上限大小:320K byte
unsigned int firmware_length;									//全局变量，用于保存固件大小

static int update_cpld(void)
{
	int ret;
	//unsigned char *firmware_buf;
	size_t offset;
	//size_t offset, count;
	unsigned int id_code;
	int error;

	printk("update_cpld enter\n");

	//firmware_buf = kzalloc(0x50000, GFP_KERNEL);  // 320K buffer
	//memset(firmware_buf, 0, 0x50000);
    //ret = firmware_load(firmware_buf, &firmware_length); // load firmware

	ret = firmware_load(cpld_firmware_data, &firmware_length); // load firmware

	if (ret < 0)
    {
		printk("fpga firmware_load failed\n");
        //goto update_fail;
    }

	//fpga_update

	update_firmware();

	return 0;
}

//static int firmware_load(struct firmware *firmeware, int *firmeware_length)
static int firmware_load(unsigned char *firmeware, unsigned int *firmeware_length)
{
    int ret = 0;
    const struct firmware *firm_data_bin;

	printk("fpga_upload_fw start load bin firmeware\n");
    ret = request_firmware(&firm_data_bin, FPGA_FW_BIN_PATH, &nt36672e_i2c_client->dev);

    if(ret < 0)
    {
		printk("fpga request_firmware failed\n");
        return -EINVAL;
    }

    *firmeware_length = (int)firm_data_bin->size;
    //memcpy(firmeware->data, firm_data_bin->data, firm_data_bin->size);
    memcpy(firmeware, firm_data_bin->data, firm_data_bin->size);	
    return 0;
}
#endif


static int update_firmware(void)
{
	printk("fpga update_firmware enter\n");
        
    pango_PGC_I2C_ProgramEFlash(dev_cpld,feature_ctl);

	printk("fpga update_firmware exit\n");

	return 0;
}


static DEVICE_ATTR(nt36672e_registers, S_IRUGO | S_IWUSR, nt36672e_show_registers,
		   nt36672e_store_registers);


static DEVICE_ATTR(vcom_registers, S_IRUGO | S_IWUSR, vcom_show_registers,
		   NULL);		

static DEVICE_ATTR(refresh_mode, S_IRUGO | S_IWUSR, refresh_mode_show,
		   NULL);				      

static struct attribute *nt36672e_attributes[] = {
	&dev_attr_nt36672e_registers.attr,
	&dev_attr_vcom_registers.attr,
	&dev_attr_refresh_mode.attr,
	NULL,
};

static const struct attribute_group nt36672e_attr_group = {
	.attrs = nt36672e_attributes,
};

static int nt36672e_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct nt36672e_i2c_dev *i2c_dev;

	pr_info("moshaoxi nt36672e_iic_probe\n");
	pr_info("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);

	i2c_dev = devm_kzalloc(&client->dev, sizeof(struct nt36672e_i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->dev = &client->dev;
	nt36672e_i2c_client = client;

	ret = sysfs_create_group(&i2c_dev->dev->kobj, &nt36672e_attr_group);
    if (ret < 0)
	{
        printk("nt36672e attr group create failed\n");
	}
    else
	{
        printk("nt36672e attr group create success!\n");
	}

	check_need_upgrade();							

	return 0;
}

static int nt36672e_remove(struct i2c_client *client)
{
	pr_info("nt36672e_remove\n");
	nt36672e_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

int fpga_send_val_only(unsigned char val)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = val;
	ret = i2c_master_send(client, write_data, 1);
	if (ret < 0)
		pr_info("fpga write data fail !!\n");
	return ret;
}

int fpga_recive_only(void)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char pBuff[1] = { 0 };

	printk("fpga_recive_only start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	ret = i2c_master_recv(client, pBuff, 1);
	if (ret < 0)
		printk("ERROR, fpga read fail !!\n");

	printk("pBuff[0] is %x !!\n",pBuff[0]);
	//memcpy(returnData, pBuff, 1);
	//*returnData = pBuff;
	//return ret;
	return pBuff[0];
}


int fpga_send_cmd_only(unsigned char cmd)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = cmd;
	ret = i2c_master_send(client, write_data, 1);
	if (ret < 0)
		pr_info("fpga write cmd fail !!\n");
	return ret;
}

int fpga_send_cmd_ops_val(unsigned char cmd,unsigned char ops,unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[3] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = cmd;
	write_data[1] = ops;
	write_data[2] = value;
	ret = i2c_master_send(client, write_data, 3);
	if (ret < 0)
		pr_info("fpga write data fail !!\n");
	return ret;
}

int fpga_send_cmd_recive(unsigned char cmd, unsigned char ops,unsigned int *returnData)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[2] = { 0 };
	char pBuff[4] = { 0 };
	//char pBuff[4] = { 0 };

	printk("fpga_send_cmd_recive start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = cmd;
	write_data[1] = ops;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("fpga write data fail !!\n");

	ret = i2c_master_recv(client, pBuff, 4);
	if (ret < 0)
		printk("ERROR, fpga read fail !!\n");

	printk("pBuff[0] is %x !!\n",pBuff[0]);
	printk("pBuff[1] is %x !!\n",pBuff[1]);
	printk("pBuff[2] is %x !!\n",pBuff[2]);
	printk("pBuff[3] is %x !!\n",pBuff[3]);
	memcpy(returnData, pBuff, 4);
	//*returnData = pBuff;
	return ret;
}


int fpga_send_cmd_recive_ops3(unsigned char cmd, unsigned char ops,unsigned int *returnData)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[4] = { 0 };
	char pBuff[4] = { 0 };
	//char pBuff[4] = { 0 };

	printk("fpga_send_cmd_recive start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = cmd;
	write_data[1] = ops;
	write_data[2] = ops;
	write_data[3] = ops;
	ret = i2c_master_send(client, write_data, 4);
	if (ret < 0)
		printk("fpga write data fail !!\n");

	ret = i2c_master_recv(client, pBuff, 4);
	if (ret < 0)
		printk("ERROR, fpga read fail !!\n");

	printk("pBuff[0] is %x !!\n",pBuff[0]);
	printk("pBuff[1] is %x !!\n",pBuff[1]);
	printk("pBuff[2] is %x !!\n",pBuff[2]);
	printk("pBuff[3] is %x !!\n",pBuff[3]);


	//memcpy(returnData, pBuff, 4);
	if(cmd != 0xA3)
	{
		memcpy(returnData, pBuff, 4);
		printk("cmd != 0xA3\n");
	}
	else if(cmd == 0xA3)				//StatusRegister
	{
		//pango_ReverseBytes(pBuff, 4);
		*returnData = pBuff[0] | (pBuff[1] << 8) | (pBuff[2] << 16) | (pBuff[3] << 24);
		printk("cmd == 0xA3\n");
	}

	//*returnData = pBuff;
	return ret;
}

int fpga_send_cmd_ops3_val(unsigned char cmd,unsigned char ops,unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[5] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = cmd;
	write_data[1] = ops;
	write_data[2] = ops;
	write_data[3] = ops;
	write_data[4] = value;
	ret = i2c_master_send(client, write_data, 5);
	if (ret < 0)
		pr_info("fpga write data fail !!\n");
	return ret;
}
		
int fpga_page_read_cmd(unsigned int page_addr,unsigned char *buf)			//MSX123456 PGC_I2C_READ
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1+4] = { 0 };
	//int i = 0; 

	printk("fpga_page_read_cmd start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = PGC_I2C_READ;
	write_data[1]= ((page_addr >> (16 - 0 * 8)) & 0xFF);
	write_data[2]= ((page_addr >> (16 - 1 * 8)) & 0xFF);
	write_data[3]= ((page_addr >> (16 - 2 * 8)) & 0xFF);
	write_data[4]= 0x00;

	ret = i2c_master_send(client, write_data, 1+4);
	if (ret < 0)
		printk("fpga write data fail !!\n");

	ret = i2c_master_recv(client, buf, 256);
	if (ret < 0)
		printk("ERROR, fpga read fail !!\n");

	//for(i = 0; i<256; ++i)
	//	printk("read buf[%d] val is %x !!\n",i,buf[i]);
	
	return ret;
}


int fpga_read_ctl_cmd(unsigned char *buf)			//MSX12345
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1+4] = { 0 };
	char pBuff[4] = { 0 };

	printk("fpga_program_cmd start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = PGC_I2C_READ_CTL;
	write_data[1]= 0x00;
	write_data[2]= 0x00;
	write_data[3]= 0x00;
	write_data[4]= 0x00;

	ret = i2c_master_send(client, write_data, 1+4);
	if (ret < 0)
		printk("fpga write data fail !!\n");

	ret = i2c_master_recv(client, pBuff, 4);
	if (ret < 0)
		printk("ERROR, fpga read fail !!\n");

	printk("pBuff[0] is %x !!\n",pBuff[0]);
	printk("pBuff[1] is %x !!\n",pBuff[1]);
	printk("pBuff[2] is %x !!\n",pBuff[2]);
	printk("pBuff[3] is %x !!\n",pBuff[3]);
	memcpy(buf, pBuff, 4);

	return ret;
}

int fpga_program_ctl_cmd(unsigned int feature_control_val)			//MSX1234
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1+3+4] = { 0 };

	printk("fpga_program_cmd start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = PGC_I2C_PROGRAM_CTL;
	write_data[1]= 0x00;
	write_data[2]= 0x00;
	write_data[3]= 0x00;

	write_data[4]= (pango_ReverseByte((feature_control_val >> (0 << 3)) & 0xFF));
	write_data[5]= (pango_ReverseByte((feature_control_val >> (1 << 3)) & 0xFF));
	write_data[6]= (pango_ReverseByte((feature_control_val >> (2 << 3)) & 0xFF));
	write_data[6]= (pango_ReverseByte((feature_control_val >> (3 << 3)) & 0xFF));

	ret = i2c_master_send(client, write_data, 1+3+4);
	if (ret < 0)
		printk("fpga write data fail !!\n");
	return ret;
}


int fpga_program_cmd(unsigned int page_addr,unsigned char *buf)			//MSX123
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1+3+256] = { 0 };
	int i;

	printk("fpga_program_cmd start!!\n");

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = 0x20;
	write_data[1]= ((page_addr >> (16 - 0 * 8)) & 0xFF);			
	write_data[2]= ((page_addr >> (16 - 1 * 8)) & 0xFF);
	write_data[3]= ((page_addr >> (16 - 2 * 8)) & 0xFF);
	memcpy(&write_data[4],buf, 256);
	ret = i2c_master_send(client, write_data, 256+1+3);
	if (ret < 0)
		printk("fpga write data fail !!\n");

	for(i = 1; i<4; ++i)
		printk("addr: write_data[%d] val is %x !!\n",i,write_data[i]);		

	return ret;
}


int fpga_program_cmd_last_time(unsigned int page_addr,unsigned char *buf,unsigned int size,unsigned int last_size)			//MSX123
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[1+3+256] = { 0 };
	char buf_temp[256] = { 0 };

	printk("fpga_program_cmd_last_time start!!,size is %d, and last_size is %d\n",size,last_size);

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = 0x20;
	write_data[1]= ((page_addr >> (16 - 0 * 8)) & 0xFF);
	write_data[2]= ((page_addr >> (16 - 1 * 8)) & 0xFF);
	write_data[3]= ((page_addr >> (16 - 2 * 8)) & 0xFF);
	memcpy(&write_data[4],buf, size);
	memcpy(&write_data[4+size],buf_temp, last_size);
	ret = i2c_master_send(client, write_data, 256+1+3);
	if (ret < 0)
		printk("fpga write data fail !!\n");
	return ret;
}


//--------------------------------------------------------------------------------------//msx-pangos
unsigned int pango_PGC_I2C_ReadIdcode(void)				
{
	unsigned int id_code = 0;
	fpga_send_cmd_recive_ops3(PGC_I2C_RDID, 0x00, &id_code);
	return id_code;
}


unsigned int pango_PGC_I2C_ReadStatusRegister(void)
{
	unsigned int sts_val;
	fpga_send_cmd_recive_ops3(PGC_I2C_RDSR, 0x00, &sts_val);
	return sts_val;
}

unsigned int pango_PGC_I2C_WakeupEmbedFlash(void)
{
	fpga_send_cmd_only(PGC_I2C_EFLASH_WAKEUP);						//Embedded FLASH Wake Up 71

	if (!pango_PGC_I2C_CheckConfigStatus(PGC_STSREG_BUSY, 1000, false))
    {
        printk("Fail to check busy flag!\n");
    }
	pango_PGC_I2C_WriteWRDIS();
    return pango_PGC_I2C_ReadStatusRegister();                       //回读状态寄存器
}

void pango_PGC_I2C_EraseSelfLoadBitstreamAndUserFlash(void)
{
	fpga_send_cmd_only(PGC_I2C_ERASE);                				 //flash擦除，0x10命令

	if (!pango_PGC_I2C_CheckConfigStatus(PGC_STSREG_BUSY, 1000, false))
    {
        printk("Fail to check busy flag!\n");
    }
	pango_PGC_I2C_WriteWRDIS();
}

static bool pango_PGC_I2C_WriteWRDIS(void)
{
	bool res = true;
	int ret = 0;
	ret = fpga_send_cmd_only(PGC_I2C_WRDIS);                         //WriteWRDIS指令，0x52
	if (ret < 0)
		res = false;
	return res;
}

static bool pango_PGC_I2C_WriteWREN(void)                    
{
	bool res = true;
	int ret = 0;
	ret = fpga_send_cmd_only(PGC_I2C_WREN);                         //WriteWRDIS指令，0x51
	if (ret < 0)
		res = false;
	return res;
}

bool pango_PGC_I2C_Reset(void)                        				//cpld复位命令
{
	bool res = true;
	int ret = 0;
	ret = fpga_send_cmd_only(PGC_I2C_RESET);                         //Reset CPLD 0x60
	if (ret < 0)
		res = false;
	return res;

}

unsigned int pango_PGC_I2C_SleepEmbedFlash(void)          			//flash休眠命令
{
	int ret = 0;
	// step 2: write EFLASH_SLEEP I2C command byte
    ret = fpga_send_cmd_only(PGC_I2C_EFLASH_SLEEP);      			// Embedded FLASH Sleep 70
    if (ret < 0)
    {
        printk("I2C bus error: Fail to write command %x with ack 0!\n", PGC_I2C_EFLASH_SLEEP);
    }

	if (!pango_PGC_I2C_CheckConfigStatus(PGC_STSREG_BUSY, 1000, false))
    {
        printk("Fail to check busy flag!\n");
    }

    pango_PGC_I2C_WriteWRDIS();
    return pango_PGC_I2C_ReadStatusRegister();
}

void pango_ClearAll(unsigned char *buf, unsigned int bytes)         //清空形参buf，置0x00
{
    //memset(buf, 0, bytes);										//MSX1234567
    int i;
    for (i = 0; i < bytes; ++i)
    {
        buf[i] = 0x00;
    }
}

bool pango_PGC_IsEmbedFlashSleep(unsigned int sts_val)
{
    return (sts_val & PGC_STSREG_SLEEP) ? true : false;             //状态寄存器[18] sleep 嵌入式FLASH休眠标志
}

bool pango_PGC_IsEmbedFlashWakeup(unsigned int sts_val)
{
    return (sts_val & PGC_STSREG_WAKE) ? true : false;              //[19] wake 嵌入式FLASH唤醒标志
}

void pango_ReverseBytes(unsigned char *buf, unsigned int num)
{
    int i;
    for (i = 0; i < num; ++i)
    {
        buf[i] = pango_ReverseByte(buf[i]);
    }
}

unsigned char pango_ReverseByte(unsigned char byte)
{
    if (byte == 0xFF || byte == 0x00)
    {
        //It's very common for BitStream data to have 0xFF and 0x00
        return byte;
    }
    byte = (byte & 0x55) << 1 | (byte & 0xAA) >> 1;
    byte = (byte & 0x33) << 2 | (byte & 0xCC) >> 2;
    byte = (byte & 0x0F) << 4 | (byte & 0xF0) >> 4;
    return byte;
}

static bool VerifyBufData(unsigned char *read_buf, unsigned char *write_buf, unsigned int total_size)
{
    unsigned int i;
	bool flag = true;
    for (i = 0; i < total_size; ++i)
    {
		printk("write_buf val is %x , and read_buf is %x , i is %d\n",write_buf[i],read_buf[i],i);
        if (write_buf[i] != read_buf[i])
        {
			flag = false;
			//printk("VerifyBufData error,exit\n");								//MSX1234567
        }
    }

	if(flag == true)
	{
		printk("VerifyBufData data ok,exit\n");
        return true;
	}

	else if(flag == false)
	{
		printk("VerifyBufData error,exit\n");
        return false;
	}
    //return true;
	return flag;
}

#if 0
//写特征控制寄存器，使能配置模式IIC，随后回读状态寄存器，确保状态
bool pango_PGC_I2C_WriteFeatureControlRegister(PGCDevice *dev, unsigned int feature_control_val)
{
	//unsigned char ack, i;
	unsigned char ack;
	unsigned char read_buf[4] = {0};
	unsigned char *buf = read_buf;
    unsigned int rb_ctl;
    bool res = true;
	if (!pango_PGC_I2C_WriteWREN())
    {
		printk("Fail to send WriteWREN 0x51\n");
        return false;
    }		

	//MSX1234
	ack =  fpga_program_ctl_cmd(feature_control_val);
	if (ack < 0)
	{
        printk("I2C bus error: Fail to write command %x with ack 0!", PGC_I2C_PROGRAM_CTL);
        res = false;
        goto i2c_wrt_ctl_error;
    }

#if 0
	// step 2: write PROGRAM_CTL I2C command byte
	ack = fpga_send_cmd_only(PGC_I2C_PROGRAM_CTL);              		//0x22改芯片特征控制位，使能配置模式IIC
	if (ack < 0)
    {
        printk("I2C bus error: Fail to write command %x with ack 0!", PGC_I2C_PROGRAM_CTL);
        res = false;
        goto i2c_wrt_ctl_error;
    }
	// step 3: write three dummy byte, indeed they are unused address bytes
    for (i = 0; i < 3; ++i)
    {
        ack = fpga_send_cmd_only(0x00);
        if (ack < 0)
        {
            printk("I2C bus error: Fail to write dummy byte %x with ack 0!\n", 0x00);
            res = false;
            goto i2c_wrt_ctl_error;
        }
    }
	// step 4: write 32 bits ctl code
    //const unsigned int feature_control_val_a_i2c = (i2c_slave_addr << 18) | DEFAULT_FEATURE_CTL | 1;
    //#define DEFAULT_FEATURE_CTL				0x20902 
    for (i = 0; i < 4; ++i)                                             //0x22 00 00 00 XX XX XX XX
    {
        ack = fpga_send_cmd_only(pango_ReverseByte((feature_control_val >> (i << 3)) & 0xFF));
        if (ack < 0)
        {
            printk("I2C bus error: Fail to write ctl byte %x with ack 0!\n", pango_ReverseByte((feature_control_val >> (i << 3)) & 0xFF));
            res = false;
            goto i2c_wrt_ctl_error;
        }
    }
#endif

	//wait for busy low，读状态寄存器，确保[21] busy 嵌入式FLASH忙碌标志为低，低不满足这个if条件
    if (!pango_PGC_I2C_CheckConfigStatus(PGC_STSREG_BUSY, 1000, false))    
    {
        printk("Error: embed flash busy flag is high!\n");
        res = false;
        goto i2c_wrt_ctl_error2;
    }

	// step 2: write READ_CTL I2C command byte							//MSX12345
	ack = fpga_read_ctl_cmd(buf);
	if (ack < 0)
    {
        printk("I2C bus error: Fail to write command %x with ack 0!\n", PGC_I2C_READ_CTL);
        res = false;
        goto i2c_wrt_ctl_error;
    }
	pango_ReverseBytes(buf, 4);

#if 0
	// step 2: write READ_CTL I2C command byte
    ack = fpga_send_cmd_only(PGC_I2C_READ_CTL);                 		 //读状态寄存器，Read Feature Control bit,0X31
    if (ack < 0)
    {
        printk("I2C bus error: Fail to write command %x with ack 0!\n", PGC_I2C_READ_CTL);
        res = false;
        goto i2c_wrt_ctl_error;
    }
	// step 3: write four dummy byte, indeed here should write 24 bits address,
    //         however, these address byte is unused in PGC, so just write dummpy bytes
    for (i = 0; i < 4; ++i)
    {
        ack = fpga_send_cmd_only(0x00);         	//0X31 AA AA AA 00,但是实际上0x31命令不会使用到地址值，所以发0x00即可
        if (ack < 0)
        {
            printk("I2C bus error: Fail to write dummy byte %x with ack 0!\n", 0x00);
            res = false;
            goto i2c_wrt_ctl_error;
        }
    }

	// step 5: read 4 ctl code bytes
    for (i = 0; i < 4; ++i)
    {
		buf[i]=fpga_recive_only();									
    }
	pango_ReverseBytes(buf, 4);
#endif


	rb_ctl = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);          
    if (rb_ctl != feature_control_val)
    {
        printk("readback control val %x not equal to feature controal val %x!\n", rb_ctl, feature_control_val);
        res = false;
    }
i2c_wrt_ctl_error:
    printk("i2c_wrt_ctl_error!\n");
i2c_wrt_ctl_error2:
    if (!pango_PGC_I2C_WriteWRDIS()) 
    {
        return false;
    }

    return res;

}
#endif

//读状态寄存器，确保[21] busy 嵌入式FLASH忙碌标志
bool pango_PGC_I2C_CheckConfigStatus(PGCStatusRegisterBitMask bit_mask, unsigned int check_times, bool is_high)
{
    unsigned int i, status_code;
    for (i = 0; i < check_times; ++i)
    {
        status_code = pango_PGC_I2C_ReadStatusRegister();
        //printk("\r\nstatus = %x\r\n",status_code);
        if (is_high)
        {
            if ((status_code & bit_mask))
            {
                return true;
            }
        }
        else
        {
            if (!(status_code & bit_mask))
            {
                return true;
            }
        }
        mdelay(20);
    }
	printk("pango_PGC_I2C_CheckConfigStatus error : PGCStatusRegister value is %d\n",status_code);
    return false;
}

/*return:
True: id_code match in list
False: id_code no match in list
*/
bool pango_PGC_DeviceEflashInit(PGCDevice *dev, unsigned int id_code)
{
    unsigned int device_num = sizeof(pgclist)/sizeof(PGCProperties);
    int i;
    for(i = 0; i<device_num; i++)
        if(pgclist[i].device_id == (id_code & DEVICE_ID_MASK))
        {
            dev->_pgc_properties = pgclist[i];
            break;
        }
    return (i != device_num ? true:false);
}

//320+59  379
//page_addr = PG_GetEflashAddr(dev, i + addr_offset);                         //addr_offset为flash烧录起始地址，i其实就是累加的PageNum
unsigned int PG_GetEflashAddr(PGCDevice *dev, unsigned int PageNum)           //要写flash的页号，PageNum
                                                                              //根据IC型号以及页号确定flash烧录地址
{        
    unsigned int addr;
    unsigned int page_num_temp;
    unsigned int total_page = dev->_pgc_properties.total_page;                 //flash容量大小信息读取
    unsigned int pages_per_heap = dev->_pgc_properties.pages_per_heap;

	printk("total_page is %d , and pages_per_heap is %d!!\n",total_page,pages_per_heap);

    if(PageNum > total_page)
    {
        printk("Warning : the input page num is larger than total page num,PageNum is %d!!\n",PageNum);
        page_num_temp = total_page;
    }
    else
        page_num_temp = PageNum;                                          		//记录页号到page_num_temp变量

	#if 0
    switch(dev->_pgc_properties.device_id)                                      //根据IC型号以及页号确定flash烧录地址
    {                                                                           //CPLD是PGC4KL_ID这个型号
        case PGC1KG_ID: case PGC1KL_ID: case PGC2KG_ID: case PGC2KL_ID:
            addr = page_num_temp << 8 | (0x3<<17);
            break;
        case PGC10KD_ID:
            addr = ((page_num_temp/pages_per_heap) << 18) | ((page_num_temp%pages_per_heap) << 8);
            break;
        default:                                                                //走这个

			//page_num_temp/pages_per_heap为堆数，第几个堆，每个堆320页，每页256byte
			//page_num_temp%pages_per_heap为页数
            addr = ((page_num_temp/pages_per_heap) << 17) | ((page_num_temp%pages_per_heap) << 8);
            break;

        // [23:19] 保留 [18:17] Ba[1:0] 堆地址 [16:8] Ra[8:0] 页地址 [7:0] 保留
        // 对于2K器件， Ba[1:0]为保留堆地址，需要设置为2’ b11

    }	
	#endif

	//page_num_temp/pages_per_heap为堆数，第几个堆，每个堆320页，每页256byte
	//page_num_temp%pages_per_heap为页数
    addr = ((page_num_temp/pages_per_heap) << 17) | ((page_num_temp%pages_per_heap) << 8);

    return addr;	
}


//res = pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify(dev, 0);         //从flash的起始地址0开始进行烧录  
//分批下固件数据包-->每次下256个字节-->每次下完256个字节的数据包做读写匹配，校验数据，直到整个固件分发完成
bool pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify(PGCDevice *dev, unsigned int addr_offset)
{
	/*
    if(dev->_pgc_properties.device_id == PGC10KD_ID)            
    //#define PGC10KD_ID		       0x042C899               但CPLD使用0x422899
        return pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify_10k(dev, addr_offset);
    else
        return pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify_n10k(dev, addr_offset);      //走这个
	*/

	printk("pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify addr_offset is %d\n",addr_offset);			//fc--252+320

	return pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify_n10k(dev, addr_offset);      //走这个
}

//这个函数涉及到固件分包以及读入fw_buf的相关处理，备注了MSX编号的地方需要回头再做处理修改
bool pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify_n10k(PGCDevice *dev, unsigned int addr_offset)    
{
	bool res = true;
	unsigned char ack;
    //unsigned char ack, addr;
    unsigned int page_size = 256;
    unsigned int write_times = dev->_pgc_properties.bitstream  / page_size;      	//计算出总共需要写flash的次数，每次写256字节
    //unsigned int left_bytes = dev->_pgc_properties.bitstream  % page_size;       	//剩余字节
    unsigned char write_buf[256];
    //unsigned int i, j, bytes_read, byte_to_read;
	unsigned int i, j, bytes_read;
    unsigned int page_addr;
	unsigned int last_size = 0;
    //unsigned char *buf = pango_GetI2CReqBuf(inf);									//MSX-1，涉及到固件文件分批分包处理
	//unsigned char fw_buf[256] = {0};
	//unsigned char *buf = fw_buf;
	unsigned char *buf = cpld_firmware_data;
	unsigned int cpld_firmware_size = sizeof(cpld_firmware_data) / sizeof(unsigned char);
	printk("pango cpld_firmware_data[%d] , and total write_times is %d\n",cpld_firmware_size,write_times);

    //dev->_cb->prepare_read((void*)dev);                                           //flash信息初始化
	for (i = 0; res && i <= write_times; ++i)
    {
        if (!pango_PGC_I2C_WriteWREN())                                          	//WriteWREN指令
		{
			printk("pango Fail to send WriteWREN 0x51\n");
            return false;
		}

		/*
        if (i == write_times)                                                       //写到了最后一次
        {
            //byte_to_read = left_bytes;                                            //接下来写剩余字节
			bytes_read = left_bytes;                                            	//接下来写剩余字节
            if (left_bytes == 0)
                break;
        }
        else
            //byte_to_read = page_size;                                             //每次从固件读256字节
			bytes_read = page_size;                                               	//每次从固件读256字节
		*/
		
		if(cpld_firmware_size >= 256)
		{
			printk("pango ((cpld_firmware_size-256) >= 256 enter , and i is %d\n",i);
		    bytes_read = 256;
			cpld_firmware_size -= 256;    				//更新剩余需要烧录的固件数组大小size

			printk("pango lastest cpld_firmware_size is %d\n",cpld_firmware_size);

			//从cpld_firmware_data拷贝bytes_read个字节到buf里
			memcpy(buf, cpld_firmware_data + i*bytes_read, bytes_read);
		}
		//else if((cpld_firmware_size < 256) && cpld_firmware_size > 0)
		else if(cpld_firmware_size > 0)
		{
			printk("pango (cpld_firmware_size-256) < 256 && cpld_firmware_size > 0 enter, and i is %d\n",i);
			printk("pango cpld_firmware_size is %d\n",cpld_firmware_size);

			//跑得进来else分支，此时的数组大小cpld_firmware_size，就已经是最后一包<256的固件数据了
			bytes_read = cpld_firmware_size;								//剩余数组大小

			printk("pango bytes_read is %d\n",bytes_read);

			memcpy(buf, cpld_firmware_data + i*256 , bytes_read);
			cpld_firmware_size = 0;											//最后一包发完后，将数组大小清0
		}
		else if(cpld_firmware_size == 0)  
		{				
			printk("pango bytes_read = 0;	 enter, and i is %d\n",i);
			//此时退出循环即可，无需拷贝buf，下发固件数据了，已经下发完整个固件了，此时的cpld_firmware_size为0
			bytes_read = 0;													//清除bytes_read
		}

        //dev->_cb->read(buf, byte_to_read, &bytes_read);                   //MSX-2,忽略，不需要从SPI-flash读进来
		//bytes_read = 256;													//MSX-3,先假设每次都从固件中读出256个字节下包

        if (bytes_read == 0)												//bytes_read记录着从SPI-flash读出的数据个数
		{        
			printk("pango bytes_read already == 0 , exit\n");                                      
            break;                                                         //如果最后一包数据，bytes_read可能会小于256
		}
        for (j = 0; j < bytes_read; ++j)
            write_buf[j] = buf[j];                                                  //buf里存着固件data
                                                                                    //存到write_buf里，后面做校验使用

		printk("pango PG_GetEflashAddr enter\n");
//从CPLD内部flash的第一个堆，第0x3b页,即59页开始刷  320 + 59
        page_addr = PG_GetEflashAddr(dev, i + addr_offset);                   //根据IC型号以及页号确定flash烧录地址 
		//page_addr = 0x00023b00 + i*256;
		printk("pango PG_GetEflashAddr exit\n");

		if(bytes_read == page_size) 
		{
			ack = fpga_program_cmd(page_addr,buf);							
			if (ack < 0)
        	{
            	printk("pango fpga_program_cmd error\n");
            	res = false;
            	//goto i2c_program_eflash_page_end;
        	} 
		}    

		else if(bytes_read != page_size)                          //但最后一包固件数据，bytes_read循环的次数会少一些
		{
			printk("pango last time to program flash,now bytes_read is %d\n",bytes_read);
			//page_size-bytes_read为要补0x00的个数
			last_size = page_size - bytes_read;
			ack =  fpga_program_cmd_last_time(page_addr,buf,bytes_read,last_size);		
			if (ack < 0)
            {
                printk("pango fpga_program_cmd_last_time error\n");
                res = false;
                //goto i2c_program_eflash_page_end;
            }

			#if 0
			//page_size-bytes_read为要补0x00的个数
            for (j = 0; j < page_size-bytes_read; ++j)       			//这里处理最后一包剩余的固件数据
            {                                                           //因为每次必须发256个字节出去
                                                                        //所以要填充满最后一包不够256缺失的固件数据
                ack = fpga_send_cmd_only(0x00);//full up     			//以0X00做为补充，补足256字节
                if (ack < 0)
                {
                    printk("I2C bus error: Fail to write page data with ack 0!\n");
                    res = false;
                    goto i2c_program_eflash_page_end;
                }
            }	
			#endif
		}	

#if 0																				//addr_offset为0，i逐渐递加
        // step 1: write PROGRAM command        
		ack = fpga_send_cmd_only(PGC_I2C_PROGRAM);                         			//Program Page,0x20   
		if (ack < 0)
        {
            printk("I2C bus error: Fail to write command %x with ack 0!\n", PGC_I2C_PROGRAM);
            res = false;
            goto i2c_program_eflash_page_end;
        }     
		for (j = 0; j < 3; ++j)     //AA AA AA,其中的AA表示地址的一个字节（8bit）    //page_addr拆分3个byte发出去
        {

        // [23:19] 保留 [18:17] Ba[1:0] 堆地址 [16:8] Ra[8:0] 页地址 [7:0] 保留
        // 对于2K器件， Ba[1:0]为保留堆地址，需要设置为2’ b11   
			ack = fpga_send_cmd_only((page_addr >> (16 - j * 8)) & 0xFF);                         			
            if (ack < 0)
            {
                printk("I2C bus error: Fail to write addr byte,and now page_addr is %d!\n",page_addr);
                res = false;
                goto i2c_program_eflash_page_end;
            }
        } 				

		//开始下固件数据包，256字节/包数据，总共要下write_times次固件数据包

        for (j = 0; j < bytes_read; ++j)                    //bytes_read记录着从SPI-flash读出的数据个数/每次从固件读出的数据个数
        {                                                   //如果最后一包数据，bytes_read可能会小于256
			//ack = pango_I2CSendByteWithACK(inf, buf[j]);                    		//buf里存着固件dat
            ack = fpga_send_cmd_only(buf[j]);          								//buf里存着固件data
            if (ack < 0)                                   //大部分时候，简单理解为每次发256字节固件数据                                            
            {                                               
                printk("I2C bus error: Fail to write page data with ack 0!\n");
                res = false;
                goto i2c_program_eflash_page_end;
            }
        }

		if(bytes_read != page_size)                          //但最后一包固件数据，bytes_read循环的次数会少一些
		{
            for (j = 0; j < page_size-bytes_read; ++j)       //这里处理最后一包剩余的固件数据
            {                                                           //因为每次必须发256个字节出去
                                                                        //所以要填充满最后一包不够256缺失的固件数据
                ack = fpga_send_cmd_only(0x00);//full up     			//以0X00做为补充，补足256字节
                if (ack < 0)
                {
                    printk("I2C bus error: Fail to write page data with ack 0!\n");
                    res = false;
                    goto i2c_program_eflash_page_end;
                }
            }	
		}	
#endif	

		//已经下完256个固件数据包了		

		// step 2: wait for busy low，读状态寄存器，确保[21] busy 嵌入式FLASH忙碌标志为低，低不满足这个if条件
        //不为低则进入if条件里break
        if (!pango_PGC_I2C_CheckConfigStatus(PGC_STSREG_BUSY, 1000, false))
        {
            printk("pango Error: embed flash busy flag is high!, i is %d\n",i);
            res = false;
            break;
        }

		#if 0
		// step 3: write READ to read back this page to verify data		//MSX123456
		ack = fpga_send_cmd_only(PGC_I2C_READ);              			//Read flash，0x30
        if (ack < 0)
        {
            printk("I2C bus error: Fail to write command %x with ack 0!", PGC_I2C_READ);
            res = false;
            goto i2c_program_eflash_page_end;
        }
		for (j = 0; j < 3; ++j)                                          //0x30 AA AA AA 00
        {
			ack = fpga_send_cmd_only((page_addr >> (16 - j * 8)) & 0xFF); 				//page_addr作为AA AA AA地址byte   
            if (ack < 0)
            {
                printk("I2C bus error: Fail to write addr byte with ack 0\n!");
                res = false;
                goto i2c_program_eflash_page_end;
            }
        }
		ack = fpga_send_cmd_only(0x00);  								 //send dummy byte,补发0x00,拼凑0x30 AA AA AA 00
        if (ack < 0)
        {
            printk("I2C bus error: Fail to write dummy byte with ack 0!");
            res = false;
            goto i2c_program_eflash_page_end;
        }																//MSX123456
		#endif

		pango_ClearAll(buf, 256);                                		 //清空形参buf，置0x00  

		fpga_page_read_cmd(page_addr,buf); 					 	  	     //回读flash，校验固件数据
		
		//pango_ReverseBytes(buf,256);									 //回读数据，byte反转处理

		#if 0
		//回读flash，校验固件数据
		for (j = 0; j < bytes_read; ++j)                                    
        {
			buf[j]=fpga_recive_only();									//回读的固件数据存放到buf里
        }
		#endif

		if (!VerifyBufData(buf, write_buf, bytes_read))                 //校验数据，读buf和写buf匹配，读写匹配
        {
            printk("pango Fail to verify data in page %i\n", i);
            res = false;
        }
		if (!pango_PGC_I2C_WriteWRDIS())                            	 //WriteWRDIS指令，0x52，Write Disable
		{
			printk("pango PGC_I2C_WriteWRDIS1111 failed, res = false!\n");
        	res = false;
		}
		

//i2c_program_eflash_page_end:
		//printk("i2c_program_eflash_page_end , Fail to write page data !\n");								
	}

	printk("pango PGC_I2C_ProgramSelfLoadBitstreamAndVerify_n10k oK,fw update oK\n");

    //dev->_cb->prog(100);
    if (!pango_PGC_I2C_WriteWRDIS())
	{
		printk("pango PGC_I2C_WriteWRDIS22222 failed, return false\n");
        return false;
	}

    return res;
}	

void pango_PGC_I2C_ProgramEFlash(PGCDevice *dev, unsigned int feature_control_val)
{
	bool res, is_sleep;
    unsigned int sts_val, id_code;
    printk("\r\n PGC I2C EFLASH configuration start ... \r\n");
    id_code = pango_PGC_I2C_ReadIdcode();                                     	 //1. writeRDID读ID
    if (!pango_PGC_DeviceEflashInit(dev, id_code))                               //2. IDCODE匹配及检查
    {                                                                            //同时赋值flash信息
        printk("\r\n Invalid PGC id coda read out: 0x%x!\r\n", id_code);
        return;
    }

	if (!pango_PGC_I2C_WriteWREN())                                          	 //WriteWREN指令, 0x51
	{
		printk("Fail to send WriteWREN 0x51\n");
        return;
	}

	sts_val = pango_PGC_I2C_ReadStatusRegister();                             //3. 读状态寄存器，RDSR命令，0xA3
    printk("\r\n Read out status code 0x%x!\r\n", sts_val);
    is_sleep = pango_PGC_IsEmbedFlashSleep(sts_val);                    //3.1 确认状态寄存器[18] sleep 嵌入式FLASH休眠标志
    sts_val = pango_PGC_I2C_WakeupEmbedFlash();                      	//4. EFLASH_WAKEUP，唤醒flash，0x71命令
                                                                        //4.1 且回读状态寄存器，保存在sts_val中		
    mdelay(30);																																	

	if (!pango_PGC_IsEmbedFlashWakeup(sts_val))                         //4.2 确认状态寄存器[19] wake 嵌入式FLASH唤醒标志
    {
        printk("\r\nError: Fail to wakeup embed flash!\r\n");
        return;
    }
    printk("\r\n After wakeup read out status code 0x%x !\r\n", sts_val);	

	#if 0
	pango_PGC_I2C_EraseSelfLoadBitstreamAndUserFlash();                 //5. flash擦除，Erase Bulk，0x10命令	

	mdelay(30);

	// step 2: wait for busy low，读状态寄存器，确保[21] busy 嵌入式FLASH忙碌标志为低，低不满足这个if条件
	//不为低则进入if条件里break
    if (!pango_PGC_I2C_CheckConfigStatus(PGC_STSREG_BUSY, 1000, false))
    {
        printk("Error: flash Erase Bulk busy flag is high!!\n");
        return;
    }
	#endif

	//分批下固件数据包-->每次下256个字节-->每次下完256个字节的数据包做读写匹配，校验数据，直到整个固件分发完成
    res = pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify(dev, 0xFC+320);     //fc -- 252+320
             
    if (!res)                                                               //如果烧录过程中出现错误，返回false
    {
        printk("Error: Fail to program bitstream in eflash!\n");
        return;
    }		
	else
		printk("Program bitstream in eflash ok!\n");	
#if 0
	if (!(feature_control_val&0x1))
        printk("Warring: Write feature control value with bit0 master auto boot off\n");												
																
	res = pango_PGC_I2C_WriteFeatureControlRegister(dev, feature_control_val); 
    if (!res)
    {
        printk("Error: Fail to write feature control value %x!\n", feature_control_val);
        return;
    }
#endif
	if (is_sleep)           //如果is_sleep为true，则原来烧录flash之前，flash为睡眠状态，为了烧录flash，使用命令唤醒了flash
    {
        // 9. sleep embed flash if sleep
        sts_val = pango_PGC_I2C_SleepEmbedFlash();       	//让flash重新进入休眠，Embedded FLASH Sleep, 0x70
        if (!pango_PGC_IsEmbedFlashSleep(sts_val))          //确认休眠状态位
        {
            printk("Error: Fail to sleep embed flash!\r\n");
            return;
        }
        printk("After sleep read out status code 0x%x\r\n", sts_val);
    }
    pango_PGC_I2C_Reset();                               //cpld复位命令， Reset CPLD 0x60
	printk("pango_PGC_I2C_Reset OK, exit\n");
}

//--------------------------------------------------------------------------------------//


/*static int nt36672e_write_bytes(unsigned char addr, unsigned char value)*/
int nt36672e_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("nt36672e write data fail !!\n");
	return ret;
}
EXPORT_SYMBOL(nt36672e_write_bytes);

int nt36672e_read_bytes(unsigned char addr, unsigned char *returnData)
{
	char pBuff;
	char puSendCmd[1];
	int ret = 0;
	struct i2c_client *client = nt36672e_i2c_client;

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	puSendCmd[0] = addr;
	ret = i2c_master_send(client, puSendCmd, 1);
	ret = i2c_master_recv(client, &pBuff, 1);
	if (ret < 0)
		printk("%s: ERROR read 0x%x fail \n",
			__func__, addr);

	*returnData = pBuff;

	return ret;
}
EXPORT_SYMBOL(nt36672e_read_bytes);

static int __init nt36672e_iic_init(void)
{
	pr_info("%s\n", __func__);
#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(TPS_I2C_BUSNUM, &nt36672e_board_info, 1);
#endif
	pr_info("nt36672e_iic_init2\n");
	i2c_add_driver(&nt36672e_iic_driver);
	pr_info("%s success\n", __func__);
	return 0;
}

static void __exit nt36672e_iic_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&nt36672e_iic_driver);
}


module_init(nt36672e_iic_init);
module_exit(nt36672e_iic_exit);

MODULE_AUTHOR("Kaiduan Cao <kaiduan.cao@mediatek.com>");
MODULE_DESCRIPTION("MTK nt36672e I2C Driver");
MODULE_LICENSE("GPL");
#endif