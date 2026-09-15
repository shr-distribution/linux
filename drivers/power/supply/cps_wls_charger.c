/*
 * Copyright © 2023, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * version:1.3
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/firmware.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
//#include <linux/wakelock.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/fs.h>

#include "cps_wls_charger.h"
//#define LOAD_HEX 0
#ifdef LOAD_HEX
#define BOOTLOADER_FILE_NAME "/data/misc/cps4021_bootloader.hex"
#define FIRMWARE_FILE_NAME "/data/misc/CPS4021.hex"
#endif
#define CPS_WLS_CHRG_DRV_NAME "cps-wls-charger"
#define CPS_WLS_CHRG_PSY_NAME "wireless"

#define CPS_WLS_BL_BIN_PATH "CPS4021_BL.bin" //only save in /drivers/base/CPS4021_BL.bin
#define CPS_WLS_FW_BIN_PATH "CPS4021.bin"    //only save in /drivers/base/CPS4021_BL.bin
struct cps_wls_chrg_chip *chip = NULL;

/*define cps rx reg enum*/
typedef enum
{                                                           
    CPS_RX_REG_VOUT_SET,                                                     
    CPS_RX_REG_SS_VAL,                                       
    CPS_RX_REG_CE_VAL,                                       
    CPS_RX_REG_RP_VAL,                                       
    CPS_RX_REG_FOP_VAL,                                    
    CPS_RX_REG_ADC_VRECT,                               
    CPS_RX_REG_ADC_MLDO_DROP,                     
    CPS_RX_REG_ADC_IOUT,                                 
    CPS_RX_REG_ADC_VOUT,                                
    CPS_RX_REG_ADC_DIE_TMP,                            
    CPS_RX_REG_PPP_HEADER,                            
    CPS_RX_REG_PPP_COMMAND,                      
    CPS_RX_REG_PPP_DATA0,                               
    CPS_RX_REG_PPP_DATA1,                               
    CPS_RX_REG_PPP_DATA2,                               
    CPS_RX_REG_PPP_DATA3,                               
    CPS_RX_REG_PPP_DATA4,                               
    CPS_RX_REG_PPP_DATA5,                             
    CPS_RX_REG_PPP_DATA6,                               
    CPS_RX_REG_BC_HEADER,                              
    CPS_RX_REG_BC_COMMAND,                        
    CPS_RX_REG_BC_DATA0,                                 
    CPS_RX_REG_BC_DATA1,                                 
    CPS_RX_REG_BC_DATA2,                                
    CPS_RX_REG_BC_DATA3,                                 
    CPS_RX_REG_BC_DATA4,                               
    CPS_RX_REG_BC_DATA5,                                 
    CPS_RX_REG_BC_DATA6, 
    CPS_RX_REG_MAX                                                        
}cps_rx_reg_e;

/*define cps tx reg enum*/
typedef enum
{
    CPS_TX_REG_PPP_HEADER,     
    CPS_TX_REG_PPP_COMMAND,     
    CPS_TX_REG_PPP_DATA0,       
    CPS_TX_REG_PPP_DATA1,       
    CPS_TX_REG_PPP_DATA2,       
    CPS_TX_REG_PPP_DATA3,       
    CPS_TX_REG_PPP_DATA4,       
    CPS_TX_REG_PPP_DATA5,      
    CPS_TX_REG_PPP_DATA6,      
    CPS_TX_REG_BC_HEADER,       
    CPS_TX_REG_BC_COMMAND,    
    CPS_TX_REG_BC_DATA0,        
    CPS_TX_REG_BC_DATA1,        
    CPS_TX_REG_BC_DATA2,       
    CPS_TX_REG_BC_DATA3,        
    CPS_TX_REG_BC_DATA4,        
    CPS_TX_REG_BC_DATA5,      
    CPS_TX_REG_BC_DATA6,             
    CPS_TX_REG_FUNC_EN,         
    CPS_TX_REG_FOP_VAL,      
    CPS_TX_REG_ADC_VIN,       
    CPS_TX_REG_ADC_VRECT,      
    CPS_TX_REG_ADC_IPA,       
    CPS_TX_REG_CE_VAL,         
    CPS_TX_REG_RP_VAL,       
    CPS_TX_REG_EPT_RSN,   
    CPS_TX_REG_ADC_DIE_TEMP,   
    CPS_TX_REG_EPT_CODE,  
    CPS_TX_REG_MAX
}cps_tx_reg_e;

typedef enum
{
    CPS_COMM_REG_CHIP_ID,  
    CPS_COMM_REG_FW_MINOR,
    CPS_COMM_REG_FW_MAJOR,
    CPS_COMM_REG_SYS_MODE,
    CPS_COMM_REG_CRC_VAL,
    CPS_COMM_REG_INT_EN,
    CPS_COMM_REG_INT_FLAG,
    CPS_COMM_REG_INT_CLR,
    CPS_COMM_REG_CMD,
    CPS_COMM_FUNC_EN,
    CPS_COMM_REG_MAX
} cps_comm_reg_e;

#define RX_REG_FOD_CUR_0              0x00C0
#define RX_REG_FOD_CUR_1              0x00C1
#define RX_REG_FOD_CUR_2              0x00C2
#define RX_REG_FOD_CUR_3              0x00C3
#define RX_REG_FOD_CUR_4              0x00C4
#define RX_REG_FOD_CUR_5              0x00C5
#define RX_REG_FOD_CUR_6              0x00C6
#define RX_REG_FOD_C0_GAIN            0x00C7
#define RX_REG_FOD_C0_OFFSET          0x00C8
#define RX_REG_FOD_C1_GAIN            0x00C9
#define RX_REG_FOD_C1_OFFSET          0x00CA
#define RX_REG_FOD_C2_GAIN            0x00CB
#define RX_REG_FOD_C2_OFFSET          0x00CC
#define RX_REG_FOD_C3_GAIN            0x00CD
#define RX_REG_FOD_C3_OFFSET          0x00CE
#define RX_REG_FOD_C4_GAIN            0x00CF
#define RX_REG_FOD_C4_OFFSET          0x00D0
#define RX_REG_FOD_C5_GAIN            0x00D1
#define RX_REG_FOD_C5_OFFSET          0x00D2
#define RX_REG_FOD_C6_GAIN            0x00D3
#define RX_REG_FOD_C6_OFFSET          0x00D4
#define RX_REG_FOD_C7_GAIN            0x00D5
#define RX_REG_FOD_C7_OFFSET          0x00D6

typedef struct
{
    uint16_t reg_name;
    uint16_t reg_bytes_len;
    uint32_t reg_addr;
} cps_reg_s;

cps_reg_s cps_comm_reg[CPS_COMM_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_COMM_REG_CHIP_ID,        2,          0x0000},
    {CPS_COMM_REG_FW_MINOR,       1,          0x0008},
    {CPS_COMM_REG_FW_MAJOR,       1,          0x0009},
    {CPS_COMM_REG_SYS_MODE,       1,          0x000A},
    {CPS_COMM_REG_CRC_VAL,        2,          0x000C},
    {CPS_COMM_REG_INT_EN,         4,          0x0020},  //4byte //***********/
    {CPS_COMM_REG_INT_FLAG,       4,          0x0024},	//4byte
    {CPS_COMM_REG_INT_CLR,        4,          0x0028},	//4byte not work
    {CPS_COMM_REG_CMD,            4,          0x002C},  //4byte//***************/
    {CPS_COMM_FUNC_EN,            4,          0x0030}
};

cps_reg_s cps_rx_reg[CPS_RX_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_RX_REG_VOUT_SET,         2,          0x0090},
    {CPS_RX_REG_SS_VAL,           2,          0x0114},
    {CPS_RX_REG_CE_VAL,           2,          0x0116},
    {CPS_RX_REG_RP_VAL,           2,          0x0118},
    {CPS_RX_REG_FOP_VAL,          2,          0x010E},
    {CPS_RX_REG_ADC_VRECT,        2,          0x0104},
    {CPS_RX_REG_ADC_MLDO_DROP,    2,          0x0106},
    {CPS_RX_REG_ADC_IOUT,         2,          0x0108},
    {CPS_RX_REG_ADC_VOUT,         2,          0x010A},
    {CPS_RX_REG_ADC_DIE_TMP,      2,          0x010C},
    {CPS_RX_REG_PPP_HEADER,       1,          0x0050},
    {CPS_RX_REG_PPP_COMMAND,      1,          0x0051},
    {CPS_RX_REG_PPP_DATA0,        1,          0x0052},
    {CPS_RX_REG_PPP_DATA1,        1,          0x0053},
    {CPS_RX_REG_PPP_DATA2,        1,          0x0054},
    {CPS_RX_REG_PPP_DATA3,        1,          0x0055},
    {CPS_RX_REG_PPP_DATA4,        1,          0x0056},
    {CPS_RX_REG_PPP_DATA5,        1,          0x0057},
    {CPS_RX_REG_PPP_DATA6,        1,          0x0058},
    {CPS_RX_REG_BC_HEADER,        1,          0x0070},
    {CPS_RX_REG_BC_COMMAND,       1,          0x0071},
    {CPS_RX_REG_BC_DATA0,         1,          0x0072},
    {CPS_RX_REG_BC_DATA1,         1,          0x0073},
    {CPS_RX_REG_BC_DATA2,         1,          0x0074},
    {CPS_RX_REG_BC_DATA3,         1,          0x0075},
    {CPS_RX_REG_BC_DATA4,         1,          0x0076},
    {CPS_RX_REG_BC_DATA5,         1,          0x0077},
    {CPS_RX_REG_BC_DATA6,         1,          0x0078}, 
};

cps_reg_s cps_tx_reg[CPS_TX_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_TX_REG_PPP_HEADER,       1,          0x0070},
    {CPS_TX_REG_PPP_COMMAND,      1,          0x0071},
    {CPS_TX_REG_PPP_DATA0,        1,          0x0072},
    {CPS_TX_REG_PPP_DATA1,        1,          0x0073},
    {CPS_TX_REG_PPP_DATA2,        1,          0x0074},
    {CPS_TX_REG_PPP_DATA3,        1,          0x0075},
    {CPS_TX_REG_PPP_DATA4,        1,          0x0076},
    {CPS_TX_REG_PPP_DATA5,        1,          0x0077},
    {CPS_TX_REG_PPP_DATA6,        1,          0x0078},
    {CPS_TX_REG_BC_HEADER,        1,          0x0050},
    {CPS_TX_REG_BC_COMMAND,       1,          0x0051},
    {CPS_TX_REG_BC_DATA0,         1,          0x0052},
    {CPS_TX_REG_BC_DATA1,         1,          0x0053},
    {CPS_TX_REG_BC_DATA2,         1,          0x0054},
    {CPS_TX_REG_BC_DATA3,         1,          0x0055},
    {CPS_TX_REG_BC_DATA4,         1,          0x0056},
    {CPS_TX_REG_BC_DATA5,         1,          0x0057},
    {CPS_TX_REG_BC_DATA6,         1,          0x0058},
    {CPS_TX_REG_FUNC_EN,          4,          0x0030}, //4byte//not work yet
    {CPS_TX_REG_FOP_VAL,          2,          0x0108},
    {CPS_TX_REG_ADC_VIN,          2,          0x0100},
    {CPS_TX_REG_ADC_VRECT,        2,          0x0102},
    {CPS_TX_REG_ADC_IPA,          2,          0x0104},
    {CPS_TX_REG_CE_VAL,           1,          0x010D},
    {CPS_TX_REG_RP_VAL,           2,          0x010E},
    {CPS_TX_REG_EPT_RSN,          4,          0x0110},
    {CPS_TX_REG_ADC_DIE_TEMP,     2,          0x0106},
    {CPS_TX_REG_EPT_CODE,         1,          0x010B},
};

/*********************************************************************************************************
*
*  I2C APT start
*
*********************************************************************************************************/

static const struct regmap_config cps4021_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

static const struct regmap_config cps4021_regmap32_config = {
    .reg_bits = 32,
    .val_bits = 8,
};

static int cps_wls_l_write_reg(int reg, int value)
{
    int ret;
    mutex_lock(&chip->i2c_lock);
    ret = regmap_write(chip->regmap, reg, value);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }

    return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_l_read_reg(int reg)
{
    int ret;
    int value;

    mutex_lock(&chip->i2c_lock);
    ret = regmap_read(chip->regmap, reg, &value);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!0x%x\n", __func__,reg);
        return CPS_WLS_FAIL;
    }
    return value;
}

/*
return -1 means fail, 0 means success 16bit reg
*/
static int cps_wls_write_reg(int reg, int value, int byte_len)
{
    int i = 0, tmp = 0;
    for (i = 0; i < byte_len; i++)
    {
        tmp = (value >> (i * 8)) & 0xff;
        if (cps_wls_l_write_reg((reg & 0xffff) + i, tmp) == CPS_WLS_FAIL)   //write sing reg 
        {
            return CPS_WLS_FAIL;
        }
    }
    return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_read_reg(int reg, int byte_len)
{
    int i = 0, tmp = 0, read_date = 0;
    printk("cps_wls_read_reg enter\n");
    for (i = 0; i < byte_len; i++)
    {
        tmp = cps_wls_l_read_reg((reg & 0xffff) + i);
        if (tmp == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }

        read_date |= (tmp << (8 * i));
    }
    printk("cps_wls_read_reg exit\n");
    return read_date;
}

/*********************************************************************************************************
*
*   FOR PROGRAM
*
*********************************************************************************************************/
/*
16 bit reg  write n bytes 
*/
static int cps_wls_write_nbyte(int reg, int value, int data_len)
{
    int ret;

    mutex_lock(&chip->i2c_lock);
    ret = regmap_raw_write(chip->regmap, reg, &value, data_len);
    mutex_unlock(&chip->i2c_lock);
	
    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    return CPS_WLS_SUCCESS;
}

/*
16 bit reg  read n bytes ,max is 4 bytes
*/
static int cps_wls_read_nbyte(int addr, int data_len)
{
    int ret;
    u8 read_date[4];
    int r_date = 0;

    mutex_lock(&chip->i2c_lock);
    // ret = regmap_raw_read(chip->regmap32, addr, read_date, 4);
    ret = regmap_raw_read(chip->regmap, addr, read_date, data_len);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    r_date = read_date[3];
    r_date = r_date << 8;
    r_date |= read_date[2];
    r_date = r_date << 8;
    r_date |= read_date[1];
    r_date = r_date << 8;
    r_date |= read_date[0];

    return r_date;
}

/*
* 16 bit reg write
*/
static int cps_wls_program_sram(int addr, u8 *date, int len)
{
    int ret;
    mutex_lock(&chip->i2c_lock);
    ret = regmap_raw_write(chip->regmap, addr, date, len);
    mutex_unlock(&chip->i2c_lock);
    // ret = cps_wls_write_reg_array(addr,date,len);
    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] sram i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    return CPS_WLS_SUCCESS;
}

static int cps_wls_program_cmd_send(int cmd)
{

    return cps_wls_write_nbyte(ADDR_CMD, cmd,4);
}

static int cps_wls_program_wait_cmd_done(void)
{
    int res;
    int cnt = 0; // ms
    while (1)
    {
        cps_wls_write_nbyte(0xFF82,0x2000,2);    
        res = cps_wls_read_nbyte(ADDR_FLAG,4);
        if (res == CPS_WLS_FAIL)
            return CPS_WLS_FAIL;

        msleep(1);

        switch (res & 0xFF)
        {
        case RUNNING:
            break;

        case PASS:
            break;

        case FAIL:
            cps_wls_log(CPS_LOG_ERR, "---> FAIL : %x\n", res);
            return res;
            break;

        case ILLEGAL:
            cps_wls_log(CPS_LOG_ERR, "---> ILLEGAL : %x\n", res);
            return res;
            break;

        default:
            cps_wls_log(CPS_LOG_ERR, "---> ERROR-CODE : %x\n", res);
            return res;
            break;
        }

        if (res == PASS)
        {
            break;
        }

        /*3s over time*/
        if ((cnt++) == 3000)
        {
            cps_wls_log(CPS_LOG_ERR, "--->[%s] CMD-OVERTIME\n", __func__);
            break;
        }
    }
    return res;

    // return CPS_WLS_SUCCESS;
}

// uint16_t get_crc(u8 *buf, int len)
// {
//     int i, j;

//     uint16_t crc_in = 0x0000;
//     uint16_t crc_poly = 0x1021;

//     for (i = 0; i < len; i++)
//     {
//         crc_in ^= (buf[i] << 8);
//         for (j = 0; j < 8; j++)
//         {
//             if (crc_in & 0x8000)
//                 crc_in = (crc_in << 1) ^ crc_poly;
//             else
//                 crc_in = crc_in << 1;
//         }
//     }

//     return crc_in;
// }
#ifdef LOAD_HEX

static int fp_size(struct file *f)
{
    int error = -EBADF;
    struct kstat stat;

    error = vfs_getattr(&f->f_path, &stat);

    if (error == 0)
    {
        return stat.size;
    }
    else
    {
        pr_err("get file file stat error\n");
        return error;
    }
}

static int cps_file_read(char *filename, char **buf)
{
    struct file *fp;
    mm_segment_t fs;
    int size = 0;
    loff_t pos = 0;

    fp = filp_open(filename, O_RDONLY, 0);
    if (IS_ERR(fp))
    {
        pr_err("open %s file error\n", filename);
        goto end;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);
    size = fp_size(fp);
    if (size <= 0)
    {
        pr_err("load file:%s error\n", filename);
        goto error;
    }

    *buf = kzalloc(size + 1, GFP_KERNEL);
    vfs_read(fp, *buf, size, &pos);

error:
    filp_close(fp, NULL);
    set_fs(fs);
end:
    return size;
}

static unsigned char chartoBcd(char iChar)
{
    unsigned char mBCD = 0;

    if (iChar >= '0' && iChar <= '9')
        mBCD = iChar - '0';
    else if (iChar >= 'A' && iChar <= 'F')
        mBCD = iChar - 'A' + 0x0a;
    else if (iChar >= 'a' && iChar <= 'f')
        mBCD = iChar - 'a' + 0x0a;

    return mBCD;
}

static unsigned char *file_parse(char *buf, int size,
                                 unsigned char *file, int *file_length)
{
    int i = 0, j = 0;
    int file_index = 0;
    char temp;

    if (!buf || !file)
        return NULL;
    for (i = 0; i < size; i++)
    {
        if (buf[i] == '\n' || buf[i] == ' ' || buf[i] == '\r')
        {
            file_index = 0;
            continue;
        }
        else
        {
            if (file_index == 1)
            {
                file_index++;
                file[j++] = (unsigned char)((chartoBcd(temp) << 4) + chartoBcd(buf[i]));
            }
            else if (file_index == 0)
            {
                file_index++;
                temp = buf[i];
            }
        }
    }
    // file[j] = '\0';
    *file_length = j;

    return file;
}


static int bootloader_load(unsigned char *bootloader, int *bootloader_length)
{
    int ret = 0;
    char *buf = NULL;
    int size = 0;
    cps_wls_log(CPS_LOG_DEBG, "%s: start load single byte hex bootloader(%s)\n", __func__, BOOTLOADER_FILE_NAME);
    size = cps_file_read(BOOTLOADER_FILE_NAME, &buf);

    if(size > 0)
    {

        if(bootloader == NULL)
        {
            kfree(buf);
            cps_wls_log(CPS_LOG_ERR, "[%s] hex file alloc error.\n", __func__);
            return -EINVAL;
        }

        if(file_parse(buf, size, bootloader, bootloader_length) == NULL)
        {
            kfree(buf);
            cps_wls_log(CPS_LOG_ERR, "[%s] hex file parse error\n", __func__);
            return -EINVAL;
        }

        kfree(buf);
    }


    return 0;
}


static int firmware_load(unsigned char *firmeware, int *firmeware_length)
{
    int ret = 0;
    char *buf = NULL;
    int size = 0;
    cps_wls_log(CPS_LOG_DEBG, "%s: start load single byte hex firmeware(%s)\n", __func__, FIRMWARE_FILE_NAME);
    size = cps_file_read(FIRMWARE_FILE_NAME, &buf);

    if(size > 0)
    {

        if(firmeware == NULL)
        {
            kfree(buf);
            cps_wls_log(CPS_LOG_ERR, "[%s] hex file alloc error.\n", __func__);
            return -EINVAL;
        }

        if(file_parse(buf, size, firmeware, firmeware_length) == NULL)
        {
            kfree(buf);
            cps_wls_log(CPS_LOG_ERR, "[%s] hex file parse error\n", __func__);
            return -EINVAL;
        }

        kfree(buf);
    }

    return 0;
}
#else

static int bootloader_load(unsigned char *bootloader, int *bootloader_length)
{
    int ret = 0;
    const struct firmware *bl_data_bin;

    cps_wls_log(CPS_LOG_DEBG, "%s: start load bin bootloader(%s)\n", __func__, CPS_WLS_BL_BIN_PATH);
    ret = request_firmware(&bl_data_bin, CPS_WLS_BL_BIN_PATH, &chip->client->dev);
    if(ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "%s: failed to request bin bootloader %s (%d)\n", __func__, CPS_WLS_FW_BIN_PATH, ret);
        return -EINVAL;
    }

    *bootloader_length = (int)bl_data_bin->size;
    memcpy(bootloader, bl_data_bin->data, bl_data_bin->size);

	//kcm added by phf, 20241110, fixbug
	if(	bl_data_bin != NULL)
		release_firmware(bl_data_bin);
		
    return 0;
}


static int firmware_load(unsigned char *firmeware, int *firmeware_length)
{
    int ret = 0;
    const struct firmware *firm_data_bin;

    cps_wls_log(CPS_LOG_ERR, " %s start load bin firmeware %s \n", __func__, CPS_WLS_FW_BIN_PATH);
    ret = request_firmware(&firm_data_bin, CPS_WLS_FW_BIN_PATH, &chip->client->dev);

    if(ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] failed to request bin firmware %s (%d)\n", __func__, CPS_WLS_FW_BIN_PATH, ret);
        return -EINVAL;
    }

    *firmeware_length = (int)firm_data_bin->size;
    memcpy(firmeware, firm_data_bin->data, firm_data_bin->size);

	//kcm added by phf, 20241110, fixbug
	if(	firm_data_bin != NULL)
		release_firmware(firm_data_bin);

    return 0;
}
#endif


static int cps_wls_write_multi_register_i2c_transfer(struct i2c_msg *regs, int num_regs)
{
    int ret = 0;

    mutex_lock(&chip->i2c_lock);    
    ret = i2c_transfer(chip->client->adapter, regs, num_regs);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }

    return CPS_WLS_SUCCESS;
}

u8 wbuf_high_addr_4000[] = {0xFF, 0x82, 0x00, 0x40};
u8 wbuf_unmask_all[] = {0xE0, 0x08, 0xFF, 0xFF};
u8 wbuf_anaglog_password[] = {0xE7, 0x5C, 0x50, 0x12};
u8 wbuf_i2c_timeout[] = {0xE0, 0x04, 0x1D};
u8 wbuf_high_addr_2000[] = {0xFF, 0x82, 0x00, 0x20};
u8 wbuf_high_addr_4004[] = {0xFF, 0x82, 0x04, 0x40};
u8 wbuf_remap[] = {0x00, 0xA0, 0xFF};
u8 wbuf_disable_trim[] = {0x00, 0x11, 0x80};
u8 wbuf_restart[] = {0xFF, 0x80, 0x00};

static int cps_wls_write_unmask_all(void)
{
    struct i2c_msg msg_unmask_all[] = {
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_4000,
            .len = ARRAY_SIZE(wbuf_high_addr_4000),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_unmask_all,
            .len = ARRAY_SIZE(wbuf_unmask_all),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_2000,
            .len = ARRAY_SIZE(wbuf_high_addr_2000),
        },
    };

    return cps_wls_write_multi_register_i2c_transfer(msg_unmask_all, ARRAY_SIZE(msg_unmask_all));
}

static int cps_wls_write_anaglog_password(void)
{
    struct i2c_msg msg_anaglog_password[] = {
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_4000,
            .len = ARRAY_SIZE(wbuf_high_addr_4000),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_anaglog_password,
            .len = ARRAY_SIZE(wbuf_anaglog_password),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_2000,
            .len = ARRAY_SIZE(wbuf_high_addr_2000),
        },
    };

    return cps_wls_write_multi_register_i2c_transfer(msg_anaglog_password, ARRAY_SIZE(msg_anaglog_password));
}


static int cps_wls_write_i2c_timeout(void)
{
    struct i2c_msg msg_i2c_timeout[] = {
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_4000,
            .len = ARRAY_SIZE(wbuf_high_addr_4000),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_i2c_timeout,
            .len = ARRAY_SIZE(wbuf_i2c_timeout),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_2000,
            .len = ARRAY_SIZE(wbuf_high_addr_2000),
        },
    };

    return cps_wls_write_multi_register_i2c_transfer(msg_i2c_timeout, ARRAY_SIZE(msg_i2c_timeout));
}

static int cps_wls_write_remap_restart(void)
{
    struct i2c_msg msg_remap_restart[] = {
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_high_addr_4004,
            .len = ARRAY_SIZE(wbuf_high_addr_4004),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_remap,
            .len = ARRAY_SIZE(wbuf_remap),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_disable_trim,
            .len = ARRAY_SIZE(wbuf_disable_trim),
        },
        {
            .addr = chip->client->addr,
            .flags = chip->client->flags & I2C_M_TEN,
            .buf = wbuf_restart,
            .len = ARRAY_SIZE(wbuf_restart),
        },
    };

    return cps_wls_write_multi_register_i2c_transfer(msg_remap_restart, ARRAY_SIZE(msg_remap_restart));
}

static int load_and_update_firmware(void)
{
    int ret, i;
	//int boot_ret;
    int firmware_length;
    int bootloader_length; // return value
    int buf0_flag = 0, buf1_flag = 0;
    unsigned char *bootloader_buf;
    unsigned char *firmware_buf;
    // unsigned char *p;
    int result;
    int cfg_buf_size;
    int addr;

    cps_wls_log(CPS_LOG_DEBG, "wireless %s enter\n", __func__);

    bootloader_buf = kzalloc(0x800, GFP_KERNEL); // 2K buffer
    firmware_buf = kzalloc(0x8000, GFP_KERNEL);  // 32K buffer
	
	// kcm added by phf, 20241221, fixbug
	if(bootloader_buf == NULL || firmware_buf == NULL)
	{
		cps_wls_log(CPS_LOG_ERR, "kzalloc bootloader_buf or firmware_buf FAIL!\n");	
		goto update_fail;
	}
	
    /***************************************************************************************
     *                                  Step1, load to sram                                *
     ***************************************************************************************/
    ret = bootloader_load(bootloader_buf, &bootloader_length);//load bootloader

    if(ret != 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] ---- bootloader get error %d\n", __func__, ret);
        goto update_fail;
    }

    msleep(10);
    cps_wls_write_nbyte(0xFF84,0x7A8B,2);  /*write password*/

    /*MCU reset and unmask all address*/
    cps_wls_write_nbyte(0xFF80,0x08,1);

    cps_wls_write_unmask_all();

    msleep(10);
    /*Write analog register password*/
    cps_wls_write_anaglog_password();

    /*Set the I2C timeout to 1s*/
    cps_wls_write_i2c_timeout();

    /*Write the bootloader code to the SRAM*/
    cps_wls_write_nbyte(0xFF82,0x2000,2);                          
    ret = cps_wls_program_sram(0x0000, bootloader_buf, 0x800);    /*Write the bootloader code to the SRAM*/

    if(ret == CPS_WLS_FAIL)
    { 
        cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM BOOTLOADER FAIL!\n", __func__);
         goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM BOOTLOADER!\n", __func__);

    /*Enable remap function*/
    cps_wls_write_remap_restart();


    /***************************************************************************************
     *                          Step2, bootloader crc check                                *
     ***************************************************************************************/

    msleep(10);
    cps_wls_write_nbyte(0xFF84,0x7A8B,2);  /*write password*/

    /*unmask all address*/
    cps_wls_write_unmask_all();

    /*Set the I2C timeout to 1s*/
    cps_wls_write_i2c_timeout();

    cps_wls_program_cmd_send(CACL_CRC_TEST);/*Enable Bootloader verification*/
    result = cps_wls_program_wait_cmd_done();

    if (result != PASS)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s]  ---> BOOTLOADER CRC FAIL\n", __func__);
        goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s]  ---> LOAD BOOTLOADER SUCCESSFUL\n", __func__);

    /***************************************************************************************
     *                          Step3, load firmware to MTP                                *
     ***************************************************************************************/
    memset(firmware_buf, 0, 0x8000);
    ret = firmware_load(firmware_buf, &firmware_length); // load bootloader

    if (ret != 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] ---- firmware get error %d\n", __func__, ret);
        goto update_fail;
    }

    cps_wls_log(CPS_LOG_DEBG, "[%s]  ---> START LOAD APP FIRMWARE \n", __func__);
    buf0_flag = 0;
    buf1_flag = 0;
    cfg_buf_size = 256;
    // cfg_buf_size = 1024;
    addr = 0;   
    cps_wls_write_nbyte(0xFF82,0x2000,2); 
    cps_wls_write_nbyte(ADDR_BUF_SIZE, cfg_buf_size, 4);

    /*ERASER MTP*/ 
    cps_wls_program_cmd_send(PGM_ERASER_0);

    result = cps_wls_program_wait_cmd_done();
    if (result != PASS)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s]  ---> ERASE MTP FAIL\n", __func__);
        goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s]  ---> ERASE MTP SUCCESSFUL\n", __func__);
    for (i = 0; i < (32 * 1024) / 4 / cfg_buf_size; i++)
    //  for (i = 0; i < (32 * 1024) /cfg_buf_size; i++)
    {
        if (buf0_flag == 0)
        {
    
            cps_wls_program_sram(ADDR_BUFFER0, firmware_buf + addr, cfg_buf_size * 4);
            addr = addr + cfg_buf_size * 4;

            if (buf1_flag == 1)
            {
                result = cps_wls_program_wait_cmd_done();
                if (result != PASS)
                {
                    pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
                    goto update_fail;
                }
                buf1_flag = 0;
            }
            cps_wls_program_cmd_send(PGM_BUFFER0);
            buf0_flag = 1;
            continue;
        }

        if (buf1_flag == 0)
        {             
            cps_wls_program_sram(ADDR_BUFFER1, firmware_buf + addr, cfg_buf_size * 4);
            addr = addr + cfg_buf_size * 4;

            if (buf0_flag == 1)
            {
                result = cps_wls_program_wait_cmd_done();
                if (result != PASS)
                {
                    pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
                    goto update_fail;
                }
                buf0_flag = 0;
            }
            cps_wls_program_cmd_send(PGM_BUFFER1);
            buf1_flag = 1;
            continue;
        }
    }

    if (buf0_flag == 1)
    {
        result = cps_wls_program_wait_cmd_done();
        if (result != PASS)
        {
            pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
            goto update_fail;
        }
        buf0_flag = 0;
    }

    if (buf1_flag == 1)
    {
        result = cps_wls_program_wait_cmd_done();
        if (result != PASS)
        {
            pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
            goto update_fail;
        }
        buf1_flag = 0;
    }
    cps_wls_log(CPS_LOG_DEBG, "%s: ---> WRITE APP FIRMWARE SUCCESSFUL\n", __func__);
    msleep(10);
    /***************************************************************************************
     *                          Step4, check app CRC                                       *
     ***************************************************************************************/
    cps_wls_program_cmd_send(CACL_CRC_APP);
    result = cps_wls_program_wait_cmd_done();
    if (result != PASS)
    {
        cps_wls_log(CPS_LOG_ERR, "%s: ---> APP CRC FAIL\n", __func__);
        goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "%s: ---> CHERK APP FIRMWARE CRC SUCCESSFUL\n", __func__);
    /***************************************************************************************
     *                          Step5, write mcu start flag                                *
     ***************************************************************************************/
    cps_wls_program_cmd_send(SYS_RESET); /*reset all system*/
    msleep(100);

    cps_wls_log(CPS_LOG_DEBG, "%s: --->  SUCCESSFUL COMPLETION\n", __func__);
	
	// kcm added by phf, 20241221, fixbug
	if(bootloader_buf != NULL)
		kfree(bootloader_buf);
	if(firmware_buf != NULL)
		kfree(firmware_buf);
	
    return CPS_WLS_SUCCESS;

update_fail:
    cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);
	// kcm added by phf, 20241221, fixbug
	if(bootloader_buf != NULL)
		kfree(bootloader_buf);
	if(firmware_buf != NULL)
		kfree(firmware_buf);
	
    return CPS_WLS_FAIL;
}

//use CPS4021_BL, CPS4021_FW to update
static int update_firmware(void)
{
    int ret, i;
	//int boot_ret;
    int firmware_length;
    int bootloader_length; // return value
    int buf0_flag = 0, buf1_flag = 0;
    unsigned char *bootloader_buf;
    unsigned char *firmware_buf;
	
    // unsigned char *p;
    int result;
    int cfg_buf_size;
    int addr;

    cps_wls_log(CPS_LOG_DEBG, "%s enter\n", __func__);
    bootloader_buf = kzalloc(0x800, GFP_KERNEL); // 2K buffer
    firmware_buf = kzalloc(0x8000, GFP_KERNEL);  // 32K buffer
	
	// kcm added by phf, 20241221, fixbug
	if(bootloader_buf == NULL || firmware_buf == NULL)
	{
		cps_wls_log(CPS_LOG_ERR, "kzalloc bootloader_buf or firmware_buf FAIL!\n");	
		goto update_fail;
	}
	
    /***************************************************************************************
     *                                  Step1, load to sram                                *
     ***************************************************************************************/
	 
	//load bootloader in buff
	bootloader_length = sizeof(CPS4021_BL);
	if(bootloader_length > 0x800)
		bootloader_length = 0x800;
	memcpy(bootloader_buf, CPS4021_BL, bootloader_length);

	msleep(10);
    cps_wls_write_nbyte(0xFF84,0x7A8B,2);  /*write password*/

    /*MCU reset and unmask all address*/
    cps_wls_write_nbyte(0xFF80,0x08,1);

    cps_wls_write_unmask_all();

    msleep(10);
    /*Write analog register password*/
    cps_wls_write_anaglog_password();

    /*Set the I2C timeout to 1s*/
    cps_wls_write_i2c_timeout();

	
    /*Write the bootloader code to the SRAM*/
    cps_wls_write_nbyte(0xFF82,0x2000,2);                          
    ret = cps_wls_program_sram(0x0000, bootloader_buf, 0x800);    /*Write the bootloader code to the SRAM*/


    if(ret == CPS_WLS_FAIL)
    { 
        cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM BOOTLOADER FAIL!\n", __func__);
         goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM BOOTLOADER!\n", __func__);

    /*Enable remap function*/
    cps_wls_write_remap_restart();

    /***************************************************************************************
     *                          Step2, bootloader crc check                                *
     ***************************************************************************************/

    msleep(10);
    cps_wls_write_nbyte(0xFF84,0x7A8B,2);  /*write password*/

    /*unmask all address*/
    cps_wls_write_unmask_all();

    /*Set the I2C timeout to 1s*/
    cps_wls_write_i2c_timeout();

    cps_wls_program_cmd_send(CACL_CRC_TEST);/*Enable Bootloader verification*/
    result = cps_wls_program_wait_cmd_done();

    if (result != PASS)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s]  ---> BOOTLOADER CRC FAIL\n", __func__);
        goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s]  ---> LOAD BOOTLOADER SUCCESSFUL\n", __func__);

    /***************************************************************************************
     *                          Step3, load firmware to MTP                                *
     ***************************************************************************************/
	//load bootloader in buff
	firmware_length = sizeof(CPS4021_FW);
	if(firmware_length > 0x8000)
		firmware_length = 0x8000;
	memcpy(firmware_buf, CPS4021_FW, firmware_length);

    cps_wls_log(CPS_LOG_DEBG, "[%s]  ---> START LOAD APP FIRMWARE \n", __func__);
    buf0_flag = 0;
    buf1_flag = 0;
    cfg_buf_size = 256;
    // cfg_buf_size = 1024;
    addr = 0;   
    cps_wls_write_nbyte(0xFF82,0x2000,2); 
    cps_wls_write_nbyte(ADDR_BUF_SIZE, cfg_buf_size, 4);

    /*ERASER MTP*/ 
    cps_wls_program_cmd_send(PGM_ERASER_0);

    result = cps_wls_program_wait_cmd_done();
    if (result != PASS)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s]  ---> ERASE MTP FAIL\n", __func__);
        goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s]  ---> ERASE MTP SUCCESSFUL\n", __func__);
    for (i = 0; i < (32 * 1024) / 4 / cfg_buf_size; i++)
    //  for (i = 0; i < (32 * 1024) /cfg_buf_size; i++)
    {
        if (buf0_flag == 0)
        {
    
            cps_wls_program_sram(ADDR_BUFFER0, firmware_buf + addr, cfg_buf_size * 4);
            addr = addr + cfg_buf_size * 4;

            if (buf1_flag == 1)
            {
                result = cps_wls_program_wait_cmd_done();
                if (result != PASS)
                {
                    pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
                    goto update_fail;
                }
                buf1_flag = 0;
            }
            cps_wls_program_cmd_send(PGM_BUFFER0);
            buf0_flag = 1;
            continue;
        }

        if (buf1_flag == 0)
        {             
            cps_wls_program_sram(ADDR_BUFFER1, firmware_buf + addr, cfg_buf_size * 4);
            addr = addr + cfg_buf_size * 4;

            if (buf0_flag == 1)
            {
                result = cps_wls_program_wait_cmd_done();
                if (result != PASS)
                {
                    pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
                    goto update_fail;
                }
                buf0_flag = 0;
            }
            cps_wls_program_cmd_send(PGM_BUFFER1);
            buf1_flag = 1;
            continue;
        }
    }

    if (buf0_flag == 1)
    {
        result = cps_wls_program_wait_cmd_done();
        if (result != PASS)
        {
            pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
            goto update_fail;
        }
        buf0_flag = 0;
    }

    if (buf1_flag == 1)
    {
        result = cps_wls_program_wait_cmd_done();
        if (result != PASS)
        {
            pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
            goto update_fail;
        }
        buf1_flag = 0;
    }
    cps_wls_log(CPS_LOG_DEBG, "%s: ---> WRITE APP FIRMWARE SUCCESSFUL\n", __func__);
    msleep(10);
    /***************************************************************************************
     *                          Step4, check app CRC                                       *
     ***************************************************************************************/
    cps_wls_program_cmd_send(CACL_CRC_APP);
    result = cps_wls_program_wait_cmd_done();
    if (result != PASS)
    {
        pr_err("%s: ---> APP CRC FAIL\n", __func__);
        goto update_fail;
    }
    cps_wls_log(CPS_LOG_DEBG, "%s: ---> CHERK APP FIRMWARE CRC SUCCESSFUL\n", __func__);
    /***************************************************************************************
     *                          Step5, write mcu start flag                                *
     ***************************************************************************************/
    cps_wls_program_cmd_send(SYS_RESET); /*reset all system*/
    msleep(100);

    cps_wls_log(CPS_LOG_DEBG, "%s: --->  SUCCESSFUL COMPLETION\n", __func__);
	
	// kcm added by phf, 20241221, fixbug
	if(bootloader_buf != NULL)
		kfree(bootloader_buf);
	if(firmware_buf != NULL)
		kfree(firmware_buf);
	
    return CPS_WLS_SUCCESS;

update_fail:
    cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);
	
	// kcm added by phf, 20241221, fixbug
	if(bootloader_buf != NULL)
		kfree(bootloader_buf);
	if(firmware_buf != NULL)
		kfree(firmware_buf);	
	
    return CPS_WLS_FAIL;
}

/*********************************************************************************************************
*
*                       I2C APT end
*------------------CPS4021 system interface-------------------
*
*********************************************************************************************************/

static int cps_wls_set_cmd(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_REG_CMD]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static uint16_t cps_wls_get_cmd(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_REG_CMD]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_fun_en(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_FUNC_EN]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static uint16_t cps_wls_get_fun_en(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_FUNC_EN]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

int cps_wls_get_int_flag(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_REG_INT_FLAG]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}
EXPORT_SYMBOL(cps_wls_get_int_flag);

static int cps_wls_set_int_clr(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_REG_INT_CLR]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_chip_id(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CHIP_ID]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_sys_fw_version(void)
{
    cps_reg_s *cps_reg;
    int cps_version = 0;
	int ret;
	
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_FW_MAJOR]);
    ret = cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	if(ret<0)
		return -1;
	
    cps_version =  ret & 0xff;
    
	cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_FW_MINOR]);
    ret = cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	if(ret<0)
		return -1;
	
	cps_version = (cps_version<<8) | ( ret & 0xff);
  
    return cps_version;
}

static int cps_wls_get_sys_mode(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_REG_SYS_MODE]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_crc_val(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_comm_reg[CPS_COMM_REG_CRC_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}


//-------------------CPS4021 RX interface-------------------
#if 0
static int cps_wls_get_rx_ept_code(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_EPT_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

// static int cps_wls_get_rx_neg_pro(void)
//{
//     cps_reg_s *cps_reg;
//     cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_NEGO_PRO]);
//     return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
// }

#endif
#if 0
static int cps_wls_set_rx_ocp_threshold(int value)
{
    if(value < 200|| value > 3300) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_OCP_TH]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_ovp_threshold(int value)
{
    if(value < 0 || value > 15) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_OVP_TH]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_send_command(uint16_t ap_command)
{
    chip->command_flag = 0;
    if(cps_wls_set_cmd(ap_command)!= CPS_WLS_SUCCESS)
    {
        return CPS_WLS_FAIL;
    }

    msleep(10);
    return CPS_WLS_SUCCESS;
}

static int cps_wls_rx_send_ept_packet(ept_reason_e value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_EPT_VAL]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);

    if(CPS_WLS_SUCCESS == cps_wls_write_reg((int)cps_reg.reg_addr, value, (int)cps_reg.reg_bytes_len))
    {
        if(CPS_WLS_SUCCESS == cps_wls_send_command(RX_CMD_SEND_EPT));
        {
            return CPS_WLS_SUCCESS;
        }
    }
    return CPS_WLS_FAIL;
}

static int cps_wls_set_rx_dummy_load_mod_val(int value)
{
    if(value < 0 || value > 31)  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_DUMY_LOAD_MOD]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_dummy_load_no_mod_val(int value)
{
    if(value < 0 || value > 31)  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_DUMY_LOAD_NO_MOD]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_fc_vpa_voltage(int value)
{
    if(value < 3500 || value > 20000)  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_FC_VPA_VOLTAGE]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_fc_mldo_voltage(int value)
{
    if(value < 3500 || value > 20000)  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_FC_MLDO_VOLTAGE]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_fc_boost_mode(int value)
{
    if(value < 0 )  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_FC_BOOST_MODE]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_max_power(int value)
{
    if(value < 0 || value > 80)  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_POWER_SET]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}



static int cps_wls_send_handshake_packet(uint8_t *data, uint8_t data_len)
{
       return cps_wls_send_ask_packet( data, data_len);    
}

#endif


static int cps_wls_get_rx_ss_pkt_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_SS_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}


static int cps_wls_get_rx_ce_pkt_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_CE_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_rp_pkt_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_RP_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_fop_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_FOP_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_vrect(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_rx_reg[CPS_RX_REG_ADC_VRECT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_mldo_drop(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_rx_reg[CPS_RX_REG_ADC_MLDO_DROP]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_irect(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_rx_reg[CPS_RX_REG_ADC_IOUT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_vout(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_rx_reg[CPS_RX_REG_ADC_VOUT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_fod_para(void)
{
    uint8_t i;

    const uint8_t FOP_CUR[7] =
        {
            // unit 10mA
            25 * 1, // C0
            25 * 2, // C1
            25 * 3, // C2
            25 * 4, // C3
            25 * 5, // C4
            25 * 6, // C5
            25 * 7, // C6
        };

    const uint8_t FOP_GAIN_OFFSET[8 * 2] =
        {
            // gain(0.01)     offset(40mW)
            // 5V
            58, 6, // C0
            58, 6, // C1
            58, 6, // C2
            58, 6, // C3
            58, 6, // C4
            58, 6, // C5
            58, 6, // C6
            58, 6, // C7
        };

    for (i = 0; i < 7; i++)
    {
        if (cps_wls_write_reg((int)(RX_REG_FOD_CUR_0 + i), FOP_CUR[i], 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    for (i = 0; i < 16; i++)
    {
        if (cps_wls_write_reg((int)(RX_REG_FOD_C0_GAIN + i), FOP_GAIN_OFFSET[i], 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    return CPS_WLS_SUCCESS;
}

static int cps_wls_set_rx_vout_target(int value)
{
    cps_reg_s *cps_reg;
    if(value < 3500 || value > 240000) 
    {
        return CPS_WLS_FAIL;
    }
    
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_VOUT_SET]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_die_tmp(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_DIE_TMP]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_send_ask_packet(uint8_t *data, uint8_t data_len)
{
  
    uint16_t cmd;
    uint8_t i;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_PPP_HEADER]);


    for(i = 0; i < data_len; i++)
    {
        if(cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    cmd = cps_wls_get_cmd();
    cmd |= RX_CMD_SEND_DATA;
    return cps_wls_set_cmd(cmd);
}

//-------------------CPS4021 TX interface-------------------
static int cps_wls_get_tx_ept_rsn(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_EPT_RSN]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_func_en(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_FUNC_EN]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_func_en(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_FUNC_EN]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_ipa(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_ADC_IPA]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_vin(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_ADC_VIN]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_vpa(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_ADC_VRECT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power smaller than RP0 threshold(0x0252),FOD ploss trigger threshold setting
 * @note
 * @param  None
 * @retval
 */
// static int cps_wls_set_tx_fod0_thresh(int value)
// {
//     cps_reg_s *cps_reg;
//     cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_FOD0_TH]);
//     return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
// }

static int cps_wls_enable_tx_mode(void)
{
    uint16_t cmd;
    cmd = cps_wls_get_cmd();
    cmd |= TX_CMD_ENTER_TX_MODE;
    return cps_wls_set_cmd(cmd);
}

static int cps_wls_disable_tx_mode(void)
{
    uint16_t cmd;
    cmd = cps_wls_get_cmd();
    cmd |= TX_CMD_ENTER_BP_MODE;
    return cps_wls_set_cmd(cmd);
}

static int cps_wls_send_fsk_packet(uint8_t *data, uint8_t data_len)
{
    uint16_t cmd;
    uint8_t i;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_PPP_HEADER]);

    for (i = 0; i < data_len; i++)
    {
        if (cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    cmd = cps_wls_get_cmd();
    cmd |= TX_CMD_SEND_FSK;
    return cps_wls_set_cmd(cmd);
}

uint8_t cps_wls_get_message_size(uint8_t header)
{
    if (header < 0x20)
    {
        return 1;
    }
    else if (header < 0x80)
    {
        return header / 16;
    }
    else if (header < 0xE0)
    {
        return header / 8 - 8;
    }
    else
    {
        return header / 4 - 36;
    }
}

static int cps_wls_get_fsk_packet(uint8_t *data)
{
    int temp;
    uint8_t i;
    uint8_t data_len;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_rx_reg[CPS_RX_REG_BC_HEADER]);

    /*get header*/
    temp = cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
   
    if (temp != CPS_WLS_FAIL)
    {
        *data = temp;
        data_len = cps_wls_get_message_size(*data);
    }
    else
    {
        return CPS_WLS_FAIL;
    }

    for (i = 0; i < data_len; i++)
    {
        temp = cps_wls_read_reg((int)(cps_reg->reg_addr + 1 + i), (int)cps_reg->reg_bytes_len);

        if (temp != CPS_WLS_FAIL)
        {
            *(data + 1 + i) = temp;
        }
        else
        {
            return CPS_WLS_FAIL;
        }
    }

    return CPS_WLS_SUCCESS;
}

static int cps_wls_get_ask_packet(uint8_t *data)
{
    int temp;
    uint8_t i;
    uint8_t data_len;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s *)(&cps_tx_reg[CPS_TX_REG_BC_HEADER]);

    /*get header*/
    temp = cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
    if (temp != CPS_WLS_FAIL)
    {
        *data = temp;
        data_len = cps_wls_get_message_size(*data);
    }
    else
    {
        return CPS_WLS_FAIL;
    }

    for (i = 0; i < data_len; i++)
    {
        temp = cps_wls_read_reg((int)(cps_reg->reg_addr + 1 + i), (int)cps_reg->reg_bytes_len);
        if (temp != CPS_WLS_FAIL)
        {
            *(data + 1 + i) = temp;
        }
        else
        {
            return CPS_WLS_FAIL;
        }
    }

    return CPS_WLS_SUCCESS;
}

static int cps_wls_get_tx_freq(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOP_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_die_tmp(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_DIE_TEMP]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_ce_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_CE_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_rp_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_RP_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_ept_code(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_EPT_CODE]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

#if 0
static int cps_wls_set_tx_ocp_threshold(int value)
{
    if(value < 1 || value > 4000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_OCP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ovp_threshold(int value)
{
    if(value < 1 || value > 13000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_OVP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_fop_min(int value)
{
    if(value < 1 || value > 255) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOP_MIN]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_fop_max(int value)
{
    if(value < 1 || value > 255) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOP_MAX]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ping_frequency(int value)
{
    if(value < 1 || value > 255) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PING_FREQ]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_huge_metal_threshold(int value)
{
    if(value < 1 || value > 4000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PING_OCP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}
#endif
//------------------------------IRQ Handler-----------------------------------
static int cps_wls_set_int_enable(void)
{
    uint16_t int_en;
    cps_reg_s *cps_reg;

    int_en = 0xFFFF;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_EN]);

    if(CPS_WLS_FAIL == cps_wls_write_reg((int)cps_reg->reg_addr, int_en, (int)cps_reg->reg_bytes_len))  goto set_int_fail;
    return CPS_WLS_SUCCESS;

set_int_fail:
    return CPS_WLS_FAIL;
}

static int cps_wls_get_fsk_pkt_event(uint8_t *data, uint8_t cnt)
{
        //int i;
        if(CPS_WLS_SUCCESS == cps_wls_get_fsk_packet(data))
        {
            //pr_err(" getfsk data : \n");
            //for ( i = 0; i < cnt; i++)
            //{
            //    pr_err(" 0x%x\n", data[i]);
            //}
            return CPS_WLS_SUCCESS;
        }
        return CPS_WLS_FAIL;
}

static int cps_wls_get_ask_pkt_event(uint8_t *data, uint8_t cnt)
{
        int i;
        if(CPS_WLS_SUCCESS == cps_wls_get_ask_packet(data))
        {
            pr_err("get ask data : \n");
            for ( i = 0; i < cnt; i++)
            {
                pr_err(" 0x%x\n", data[i]);
            }
            return CPS_WLS_SUCCESS;
        }
        return CPS_WLS_FAIL;
}

static int cps_wls_rx_irq_handler(int int_flag)
{
    int rc = 0;
    uint8_t data[8] = {0};
    if (int_flag & RX_INT_POWER_ON)
    {
    }
    if (int_flag & RX_INT_MLDO_OFF)
    {
    }
    if (int_flag & RX_INT_MLDO_ON)
    {
    }
    if (int_flag & RX_INT_READY)
    {
    }
    if (int_flag & RX_INT_FSK_TIMEOUT)
    {
    }
    if (int_flag & RX_INT_FSK_PKT)
    {
        cps_wls_get_fsk_pkt_event(data, sizeof(data));
    }
    if (int_flag & RX_INT_HEAVY_LOAD)
    {
    }
    if (int_flag & RX_INT_LIGHT_LOAD)
    {
    }
    if (int_flag & RX_INT_VRECT_OVP)
    {
    }
    if (int_flag & RX_INT_VRECT_OVP_TO)
    {
    }
    if (int_flag & RX_INT_VRECT_OVP_BVP)
    {
    }
    if (int_flag & RX_INT_CHIP_OTP)
    {
    }
    if (int_flag & RX_INT_CHIP_HTP)
    {
    }
    if (int_flag & RX_INT_MLDO_OCP)
    {
    }
    if (int_flag & RX_INT_MLDO_HOCP)
    {
    }
    if (int_flag & RX_INT_MLDO_OPP)
    {
    }
    if (int_flag & RX_INT_MLDO_UVP)
    {
    }
    if (int_flag & RX_INT_MLDO_OVP)
    {
    }
    if (int_flag & RX_INT_AC_LOSS)
    {
    }
    if (int_flag & RX_INT_SR_BR_SW_FAIL)
    {
    }
    if (int_flag & RX_INT_SR_BR_SW_SUCC)
    {
    }
    if (int_flag & RX_INT_START_OV)
    {
    }

    return rc;
}

static int cps_wls_tx_irq_handler(int int_flag)
{
    int rc = 0;
    uint8_t data[8] = {0};

    if (int_flag & TX_INT_PING)
    {
        // todo
    }
    if (int_flag & TX_INT_SSP)
    {
        // todo
    }
    if (int_flag & TX_INT_INIT_DONE)
    {
    }
    if (int_flag & TX_INT_IDP)
    {
    }
    if (int_flag & TX_INT_CFGP)
    {
    }
    if (int_flag & TX_INT_EPT)
    {
    }
    if (int_flag & TX_INT_AC_DET)
    {
    }
    if (int_flag & TX_INT_HTP)
    {
    }
    if (int_flag & TX_INT_BR_H_T_F)
    {
    }
    if (int_flag & TX_INT_BR_F_T_H)
    {
    }
    if (int_flag & TX_INT_LP_END)
    {
    }

    // if(int_flag & TX_INT_NRG_WEAK){}
    // if(int_flag & TX_INT_NRG_STRONG) {}
    if (int_flag & TX_INT_PVT_ASK)
    {
        
      cps_wls_get_ask_pkt_event(data,sizeof(data));
    }

    return rc;
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
    int int_flag;
    int int_clr;
    cps_wls_log(CPS_LOG_DEBG, "[%s] IRQ triggered\n", __func__);
    mutex_lock(&chip->irq_lock);
    int_flag = cps_wls_get_int_flag();
    cps_wls_log(CPS_LOG_DEBG, ">>>>>int_flag = 0x%x, mldo pin = %d\n", int_flag, wls_mod0_gp1_get());
    if (int_flag == CPS_WLS_FAIL)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] read wls irq reg failed\n", __func__);
        mutex_unlock(&chip->irq_lock);
        return IRQ_HANDLED;
    }

    int_clr = int_flag;
    cps_wls_set_int_clr(int_flag);
    mutex_unlock(&chip->irq_lock);
    if (cps_wls_get_sys_mode() == SYS_MODE_RX)
    {
        cps_wls_rx_irq_handler(int_flag);
    }
    else
    {
        cps_wls_tx_irq_handler(int_flag);
    }
    return IRQ_HANDLED;
}

static enum power_supply_property cps_wls_chrg_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CALIBRATE,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
};


static int cps_wls_chrg_property_is_writeable(struct power_supply *psy,
                                              enum power_supply_property psp)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_CURRENT_MAX:
    //case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        return 1;

    default:
        break;
    }

    return 0;
}

static int cps_wls_chrg_get_property(struct power_supply *psy,
                                     enum power_supply_property psp,
                                     union power_supply_propval *val)
{
    //int ret;
    switch (psp)
    {
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = 0;
        break;
/*
    case POWER_SUPPLY_PROP_VRECT:
        ret = cps_wls_get_rx_vrect();
        if (ret != CPS_WLS_FAIL)
        {
            chip->rx_vrect = ret;
        }
        val->intval = chip->rx_vrect;
        break;
*/

/*
    case POWER_SUPPLY_PROP_IRECT:
        ret = cps_wls_get_rx_irect();
        if (ret != CPS_WLS_FAIL)
        {
            chip->rx_irect = ret;
        }
        val->intval = chip->rx_irect;
        break;
*/

/*
    case POWER_SUPPLY_PROP_PROTOCOL:
        // ret = cps_wls_get_rx_neg_pro();
        // if(ret != CPS_WLS_FAIL)
        // {
        //     chip->rx_neg_protocol = cps_wls_get_rx_neg_pro();
        // }
        // val->intval = chip->rx_neg_protocol;
        break;
*/
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static int cps_wls_chrg_set_property(struct power_supply *psy,
                                     enum power_supply_property psp,
                                     const union power_supply_propval *val)
{
    int ret = 0;
    struct cps_wls_chrg_chip *chip = power_supply_get_drvdata(psy);
    cps_wls_log(CPS_LOG_DEBG, "[%s] psp = %d.\n", __func__, psp);
    chip->state = 1;
    return ret;
}

static void cps_wls_charger_external_power_changed(struct power_supply *psy)
{
    ;
}

/*********************************************************************************************************
*
*  COMME  DEVICE ATTR 
*
*********************************************************************************************************/
//-----------------------------show_comme_cmd-----------------------------------------------
static ssize_t show_comme_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "CMD set 0x%x\n", cps_wls_get_cmd());
}
static DEVICE_ATTR(get_cmd, 0444, show_comme_cmd, NULL);
//----------------------set_comme_cmd------------------------------------------
static ssize_t set_comme_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    cps_wls_set_cmd(tmp);
    return count;
}
static DEVICE_ATTR(set_cmd, 0664, NULL, set_comme_cmd);
//-----------------------------show_comme_fun_en-----------------------------------------------
static ssize_t show_comme_fun_en(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Fun_en :0x%x\n", cps_wls_get_fun_en());
}
static DEVICE_ATTR(get_fun_en, 0444, show_comme_fun_en, NULL);
//----------------------set_comme_fun_en------------------------------------------
static ssize_t set_comme_fun_en(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    cps_wls_set_fun_en(tmp);
    return count;
}
static DEVICE_ATTR(set_fun_en, 0664, NULL, set_comme_fun_en);

//----------------------set_int_enable------------------------------------------
static ssize_t set_int_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(cps_wls_set_int_enable() == 0) 
    {
        pr_err("enable irq set success\n");
        return CPS_WLS_SUCCESS;
    }
    else
    {
        pr_err("enable irq failed !\n");
        return CPS_WLS_FAIL;
    }
}
static DEVICE_ATTR(sets_int_enable, 0444, set_int_enable, NULL);
//-----------------------------reg addr----------------------------------
static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "reg addr 0x%08x\n", chip->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    chip->reg_addr = tmp;
    return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);
//-----------------------------reg data----------------------------------
static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    chip->reg_data = cps_wls_read_reg(chip->reg_addr, 4);
    return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", chip->reg_addr, chip->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp;

    tmp = simple_strtoul(buf, NULL, 0);
    chip->reg_data = tmp;
    cps_wls_write_reg(chip->reg_addr, chip->reg_data, 4);

    return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);
//---------------------------- store_update_fw--------------------------------------------------
static ssize_t store_update_fw(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /*
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    if (tmp != 0)
    {
        cps_wls_log(CPS_LOG_DEBG, "[%s] -------start update fw\n", __func__);
        load_and_update_firmware();
    }
    return count;
    */
    int i = 0;
    cps_wls_log(CPS_LOG_DEBG, "[%s] -------start update fw\n", __func__);
    for(i = 0; i < 3; i++){
		if(load_and_update_firmware() == 0){
			cps_wls_log(CPS_LOG_DEBG,"update_firmware success\n");
			break;
		}else{
			cps_wls_log(CPS_LOG_DEBG,"update_firmware failed\n");
		}
	}	
    return count;
}
static DEVICE_ATTR(update_fw, 0664, NULL, store_update_fw);
/*********************************************************************************************************
*
*   RX DEVICE ATTR
*
*********************************************************************************************************/
//-----------------------------show_rx_rp_value------------------------------------------------
static ssize_t show_fsk_pkt(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t data[4];
    memset(data, 0, sizeof(data));
    cps_wls_get_fsk_pkt_event(data,sizeof(data));
    return sprintf(buf, " data0: 0x%x\n data1: 0x%x\n data2: 0x%x\n data3: 0x%x\n",data[0],data[1],data[2],data[3]);
}
static DEVICE_ATTR(get_rx_fsk_pkt, 0444, show_fsk_pkt, NULL);
//-----------------------------rx_send_ask_packet------------------------------------------------
static ssize_t set_rx_send_ask_packet(struct device *dev, struct device_attribute *attr,char *buf)
{
    uint8_t data[4];
    data[0] = 0x38;
    data[1] = 0x12;
    cps_wls_send_ask_packet(data,2);
    return sprintf(buf, "Rx send : 0x%x 0x%x\n",0x38,0x12);
}
static DEVICE_ATTR(rx_send_ask_packet_example, 0444, set_rx_send_ask_packet, NULL);
//--------------------------------rx_rp_value-------------------------------------------------
static ssize_t show_rx_rp_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx rp: 0x%x\n", cps_wls_get_rx_rp_pkt_value());
}
static DEVICE_ATTR(get_rx_rp_value, 0444, show_rx_rp_value, NULL);
//-------------------------------rx_fop_value-------------------------------------------------
static ssize_t show_rx_fop_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx fop: 0x%x\n", cps_wls_get_rx_fop_value());
}
static DEVICE_ATTR(get_rx_fop_value, 0444, show_rx_fop_value, NULL);
//----------------------------------rx_ce_value-----------------------------------------------
static ssize_t show_rx_ce_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx ce: 0x%x\n", cps_wls_get_rx_ce_pkt_value());
}
static DEVICE_ATTR(get_rx_ce_value, 0444, show_rx_ce_value, NULL);
//---------------------------------rx_ss_value------------------------------------------------
static ssize_t show_rx_ss_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx ss: 0x%x\n", cps_wls_get_rx_ss_pkt_value());
}
static DEVICE_ATTR(get_rx_ss_value, 0444, show_rx_ss_value, NULL);
//----------------------------------show_rx_die_tmp-------------------------------------------
static ssize_t show_rx_die_tmp(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx die tmp: 0x%x\n", cps_wls_get_rx_die_tmp());
}
static DEVICE_ATTR(get_rx_die_tmp, 0444, show_rx_die_tmp, NULL);
//------------------------------- set&get _rx_vout_target-----------------------------------------
static ssize_t set_rx_vout_target(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    if(tmp <3500 || tmp >24000)
    {
        return count;
    }
    cps_wls_set_rx_vout_target(tmp);
    return count;
}
static DEVICE_ATTR(set_rx_vout, 0664, NULL, set_rx_vout_target);
//------------------------------show_rx_irect------------------------------------------------
static ssize_t show_rx_irect(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx irect = %dmA\n", cps_wls_get_rx_irect());
}
static DEVICE_ATTR(get_rx_irect, 0444, show_rx_irect, NULL);
//------------------------------show_rx_vrect------------------------------------------------
static ssize_t show_rx_vrect(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx vrect = %dmV\n", cps_wls_get_rx_vrect());
}
static DEVICE_ATTR(get_rx_vrect, 0444, show_rx_vrect, NULL);
//------------------------------show_rx_mldo_drop------------------------------------------------
static ssize_t show_rx_mldo_drop(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx mldo drop = %dmV\n", cps_wls_get_rx_mldo_drop());
}
static DEVICE_ATTR(get_rx_mldo_drop, 0444, show_rx_mldo_drop, NULL);
//------------------------------show_rx_vout------------------------------------------------
static ssize_t show_rx_vout(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "rx vout = %dmV\n", cps_wls_get_rx_vout());
}
static DEVICE_ATTR(get_rx_vout, 0444, show_rx_vout, NULL);
//-----------------------execute_rx_test_process-------------------------------------
static ssize_t execute_rx_test_process(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"rx rp: 0x%x\nrx fop: 0x%x\nrx ce: 0x%x\nrx ss: 0x%x\nrx die tmp: 0x%x\nrx irect = %dmA\nrx vrect = %dmV\nrx vout = %dmV\nrx mldo drop = %dmV\n",\
                    cps_wls_get_rx_rp_pkt_value(), cps_wls_get_rx_fop_value(), cps_wls_get_rx_ce_pkt_value(),cps_wls_get_rx_ss_pkt_value(), cps_wls_get_rx_die_tmp(),\
                    cps_wls_get_rx_irect(), cps_wls_get_rx_vrect(), cps_wls_get_rx_vout(),cps_wls_get_rx_mldo_drop());
}
static DEVICE_ATTR(start_execute_rx_test_process, 0664, execute_rx_test_process, NULL);
/*********************************************************************************************************
*
*   TX DEVICE ATTR
*
*********************************************************************************************************/
//----------------------------show_ask_pkt--------------------------------------------
static ssize_t show_ask_pkt(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t data[8];
    cps_wls_get_ask_pkt_event(data,sizeof(data));
    return sprintf(buf, " data0: 0x%x\n data1: 0x%x\n data2: 0x%x\n data3: 0x%x\n",data[0],data[1],data[2],data[3]);
}
static DEVICE_ATTR(get_ask_pkt, 0444, show_ask_pkt, NULL);
//----------------------------show_tx_ept_rsn--------------------------------------------
static ssize_t show_tx_ept_rsn(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, " tx ept rsn: 0x%x\n", cps_wls_get_tx_ept_rsn());
}
static DEVICE_ATTR(get_tx_ept_rsn, 0444, show_tx_ept_rsn, NULL);
//----------------------------show_tx_ept_code-------------------------------------------
static ssize_t show_tx_ept_code(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, " tx ept  code: 0x%x\n", cps_wls_get_tx_ept_code());
}
static DEVICE_ATTR(get_tx_ept_code, 0444, show_tx_ept_code, NULL);
//---------------------------show_tx_die_tmp---------------------------------------------
static ssize_t show_tx_die_tmp(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, " tx die tmp: 0x%x\n", cps_wls_get_tx_die_tmp());
}
static DEVICE_ATTR(get_tx_die_tmp, 0444, show_tx_die_tmp, NULL);
//---------------------------show_rp_value------------------------------------------------
static ssize_t show_rp_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "RP: 0x%x\n", cps_wls_get_tx_rp_value());
}
static DEVICE_ATTR(get_tx_rp_value, 0444, show_rp_value, NULL);
//---------------------------show_ce_value------------------------------------------------
static ssize_t show_ce_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "CE: 0x%x\n", cps_wls_get_tx_ce_value());
}
static DEVICE_ATTR(get_tx_ce_value, 0444, show_ce_value, NULL);
//---------------------------show_tx_freq-------------------------------------------------
static ssize_t show_tx_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "operating frequency value :0x%x\n", cps_wls_get_tx_freq());
}
static DEVICE_ATTR(get_tx_freq, 0444, show_tx_freq, NULL);
//------------------------show_tx_func_en-------------------------------------------------
static ssize_t show_tx_func_en(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "tx func en is :0x%x\n", cps_wls_get_tx_func_en());
}
static DEVICE_ATTR(get_tx_func_en,0444, show_tx_func_en, NULL);
//--------------------------set_tx_func_en-----------------------------------------------
static ssize_t set_tx_func_ens(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    cps_wls_set_tx_func_en(tmp);
    return count;
}
static DEVICE_ATTR(set_tx_func_en,0664, NULL, set_tx_func_ens);
//--------------------------set_tx_send_fsk_packet--------------------------------------------
static ssize_t set_tx_send_fsk_packet(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t data[8];
    data[0] = 0x1E;
    data[1] = 0XFF;
    cps_wls_send_fsk_packet(data,2);
    return sprintf(buf, "Tx send : 0x%x 0x%x\n",0x1E,0XFF);
}
static DEVICE_ATTR(tx_send_fsk_packet_example, 0444,set_tx_send_fsk_packet,NULL);
//----------------------------show_tx_vin--------------------------------------------------
static ssize_t show_tx_vin(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "tx vin : %d\n", cps_wls_get_tx_vin());
}
static DEVICE_ATTR(get_tx_vin, 0444, show_tx_vin, NULL);
//---------------------------- show_tx_ipa--------------------------------------------------
static ssize_t show_tx_ipa(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "tx ipa : %d\n", cps_wls_get_tx_ipa());
}
static DEVICE_ATTR(get_tx_ipa, 0444, show_tx_ipa, NULL);
//------------------------------show_tx_vpa------------------------------------------------
static ssize_t show_tx_vpa(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "tx vpa : %d\n", cps_wls_get_tx_vpa());
}
static DEVICE_ATTR(get_tx_vpa, 0444, show_tx_vpa, NULL);
//-----------------------------show_chip_id-------------------------------------------------
static ssize_t show_chip_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "chip id : %x\n", cps_wls_get_chip_id());
}
static DEVICE_ATTR(get_chip_id, 0444, show_chip_id, NULL);
//------------------------------show_fw_version------------------------------------------------
static ssize_t show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "fw version : 0x%x\n", cps_wls_get_sys_fw_version());
}
static DEVICE_ATTR(get_version, 0444, show_fw_version, NULL);
//-------------------------------show_sys_mode-----------------------------------------------
static ssize_t show_sys_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (cps_wls_get_sys_mode() == SYS_MODE_RX)
    {
        return sprintf(buf,"sys mode :RX %d\n",1);
    }
    else
    {
        return sprintf(buf,"sys mode :TX %d\n",2);
    }
    
}
static DEVICE_ATTR(get_sys_mode, 0444, show_sys_mode, NULL);
//-------------------------------show_sys_mode-----------------------------------------------
static ssize_t show_crc_val(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "crc val : 0x%x\n",cps_wls_get_crc_val()); 
}
static DEVICE_ATTR(get_crc_val, 0444, show_crc_val, NULL);

//-----------------------execute_tx_test_process------------------------------------
static ssize_t execute_tx_test_process(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t cmd;
    cps_wls_set_tx_func_en(0x21);
    cmd = cps_wls_get_cmd();
    cmd |= TX_CMD_ENTER_TX_MODE;
    cps_wls_set_cmd(cmd);
    return sprintf(buf,"chip id : 0x%x\nfw version : 0x%x\nCMD TO_TX_MODE SET 0x8\n tx ept rsn: 0x%x\ntx die tmp: %d C\nRP: 0x%x\nCE: 0x%x\ntx ept  code: 0x%x\n \
                    operating frequency value :0x%x\ntx func en is :0x%x\ntx vin : %d\ntx ipa : %d\ntx vpa : %d\n", cps_wls_get_chip_id(), cps_wls_get_sys_fw_version(),\
                    cps_wls_get_tx_ept_rsn(), cps_wls_get_tx_die_tmp(), cps_wls_get_tx_rp_value(), cps_wls_get_tx_ce_value(), cps_wls_get_tx_ept_code(), \
                    cps_wls_get_tx_freq(), cps_wls_get_tx_func_en(), cps_wls_get_tx_vin(), cps_wls_get_tx_ipa(), cps_wls_get_tx_vpa());
}
static DEVICE_ATTR(start_execute_tx_test_process, 0444, execute_tx_test_process, NULL);

/*********************************************************************************************************
*
*   ADD DEVICE ATTR
*
*********************************************************************************************************/

static void cps_wls_create_device_node(struct device *dev)
{
     //-----------------------COMME------------------------
     
    device_create_file(dev, &dev_attr_set_cmd);
    device_create_file(dev, &dev_attr_get_cmd);
    device_create_file(dev, &dev_attr_set_fun_en);
    device_create_file(dev, &dev_attr_get_fun_en);
    device_create_file(dev, &dev_attr_get_chip_id);
    device_create_file(dev, &dev_attr_get_version);
    device_create_file(dev, &dev_attr_get_crc_val);
    device_create_file(dev, &dev_attr_get_sys_mode);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_sets_int_enable);
    //-----------------------program---------------------
    device_create_file(dev, &dev_attr_update_fw);
    //-----------------------write password--------------
    // device_create_file(dev, &dev_attr_write_password);

    //-----------------------RX--------------------------
    device_create_file(dev, &dev_attr_set_rx_vout);
    device_create_file(dev, &dev_attr_rx_send_ask_packet_example);

    device_create_file(dev, &dev_attr_get_rx_irect);
    device_create_file(dev, &dev_attr_get_rx_vrect);
    device_create_file(dev, &dev_attr_get_rx_mldo_drop);
    device_create_file(dev, &dev_attr_get_rx_vout);
    device_create_file(dev, &dev_attr_get_rx_ss_value);
    device_create_file(dev, &dev_attr_get_rx_ce_value);
    device_create_file(dev, &dev_attr_get_rx_rp_value);
    device_create_file(dev, &dev_attr_get_rx_fop_value);
    device_create_file(dev, &dev_attr_get_rx_die_tmp);
    device_create_file(dev, &dev_attr_start_execute_rx_test_process);
    device_create_file(dev, &dev_attr_get_rx_fsk_pkt);


    //-----------------------TX--------------------------
    device_create_file(dev, &dev_attr_get_tx_vin);
    device_create_file(dev, &dev_attr_get_tx_ipa);
    device_create_file(dev, &dev_attr_get_tx_vpa);
    device_create_file(dev, &dev_attr_set_tx_func_en);
    device_create_file(dev, &dev_attr_get_tx_func_en);
    device_create_file(dev, &dev_attr_get_tx_freq);
    device_create_file(dev, &dev_attr_get_tx_ce_value);
    device_create_file(dev, &dev_attr_get_tx_rp_value);
    device_create_file(dev, &dev_attr_get_tx_die_tmp);
    device_create_file(dev, &dev_attr_get_tx_ept_code);
    device_create_file(dev, &dev_attr_get_tx_ept_rsn);
    device_create_file(dev, &dev_attr_tx_send_fsk_packet_example);
    device_create_file(dev, &dev_attr_start_execute_tx_test_process);
    device_create_file(dev, &dev_attr_get_ask_pkt);
    

}

static int cps_wls_parse_dt(struct cps_wls_chrg_chip *chip)
{
    struct device_node *node = chip->dev->of_node;
	printk("%s enter\n", __func__);
    if (!node)
    {
        cps_wls_log(CPS_LOG_ERR, "devices tree node missing \n");
        return -EINVAL;
    }

    chip->wls_charge_int = of_get_named_gpio(node, "cps,cps_wls_int", 0);
    if (!gpio_is_valid(chip->wls_charge_int)){
		cps_wls_log(CPS_LOG_ERR, "cps_wls_int gpio missing \n");
        return -EINVAL;
	}
	chip->wls_charger_ovp_en = of_get_named_gpio(node, "cps,wls_charger_ovp_en", 0);
    if (!gpio_is_valid(chip->wls_charger_ovp_en)){
		cps_wls_log(CPS_LOG_ERR, "wls_charger_ovp_en gpio missing \n");
        return -EINVAL;
	}
	chip->wls_mod0_gp1 = of_get_named_gpio(node, "cps,wls_mod0_gp1", 0);
    if (!gpio_is_valid(chip->wls_mod0_gp1)){
		cps_wls_log(CPS_LOG_ERR, "wls_mod0_gp1 gpio missing \n");
        return -EINVAL;
	}
	chip->wls_mod1_gp2 = of_get_named_gpio(node, "cps,wls_mod1_gp2", 0);
    if (!gpio_is_valid(chip->wls_mod1_gp2)){
		cps_wls_log(CPS_LOG_ERR, "wls_mod1_gp2 gpio missing \n");
        return -EINVAL;
	}
	chip->wls_charger_ovp_flag = of_get_named_gpio(node, "cps,wls_charger_ovp_flag", 0);
    if (!gpio_is_valid(chip->wls_charger_ovp_flag)){
		cps_wls_log(CPS_LOG_ERR, "wls_charger_ovp_flag gpio missing \n");
        return -EINVAL;
	}
    return 0;
}
static int cps_wls_gpio_request(struct cps_wls_chrg_chip *chip)
{
    int ret = 0;
    int irqn = 0;
	printk("%s enter\n", __func__);
    if (gpio_is_valid(chip->wls_charge_int))
    {
        ret = gpio_request_one(chip->wls_charge_int, GPIOF_DIR_IN, "cps4021_ap_int");
        if (ret)
        {
            cps_wls_log(CPS_LOG_ERR, "[%s] int gpio request failed\n", __func__);
            goto err_irq_gpio;
        }
        irqn = gpio_to_irq(chip->wls_charge_int);
        if (irqn < 0)
        {
            ret = irqn;
            cps_wls_log(CPS_LOG_ERR, "[%s] failed to gpio to irq\n", __func__);
            goto err_irq_gpio;
        }
        chip->cps_wls_irq = irqn;
    }
    else
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] reset gpio not provided\n", __func__);
        goto err_irq_gpio;
    }
	if(gpio_is_valid(chip->wls_charger_ovp_en)){
		ret = gpio_request_one(chip->wls_charger_ovp_en, GPIOF_DIR_OUT, "wls_charger_ovp_en");
		if (ret)
        {
            cps_wls_log(CPS_LOG_ERR, "[%s] wls_charger_ovp_en gpio request failed\n", __func__);
            goto err_irq_gpio;
        }
	}
	if(gpio_is_valid(chip->wls_charger_ovp_flag)){
		ret = gpio_request_one(chip->wls_charger_ovp_flag, GPIOF_DIR_OUT, "wls_charger_ovp_flag");
		if (ret)
        {
            cps_wls_log(CPS_LOG_ERR, "[%s] wls_charger_ovp_flag gpio request failed\n", __func__);
            goto err_irq_gpio;
        }
	}
	if(gpio_is_valid(chip->wls_mod0_gp1)){
		ret = gpio_request_one(chip->wls_mod0_gp1, GPIOF_DIR_IN, "wls_mod0_gp1");
		if (ret)
        {
            cps_wls_log(CPS_LOG_ERR, "[%s] wls_mod0_gp1 gpio request failed\n", __func__);
            goto err_irq_gpio;
        }
	}
	if(gpio_is_valid(chip->wls_mod1_gp2)){
		ret = gpio_request_one(chip->wls_mod1_gp2, GPIOF_DIR_OUT, "wls_mod1_gp2");
		if (ret)
        {
            cps_wls_log(CPS_LOG_ERR, "[%s] wls_mod1_gp2 gpio request failed\n", __func__);
            goto err_irq_gpio;
        }
	}
err_irq_gpio:
    gpio_free(chip->wls_charge_int);
	gpio_free(chip->wls_charger_ovp_en);
	gpio_free(chip->wls_mod0_gp1);
	gpio_free(chip->wls_charger_ovp_flag);
	gpio_free(chip->wls_mod1_gp2);
    return ret;
}

void wls_charger_ovp_en_set(int en)
{
	cps_wls_log(CPS_LOG_ERR, "[%s] en = %d\n", __func__, en);
	if(gpio_is_valid(chip->wls_charger_ovp_en)){
		gpio_set_value(chip->wls_charger_ovp_en, en);
		usleep_range(1000, 2000);
	}
}
EXPORT_SYMBOL(wls_charger_ovp_en_set);

int wls_mod0_gp1_get(void)
{
	int ret;
	if(gpio_is_valid(chip->wls_mod0_gp1)){		 
		ret = gpio_get_value(chip->wls_mod0_gp1);
		cps_wls_log(CPS_LOG_ERR, "[%s] wls_mod0_gp1 = %d\n", __func__, ret);
		return ret;
	}else{
		cps_wls_log(CPS_LOG_ERR, "[%s] wls_mod0_gp1 gpio not valid\n", __func__);
		return -1;
	}
}
EXPORT_SYMBOL(wls_mod0_gp1_get);

void wls_charger_ovp_flag_set(int en)
{
	cps_wls_log(CPS_LOG_ERR, "[%s] en = %d\n", __func__, en);
	if(gpio_is_valid(chip->wls_charger_ovp_flag)){
		gpio_set_value(chip->wls_charger_ovp_flag, en);
		usleep_range(1000, 2000);
	}
}
EXPORT_SYMBOL(wls_charger_ovp_flag_set);

void wls_mod1_gp2_set(int en)
{
	cps_wls_log(CPS_LOG_ERR, "[%s] en = %d\n", __func__, en);
	if(gpio_is_valid(chip->wls_mod1_gp2)){
		gpio_set_value(chip->wls_mod1_gp2, en);
		usleep_range(1000, 2000);
	}
}
EXPORT_SYMBOL(wls_mod1_gp2_set);

static void cps_wls_lock_work_init(struct cps_wls_chrg_chip *chip)
{
    mutex_init(&chip->irq_lock);
    mutex_init(&chip->i2c_lock);
    //wake_lock_init(&chip->cps_wls_wake_lock, WAKE_LOCK_SUSPEND, "cps_wls_wake_lock");
    // INIT_DELAYED_WORK(&chip->cps_wls_monitor_work, cps_wls_monitor_work_func);
}

static void cps_wls_lock_destroy(struct cps_wls_chrg_chip *chip)
{
    mutex_destroy(&chip->irq_lock);
    mutex_destroy(&chip->i2c_lock);
    //wake_lock_destroy(&chip->cps_wls_wake_lock);
    // cancel_delayed_work_sync(&chip->cps_wls_monitor_work);
}

static void cps_wls_free_gpio(struct cps_wls_chrg_chip *chip)
{
    if (gpio_is_valid(chip->wls_charge_int))
        gpio_free(chip->wls_charge_int);
}


static int cps_wls_register_psy(struct cps_wls_chrg_chip *chip)
{
    struct power_supply_config cps_wls_psy_cfg = {};

    chip->wl_psd.name = CPS_WLS_CHRG_PSY_NAME;
    chip->wl_psd.type = POWER_SUPPLY_TYPE_UNKNOWN;
    chip->wl_psd.properties = cps_wls_chrg_props;
    chip->wl_psd.num_properties = ARRAY_SIZE(cps_wls_chrg_props);
    chip->wl_psd.get_property = cps_wls_chrg_get_property;
    chip->wl_psd.set_property = cps_wls_chrg_set_property;
    chip->wl_psd.property_is_writeable = cps_wls_chrg_property_is_writeable;
    chip->wl_psd.external_power_changed = cps_wls_charger_external_power_changed;

    cps_wls_psy_cfg.drv_data = chip;
    cps_wls_psy_cfg.of_node = chip->dev->of_node;
    chip->wl_psy = devm_power_supply_register(chip->dev,
                                         &chip->wl_psd,
                                         &cps_wls_psy_cfg);
    if (IS_ERR(chip->wl_psy))
    {
        cps_wls_log(CPS_LOG_ERR, "devm_power_supply_register error\n");
        return PTR_ERR(chip->wl_psy);
    }
    return CPS_WLS_SUCCESS;
}

//kcm added by phf, 20241110
static bool cps_wls_is_meta_mode(void)
{
	struct device_node *boot_np; //, *np = chip->dev->of_node;
	const struct {
		u32 size;
		u32 tag;
		u32 boot_mode;
		u32 boot_type;
	} *tag;
	
	/* mediatek boot mode */
#if 1
	boot_np = of_find_node_by_path("/chosen");
	if (!boot_np) {
		cps_wls_log(CPS_LOG_ERR, "warning: not find node: '/chosen'\n");

		boot_np = of_find_node_by_path("/chosen@0");
		if (!boot_np) {
			cps_wls_log(CPS_LOG_ERR,
				"[%s] error: not find node: '/chosen@0'\n",
				__func__);
			return false;
		}
	}
#else	
	boot_np = of_parse_phandle(np, "boot_mode", 0);
	if (!boot_np) {
		cps_wls_log(CPS_LOG_ERR, "failed to get bootmode phandle\n");
		return false;
	}
#endif	
	tag = of_get_property(boot_np, "atag,boot", NULL);
	if (!tag) {
		cps_wls_log(CPS_LOG_ERR, "failed to get atag,boot\n");
		return false;
	}
	cps_wls_log(CPS_LOG_DEBG, "sz:0x%x tag:0x%x mode:0x%x type:0x%x\n",
		 tag->size, tag->tag, tag->boot_mode, tag->boot_type);
		 
	/* set aicr = 200mA in 1:META_BOOT 5:ADVMETA_BOOT */
	if (tag->boot_mode == 1 || tag->boot_mode == 5)
		return true;
	else
		return false;
}

static int cps_wls_get_update_fw_version(void)
{
	int ver_offset = 196;
	const u8 *ptr = CPS4021_FW;
	int version;

	
	version =  (*(ptr+ver_offset)<<8) | *(ptr+ver_offset+1);
	
	return version;
}

static void cps_wls_auto_upate_firmware(void)
{
	int i;
	int sys_fm_ver, fw_ver;
	
	if( !cps_wls_is_meta_mode() )
		return;
	
	//delay 100ms
	msleep(6500);
	
	cps_wls_log(CPS_LOG_DEBG, "Is in META mode !");
	
	fw_ver = cps_wls_get_update_fw_version();
	
	for(i=0; i<3; i++)
	{
		sys_fm_ver = cps_wls_get_sys_fw_version();
		if(sys_fm_ver != -1)
			break;
		msleep(10);
	}
	
	cps_wls_log(CPS_LOG_DEBG, "sys_fm_ver:0x%x, fw_ver:0x%x", sys_fm_ver, fw_ver);
	if(sys_fm_ver == fw_ver)
	{
		cps_wls_log(CPS_LOG_DEBG, "FW version is the same, no need update !");
		return;
	}
	
	//do fimrware upate, retry for 3 times
	//for(i=0; i<3; i++)
	while(1)
	{
		if(update_firmware() == 0)
		{
			cps_wls_log(CPS_LOG_DEBG, "FW update successful");
			return;
#if 0			
			//read sys fw version
			for(j=0; j<3; j++)
			{
				sys_fm_ver = cps_wls_get_sys_fw_version();
				cps_wls_log(CPS_LOG_DEBG, "get sys fw version:%x, i=%d", sys_fm_ver, i);
				if(sys_fm_ver != -1)
					break;
				msleep(10);
			}
			
			if(sys_fm_ver == fw_ver)
			{
				cps_wls_log(CPS_LOG_DEBG, "sys_fm_ver == fw_ver, FW update OK!!!\n");
				return;
			}
			else
			{
				cps_wls_log(CPS_LOG_DEBG, "read sys != fw_ver, FW update NOT OK !!!\n");	
				//reset i=0, for retry
				i = 0;
			}
#endif
		}
		msleep(1000);
	}
	
	cps_wls_log(CPS_LOG_DEBG, "FW update failed !");
}

static int cps_wls_notify_task_threadfn(void *data)
{
	printk("cps_wls_notify_task_threadfn===\n");
	
	// kcm added by phf, 20241109, in meta mode, auto do firmware update
	cps_wls_auto_upate_firmware();
	//while (!kthread_should_stop()) {
	//	if (kthread_should_stop())
	//		goto out;				
	//}
//out:
	return 0;
}
// kcm added end

static int cps_wls_chrg_probe(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
    int ret = 0;
    int id_val = 0;
	
    cps_wls_log(CPS_LOG_DEBG, "%s enter\n", __func__);
	
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] cps_debug: Unable to allocate memory\n", __func__);
        return -ENOMEM;
    }
    chip->client = client;
    chip->dev = &client->dev;
    chip->name = "cps_wls";
    chip->regmap = devm_regmap_init_i2c(client, &cps4021_regmap_config); //16bit reg
    if (IS_ERR(chip->regmap))
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
        devm_kfree(&client->dev, chip);
        return PTR_ERR(chip->regmap);
    }
    chip->regmap32 = devm_regmap_init_i2c(client, &cps4021_regmap32_config); //32bit reg
    if (IS_ERR(chip->regmap32))
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
        devm_kfree(&client->dev, chip);
        return PTR_ERR(chip->regmap32);
    }

    i2c_set_clientdata(client, chip);
    dev_set_drvdata(&(client->dev), chip);

    ret = cps_wls_parse_dt(chip);
    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] Couldn't parse DT nodes ret = %d\n", __func__, ret);
        goto free_source;
    }

    ret = cps_wls_gpio_request(chip);
    if (ret < 0)
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] gpio request failed ret = %d\n", __func__, ret);
        goto free_source;
    }

    if (chip->cps_wls_irq)
    {
        ret = devm_request_threaded_irq(&client->dev, chip->cps_wls_irq, NULL,
                                        cps_wls_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "cps_wls_irq", chip);
        if (ret)
        {
            cps_wls_log(CPS_LOG_ERR, "[%s] request cps_wls_int irq failed ret = %d\n", __func__, ret);
            goto free_source;
        }
        enable_irq_wake(chip->cps_wls_irq);
    }
    cps_wls_lock_work_init(chip);

    cps_wls_create_device_node(&(client->dev));

    
    //ret = cps_wls_register_psy(chip);
    
    cps_wls_register_psy(chip);
    /*
    if (IS_ERR(chip->wl_psy))
    {
        cps_wls_log(CPS_LOG_ERR, "[%s] power_supply_register wireless failed , ret = %d\n", __func__, ret);
        goto free_source;
    }
    */
    
    //cps_wls_get_sys_fw_version();
    id_val = cps_wls_get_chip_id();
    printk("cps_wls_chrg_probe chip id_val is %x\n",id_val);

    //cps_wls_write_nbyte(0xFF84,0x7A8B,2);  /*write password*/

    //wake_lock(&chip->cps_wls_wake_lock);
	
	chip->notify_task = kthread_run(cps_wls_notify_task_threadfn, chip,
					"cps_update_fw__thread");
	if (IS_ERR(chip->notify_task)) {
		dev_notice(chip->dev, "%s run notify thread fail(%d)\n",
			__func__, ret);
		ret = PTR_ERR(chip->notify_task);
		goto free_source;
	}
	
    cps_wls_log(CPS_LOG_DEBG, "[%s] wireless charger addr low probe successful!\n", __func__);
    return ret;

free_source:
    cps_wls_free_gpio(chip);
    cps_wls_lock_destroy(chip);
    cps_wls_log(CPS_LOG_ERR, "[%s] error: free resource.\n", __func__);

    return ret;
}

static void not_called_api(void)
{
    /*int rc;
    rc = cps_wls_get_rx_ss_pkt_value();
    rc = cps_wls_get_rx_ce_pkt_value();
    rc = cps_wls_get_rx_rp_pkt_value();
    rc = cps_wls_get_rx_fop_value();
    rc = cps_wls_get_rx_ept_code();
    //rc = cps_wls_get_rx_neg_pro();
   rc = cps_wls_get_rx_vrect();
    rc = cps_wls_get_rx_mldo_drop();
    rc = cps_wls_get_rx_irect();
    rc = cps_wls_get_rx_vout();
    rc = cps_wls_get_rx_die_tmp();
    rc = cps_wls_set_rx_vout_target(5000);
    rc = cps_wls_set_rx_max_power(50);
    rc = cps_wls_set_rx_dummy_load_mod_val(6);
     rc = cps_wls_set_rx_dummy_load_no_mod_val(3);
    rc = cps_wls_set_rx_fc_vpa_voltage(0x1388)
    rc = cps_wls_set_rx_fc_mldo_voltage(0x1388)
    rc = cps_wls_set_rx_fc_boost_mode(0)
    rc = cps_wls_get_tx_ce_value();
    rc = cps_wls_get_tx_rp_value();*/
    int rc;
    uint8_t data[2] = {0x1F, 0xAC};
   // rc = cps_wls_set_tx_fod0_thresh(3000);
    rc = cps_wls_enable_tx_mode();
    rc = cps_wls_disable_tx_mode();
    rc = cps_wls_send_fsk_packet(data, 2);
    rc = cps_wls_set_fod_para();
    return;
}

static int cps_wls_chrg_remove(struct i2c_client *client)
{
    not_called_api();
    // cps_wls_lock_destroy(chip);
	if (chip->notify_task)
		kthread_stop(chip->notify_task);
    kfree(chip);
    return 0;
}

static const struct i2c_device_id cps_wls_charger_id[] = {
    {"cps-wls-charger", 0},
    {},
};

static const struct of_device_id cps_wls_chrg_of_tbl[] = {
    {.compatible = "cps,wls-charger-cps4021", .data = NULL},
    {},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id);

static struct i2c_driver cps_wls_charger_driver = {
    .driver = {
        .name = CPS_WLS_CHRG_DRV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = cps_wls_chrg_of_tbl,
    },
    .probe = cps_wls_chrg_probe,
    .remove = cps_wls_chrg_remove,
    .id_table = cps_wls_charger_id,
};

static int __init cps_wls_driver_init(void)
{
    return (i2c_add_driver(&cps_wls_charger_driver));
}

late_initcall(cps_wls_driver_init);

static void __exit cps_wls_driver_exit(void)
{
    i2c_del_driver(&cps_wls_charger_driver);
}

module_exit(cps_wls_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jian.deng@convenientpower.com");
MODULE_DESCRIPTION("cps_wls_charger driver");
MODULE_ALIAS("i2c:cps_wls_charger");
