/*
 * EEPROM/OTP helpers for the OV16880.
 *
 * The module info table below is written by ov16880CheckVersion() and read by
 * nothing in this tree: the stock image keeps it in this file and no other
 * driver touches it.
 */
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#define PFX "OV16880_pdafotp"
#define LOG_INF(fmt, args...)   pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

#define BYTE                unsigned char

#define EEPROM_READ_ID      (0xa0)
#define MAX_OFFSET          (0xffff)
#define TOTAL_DATA_SIZE     (496+876)

#define MODULE_INFO_NAME_LEN 32

struct ov16880_module_info {
	kal_uint32 rsvd0[16];
	kal_uint32 module_id;
	char module_vendor[MODULE_INFO_NAME_LEN];
	kal_uint32 lens_id;
	char lens_vendor[MODULE_INFO_NAME_LEN];
	kal_uint32 vcm_id;
	kal_uint32 driver_ic_id;
	kal_uint32 pdaf_id;
	kal_uint32 year;
	kal_uint32 rsvd152;
};

static struct ov16880_module_info module_info[1] = {
	{ .module_vendor = "Unknown", .lens_vendor = "Unknown" }
};
static int module_info_idx;

static BYTE eeprom_data[TOTAL_DATA_SIZE] = {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

bool selective_read_eeprom(u16 addr, BYTE *data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	if (addr > MAX_OFFSET)
		return false;

	if (iReadRegI2C(pu_send_cmd, 2, (u8 *)data, 1, EEPROM_READ_ID) < 0)
		return false;
	return true;
}

static bool _read_eeprom(u16 addr, BYTE *data, u32 size)
{
	int i = 0;
	int offset = addr;

	for (i = 0; i < size; i++) {
		if (!selective_read_eeprom(offset, &data[i]))
			return false;
		LOG_INF("read_eeprom 0x%x 0x%x\n", offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
	return true;
}

bool read_eeprom(u16 addr, BYTE *data, u32 size)
{
	addr = 0x0801;
	size = TOTAL_DATA_SIZE;
	LOG_INF("read ov16880 eeprom, addr = %d; size = %d\n", addr, size);
	if (!get_done || last_size != size || last_offset != addr) {
		if (!_read_eeprom(addr, eeprom_data, size)) {
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			return false;
		}
		LOG_INF("Eeprom read completed\n");
	} else {
		LOG_INF("ERROR: read failed in entry 1\n");
	}
	memcpy(data, eeprom_data, size);
	return true;
}

bool OV16880_MIPI_read_otp_wb(void)
{
	BYTE awb[13] = {0};
	int i;

	selective_read_eeprom(0x001B, &awb[0]);
	if (awb[0] != 1) {
		pr_err("err:read ov16880 eeprom no awb data\n");
		return false;
	}
	for (i = 1; i < 13; i++)
		selective_read_eeprom(0x001B + i, &awb[i]);
	return true;
}

kal_uint32 ov16880CheckVersion(kal_uint8 i2c_write_id)
{
	BYTE flag = 0;
	BYTE info[23] = {0};
	struct ov16880_module_info *p = &module_info[module_info_idx];

	_read_eeprom(0x0000, &flag, 1);
	if (flag == 0) {
		pr_err("[MaJian]read otp failed!\n");
		return 0;
	}

	_read_eeprom(0x0001, info, sizeof(info));

	p->module_id = info[2];		/* eeprom 0x0003 */
	p->lens_id = info[3];		/* eeprom 0x0004 */
	p->vcm_id = info[15];		/* eeprom 0x0010 */
	p->driver_ic_id = info[16];	/* eeprom 0x0011 */
	p->pdaf_id = info[17];		/* eeprom 0x0012 */
	p->year = info[11];		/* eeprom 0x000C */
	return 1;
}
