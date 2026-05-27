// SPDX-License-Identifier: GPL-2.0-only
/*
 * High level functions for A6 Spy-by-Wire programming
 */

#include "a6_host_adapter.h"
#include "jtag_funcs.h"
#include "low_level_funcs.h"
#include "high_level_funcs.h"

#define LOCAL_TRACE 0

/* definition for current implementation mappings used by the sbw code */
uint16_t (*SetSBWTCK)(void);
uint16_t (*ClrSBWTCK)(void);
uint16_t (*SetSBWTDIO)(void);
uint16_t (*ClrSBWTDIO)(void);
uint16_t (*SetInSBWTDIO)(void);
uint16_t (*SetOutSBWTDIO)(void);
uint16_t (*GetSBWTDIO)(void);
uint16_t (*SetSBWAKEUP)(void);
uint16_t (*ClrSBWAKEUP)(void);
void (*delay)(uint32_t delay_us);

enum sbw_state_code {
	SBW_OK = 0,
	SBW_TOK,
	SBW_EOL,
	SBW_EOS,
	SBW_SOS,
	SBW_EOI,
	SBW_STATE_ERROR
};

#define SIZEOF_NEWLINE 1

static int hexval(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	else if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return 0;
}

static enum sbw_state_code sbw_get_token(uint8_t *read_p, uint8_t *write_p,
				    uint32_t *read_len_p, uint32_t *write_len_p)
{
	enum sbw_state_code ret;

	/* end-of-line */
	if (*read_p == 0x0d && *(read_p + 1) == 0x0a) {
		*read_len_p = 2;
		*write_len_p = 0;
		ret = SBW_EOL;
	} else if (*read_p == 'q' || *read_p == 'Q') {
		/* end-of-image */
		*read_len_p = 1;
		*write_len_p = 0;
		ret = SBW_EOI;
	} else {
		uint32_t val = 0;

		/* section start */
		if (read_p[0] == '@') {
			ret = SBW_SOS;
			val = hexval(read_p[1 + 0]) << 12;
			val |= hexval(read_p[1 + 1]) << 8;
			val |= hexval(read_p[1 + 2]) << 4;
			val |= hexval(read_p[1 + 3]);

			*read_len_p = 1 + 4 + 2;	/* '@' + XXXX + CRLF */
			*write_len_p = 2;		/* two bytes written */
		} else {
			/* data */
			ret = SBW_TOK;
			val = hexval(read_p[0]) << 4;
			val |= hexval(read_p[1]);

			/*
			 * handle variation: the last 2-u8 value on a line
			 * may not include trailing space
			 */
			*read_len_p = 2 + 1;	/* XX + ' ' */
			*write_len_p = 1;	/* one u8 written */
		}

		/* no target? skip the actual write */
		if (write_p) {
			write_p[0] = val & 0x000000ff;
			write_p[1] = (val >> 8) & 0x000000ff;
			write_p[2] = (val >> 16) & 0x000000ff;
			write_p[3] = (val >> 24) & 0x000000ff;
		}
	}

	return ret;
}

static enum sbw_state_code sbw_parse_line(uint8_t *read_p, uint8_t *write_p,
				     uint32_t *read_len_p, uint32_t *write_len_p)
{
	enum sbw_state_code ret;
	uint32_t total_read_len = 0, total_write_len = 0;
	uint32_t val = 0, r_len = 0, w_len = 0;

	do {
		ret = sbw_get_token(read_p, (uint8_t *)&val, &r_len, &w_len);
		/* end-of-line; break out of loop */
		if (ret == SBW_EOL) {
			total_read_len += r_len;
			*read_len_p = total_read_len;
			total_write_len += w_len;
			*write_len_p = total_write_len;
		} else if (ret == SBW_TOK) {
			/* regular token; keep looping */
			*((uint8_t *)write_p) = (uint8_t)val;
			total_read_len += r_len;
			read_p += r_len;
			total_write_len += w_len;
			write_p += w_len;
		} else if (ret == SBW_SOS || ret == SBW_EOI) {
			/* map start-of-section/end-of-image to end-of-section */
			*read_len_p = *write_len_p = 0;
			ret = SBW_EOS;
		} else {
			/* state mismatch */
			pr_err("SBW_ERROR[%s]: wrong state returned; state: %d\n",
			       __func__, ret);
			ret = SBW_STATE_ERROR;
		}
	} while (ret == SBW_TOK);

	return ret;
}

static enum sbw_state_code sbw_parse_section(uint8_t *read_p, uint8_t *write_p,
					uint32_t *read_len_p, uint32_t *write_len_p)
{
	enum sbw_state_code ret;
	uint32_t total_read_len = 0, total_write_len = 0;
	uint32_t r_len = 0, w_len = 0;

	do {
		ret = sbw_parse_line(read_p, write_p, &r_len, &w_len);
		/* end-of-section; break out of loop */
		if (ret == SBW_EOS) {
			total_read_len += r_len;
			*read_len_p = total_read_len;
			total_write_len += w_len;
			*write_len_p = total_write_len;
		} else if (ret == SBW_EOL) {
			/* end-of-line; keep looping */
			total_read_len += r_len;
			read_p += r_len;
			total_write_len += w_len;
			write_p += w_len;
		} else {
			/* state mismatch */
			pr_err("SBW_ERROR[%s]: wrong state returned; state: %d\n",
			       __func__, ret);
			ret = SBW_STATE_ERROR;
		}
	} while (ret == SBW_EOL);

	return ret;
}

struct sec_info {
	uint32_t sec_addr[75];
	uint32_t sec_len[75];
	uint32_t num_sections;
};

static struct sec_info sec_info;

static enum sbw_state_code sbw_parse_image(uint8_t *read_p, uint8_t *write_p,
				      uint32_t *read_len_p, uint32_t *write_len_p)
{
	enum sbw_state_code ret;
	uint32_t total_read_len = 0, total_write_len = 0;
	uint32_t r_len = 0, w_len = 0, val = 0;

	memset(&sec_info, 0, sizeof(sec_info));

	do {
		ret = sbw_get_token(read_p, (uint8_t *)&val, &r_len, &w_len);
		if (ret != SBW_SOS) {
			if (!sec_info.num_sections) {
				pr_err("SBW_ERROR[%s]: does not start with section; value: %d\n",
				       __func__, ret);
				return SBW_STATE_ERROR;
			}

			if (ret == SBW_EOI) {
				*read_len_p = total_read_len;
				*write_len_p = total_write_len;
				ret = SBW_OK;
				break;
			}

			pr_err("SBW_ERROR[%s]: wrong state returned; state: %d\n",
			       __func__, ret);
			ret = SBW_STATE_ERROR;
			break;
		}

		total_read_len += r_len;
		read_p += r_len;

		sec_info.sec_addr[sec_info.num_sections] = val;

		ret = sbw_parse_section(read_p, write_p, &r_len, &w_len);
		/* end-of-section; keep looping */
		if (ret == SBW_EOS) {
			total_read_len += r_len;
			read_p += r_len;

			if (w_len & 1) {
				write_p[w_len] = 0xff;
				w_len++;
			}
			total_write_len += w_len;
			write_p += w_len;

			/* sec_len converted to A6 words (16-bit) */
			sec_info.sec_len[sec_info.num_sections] = w_len / 2;
			sec_info.num_sections++;
		} else {
			/* state mismatch */
			pr_err("SBW_ERROR[%s:1]: wrong state returned; state: %d\n",
			       __func__, ret);
			ret = SBW_STATE_ERROR;
		}
	} while (ret == SBW_EOS);

	if (ret == SBW_OK) {
		int idx = 0;

		pr_info("Parsing complete. Read size: %d, Write size: %d. Num sections: %d\n",
			*read_len_p, *write_len_p, sec_info.num_sections);
		while (idx < (int)sec_info.num_sections) {
			pr_info("Section idx: %d; Addr: 0x%04x; Length: %d\n",
				idx, sec_info.sec_addr[idx], sec_info.sec_len[idx]);
			idx++;
		}
	}

	return ret;
}

int program_device_sbw(struct a6_sbw_interface *sbw_ops, uint32_t read_address)
{
	uint32_t read_len = 0, write_len = 0;
	enum sbw_state_code parse_ret;
	int retry = 0, ret_val = 0;
	uint16_t addr;

	if (read_address & 1) {
		pr_err("program_fw: Please enter an even read address.\n");
		return -1;
	}

	/* set up the current mappings for the sbw code */
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	parse_ret = sbw_parse_image((uint8_t *)read_address,
				    (uint8_t *)read_address,
				    &read_len, &write_len);
	if (parse_ret != SBW_OK) {
		pr_err("Error in parsing A6 fw file...\n");
		return -1;
	}

	/*
	 * TEMP: Workaround for occasional verification failure.
	 * Not root-caused yet but, empirically, a retry always works. Revisit.
	 */
retry_0:
	InitTarget();

	/* Start of SBW access to the Target */
	if (GetDevice() != STATUS_OK) {
		/* stop here if invalid JTAG ID or time-out */
		pr_err("Error in GetDevice()\n");
		ret_val = -1;
		goto err0;
	}

	/* Program the boot code */
	if (!WriteAllSections((const unsigned short *)read_address,
			      (const unsigned long *)&sec_info.sec_addr[0],
			      (const unsigned long *)&sec_info.sec_len[0],
			      sec_info.num_sections)) {
		pr_err("Error in WriteAllSections(all)\n");
		ret_val = -1;
		goto err0;
	}

	if (!VerifyAllSections((const unsigned short *)read_address,
			       (const unsigned long *)&sec_info.sec_addr[0],
			       (const unsigned long *)&sec_info.sec_len[0],
			       sec_info.num_sections)) {
		pr_err("Error in VerifyAllSections(all)\n");
		pr_err("Retrying...\n\n");
		if (retry++ < 15) {
			addr = ReadMem_430Xv2(F_WORD, V_RESET);
			ReleaseDevice(addr, ERROR);
			ReleaseTarget();
			goto retry_0;
		}

		pr_err("Failure to write and verify fw file after %d retries\n", retry);
		ret_val = -1;
	}

err0:
	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	if (ReleaseDevice(addr, PROGRAM) < 0) {
		pr_err("Checksum validation failed post-flashing.\n");
		if (retry < 15) {
			pr_err("Retrying...\n\n");
			retry++;
			ReleaseTarget();
			goto retry_0;
		}

		pr_err("Failure to program fw after %d retries.\n", retry);
		ret_val = -1;
	}

	/* if fail to set JTAG mode */
	if (ret_val == -1 && retry == 0) {
		retry++;
		ret_val = 0;
		goto retry_0;
	}

	ReleaseTarget();
	return ret_val;
}

int verify_device_sbw(struct a6_sbw_interface *sbw_ops, uint32_t read_address)
{
	uint32_t read_len = 0, write_len = 0;
	enum sbw_state_code parse_ret;
	int ret_val = 0;
	uint16_t addr;

	if (read_address & 1) {
		pr_err("program_fw: Please enter an even read address.\n");
		return -1;
	}

	/* set up the current mappings for the sbw code */
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	parse_ret = sbw_parse_image((uint8_t *)read_address,
				    (uint8_t *)read_address,
				    &read_len, &write_len);
	if (parse_ret != SBW_OK) {
		pr_err("Error in parsing A6 fw file...\n");
		return -1;
	}

	InitTarget();

	/* Start of SBW access to the Target */
	if (GetDevice() != STATUS_OK) {
		/* stop here if invalid JTAG ID or time-out */
		pr_err("Error in GetDevice()\n");
		ret_val = -1;
		goto err0;
	}

	if (!VerifyAllSections((const unsigned short *)read_address,
			       (const unsigned long *)&sec_info.sec_addr[0],
			       (const unsigned long *)&sec_info.sec_len[0],
			       sec_info.num_sections)) {
		pr_err("Error in VerifyAllSections(all)\n");
		ret_val = -1;
	}

err0:
	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	ReleaseDevice(addr, VERIFY);
	ReleaseTarget();

	return ret_val;
}

int ttf_extract_fw_sbw(struct a6_sbw_interface *sbw_ops)
{
	int ret_val = 0;
	uint16_t addr;

	/* set up the current mappings for the sbw code */
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	InitTarget();

	/* Start of SBW access to the Target */
	if (GetDevice() != STATUS_OK) {
		/* stop here if invalid JTAG ID or time-out */
		pr_err("Error in GetDevice()\n");
		ret_val = -1;
		goto err0;
	}

	if (!TTFExtractAllSections()) {
		pr_err("Error in TTFExtractAllSections\n");
		ret_val = -1;
	}

err0:
	addr = ReadMem_430Xv2(F_WORD, V_RESET);
	ReleaseDevice(addr, VERIFY);
	ReleaseTarget();
	return ret_val;
}

int ttf_image_read(char *buf, size_t count, loff_t *ppos)
{
	return TTFImageRead(buf, count, ppos);
}

int ttf_extract_cache_clear(void)
{
	TTFExtractCacheClear();
	return 0;
}

int get_checksum_data_sbw(struct a6_sbw_interface *sbw_ops, unsigned short *cksum1,
			  unsigned short *cksum2, unsigned short *cksum_cycles,
			  unsigned short *cksum_errors)
{
	/* set up the current mappings for the sbw code */
	SetSBWTCK = sbw_ops->a6_per_device_interface.SetSBWTCK;
	ClrSBWTCK = sbw_ops->a6_per_device_interface.ClrSBWTCK;
	SetSBWTDIO = sbw_ops->a6_per_device_interface.SetSBWTDIO;
	ClrSBWTDIO = sbw_ops->a6_per_device_interface.ClrSBWTDIO;
	SetInSBWTDIO = sbw_ops->a6_per_device_interface.SetInSBWTDIO;
	SetOutSBWTDIO = sbw_ops->a6_per_device_interface.SetOutSBWTDIO;
	GetSBWTDIO = sbw_ops->a6_per_device_interface.GetSBWTDIO;
	SetSBWAKEUP = sbw_ops->a6_per_device_interface.SetSBWAKEUP;
	ClrSBWAKEUP = sbw_ops->a6_per_device_interface.ClrSBWAKEUP;
	delay = sbw_ops->a6_per_target_interface.delay;

	return GetChecksumData(cksum1, cksum2, cksum_cycles, cksum_errors);
}
