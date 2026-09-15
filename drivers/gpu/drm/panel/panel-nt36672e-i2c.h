#ifndef _NT36672E_I2C_SW_H_
#define _NT36672E_I2C_SW_H_

#define DEVICE_ID_MASK 	     0x0FFFFFF

#define PGC1KG_ID            0x0499899
#define PGC1KL_ID	           0x0410899
#define PGC2KG_ID            0x0419899
#define PGC2KL_ID	           0x0411899
#define PGC4KD_ID			       0x042A899
#define PGC4KL_ID			       0x0422899                //CPLD参考这个,0x0422899
#define PGC7KD_ID			       0x042B899
#define PGC10KD_ID		       0x042C899                    

#define PGC1KL_BITSTREAM_SIZE	    54116
#define PGC1KG_BITSTREAM_SIZE			84936
#define PGC2KG_BITSTREAM_SIZE			84936
#define PGC2KL_BITSTREAM_SIZE			84936
//#define PGC4KD_BITSTREAM_SIZE			146072
//#define PGC4KL_BITSTREAM_SIZE			146072
#define PGC4KD_BITSTREAM_SIZE			206072
#define PGC4KL_BITSTREAM_SIZE			206072
#define PGC7KD_BITSTREAM_SIZE			227972
#define PGC10KD_BITSTREAM_SIZE		326900

#define PGC1KL_TOTAL_PAGE         332
#define PGC1KG_TOTAL_PAGE         332
#define PGC2KG_TOTAL_PAGE         332
#define PGC2KL_TOTAL_PAGE         332
#define PGC4KD_TOTAL_PAGE         1280
#define PGC4KL_TOTAL_PAGE         1280
#define PGC7KD_TOTAL_PAGE         1808
#define PGC10KD_TOTAL_PAGE        2560

#define DEFAULT_FEATURE_CTL				0x20902

#define PGC4KD_PAGES_PER_HEAP			320
#define PGC4KL_PAGES_PER_HEAP			320
#define PGC7KD_PAGES_PER_HEAP			452
#define PGC10KD_PAGES_PER_HEAP	  640

#define PG_PGC_EFLASH_BYTE_PER_PAGE   256


typedef struct PGCProperties
{
		unsigned int device_id;
		unsigned int bitstream;
		unsigned int total_page;
		unsigned int pages_per_heap;
}PGCProperties;

static PGCProperties pgclist[] = {
			{PGC1KL_ID,PGC1KL_BITSTREAM_SIZE,PGC1KL_TOTAL_PAGE,PGC1KL_TOTAL_PAGE},
			{PGC1KG_ID,PGC1KG_BITSTREAM_SIZE,PGC1KG_TOTAL_PAGE,PGC1KG_TOTAL_PAGE},
			{PGC2KL_ID,PGC2KL_BITSTREAM_SIZE,PGC2KL_TOTAL_PAGE,PGC2KL_TOTAL_PAGE},
			{PGC2KG_ID,PGC2KG_BITSTREAM_SIZE,PGC2KG_TOTAL_PAGE,PGC2KG_TOTAL_PAGE},
			{PGC4KL_ID,PGC4KL_BITSTREAM_SIZE,PGC4KL_TOTAL_PAGE,PGC4KL_PAGES_PER_HEAP},          //这个
            //              146072              1280                320  
			{PGC4KD_ID,PGC4KD_BITSTREAM_SIZE,PGC4KD_TOTAL_PAGE,PGC4KD_PAGES_PER_HEAP},
			{PGC7KD_ID,PGC7KD_BITSTREAM_SIZE,PGC7KD_TOTAL_PAGE,PGC7KD_PAGES_PER_HEAP},
			{PGC10KD_ID,PGC10KD_BITSTREAM_SIZE,PGC10KD_TOTAL_PAGE,PGC10KD_PAGES_PER_HEAP},
		};

typedef struct PGCDevice
{
    //PGIOCallback *_cb;
    //I2CInterface *_i2c_inf;
    PGCProperties _pgc_properties;
} PGCDevice;


typedef enum PGCStatusRegisterBitMask
{
    PGC_STSREG_PRESIST_MSPI  = 0x20000000, //[29] persist_mspi 用户模式下主SPI接口使能
    PGC_STSREG_PERSIST_SI2C  = 0x10000000, //[28] persist_si2c 用户模式下从I2C接口使能
    PGC_STSREG_PERSIST_SSPI  = 0x08000000, //[27] persist_sspi 用户模式下从SPI接口使能
    PGC_STSREG_PERSIST_JTAG  = 0x04000000, //[26] persist_jtag 用户模式下JTAG接口使能
    PGC_STSREG_PERSIST_DONE  = 0x02000000, //[25] persist_done 用户模式下DONE管脚使能
    PGC_STSREG_PERSIST_INIT  = 0x01000000, //[24] persist_init 用户模式下INIT_N管脚使能
    PGC_STSREG_PERSIST_RSTN  = 0x00800000, //[23] persist_rstn 用户模式下RST_N管脚使能
    PGC_STSREG_PERSIST_CAL   = 0x00400000, //[22] pass_cal 嵌入式FLASH校准成功标志 4K/7K器件此位保留
    PGC_STSREG_BUSY          = 0x00200000, //[21] busy 嵌入式FLASH忙碌标志
    PGC_STSREG_LOCK          = 0x00100000, //[20] lock 嵌入式FLASH锁定标志
    PGC_STSREG_WAKE          = 0x00080000, //[19] wake 嵌入式FLASH唤醒标志
    PGC_STSREG_SLEEP         = 0x00040000, //[18] sleep 嵌入式FLASH休眠标志
    PGC_STSREG_FALLBACK      = 0x00020000, //[17] fallback 回退指示标志
    PGC_STSREG_PLL_LOCK      = 0x00010000, //[16] pll_lock PLL锁定标志
    PGC_STSREG_GWEN          = 0x00008000, //[15] gwen 全局写使能
    PGC_STSREG_GRS_N         = 0x00004000, //[14] grs_n 全局寄存器置位复位
    PGC_STSREG_GOUTEN        = 0x00002000, //[13] gouten 全局IO输出使能
    //[12] 保留              
    PGC_STSREG_GLOGEN_FB     = 0x00000800, //[11] glogen_fb 全局逻辑使能反馈
    PGC_STSREG_GLOGEN        = 0x00000400, //[10] glogen 全局逻辑使能
    PGC_STSREG_DONE_I        = 0x00000200, //[9] done_i DONE管脚输入
    PGC_STSREG_DONE          = 0x00000100, //[8] done 器件唤醒成功标志
    PGC_STSREG_INIT_N        = 0x00000080, //[7] init_n INIT_N管脚输入
    PGC_STSREG_INIT_COMPLETE = 0x00000040, //[6] init_complete 初始化完成和配置错误指示
    PGC_STSREG_WAKEDOWN_OVER = 0x00000020, //[5] wakedown_over 唤醒关断结束
    PGC_STSREG_WAKEUP_OVER   = 0x00000010, //[4] wakeup_over 唤醒结束
    PGC_STSREG_TIMEOUT       = 0x00000008, //[3] timeout 看门狗超时Compact CPLD配置控制系统
    PGC_STSREG_RBCRC_ERR     = 0x00000004, //[2] rbcrc_err 回读CRC检测结果 0: CRC正确 1: CRC错误
    PGC_STSREG_CRC_ERR       = 0x00000002, //[1] crc_err CRC检测结果 0: CRC正确 1: CRC错误
    PGC_STSREG_ID_ERR        = 0x00000001, //[0] id_err ID检测结果 0: 正确 1: 错误
} PGCStatusRegisterBitMask;

typedef enum PGCI2CCmd
{
    PGC_I2C_NOP = 0xFF, // No Operation FF
    PGC_I2C_RDID = 0xA1, // Read Identification A1
    PGC_I2C_RDUSER = 0xA2, // Read Usercode A2
    PGC_I2C_RDSR = 0xA3, // Read Status Register A3
    PGC_I2C_RDUID = 0xA4, // Read Unique Identification A4
    PGC_I2C_RDLOCK = 0xA5, // Read Embedded FLASH Lock Information A5
    PGC_I2C_CFG = 0x50, // Config Bitstream 50
    PGC_I2C_WREN = 0x51, // Write Enable 51
    PGC_I2C_WRDIS = 0x52, // Write Disable 52
    PGC_I2C_RESET = 0x60, // Reset CPLD 60
    PGC_I2C_ERASE = 0x10, // Erase Bulk 10
    PGC_I2C_ERASE_PAGE = 0x11, // Erase Page 11
    PGC_I2C_ERASE_CTL = 0x12, // Erase Feature Control bit 12
    PGC_I2C_PROGRAM = 0x20, // Program Page 20
    PGC_I2C_PROGRAM_UID = 0x21, // Program Unique Identification 21
    PGC_I2C_PROGRAM_CTL = 0x22, // Program Feature Control bit 22
    PGC_I2C_READ = 0x30, // Read 30
    PGC_I2C_READ_CTL = 0x31, // Read Feature Control bit 31
    PGC_I2C_PROGRAM_LOCK = 0x40, // Lock Embedded FLASH 40
    PGC_I2C_EFLASH_SLEEP = 0x70, // Embedded FLASH Sleep 70
    PGC_I2C_EFLASH_WAKEUP = 0x71 // Embedded FLASH Wake Up 71
} PGCI2CCmd;

unsigned int pango_PGC_I2C_ReadIdcode(void);
unsigned int pango_PGC_I2C_ReadStatusRegister(void);
unsigned int pango_PGC_I2C_WakeupEmbedFlash(void);
void pango_PGC_I2C_EraseSelfLoadBitstreamAndUserFlash(void);
static bool pango_PGC_I2C_WriteWRDIS(void);
static bool pango_PGC_I2C_WriteWREN(void);  
bool pango_PGC_IsEmbedFlashWakeup(unsigned int sts_val);
bool pango_PGC_IsEmbedFlashSleep(unsigned int sts_val);
bool pango_PGC_I2C_CheckConfigStatus(PGCStatusRegisterBitMask bit_mask, unsigned int check_times, bool is_high);
bool pango_PGC_DeviceEflashInit(PGCDevice *dev, unsigned int id_code);
unsigned int PG_GetEflashAddr(PGCDevice *dev, unsigned int PageNum);
void pango_PGC_I2C_ProgramEFlash(PGCDevice *dev, unsigned int feature_control_val);
bool pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify(PGCDevice *dev, unsigned int addr_offset);
bool pango_PGC_I2C_ProgramSelfLoadBitstreamAndVerify_n10k(PGCDevice *dev, unsigned int addr_offset);
void pango_ClearAll(unsigned char *buf, unsigned int bytes);
//static bool VerifyBufData(unsigned char *read_buf, unsigned char *write_buf, unsigned int total_size);
//bool pango_PGC_I2C_WriteFeatureControlRegister(PGCDevice *dev, unsigned int feature_control_val);
unsigned char pango_ReverseByte(unsigned char byte);
void pango_ReverseBytes(unsigned char *buf, unsigned int num);
unsigned int pango_PGC_I2C_SleepEmbedFlash(void);
bool pango_PGC_I2C_Reset(void); 
int fpga_read_ctl_cmd(unsigned char *buf);
int fpga_program_ctl_cmd(unsigned int feature_control_val);
int fpga_program_cmd(unsigned int page_addr,unsigned char *buf);
int fpga_program_cmd_last_time(unsigned int page_addr,unsigned char *buf,unsigned int size,unsigned int last_size);
int fpga_page_read_cmd(unsigned int page_addr,unsigned char *buf);
#endif



