// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Linux community
 *
 * Qualcomm MSM7x30 pinctrl driver
 * Based on pinctrl-msm8660.c
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "pinctrl-msm.h"

/*
 * MSM7x30 has 182 GPIOs split across two banks:
 * - Bank 1: GPIOs 0-15 (at 0xAC001000)
 * - Bank 2: GPIOs 16-181 (at 0xAC101000)
 *
 * Each GPIO has up to 16 functions (0 = GPIO, 1-15 = alternate functions)
 */

static const struct pinctrl_pin_desc msm7x30_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
	PINCTRL_PIN(18, "gpio18"),
	PINCTRL_PIN(19, "gpio19"),
	PINCTRL_PIN(20, "gpio20"),
	PINCTRL_PIN(21, "gpio21"),
	PINCTRL_PIN(22, "gpio22"),
	PINCTRL_PIN(23, "gpio23"),
	PINCTRL_PIN(24, "gpio24"),
	PINCTRL_PIN(25, "gpio25"),
	PINCTRL_PIN(26, "gpio26"),
	PINCTRL_PIN(27, "gpio27"),
	PINCTRL_PIN(28, "gpio28"),
	PINCTRL_PIN(29, "gpio29"),
	PINCTRL_PIN(30, "gpio30"),
	PINCTRL_PIN(31, "gpio31"),
	PINCTRL_PIN(32, "gpio32"),
	PINCTRL_PIN(33, "gpio33"),
	PINCTRL_PIN(34, "gpio34"),
	PINCTRL_PIN(35, "gpio35"),
	PINCTRL_PIN(36, "gpio36"),
	PINCTRL_PIN(37, "gpio37"),
	PINCTRL_PIN(38, "gpio38"),
	PINCTRL_PIN(39, "gpio39"),
	PINCTRL_PIN(40, "gpio40"),
	PINCTRL_PIN(41, "gpio41"),
	PINCTRL_PIN(42, "gpio42"),
	PINCTRL_PIN(43, "gpio43"),
	PINCTRL_PIN(44, "gpio44"),
	PINCTRL_PIN(45, "gpio45"),
	PINCTRL_PIN(46, "gpio46"),
	PINCTRL_PIN(47, "gpio47"),
	PINCTRL_PIN(48, "gpio48"),
	PINCTRL_PIN(49, "gpio49"),
	PINCTRL_PIN(50, "gpio50"),
	PINCTRL_PIN(51, "gpio51"),
	PINCTRL_PIN(52, "gpio52"),
	PINCTRL_PIN(53, "gpio53"),
	PINCTRL_PIN(54, "gpio54"),
	PINCTRL_PIN(55, "gpio55"),
	PINCTRL_PIN(56, "gpio56"),
	PINCTRL_PIN(57, "gpio57"),
	PINCTRL_PIN(58, "gpio58"),
	PINCTRL_PIN(59, "gpio59"),
	PINCTRL_PIN(60, "gpio60"),
	PINCTRL_PIN(61, "gpio61"),
	PINCTRL_PIN(62, "gpio62"),
	PINCTRL_PIN(63, "gpio63"),
	PINCTRL_PIN(64, "gpio64"),
	PINCTRL_PIN(65, "gpio65"),
	PINCTRL_PIN(66, "gpio66"),
	PINCTRL_PIN(67, "gpio67"),
	PINCTRL_PIN(68, "gpio68"),
	PINCTRL_PIN(69, "gpio69"),
	PINCTRL_PIN(70, "gpio70"),
	PINCTRL_PIN(71, "gpio71"),
	PINCTRL_PIN(72, "gpio72"),
	PINCTRL_PIN(73, "gpio73"),
	PINCTRL_PIN(74, "gpio74"),
	PINCTRL_PIN(75, "gpio75"),
	PINCTRL_PIN(76, "gpio76"),
	PINCTRL_PIN(77, "gpio77"),
	PINCTRL_PIN(78, "gpio78"),
	PINCTRL_PIN(79, "gpio79"),
	PINCTRL_PIN(80, "gpio80"),
	PINCTRL_PIN(81, "gpio81"),
	PINCTRL_PIN(82, "gpio82"),
	PINCTRL_PIN(83, "gpio83"),
	PINCTRL_PIN(84, "gpio84"),
	PINCTRL_PIN(85, "gpio85"),
	PINCTRL_PIN(86, "gpio86"),
	PINCTRL_PIN(87, "gpio87"),
	PINCTRL_PIN(88, "gpio88"),
	PINCTRL_PIN(89, "gpio89"),
	PINCTRL_PIN(90, "gpio90"),
	PINCTRL_PIN(91, "gpio91"),
	PINCTRL_PIN(92, "gpio92"),
	PINCTRL_PIN(93, "gpio93"),
	PINCTRL_PIN(94, "gpio94"),
	PINCTRL_PIN(95, "gpio95"),
	PINCTRL_PIN(96, "gpio96"),
	PINCTRL_PIN(97, "gpio97"),
	PINCTRL_PIN(98, "gpio98"),
	PINCTRL_PIN(99, "gpio99"),
	PINCTRL_PIN(100, "gpio100"),
	PINCTRL_PIN(101, "gpio101"),
	PINCTRL_PIN(102, "gpio102"),
	PINCTRL_PIN(103, "gpio103"),
	PINCTRL_PIN(104, "gpio104"),
	PINCTRL_PIN(105, "gpio105"),
	PINCTRL_PIN(106, "gpio106"),
	PINCTRL_PIN(107, "gpio107"),
	PINCTRL_PIN(108, "gpio108"),
	PINCTRL_PIN(109, "gpio109"),
	PINCTRL_PIN(110, "gpio110"),
	PINCTRL_PIN(111, "gpio111"),
	PINCTRL_PIN(112, "gpio112"),
	PINCTRL_PIN(113, "gpio113"),
	PINCTRL_PIN(114, "gpio114"),
	PINCTRL_PIN(115, "gpio115"),
	PINCTRL_PIN(116, "gpio116"),
	PINCTRL_PIN(117, "gpio117"),
	PINCTRL_PIN(118, "gpio118"),
	PINCTRL_PIN(119, "gpio119"),
	PINCTRL_PIN(120, "gpio120"),
	PINCTRL_PIN(121, "gpio121"),
	PINCTRL_PIN(122, "gpio122"),
	PINCTRL_PIN(123, "gpio123"),
	PINCTRL_PIN(124, "gpio124"),
	PINCTRL_PIN(125, "gpio125"),
	PINCTRL_PIN(126, "gpio126"),
	PINCTRL_PIN(127, "gpio127"),
	PINCTRL_PIN(128, "gpio128"),
	PINCTRL_PIN(129, "gpio129"),
	PINCTRL_PIN(130, "gpio130"),
	PINCTRL_PIN(131, "gpio131"),
	PINCTRL_PIN(132, "gpio132"),
	PINCTRL_PIN(133, "gpio133"),
	PINCTRL_PIN(134, "gpio134"),
	PINCTRL_PIN(135, "gpio135"),
	PINCTRL_PIN(136, "gpio136"),
	PINCTRL_PIN(137, "gpio137"),
	PINCTRL_PIN(138, "gpio138"),
	PINCTRL_PIN(139, "gpio139"),
	PINCTRL_PIN(140, "gpio140"),
	PINCTRL_PIN(141, "gpio141"),
	PINCTRL_PIN(142, "gpio142"),
	PINCTRL_PIN(143, "gpio143"),
	PINCTRL_PIN(144, "gpio144"),
	PINCTRL_PIN(145, "gpio145"),
	PINCTRL_PIN(146, "gpio146"),
	PINCTRL_PIN(147, "gpio147"),
	PINCTRL_PIN(148, "gpio148"),
	PINCTRL_PIN(149, "gpio149"),
	PINCTRL_PIN(150, "gpio150"),
	PINCTRL_PIN(151, "gpio151"),
	PINCTRL_PIN(152, "gpio152"),
	PINCTRL_PIN(153, "gpio153"),
	PINCTRL_PIN(154, "gpio154"),
	PINCTRL_PIN(155, "gpio155"),
	PINCTRL_PIN(156, "gpio156"),
	PINCTRL_PIN(157, "gpio157"),
	PINCTRL_PIN(158, "gpio158"),
	PINCTRL_PIN(159, "gpio159"),
	PINCTRL_PIN(160, "gpio160"),
	PINCTRL_PIN(161, "gpio161"),
	PINCTRL_PIN(162, "gpio162"),
	PINCTRL_PIN(163, "gpio163"),
	PINCTRL_PIN(164, "gpio164"),
	PINCTRL_PIN(165, "gpio165"),
	PINCTRL_PIN(166, "gpio166"),
	PINCTRL_PIN(167, "gpio167"),
	PINCTRL_PIN(168, "gpio168"),
	PINCTRL_PIN(169, "gpio169"),
	PINCTRL_PIN(170, "gpio170"),
	PINCTRL_PIN(171, "gpio171"),
	PINCTRL_PIN(172, "gpio172"),
	PINCTRL_PIN(173, "gpio173"),
	PINCTRL_PIN(174, "gpio174"),
	PINCTRL_PIN(175, "gpio175"),
	PINCTRL_PIN(176, "gpio176"),
	PINCTRL_PIN(177, "gpio177"),
	PINCTRL_PIN(178, "gpio178"),
	PINCTRL_PIN(179, "gpio179"),
	PINCTRL_PIN(180, "gpio180"),
	PINCTRL_PIN(181, "gpio181"),
};

#define DECLARE_MSM_GPIO_PINS(pin) \
	static const unsigned int gpio##pin##_pins[] = { pin }
DECLARE_MSM_GPIO_PINS(0);
DECLARE_MSM_GPIO_PINS(1);
DECLARE_MSM_GPIO_PINS(2);
DECLARE_MSM_GPIO_PINS(3);
DECLARE_MSM_GPIO_PINS(4);
DECLARE_MSM_GPIO_PINS(5);
DECLARE_MSM_GPIO_PINS(6);
DECLARE_MSM_GPIO_PINS(7);
DECLARE_MSM_GPIO_PINS(8);
DECLARE_MSM_GPIO_PINS(9);
DECLARE_MSM_GPIO_PINS(10);
DECLARE_MSM_GPIO_PINS(11);
DECLARE_MSM_GPIO_PINS(12);
DECLARE_MSM_GPIO_PINS(13);
DECLARE_MSM_GPIO_PINS(14);
DECLARE_MSM_GPIO_PINS(15);
DECLARE_MSM_GPIO_PINS(16);
DECLARE_MSM_GPIO_PINS(17);
DECLARE_MSM_GPIO_PINS(18);
DECLARE_MSM_GPIO_PINS(19);
DECLARE_MSM_GPIO_PINS(20);
DECLARE_MSM_GPIO_PINS(21);
DECLARE_MSM_GPIO_PINS(22);
DECLARE_MSM_GPIO_PINS(23);
DECLARE_MSM_GPIO_PINS(24);
DECLARE_MSM_GPIO_PINS(25);
DECLARE_MSM_GPIO_PINS(26);
DECLARE_MSM_GPIO_PINS(27);
DECLARE_MSM_GPIO_PINS(28);
DECLARE_MSM_GPIO_PINS(29);
DECLARE_MSM_GPIO_PINS(30);
DECLARE_MSM_GPIO_PINS(31);
DECLARE_MSM_GPIO_PINS(32);
DECLARE_MSM_GPIO_PINS(33);
DECLARE_MSM_GPIO_PINS(34);
DECLARE_MSM_GPIO_PINS(35);
DECLARE_MSM_GPIO_PINS(36);
DECLARE_MSM_GPIO_PINS(37);
DECLARE_MSM_GPIO_PINS(38);
DECLARE_MSM_GPIO_PINS(39);
DECLARE_MSM_GPIO_PINS(40);
DECLARE_MSM_GPIO_PINS(41);
DECLARE_MSM_GPIO_PINS(42);
DECLARE_MSM_GPIO_PINS(43);
DECLARE_MSM_GPIO_PINS(44);
DECLARE_MSM_GPIO_PINS(45);
DECLARE_MSM_GPIO_PINS(46);
DECLARE_MSM_GPIO_PINS(47);
DECLARE_MSM_GPIO_PINS(48);
DECLARE_MSM_GPIO_PINS(49);
DECLARE_MSM_GPIO_PINS(50);
DECLARE_MSM_GPIO_PINS(51);
DECLARE_MSM_GPIO_PINS(52);
DECLARE_MSM_GPIO_PINS(53);
DECLARE_MSM_GPIO_PINS(54);
DECLARE_MSM_GPIO_PINS(55);
DECLARE_MSM_GPIO_PINS(56);
DECLARE_MSM_GPIO_PINS(57);
DECLARE_MSM_GPIO_PINS(58);
DECLARE_MSM_GPIO_PINS(59);
DECLARE_MSM_GPIO_PINS(60);
DECLARE_MSM_GPIO_PINS(61);
DECLARE_MSM_GPIO_PINS(62);
DECLARE_MSM_GPIO_PINS(63);
DECLARE_MSM_GPIO_PINS(64);
DECLARE_MSM_GPIO_PINS(65);
DECLARE_MSM_GPIO_PINS(66);
DECLARE_MSM_GPIO_PINS(67);
DECLARE_MSM_GPIO_PINS(68);
DECLARE_MSM_GPIO_PINS(69);
DECLARE_MSM_GPIO_PINS(70);
DECLARE_MSM_GPIO_PINS(71);
DECLARE_MSM_GPIO_PINS(72);
DECLARE_MSM_GPIO_PINS(73);
DECLARE_MSM_GPIO_PINS(74);
DECLARE_MSM_GPIO_PINS(75);
DECLARE_MSM_GPIO_PINS(76);
DECLARE_MSM_GPIO_PINS(77);
DECLARE_MSM_GPIO_PINS(78);
DECLARE_MSM_GPIO_PINS(79);
DECLARE_MSM_GPIO_PINS(80);
DECLARE_MSM_GPIO_PINS(81);
DECLARE_MSM_GPIO_PINS(82);
DECLARE_MSM_GPIO_PINS(83);
DECLARE_MSM_GPIO_PINS(84);
DECLARE_MSM_GPIO_PINS(85);
DECLARE_MSM_GPIO_PINS(86);
DECLARE_MSM_GPIO_PINS(87);
DECLARE_MSM_GPIO_PINS(88);
DECLARE_MSM_GPIO_PINS(89);
DECLARE_MSM_GPIO_PINS(90);
DECLARE_MSM_GPIO_PINS(91);
DECLARE_MSM_GPIO_PINS(92);
DECLARE_MSM_GPIO_PINS(93);
DECLARE_MSM_GPIO_PINS(94);
DECLARE_MSM_GPIO_PINS(95);
DECLARE_MSM_GPIO_PINS(96);
DECLARE_MSM_GPIO_PINS(97);
DECLARE_MSM_GPIO_PINS(98);
DECLARE_MSM_GPIO_PINS(99);
DECLARE_MSM_GPIO_PINS(100);
DECLARE_MSM_GPIO_PINS(101);
DECLARE_MSM_GPIO_PINS(102);
DECLARE_MSM_GPIO_PINS(103);
DECLARE_MSM_GPIO_PINS(104);
DECLARE_MSM_GPIO_PINS(105);
DECLARE_MSM_GPIO_PINS(106);
DECLARE_MSM_GPIO_PINS(107);
DECLARE_MSM_GPIO_PINS(108);
DECLARE_MSM_GPIO_PINS(109);
DECLARE_MSM_GPIO_PINS(110);
DECLARE_MSM_GPIO_PINS(111);
DECLARE_MSM_GPIO_PINS(112);
DECLARE_MSM_GPIO_PINS(113);
DECLARE_MSM_GPIO_PINS(114);
DECLARE_MSM_GPIO_PINS(115);
DECLARE_MSM_GPIO_PINS(116);
DECLARE_MSM_GPIO_PINS(117);
DECLARE_MSM_GPIO_PINS(118);
DECLARE_MSM_GPIO_PINS(119);
DECLARE_MSM_GPIO_PINS(120);
DECLARE_MSM_GPIO_PINS(121);
DECLARE_MSM_GPIO_PINS(122);
DECLARE_MSM_GPIO_PINS(123);
DECLARE_MSM_GPIO_PINS(124);
DECLARE_MSM_GPIO_PINS(125);
DECLARE_MSM_GPIO_PINS(126);
DECLARE_MSM_GPIO_PINS(127);
DECLARE_MSM_GPIO_PINS(128);
DECLARE_MSM_GPIO_PINS(129);
DECLARE_MSM_GPIO_PINS(130);
DECLARE_MSM_GPIO_PINS(131);
DECLARE_MSM_GPIO_PINS(132);
DECLARE_MSM_GPIO_PINS(133);
DECLARE_MSM_GPIO_PINS(134);
DECLARE_MSM_GPIO_PINS(135);
DECLARE_MSM_GPIO_PINS(136);
DECLARE_MSM_GPIO_PINS(137);
DECLARE_MSM_GPIO_PINS(138);
DECLARE_MSM_GPIO_PINS(139);
DECLARE_MSM_GPIO_PINS(140);
DECLARE_MSM_GPIO_PINS(141);
DECLARE_MSM_GPIO_PINS(142);
DECLARE_MSM_GPIO_PINS(143);
DECLARE_MSM_GPIO_PINS(144);
DECLARE_MSM_GPIO_PINS(145);
DECLARE_MSM_GPIO_PINS(146);
DECLARE_MSM_GPIO_PINS(147);
DECLARE_MSM_GPIO_PINS(148);
DECLARE_MSM_GPIO_PINS(149);
DECLARE_MSM_GPIO_PINS(150);
DECLARE_MSM_GPIO_PINS(151);
DECLARE_MSM_GPIO_PINS(152);
DECLARE_MSM_GPIO_PINS(153);
DECLARE_MSM_GPIO_PINS(154);
DECLARE_MSM_GPIO_PINS(155);
DECLARE_MSM_GPIO_PINS(156);
DECLARE_MSM_GPIO_PINS(157);
DECLARE_MSM_GPIO_PINS(158);
DECLARE_MSM_GPIO_PINS(159);
DECLARE_MSM_GPIO_PINS(160);
DECLARE_MSM_GPIO_PINS(161);
DECLARE_MSM_GPIO_PINS(162);
DECLARE_MSM_GPIO_PINS(163);
DECLARE_MSM_GPIO_PINS(164);
DECLARE_MSM_GPIO_PINS(165);
DECLARE_MSM_GPIO_PINS(166);
DECLARE_MSM_GPIO_PINS(167);
DECLARE_MSM_GPIO_PINS(168);
DECLARE_MSM_GPIO_PINS(169);
DECLARE_MSM_GPIO_PINS(170);
DECLARE_MSM_GPIO_PINS(171);
DECLARE_MSM_GPIO_PINS(172);
DECLARE_MSM_GPIO_PINS(173);
DECLARE_MSM_GPIO_PINS(174);
DECLARE_MSM_GPIO_PINS(175);
DECLARE_MSM_GPIO_PINS(176);
DECLARE_MSM_GPIO_PINS(177);
DECLARE_MSM_GPIO_PINS(178);
DECLARE_MSM_GPIO_PINS(179);
DECLARE_MSM_GPIO_PINS(180);
DECLARE_MSM_GPIO_PINS(181);

/*
 * Function groups for MSM7x30
 * Based on HP Pre3 (Rib) board GPIO usage
 */

/* I2C */
static const char * const i2c_groups[] = {
	"gpio16", "gpio17"  /* I2C SCL/SDA */
};

static const char * const i2c2_groups[] = {
	"gpio70", "gpio71"  /* I2C2 SCL/SDA (camera) */
};

static const char * const qup_i2c_groups[] = {
	"gpio16", "gpio17"  /* QUP I2C (GSBI-based) */
};

/* UART */
static const char * const uart1_groups[] = {
	"gpio138", "gpio139"  /* UART1 TX/RX */
};

static const char * const uart2_groups[] = {
	"gpio49", "gpio50"  /* UART2 TX/RX */
};

static const char * const uart3_groups[] = {
	"gpio53", "gpio54", "gpio55", "gpio57"  /* UART3 RX/TX/CTS/RTS (debug) */
};

static const char * const uart1dm_groups[] = {
	"gpio134", "gpio135", "gpio136", "gpio137"  /* UART1DM RTS/CTS/RX/TX (Bluetooth) */
};

static const char * const uart2dm_groups[] = {
	"gpio19", "gpio20", "gpio21", "gpio108"  /* UART2DM */
};

/* SDC/MMC */
static const char * const sdc1_groups[] = {
	"gpio64", "gpio65", "gpio66", "gpio67", "gpio68", "gpio69",
	"gpio112", "gpio113", "gpio114", "gpio115"  /* SDC1 CLK/CMD/DATA0-7 (eMMC) */
};

static const char * const sdc3_groups[] = {
	"gpio110", "gpio111", "gpio116", "gpio117", "gpio118", "gpio119"  /* SDC3 CLK/CMD/DATA0-3 (WiFi) */
};

static const char * const sdc4_groups[] = {
	"gpio58", "gpio59", "gpio60", "gpio61", "gpio62", "gpio63"  /* SDC4 */
};

/* Camera */
static const char * const cam_mclk_groups[] = {
	"gpio15"  /* Camera MCLK */
};

/* Display */
static const char * const mddi_groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7",
	"gpio8", "gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14"  /* MDDI data */
};

static const char * const lcdc_groups[] = {
	"gpio90", "gpio91", "gpio92", "gpio93", "gpio94", "gpio95", "gpio96", "gpio97"  /* LCDC */
};

/* USB */
static const char * const usb_groups[] = {
	"gpio21", "gpio33", "gpio34", "gpio35"  /* USB */
};

/* Audio */
static const char * const mi2s_groups[] = {
	"gpio101", "gpio102", "gpio103", "gpio104", "gpio105", "gpio106", "gpio107"  /* MI2S */
};

static const char * const pcm_groups[] = {
	"gpio138", "gpio139", "gpio140", "gpio141"  /* PCM */
};

/* SPI */
static const char * const spi_groups[] = {
	"gpio45", "gpio46", "gpio47", "gpio48"  /* SPI */
};

/* Misc */
static const char * const gp_clk_groups[] = {
	"gpio30", "gpio31"  /* GP clock outputs */
};

static const char * const gpio_groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7",
	"gpio8", "gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14", "gpio15",
	"gpio16", "gpio17", "gpio18", "gpio19", "gpio20", "gpio21", "gpio22", "gpio23",
	"gpio24", "gpio25", "gpio26", "gpio27", "gpio28", "gpio29", "gpio30", "gpio31",
	"gpio32", "gpio33", "gpio34", "gpio35", "gpio36", "gpio37", "gpio38", "gpio39",
	"gpio40", "gpio41", "gpio42", "gpio43", "gpio44", "gpio45", "gpio46", "gpio47",
	"gpio48", "gpio49", "gpio50", "gpio51", "gpio52", "gpio53", "gpio54", "gpio55",
	"gpio56", "gpio57", "gpio58", "gpio59", "gpio60", "gpio61", "gpio62", "gpio63",
	"gpio64", "gpio65", "gpio66", "gpio67", "gpio68", "gpio69", "gpio70", "gpio71",
	"gpio72", "gpio73", "gpio74", "gpio75", "gpio76", "gpio77", "gpio78", "gpio79",
	"gpio80", "gpio81", "gpio82", "gpio83", "gpio84", "gpio85", "gpio86", "gpio87",
	"gpio88", "gpio89", "gpio90", "gpio91", "gpio92", "gpio93", "gpio94", "gpio95",
	"gpio96", "gpio97", "gpio98", "gpio99", "gpio100", "gpio101", "gpio102", "gpio103",
	"gpio104", "gpio105", "gpio106", "gpio107", "gpio108", "gpio109", "gpio110", "gpio111",
	"gpio112", "gpio113", "gpio114", "gpio115", "gpio116", "gpio117", "gpio118", "gpio119",
	"gpio120", "gpio121", "gpio122", "gpio123", "gpio124", "gpio125", "gpio126", "gpio127",
	"gpio128", "gpio129", "gpio130", "gpio131", "gpio132", "gpio133", "gpio134", "gpio135",
	"gpio136", "gpio137", "gpio138", "gpio139", "gpio140", "gpio141", "gpio142", "gpio143",
	"gpio144", "gpio145", "gpio146", "gpio147", "gpio148", "gpio149", "gpio150", "gpio151",
	"gpio152", "gpio153", "gpio154", "gpio155", "gpio156", "gpio157", "gpio158", "gpio159",
	"gpio160", "gpio161", "gpio162", "gpio163", "gpio164", "gpio165", "gpio166", "gpio167",
	"gpio168", "gpio169", "gpio170", "gpio171", "gpio172", "gpio173", "gpio174", "gpio175",
	"gpio176", "gpio177", "gpio178", "gpio179", "gpio180", "gpio181"
};

static const struct pinfunction msm7x30_functions[] = {
	MSM_PIN_FUNCTION(gpio),
	MSM_PIN_FUNCTION(i2c),
	MSM_PIN_FUNCTION(i2c2),
	MSM_PIN_FUNCTION(qup_i2c),
	MSM_PIN_FUNCTION(uart1),
	MSM_PIN_FUNCTION(uart2),
	MSM_PIN_FUNCTION(uart3),
	MSM_PIN_FUNCTION(uart1dm),
	MSM_PIN_FUNCTION(uart2dm),
	MSM_PIN_FUNCTION(sdc1),
	MSM_PIN_FUNCTION(sdc3),
	MSM_PIN_FUNCTION(sdc4),
	MSM_PIN_FUNCTION(cam_mclk),
	MSM_PIN_FUNCTION(mddi),
	MSM_PIN_FUNCTION(lcdc),
	MSM_PIN_FUNCTION(usb),
	MSM_PIN_FUNCTION(mi2s),
	MSM_PIN_FUNCTION(pcm),
	MSM_PIN_FUNCTION(spi),
	MSM_PIN_FUNCTION(gp_clk),
};

/*
 * Pin groups - each GPIO can be in GPIO mode or one of its alternate functions
 * The function numbers (f1-f7) map to the alternate function select values
 *
 * Based on legacy gpiomux-7x00.h:
 * - Function 0 = GPIO
 * - Functions 1-15 = Alternate functions (device-specific)
 */

#define PINGROUP(id, f1, f2, f3, f4, f5, f6, f7)		\
	{							\
		.grp = PINCTRL_PINGROUP("gpio" #id,		\
			gpio##id##_pins,			\
			ARRAY_SIZE(gpio##id##_pins)),		\
		.funcs = (int[]){				\
			msm_mux_gpio,				\
			msm_mux_##f1,				\
			msm_mux_##f2,				\
			msm_mux_##f3,				\
			msm_mux_##f4,				\
			msm_mux_##f5,				\
			msm_mux_##f6,				\
			msm_mux_##f7				\
		},						\
		.nfuncs = 8,					\
		.ctl_reg = 0x1000 + 0x10 * id,			\
		.io_reg = 0x1004 + 0x10 * id,			\
		.intr_cfg_reg = 0x1008 + 0x10 * id,		\
		.intr_status_reg = 0x100c + 0x10 * id,		\
		.intr_target_reg = 0x1008 + 0x10 * id,		\
		.mux_bit = 2,					\
		.pull_bit = 15,					\
		.drv_bit = 17,					\
		.oe_bit = 9,					\
		.in_bit = 0,					\
		.out_bit = 1,					\
		.intr_enable_bit = 0,				\
		.intr_status_bit = 0,				\
		.intr_target_bit = 5,				\
		.intr_target_kpss_val = 0,			\
		.intr_raw_status_bit = 4,			\
		.intr_polarity_bit = 3,				\
		.intr_detection_bit = 2,			\
		.intr_detection_width = 1,			\
	}

/* Use _ for "no function" placeholder */
#define _ gpio

/*
 * MSM7x30 pingroups
 * Note: Function assignments are approximate based on known HP Pre3 usage
 * Full verification requires datasheets or additional reverse engineering
 */
static const struct msm_pingroup msm7x30_groups[] = {
	/* MDDI data lines (0-14) */
	PINGROUP(0, mddi, _, _, _, _, _, _),
	PINGROUP(1, mddi, _, _, _, _, _, _),
	PINGROUP(2, mddi, _, _, _, _, _, _),
	PINGROUP(3, mddi, _, _, _, _, _, _),
	PINGROUP(4, mddi, _, _, _, _, _, _),
	PINGROUP(5, mddi, _, _, _, _, _, _),
	PINGROUP(6, mddi, _, _, _, _, _, _),
	PINGROUP(7, mddi, _, _, _, _, _, _),
	PINGROUP(8, mddi, _, _, _, _, _, _),
	PINGROUP(9, mddi, _, _, _, _, _, _),
	PINGROUP(10, mddi, _, _, _, _, _, _),
	PINGROUP(11, mddi, _, _, _, _, _, _),
	PINGROUP(12, mddi, _, _, _, _, _, _),
	PINGROUP(13, mddi, _, _, _, _, _, _),
	PINGROUP(14, mddi, _, _, _, _, _, _),
	/* Camera MCLK */
	PINGROUP(15, cam_mclk, _, _, _, _, _, _),
	/* I2C */
	PINGROUP(16, i2c, qup_i2c, _, _, _, _, _),
	PINGROUP(17, i2c, qup_i2c, _, _, _, _, _),
	/* Misc GPIOs */
	PINGROUP(18, _, _, _, _, _, _, _),
	PINGROUP(19, uart2dm, _, _, _, _, _, _),
	PINGROUP(20, uart2dm, _, _, _, _, _, _),  /* Also slider open */
	PINGROUP(21, uart2dm, usb, _, _, _, _, _),
	PINGROUP(22, _, _, _, _, _, _, _),
	PINGROUP(23, _, _, _, _, _, _, _),
	PINGROUP(24, _, _, _, _, _, _, _),  /* LCD TE */
	PINGROUP(25, _, _, _, _, _, _, _),
	PINGROUP(26, _, _, _, _, _, _, _),
	PINGROUP(27, _, _, _, _, _, _, _),  /* PMIC IRQ */
	PINGROUP(28, _, _, _, _, _, _, _),
	PINGROUP(29, _, _, _, _, _, _, _),
	PINGROUP(30, gp_clk, _, _, _, _, _, _),  /* CTP_SHDN */
	PINGROUP(31, gp_clk, _, _, _, _, _, _),
	PINGROUP(32, _, _, _, _, _, _, _),
	PINGROUP(33, usb, _, _, _, _, _, _),
	PINGROUP(34, usb, _, _, _, _, _, _),
	PINGROUP(35, usb, _, _, _, _, _, _),
	PINGROUP(36, _, _, _, _, _, _, _),
	PINGROUP(37, _, _, _, _, _, _, _),  /* VOL_DOWN */
	PINGROUP(38, _, _, _, _, _, _, _),
	PINGROUP(39, _, _, _, _, _, _, _),
	PINGROUP(40, _, _, _, _, _, _, _),  /* POWER key */
	PINGROUP(41, _, _, _, _, _, _, _),
	PINGROUP(42, _, _, _, _, _, _, _),  /* LM8502 IRQ */
	PINGROUP(43, _, _, _, _, _, _, _),  /* VOL_UP */
	PINGROUP(44, _, _, _, _, _, _, _),  /* Slider close */
	/* SPI */
	PINGROUP(45, spi, _, _, _, _, _, _),
	PINGROUP(46, spi, _, _, _, _, _, _),
	PINGROUP(47, spi, _, _, _, _, _, _),
	PINGROUP(48, spi, _, _, _, _, _, _),  /* CTP_WAKE */
	PINGROUP(49, uart2, _, _, _, _, _, _),
	PINGROUP(50, uart2, _, _, _, _, _, _),
	PINGROUP(51, _, _, _, _, _, _, _),
	PINGROUP(52, _, _, _, _, _, _, _),
	/* UART3 (debug) */
	PINGROUP(53, uart3, _, _, _, _, _, _),  /* RX */
	PINGROUP(54, uart3, _, _, _, _, _, _),  /* TX */
	PINGROUP(55, uart3, _, _, _, _, _, _),  /* CTS */
	PINGROUP(56, _, _, _, _, _, _, _),
	PINGROUP(57, uart3, _, _, _, _, _, _),  /* RTS */
	/* SDC4 */
	PINGROUP(58, sdc4, _, _, _, _, _, _),
	PINGROUP(59, sdc4, _, _, _, _, _, _),
	PINGROUP(60, sdc4, _, _, _, _, _, _),
	PINGROUP(61, sdc4, _, _, _, _, _, _),
	PINGROUP(62, sdc4, _, _, _, _, _, _),
	PINGROUP(63, sdc4, _, _, _, _, _, _),
	/* SDC1 (eMMC) CLK/CMD */
	PINGROUP(64, sdc1, _, _, _, _, _, _),  /* CLK */
	PINGROUP(65, sdc1, _, _, _, _, _, _),  /* CMD */
	/* SDC1 DATA 0-3 */
	PINGROUP(66, sdc1, _, _, _, _, _, _),
	PINGROUP(67, sdc1, _, _, _, _, _, _),
	PINGROUP(68, sdc1, _, _, _, _, _, _),
	PINGROUP(69, sdc1, _, _, _, _, _, _),
	/* I2C2 (camera) */
	PINGROUP(70, i2c2, _, _, _, _, _, _),  /* SCL */
	PINGROUP(71, i2c2, _, _, _, _, _, _),  /* SDA */
	/* More GPIOs */
	PINGROUP(72, _, _, _, _, _, _, _),
	PINGROUP(73, _, _, _, _, _, _, _),
	PINGROUP(74, _, _, _, _, _, _, _),
	PINGROUP(75, _, _, _, _, _, _, _),
	PINGROUP(76, _, _, _, _, _, _, _),
	PINGROUP(77, _, _, _, _, _, _, _),
	PINGROUP(78, _, _, _, _, _, _, _),
	PINGROUP(79, _, _, _, _, _, _, _),
	PINGROUP(80, _, _, _, _, _, _, _),
	PINGROUP(81, _, _, _, _, _, _, _),
	PINGROUP(82, _, _, _, _, _, _, _),  /* Speaker amp */
	PINGROUP(83, _, _, _, _, _, _, _),
	PINGROUP(84, _, _, _, _, _, _, _),
	PINGROUP(85, _, _, _, _, _, _, _),  /* CTP_RX */
	PINGROUP(86, _, _, _, _, _, _, _),
	PINGROUP(87, _, _, _, _, _, _, _),
	PINGROUP(88, _, _, _, _, _, _, _),
	PINGROUP(89, _, _, _, _, _, _, _),
	/* LCDC */
	PINGROUP(90, lcdc, _, _, _, _, _, _),
	PINGROUP(91, lcdc, _, _, _, _, _, _),
	PINGROUP(92, lcdc, _, _, _, _, _, _),  /* Also MT9P013 reset */
	PINGROUP(93, lcdc, _, _, _, _, _, _),
	PINGROUP(94, lcdc, _, _, _, _, _, _),
	PINGROUP(95, lcdc, _, _, _, _, _, _),
	PINGROUP(96, lcdc, _, _, _, _, _, _),
	PINGROUP(97, lcdc, _, _, _, _, _, _),
	PINGROUP(98, _, _, _, _, _, _, _),
	PINGROUP(99, _, _, _, _, _, _, _),
	PINGROUP(100, _, _, _, _, _, _, _),
	/* MI2S */
	PINGROUP(101, mi2s, _, _, _, _, _, _),
	PINGROUP(102, mi2s, _, _, _, _, _, _),
	PINGROUP(103, mi2s, _, _, _, _, _, _),
	PINGROUP(104, mi2s, _, _, _, _, _, _),
	PINGROUP(105, mi2s, _, _, _, _, _, _),
	PINGROUP(106, mi2s, _, _, _, _, _, _),
	PINGROUP(107, mi2s, _, _, _, _, _, _),  /* Camera select */
	PINGROUP(108, uart2dm, _, _, _, _, _, _),
	PINGROUP(109, _, _, _, _, _, _, _),  /* MT9P013 power */
	/* SDC3 (WiFi) */
	PINGROUP(110, sdc3, _, _, _, _, _, _),  /* CLK */
	PINGROUP(111, sdc3, _, _, _, _, _, _),  /* CMD */
	/* SDC1 DATA 4-7 */
	PINGROUP(112, sdc1, _, _, _, _, _, _),
	PINGROUP(113, sdc1, _, _, _, _, _, _),
	PINGROUP(114, sdc1, _, _, _, _, _, _),
	PINGROUP(115, sdc1, _, _, _, _, _, _),
	/* SDC3 DATA 0-3 */
	PINGROUP(116, sdc3, _, _, _, _, _, _),
	PINGROUP(117, sdc3, _, _, _, _, _, _),
	PINGROUP(118, sdc3, _, _, _, _, _, _),
	PINGROUP(119, sdc3, _, _, _, _, _, _),
	PINGROUP(120, _, _, _, _, _, _, _),
	PINGROUP(121, _, _, _, _, _, _, _),
	PINGROUP(122, _, _, _, _, _, _, _),
	PINGROUP(123, _, _, _, _, _, _, _),
	PINGROUP(124, _, _, _, _, _, _, _),  /* LM8502 enable */
	PINGROUP(125, _, _, _, _, _, _, _),  /* LCD reset */
	PINGROUP(126, _, _, _, _, _, _, _),  /* OV7739 power */
	PINGROUP(127, _, _, _, _, _, _, _),  /* OV7739 reset */
	PINGROUP(128, _, _, _, _, _, _, _),  /* CTP reset */
	PINGROUP(129, _, _, _, _, _, _, _),
	PINGROUP(130, _, _, _, _, _, _, _),
	PINGROUP(131, _, _, _, _, _, _, _),
	PINGROUP(132, _, _, _, _, _, _, _),
	PINGROUP(133, _, _, _, _, _, _, _),
	/* UART1DM (Bluetooth) */
	PINGROUP(134, uart1dm, _, _, _, _, _, _),  /* RTS */
	PINGROUP(135, uart1dm, _, _, _, _, _, _),  /* CTS */
	PINGROUP(136, uart1dm, _, _, _, _, _, _),  /* RX */
	PINGROUP(137, uart1dm, _, _, _, _, _, _),  /* TX */
	/* PCM */
	PINGROUP(138, uart1, pcm, _, _, _, _, _),
	PINGROUP(139, uart1, pcm, _, _, _, _, _),
	PINGROUP(140, pcm, _, _, _, _, _, _),
	PINGROUP(141, pcm, _, _, _, _, _, _),
	PINGROUP(142, _, _, _, _, _, _, _),  /* WiFi IRQ */
	PINGROUP(143, _, _, _, _, _, _, _),
	PINGROUP(144, _, _, _, _, _, _, _),
	PINGROUP(145, _, _, _, _, _, _, _),  /* BT wake host */
	PINGROUP(146, _, _, _, _, _, _, _),
	PINGROUP(147, _, _, _, _, _, _, _),
	PINGROUP(148, _, _, _, _, _, _, _),
	PINGROUP(149, _, _, _, _, _, _, _),
	PINGROUP(150, _, _, _, _, _, _, _),
	PINGROUP(151, _, _, _, _, _, _, _),
	PINGROUP(152, _, _, _, _, _, _, _),
	PINGROUP(153, _, _, _, _, _, _, _),
	PINGROUP(154, _, _, _, _, _, _, _),
	PINGROUP(155, _, _, _, _, _, _, _),
	PINGROUP(156, _, _, _, _, _, _, _),
	PINGROUP(157, _, _, _, _, _, _, _),
	PINGROUP(158, _, _, _, _, _, _, _),
	PINGROUP(159, _, _, _, _, _, _, _),
	PINGROUP(160, _, _, _, _, _, _, _),
	PINGROUP(161, _, _, _, _, _, _, _),
	PINGROUP(162, _, _, _, _, _, _, _),  /* WiFi reset */
	PINGROUP(163, _, _, _, _, _, _, _),  /* BT reset */
	PINGROUP(164, _, _, _, _, _, _, _),  /* WiFi enable */
	PINGROUP(165, _, _, _, _, _, _, _),
	PINGROUP(166, _, _, _, _, _, _, _),
	PINGROUP(167, _, _, _, _, _, _, _),  /* LDO2 */
	PINGROUP(168, _, _, _, _, _, _, _),  /* LDO1 */
	PINGROUP(169, _, _, _, _, _, _, _),
	PINGROUP(170, _, _, _, _, _, _, _),
	PINGROUP(171, _, _, _, _, _, _, _),
	PINGROUP(172, _, _, _, _, _, _, _),
	PINGROUP(173, _, _, _, _, _, _, _),
	PINGROUP(174, _, _, _, _, _, _, _),
	PINGROUP(175, _, _, _, _, _, _, _),
	PINGROUP(176, _, _, _, _, _, _, _),
	PINGROUP(177, _, _, _, _, _, _, _),
	PINGROUP(178, _, _, _, _, _, _, _),
	PINGROUP(179, _, _, _, _, _, _, _),
	PINGROUP(180, _, _, _, _, _, _, _),  /* Ringer switch */
	PINGROUP(181, _, _, _, _, _, _, _),
};

#undef _

#define NUM_GPIO_PINGROUPS 182

static const struct msm_pinctrl_soc_data msm7x30_pinctrl = {
	.pins = msm7x30_pins,
	.npins = ARRAY_SIZE(msm7x30_pins),
	.functions = msm7x30_functions,
	.nfunctions = ARRAY_SIZE(msm7x30_functions),
	.groups = msm7x30_groups,
	.ngroups = ARRAY_SIZE(msm7x30_groups),
	.ngpios = NUM_GPIO_PINGROUPS,
};

static int msm7x30_pinctrl_probe(struct platform_device *pdev)
{
	return msm_pinctrl_probe(pdev, &msm7x30_pinctrl);
}

static const struct of_device_id msm7x30_pinctrl_of_match[] = {
	{ .compatible = "qcom,msm7x30-pinctrl", },
	{ },
};
MODULE_DEVICE_TABLE(of, msm7x30_pinctrl_of_match);

static struct platform_driver msm7x30_pinctrl_driver = {
	.driver = {
		.name = "msm7x30-pinctrl",
		.of_match_table = msm7x30_pinctrl_of_match,
	},
	.probe = msm7x30_pinctrl_probe,
	.remove = msm_pinctrl_remove,
};

static int __init msm7x30_pinctrl_init(void)
{
	return platform_driver_register(&msm7x30_pinctrl_driver);
}
arch_initcall(msm7x30_pinctrl_init);

static void __exit msm7x30_pinctrl_exit(void)
{
	platform_driver_unregister(&msm7x30_pinctrl_driver);
}
module_exit(msm7x30_pinctrl_exit);

MODULE_AUTHOR("Linux community");
MODULE_DESCRIPTION("Qualcomm MSM7x30 pinctrl driver");
MODULE_LICENSE("GPL v2");
