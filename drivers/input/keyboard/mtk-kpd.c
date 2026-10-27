// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 * Author Terry Chang <terry.chang@mediatek.com>
 */
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#ifdef CONFIG_KPD_CUSTOM_GPIO_KEY
#include <linux/of_gpio.h>
#endif

#define KPD_NAME	"mtk-kpd"

#define KP_STA			(0x0000)
#define KP_MEM1			(0x0004)
#define KP_MEM2			(0x0008)
#define KP_MEM3			(0x000c)
#define KP_MEM4			(0x0010)
#define KP_MEM5			(0x0014)
#define KP_DEBOUNCE		(0x0018)
#define KP_SEL			(0X0020)
#define KP_EN			(0x0024)

#define KPD_DEBOUNCE_MASK	((1U << 14) - 1)
#define KPD_DOUBLE_KEY_MASK	(1U << 0)

#define KPD_NUM_MEMS	5
#define KPD_MEM5_BITS	8
#define KPD_NUM_KEYS	72	/* 4 * 16 + KPD_MEM5_BITS */

struct mtk_keypad {
	struct input_dev *input_dev;
	struct wakeup_source *suspend_lock;
	struct tasklet_struct tasklet;
	struct clk *clk;
	void __iomem *base;
	unsigned int irqnr;
	u32 key_debounce;
	u32 use_extend_type;
	u32 hw_map_num;
	u32 hw_init_map[KPD_NUM_KEYS];
	u16 keymap_state[KPD_NUM_MEMS];
};

static void kpd_get_keymap_state(void __iomem *kp_base, u16 state[])
{
	state[0] = readw(kp_base + KP_MEM1);
	state[1] = readw(kp_base + KP_MEM2);
	state[2] = readw(kp_base + KP_MEM3);
	state[3] = readw(kp_base + KP_MEM4);
	state[4] = readw(kp_base + KP_MEM5);
}

static void kpd_double_key_enable(void __iomem *kp_base, int en)
{
	u16 tmp;

	tmp = *(u16*)KP_SEL;
	if (en)
		writew((u16)(tmp | KPD_DOUBLE_KEY_MASK), kp_base + KP_SEL);
	else
		writew((u16)(tmp & ~KPD_DOUBLE_KEY_MASK), kp_base + KP_SEL);
}

static void enable_kpd(void __iomem *kp_base, int en)
{
	writew((u16)(en), kp_base + KP_EN);
}

static void kpd_keymap_handler(unsigned long data)
{
	int i, j;
	int pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, keycode;
	void *dest;
	struct mtk_keypad *keypad = (struct mtk_keypad *)data;

	kpd_get_keymap_state(keypad->base, new_state);

	__pm_wakeup_event(keypad->suspend_lock, 500);

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ keypad->keymap_state[i];
		if (!change)
			continue;

		for (j = 0; j < 16U; j++) {
			mask = (u16) 1 << j;
			if (!(change & mask))
				continue;

			hw_keycode = (i << 4) + j;

			if (hw_keycode >= KPD_NUM_KEYS)
				continue;

			/* bit is 1: not pressed, 0: pressed */
			pressed = (new_state[i] & mask) == 0U;
			pr_info("(%s) HW keycode = %d\n",
				(pressed) ? "pressed" : "released",
					hw_keycode);

			keycode = keypad->hw_init_map[hw_keycode];
			if (!keycode)
				continue;
			input_report_key(keypad->input_dev, keycode, pressed);
			input_sync(keypad->input_dev);
			pr_info("report Linux keycode = %d\n", keycode);
		}
	}

	dest = memcpy(keypad->keymap_state, new_state, sizeof(new_state));
	enable_irq(keypad->irqnr);
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	struct mtk_keypad *keypad = dev_id;

	disable_irq_nosync(keypad->irqnr);
	tasklet_schedule(&keypad->tasklet);
	return IRQ_HANDLED;
}

static int kpd_get_dts_info(struct mtk_keypad *keypad,
				struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "mediatek,key-debounce-ms",
		&keypad->key_debounce);
	if (ret) {
		pr_err("read mediatek,key-debounce-ms error.\n");
		return ret;
	}

	ret = of_property_read_u32(node, "mediatek, use-extend-type",
		&keypad->use_extend_type);
	if (ret) {
		pr_err("read mediatek,use-extend-type error.\n");
		keypad->use_extend_type = 0;
	}

	ret = of_property_read_u32(node, "mediatek,hw-map-num",
		&keypad->hw_map_num);
	if (ret) {
		pr_err("read mediatek,hw-map-num error.\n");
		return ret;
	}

	if (keypad->hw_map_num > KPD_NUM_KEYS) {
		pr_err("hw-map-num error, it cannot bigger than %d.\n",
			KPD_NUM_KEYS);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(node, "mediatek,hw-init-map",
		keypad->hw_init_map, keypad->hw_map_num);

	if (ret) {
		pr_err("hw-init-map was not defined in dts.\n");
		return ret;
	}

	pr_debug("deb= %d\n", keypad->key_debounce);

	return 0;
}
#ifdef CONFIG_KPD_CUSTOM_GPIO_KEY
int hall_state;//low for near;high for leave
static int hall_ext_irq;
static bool hall_irq_disabled=false;
static   unsigned int hall_gpio;

int nstart_get_hall_state(void){
    return hall_state;
}
EXPORT_SYMBOL(nstart_get_hall_state);
void hall_irq_disable(void)
{
	if (!hall_irq_disabled) {
		disable_irq_nosync(hall_ext_irq);
		hall_irq_disabled = true;
	}
}

void hall_irq_enable(void)
{
	if (hall_irq_disabled) {
		enable_irq(hall_ext_irq);
		hall_irq_disabled = false;
	}
}

static irqreturn_t hall_eint_handler(int irq, void *data)
{
	int status = 0;
	struct mtk_keypad *keypad = data;

	hall_irq_disable();
	status = gpio_get_value(hall_gpio);
	if (hall_state == 0 && status == 0) {
		irq_set_irq_type(hall_ext_irq, IRQF_TRIGGER_RISING);
		hall_state = 1;
		input_report_key(keypad->input_dev, KEY_F2, 1);
		input_sync(keypad->input_dev);
		input_report_key(keypad->input_dev, KEY_F2, 0);
		input_sync(keypad->input_dev);
	} else if(hall_state == 1 && status == 1){	
		irq_set_irq_type(hall_ext_irq, IRQF_TRIGGER_FALLING);
		hall_state = 0;
		input_report_key(keypad->input_dev, KEY_F3, 1);
		input_sync(keypad->input_dev);
		input_report_key(keypad->input_dev, KEY_F3, 0);
		input_sync(keypad->input_dev);
	}
	pr_info("%s:pio level=%d\n",__func__,gpio_get_value(hall_gpio));
	hall_irq_enable();
	return IRQ_HANDLED;
}

void hall_eint_register(struct platform_device *pdev)
{
	int ints[2] = {0, 0};
	int ret;
	struct device_node *node;
	struct mtk_keypad *keypad = platform_get_drvdata(pdev);

	pr_info("%s:enter \n",__func__);
	node = of_find_compatible_node(NULL, NULL, "mediatek,hall");
	if (!node)
	{
		pr_info("%s: can't find compatible node\n",__func__);
	}
	else {
		hall_ext_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(hall_ext_irq, hall_eint_handler,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "hall-eint", keypad);
		pr_info("hall_ext_irq request_irq ret=%d\n",ret);
		if (ret > 0)
		{
			pr_info("%s:EINT IRQ LINE NOT AVAILABLE\n",__func__);
		}
		enable_irq_wake(hall_ext_irq);
		hall_gpio = of_get_named_gpio(node, "cfg-pin", 0);
 		of_property_read_u32(node, "debounce", &ints[0]);
		pr_info("%s:cfg-pin=%d,debounce=%d\n",__func__,hall_gpio,ints[0]);
		if (gpio_is_valid(hall_gpio)) {
		    ret = gpio_request(hall_gpio, "hall gpio");
		    if (ret) {
		        pr_err("%s: unable to request hall  gpio[%d]\n",__func__, hall_gpio);
		       return;
		    }
			gpio_set_debounce(hall_gpio, ints[0]);
			if (gpio_get_value(hall_gpio)){
				hall_state = 0;
				irq_set_irq_type(hall_ext_irq, IRQF_TRIGGER_FALLING);
			}else{
				hall_state = 1;
				irq_set_irq_type(hall_ext_irq, IRQF_TRIGGER_RISING);
			}	
		    msleep(4);
			return;
		}
		else {
		    pr_err("%s: cfg gpio not provided\\n", __func__);
		    return;
		}
	}
}

void hall_eint_unregister(struct platform_device *pdev){
	free_irq(hall_ext_irq,NULL);
	if (gpio_is_valid(hall_gpio))
		gpio_free(hall_gpio);
}
static ssize_t hall_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	pr_info("%s:enter\n", __func__);
	len += snprintf(buf + len, PAGE_SIZE - len,
					"%d\n", hall_state);
	return len;
}

static DEVICE_ATTR( hall_state, S_IWUSR | S_IRUGO,
		 hall_state_show,  NULL);

static int touch_key_state;//low for near;high for leave
static int touch_key_ext_irq;
static bool touch_key_irq_disabled=false;
static   unsigned int touch_key_gpio;
static   unsigned int touch_key_pwr_gpio;

void touch_key_irq_disable(void)
{
	if (!touch_key_irq_disabled) {
		disable_irq_nosync(touch_key_ext_irq);
		touch_key_irq_disabled = true;
	}
}

void touch_key_irq_enable(void)
{
	if (touch_key_irq_disabled) {
		enable_irq(touch_key_ext_irq);
		touch_key_irq_disabled = false;
	}
}

static irqreturn_t touch_key_eint_handler(int irq, void *data)
{
	int status = 0;
	struct mtk_keypad *keypad = data;

	touch_key_irq_disable();
	status = gpio_get_value(touch_key_gpio);
	if (touch_key_state == 0 && status == 0) {
		irq_set_irq_type(touch_key_ext_irq, IRQF_TRIGGER_RISING);
		touch_key_state = 1;
		input_report_key(keypad->input_dev, KEY_F4, 1);
		input_sync(keypad->input_dev);
	} else if(touch_key_state == 1 && status == 1){	
		irq_set_irq_type(touch_key_ext_irq, IRQF_TRIGGER_FALLING);
		touch_key_state = 0;
		input_report_key(keypad->input_dev, KEY_F4, 0);
		input_sync(keypad->input_dev);
	}
	pr_info("%s:pio level=%d\n",__func__,gpio_get_value(touch_key_gpio));
	touch_key_irq_enable();
	return IRQ_HANDLED;
}

void touch_key_eint_register(struct platform_device *pdev)
{
	int ints[2] = {0, 0};
	int ret;
	struct device_node *node;
	struct mtk_keypad *keypad = platform_get_drvdata(pdev);

	pr_info("%s:enter \n",__func__);
	node = of_find_compatible_node(NULL, NULL, "mediatek,touch_key");
	if (!node)
	{
		pr_info("%s: can't find compatible node\n",__func__);
	}
	else {
		touch_key_ext_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_key_ext_irq, touch_key_eint_handler,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "touch_key-eint", keypad);
		pr_info("touch_key request_irq ret=%d\n",ret);
		if (ret > 0)
		{
			pr_info("%s:EINT IRQ LINE NOT AVAILABLE\n",__func__);
		}
//		enable_irq_wake(touch_key_ext_irq);
		touch_key_gpio = of_get_named_gpio(node, "cfg-pin", 0);
 		of_property_read_u32(node, "debounce", &ints[0]);
		pr_info("%s:cfg-pin=%d,debounce=%d\n",__func__,touch_key_gpio,ints[0]);
		if (gpio_is_valid(touch_key_gpio)) {			
			ret = gpio_request(touch_key_gpio, "touch_key gpio");
			if (ret) {
			    pr_err("%s: unable to request touch_key  gpio[%d]\n",__func__, touch_key_gpio);
			   return;
			}
			gpio_set_debounce(touch_key_gpio, ints[0]);
			if (gpio_get_value(touch_key_gpio)){
				touch_key_state = 0;
				irq_set_irq_type(touch_key_ext_irq, IRQF_TRIGGER_FALLING);
			}else{
				touch_key_state = 1;
				irq_set_irq_type(touch_key_ext_irq, IRQF_TRIGGER_RISING);
			}	
		    	msleep(4);
		}
		else {
		    pr_err("%s: cfg gpio not provided\\n", __func__);
		    return;
		}
		touch_key_pwr_gpio = of_get_named_gpio(node, "pwr-pin", 0);
		if (gpio_is_valid(touch_key_pwr_gpio)) {			
			ret = gpio_request(touch_key_pwr_gpio, "touch_key pwr gpio");
			if (ret) {
			    pr_err("%s: unable to request touch_key pwr gpio[%d]\n",__func__, touch_key_pwr_gpio);
			   return;
			}
			gpio_direction_output(touch_key_pwr_gpio, 1);
		}
		else {
		    pr_err("%s:  touch_key pwr gpio not provided\\n", __func__);
		}
	}
}

void touch_key_eint_unregister(struct platform_device *pdev){
	free_irq(touch_key_ext_irq,NULL);
	if (gpio_is_valid(touch_key_gpio))
		gpio_free(touch_key_gpio);
	if (gpio_is_valid(touch_key_pwr_gpio)){
		gpio_direction_output(touch_key_pwr_gpio, 0);
		gpio_free(touch_key_pwr_gpio);
	}
}

static ssize_t touch_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	pr_info("%s:enter\n", __func__);
	len += snprintf(buf + len, PAGE_SIZE - len,
					"%d\n", touch_key_state);
	return len;
}

static DEVICE_ATTR( touch_state, S_IWUSR | S_IRUGO,
		 touch_state_show,  NULL);


void custom_key_eint_unregister(struct platform_device *pdev){
	hall_eint_unregister(pdev);
	touch_key_eint_unregister(pdev);
}

void custom_key_eint_register(struct platform_device *pdev)
{
	struct mtk_keypad *keypad = platform_get_drvdata(pdev);

	__set_bit(KEY_F4,keypad->input_dev->keybit);//touch key
	__set_bit(KEY_F2, keypad->input_dev->keybit);//hall key
	__set_bit(KEY_F3, keypad->input_dev->keybit);//hall key
	hall_eint_register(pdev);
	touch_key_eint_register(pdev);
}		 

static struct attribute *custom_key_attributes[] = {
	&dev_attr_hall_state.attr,
	&dev_attr_touch_state.attr,
	NULL
};

static struct attribute_group custom_key_attribute_group = {
	.attrs = custom_key_attributes
};

#endif

static int kpd_pdrv_probe(struct platform_device *pdev)
{
	struct mtk_keypad *keypad;
	struct resource *res;
	int i;
	int ret;

	keypad = devm_kzalloc(&pdev->dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	keypad->clk = devm_clk_get(&pdev->dev, "kpd");
	if (IS_ERR(keypad->clk))
		return PTR_ERR(keypad->clk);

	ret = clk_prepare_enable(keypad->clk);
	if (ret) {
		pr_err("cannot prepare/enable keypad clock\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err_unprepare_clk;
	}

	keypad->base = devm_ioremap(&pdev->dev, res->start,
			resource_size(res));
	if (!keypad->base) {
		pr_err("KP iomap failed\n");
		ret = -EBUSY;
		goto err_unprepare_clk;
	}

	keypad->irqnr = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!keypad->irqnr) {
		pr_err("KP get irqnr failed\n");
		ret = -ENODEV;
		goto err_unprepare_clk;
	}

	ret = kpd_get_dts_info(keypad, pdev->dev.of_node);
	if (ret) {
		pr_err("get dts info failed.\n");
		goto err_unprepare_clk;
	}

	memset(keypad->keymap_state, 0xff, sizeof(keypad->keymap_state));

	keypad->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!keypad->input_dev) {
		pr_err("input allocate device fail.\n");
		ret = -ENOMEM;
		goto err_unprepare_clk;
	}

	keypad->input_dev->name = KPD_NAME;
	keypad->input_dev->id.bustype = BUS_HOST;
	keypad->input_dev->dev.parent = &pdev->dev;

	__set_bit(EV_KEY, keypad->input_dev->evbit);

	if (!keypad->use_extend_type) {
		for (i = 17; i < KPD_NUM_KEYS; i += 9)
			keypad->hw_init_map[i] = 0;
	}

	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (keypad->hw_init_map[i])
			__set_bit(keypad->hw_init_map[i],
				keypad->input_dev->keybit);
	}

	if (keypad->use_extend_type)
		kpd_double_key_enable(keypad->base, 1);

	ret = input_register_device(keypad->input_dev);
	if (ret) {
		pr_err("register input device failed (%d)\n", ret);
		goto err_unprepare_clk;
	}

	input_set_drvdata(keypad->input_dev, keypad);

	keypad->suspend_lock = wakeup_source_register(NULL, "kpd wakelock");
	if (!keypad->suspend_lock) {
		pr_err("wakeup source init failed.\n");
		goto err_unregister_device;
	}

	tasklet_init(&keypad->tasklet, kpd_keymap_handler,
					(unsigned long)keypad);

	writew((u16)(keypad->key_debounce & KPD_DEBOUNCE_MASK),
			keypad->base + KP_DEBOUNCE);

	/* register IRQ */
	ret = request_irq(keypad->irqnr, kpd_irq_handler, IRQF_TRIGGER_NONE,
			KPD_NAME, keypad);
	if (ret) {
		pr_err("register IRQ failed (%d)\n", ret);
		goto err_irq;
	}

	ret = enable_irq_wake(keypad->irqnr);
	if (ret < 0)
		pr_err("irq %d enable irq wake fail\n", keypad->irqnr);

	platform_set_drvdata(pdev,keypad);
	enable_kpd(keypad->base, 1);
#ifdef CONFIG_KPD_CUSTOM_GPIO_KEY
	custom_key_eint_register(pdev);
	ret = sysfs_create_group(&pdev->dev.kobj,&custom_key_attribute_group);
	if (ret < 0) {
		pr_err("%s:error creating sysfs attr files",__func__);
		goto err_irq;
	}
#endif

	return 0;

err_irq:
	tasklet_kill(&keypad->tasklet);

err_unregister_device:
	input_unregister_device(keypad->input_dev);

err_unprepare_clk:
	clk_disable_unprepare(keypad->clk);

	return ret;
}

static int kpd_pdrv_remove(struct platform_device *pdev)
{
	struct mtk_keypad *keypad = platform_get_drvdata(pdev);

	tasklet_kill(&keypad->tasklet);
	wakeup_source_unregister(keypad->suspend_lock);
	input_unregister_device(keypad->input_dev);
	clk_disable_unprepare(keypad->clk);
#ifdef CONFIG_KPD_CUSTOM_GPIO_KEY
	custom_key_eint_unregister(pdev);
	sysfs_remove_group(&pdev->dev.kobj,&custom_key_attribute_group);
#endif

	return 0;
}

static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_KPD_CUSTOM_GPIO_KEY
	if (gpio_is_valid(touch_key_pwr_gpio)){
		touch_key_irq_disable();
		gpio_direction_output(touch_key_pwr_gpio, 0);
		pr_err("%s:enter",__func__);
	}
#endif
	return 0;
}

static int kpd_pdrv_resume(struct platform_device *pdev)
{
#ifdef CONFIG_KPD_CUSTOM_GPIO_KEY
	if (gpio_is_valid(touch_key_pwr_gpio)){
		gpio_direction_output(touch_key_pwr_gpio, 1);
	    	msleep(50);
		touch_key_irq_enable();
		pr_err("%s:enter",__func__);
	}
#endif
	return 0;
}

static const struct of_device_id kpd_of_match[] = {
	{.compatible = "mediatek,mt6779-keypad"},
	{.compatible = "mediatek,kp"},
	{}
};

static struct platform_driver kpd_pdrv = {
	.probe = kpd_pdrv_probe,
	.remove = kpd_pdrv_remove,
	.suspend = kpd_pdrv_suspend,
	.resume = kpd_pdrv_resume,
	.driver = {
		   .name = KPD_NAME,
		   .of_match_table = kpd_of_match,
	},
};
module_platform_driver(kpd_pdrv);

MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver");
MODULE_LICENSE("GPL");
