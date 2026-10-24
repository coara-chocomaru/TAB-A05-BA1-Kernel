/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* Copyright (C) 2019 SANYO Techno Solutions Tottori Co., Ltd.
*
*/

#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <linux/i2c.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#endif

#include "lcm_drv.h"

static struct regulator *lcm_vgp;

/***********************************************/
/***********************************************/

static unsigned int GPIO_LCD_PWR;
static unsigned int GPIO_LCD_PWR2;
static unsigned int GPIO_LCD_PWR3;
static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LCD_RST;

static unsigned int GPIO_TP_RST;

/* get LDO supply */
static int kd_lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_info("LCM: %s enter\n", __func__);

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		pr_info("failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_info("LCM: lcm get supply ok.\n");

	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_info("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

static int kd_lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_info("LCM: %s enter\n", __func__);

	if (lcm_vgp == NULL)
		return 0;

	pr_info("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_info("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		pr_info("LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_info("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n",
			volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_info("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

static int kd_lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (lcm_vgp == NULL)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_info("LCM: lcm query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_info("LCM: lcm failed to disable lcm_vgp: %d\n",
				ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_info("LCM: lcm regulator disable pass\n");
	}

	return ret;
}

static int lcm_get_gpio(struct device *dev)
{
	GPIO_LCD_PWR = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr", 0);
	gpio_request(GPIO_LCD_PWR, "GPIO_LCD_PWR");
	pr_err("[Kernel/LCM] GPIO_LCD_PWR = 0x%x\n", GPIO_LCD_PWR);

	GPIO_LCD_PWR2 = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr2", 0);
	gpio_request(GPIO_LCD_PWR2, "GPIO_LCD_PWR2");
	pr_err("[Kernel/LCM] GPIO_LCD_PWR2 = 0x%x\n", GPIO_LCD_PWR2);

	GPIO_LCD_PWR3 = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr3", 0);
	gpio_request(GPIO_LCD_PWR3, "GPIO_LCD_PWR3");
	pr_err("[Kernel/LCM] GPIO_LCD_PWR3 = 0x%x\n", GPIO_LCD_PWR3);

	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	pr_err("[Kernel/LCM] GPIO_LCD_BL_EN = 0x%x\n", GPIO_LCD_BL_EN);

	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	pr_err("[Kernel/LCM] GPIO_LCD_RST = 0x%x\n", GPIO_LCD_RST);

	GPIO_TP_RST = of_get_named_gpio(dev->of_node, "gpio_tp_rst", 0);
	gpio_request(GPIO_TP_RST, "GPIO_TP_RST");
	pr_err("[Kernel/LCM] GPIO_TP_RST = 0x%x\n", GPIO_TP_RST);
	if (gpio_is_valid(GPIO_TP_RST))
		gpio_free(GPIO_TP_RST);

	return 0;
}

static int lcm_gpio_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	pr_err("Kernel/LCM: %s start \n",__func__);

	lcm_get_gpio(dev);
	kd_lcm_get_vgp_supply(dev);
	kd_lcm_vgp_supply_enable();
	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,kd_wuxga_incell_dsi",},
	{}
};

static struct platform_driver lcm_driver = {
    .probe = lcm_gpio_probe,
	.driver = {
		   .name = "kd_wuxga_incell_dsi",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH  (1200)
#define FRAME_HEIGHT (1920)

#ifndef	GPIO_OUT_ZERO
#define	GPIO_OUT_ZERO	(0)
#endif

#ifndef	GPIO_OUT_ONE
#define	GPIO_OUT_ONE	(1)
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */
static struct LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)

#define MDELAY(n) (lcm_util.mdelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)

#define REGFLAG_DELAY             0xFEFE
#define REGFLAG_END_OF_TABLE      0xFFFF   // END OF REGISTERS MARKER

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x00, 1, {0x00}},
	{0xFA, 1, {0x5A}},
	{0x35, 0, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {0x00}}
};

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_set_value(GPIO, output);
}

static void lcm_init_power(void)
{
	pr_err("[Kernel/LCM] lcm_init_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_PWR2, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_PWR3, GPIO_OUT_ONE);
}

static void lcm_suspend_power(void)
{
	pr_err("[Kernel/LCM] lcm_suspend_power() enter\n");

//	lcm_set_gpio_output(GPIO_LCD_PWR3, GPIO_OUT_ZERO);
//	lcm_set_gpio_output(GPIO_LCD_PWR2, GPIO_OUT_ZERO);
	MDELAY(2);
//	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
//	MDELAY(500);
}

static void lcm_resume_power(void)
{
	pr_err("[Kernel/LCM] lcm_resume_power() enter\n");
	kd_lcm_vgp_supply_enable();
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_PWR2, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_PWR3, GPIO_OUT_ONE);
	MDELAY(100);
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE :
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				break;
		}
	}
}

static void init_lcm_registers(void)
{
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	pr_err("[Kernel/LCM] lcm_get_params() enter\n");

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;

	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	params->dsi.intermediat_buffer_num = 0;	/* This para should be 0 at video mode */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.horizontal_sync_active 	= 10;
	params->dsi.horizontal_backporch  	= 30;
	params->dsi.horizontal_frontporch 	= 28;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.vertical_sync_active 	= 4;
	params->dsi.vertical_backporch  	= 28;
	params->dsi.vertical_frontporch 	= 242;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.PLL_CLOCK 		  	= 532;
	params->dsi.ssc_disable = 1;
}

static void lcm_init_lcm(void)
{
	unsigned int data_array[16];

	pr_err("[Kernel/LCM] lcm_init() enter\n");
	init_lcm_registers();

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(180);

	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);

	MDELAY(120);
	gpio_request(GPIO_TP_RST, "GPIO_TP_RST");
	lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ZERO);
	if (gpio_is_valid(GPIO_TP_RST))
		gpio_free(GPIO_TP_RST);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_PWR3, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_PWR2, GPIO_OUT_ZERO);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
	MDELAY(150);
	kd_lcm_vgp_supply_disable();
	MDELAY(100);

	pr_err("[Kernel/LCM] lcm_suspend() enter\n");
}

static void lcm_resume(void)
{
	unsigned int data_array[16];

	pr_err("[Kernel/LCM] lcm_resume() enter\n");

#if 1
	gpio_request(GPIO_TP_RST, "GPIO_TP_RST");
	lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ONE);
	if (gpio_is_valid(GPIO_TP_RST))
		gpio_free(GPIO_TP_RST);
	MDELAY(100);
#endif
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(100);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(100);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(100);

	init_lcm_registers();

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(33);
}

struct LCM_DRIVER kd_wuxga_incell_dsi_lcm_drv = {
    .name		= "kd_wuxga_incell_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power		= lcm_init_power,
	.resume_power 	= lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
};
