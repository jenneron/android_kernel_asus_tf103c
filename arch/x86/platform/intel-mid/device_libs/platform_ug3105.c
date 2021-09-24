/*
 * platform_bq24192.c: bq24192 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/battery_id.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/intel-mid.h>
#include <asm/spid.h>
#include <linux/usb/otg.h>

static struct i2c_board_info __initdata ug3105_i2c_device = {
	I2C_BOARD_INFO("ug31xx-gauge", 0x70),
	.flags         = 0x00,
	.platform_data = NULL,
	.archdata      = NULL,
	.irq           = -1,
};

static int __init ug3105_platform_init(void)
{
	return i2c_register_board_info(1, &ug3105_i2c_device, 1);
}
module_init(ug3105_platform_init);
