/*
 * platform_bcm2079x.c: bcm2079x platform data initilization file
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
#include <linux/nfc/bcm2079x.h>
#include <asm/intel-mid.h>
#include "platform_bcm2079x.h"
#include <linux/lnw_gpio.h>

static struct bcm2079x_platform_data bcmi2cnfc_pdata = {
    .irq_gpio = NFC_INT,
    .en_gpio = NFC_ENABLE,
    .wake_gpio = NFC_WAKE,
};

static struct i2c_board_info __initdata bcm2079x_i2c_device = {
                .type           = "bcm2079x-i2c",
                .addr           = BCM_NFC_ADDR,
                .flags          = 0,
                .irq            = (NFC_INT+INTEL_MID_IRQ_OFFSET),
                .platform_data = (void *)&bcmi2cnfc_pdata,
};

static int __init bcm2079x_platform_init(void)
{
	return i2c_register_board_info(BCM_NFC_BUS, &bcm2079x_i2c_device, 1);
}
module_init(bcm2079x_platform_init);
