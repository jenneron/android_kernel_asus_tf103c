#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include <asm/intel_crystalcove_pwrsrc.h>

int atmel_mxt_info[2] = {
	158,
	60
};


static struct i2c_board_info __initdata atmel_i2c_device[] = {
        {
         	.type          = "atmel_mxt_ts",
		    .addr          = 0x4a,
		    .flags         = 0,
		    .irq           = 158,
		    .platform_data = atmel_mxt_info,
        },

};


static int __init atmel_mxt_i2c_init(void)
{
	printk("RED register board info...\n");
	return i2c_register_board_info(6, atmel_i2c_device, ARRAY_SIZE(atmel_i2c_device));
}
module_init(atmel_mxt_i2c_init);

