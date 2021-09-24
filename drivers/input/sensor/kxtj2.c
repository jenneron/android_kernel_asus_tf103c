/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtj2.h>
#include <linux/input-polldev.h>
#include <linux/acpi.h>

#define NAME			"KXTJ2100"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F		3
#define ODR200F		4
#define ODR400F		5
#define ODR800F		6
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define kxtj2_IEL		(1 << 3)
#define kxtj2_IEA		(1 << 4)
#define kxtj2_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

#define ENABLE_CALIBRATION_INTERFACE 1

//the Makefile will trans this macro to me
//#define CONFIG_INPUT_SENSOR_KXTJ2_POLLED_MODE 1

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtj2_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};
struct kxtj2_data {
	struct i2c_client *client;
	struct KXTJ2_platform_data pdata;
	struct input_dev *input_dev;
#ifdef CONFIG_INPUT_SENSOR_KXTJ2_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
	unsigned int last_poll_interval;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
        
        //bool enable;
	atomic_t enable;
        s16 axis_x_buf;
        s16 axis_y_buf;
        s16 axis_z_buf;
        spinlock_t axis_buf_lock;

	//add for calibration
        #ifdef ENABLE_CALIBRATION_INTERFACE
        atomic_t cal_enable;
        s16 cal_data_x;
        s16 cal_data_y;
        s16 cal_data_z;
        s16 no_cal_data_x;
        s16 no_cal_data_y;
        s16 no_cal_data_z;
        spinlock_t axis_cal_lock;
        #endif
};


#define KXTJ2_I2C_ADAPTER       0x05
#define KXTJ2_I2C_ADDRESS       0x0F

static struct KXTJ2_platform_data kxtj2_pdata = {
    .min_interval = 10,
    .init_interval = 50,
    .axis_map_x = 1,
    .axis_map_y = 0,
    .axis_map_z = 2,

    .negate_x = 0,
    .negate_y = 0,
    .negate_z = 1,
    .res_12bit = RES_12BIT,
    .g_range = KXTJ2_G_2G,

    .init = NULL,
    .exit = NULL,
    .power_on = NULL,
    .power_off = NULL,
};
static struct i2c_board_info kxtj2_board_info = {
    I2C_BOARD_INFO(NAME, KXTJ2_I2C_ADDRESS),
    .irq = 0,
    .platform_data = &kxtj2_pdata,
};
static int kxtj2_i2c_read(struct kxtj2_data *tj2, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj2->client->addr,
			.flags = tj2->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj2->client->addr,
			.flags = tj2->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj2->client->adapter, msgs, 2);
}
static void kxtj2_report_acceleration_data(struct kxtj2_data *tj2)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;

        

	err = kxtj2_i2c_read(tj2, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0)
		dev_err(&tj2->client->dev, "accelerometer data read failed\n");
	x = le16_to_cpu(acc_data[tj2->pdata.axis_map_x]);
	y = le16_to_cpu(acc_data[tj2->pdata.axis_map_y]);
	z = le16_to_cpu(acc_data[tj2->pdata.axis_map_z]);

	x >>= tj2->shift;
	y >>= tj2->shift;
	z >>= tj2->shift;

	x = tj2->pdata.negate_x ? -x : x;
        y = tj2->pdata.negate_y ? -y : y;
        z = tj2->pdata.negate_z ? -z : z;

        #ifdef ENABLE_CALIBRATION_INTERFACE
        spin_lock(&tj2->axis_cal_lock);

        tj2->no_cal_data_x = x;
        tj2->no_cal_data_y = y;
        tj2->no_cal_data_z = z;

        if( atomic_read(&tj2->cal_enable) )     {
                        x += tj2->cal_data_x;
                        y += tj2->cal_data_y;
                        z += tj2->cal_data_z;
        }

        spin_unlock(&tj2->axis_cal_lock);
        #endif


        spin_lock(&tj2->axis_buf_lock);
        tj2->axis_x_buf = x;
        tj2->axis_y_buf = y;
        tj2->axis_z_buf = z;
        spin_unlock(&tj2->axis_buf_lock);

        input_report_abs(tj2->input_dev, ABS_X, x);
        input_report_abs(tj2->input_dev, ABS_Y, y);
        input_report_abs(tj2->input_dev, ABS_Z, z);
        input_sync(tj2->input_dev);
}

static irqreturn_t kxtj2_isr(int irq, void *dev)
{
	struct kxtj2_data *tj2 = dev;
	int err;

	/* data ready is the only possible interrupt type */
	kxtj2_report_acceleration_data(tj2);

	err = i2c_smbus_read_byte_data(tj2->client, INT_REL);
	if (err < 0)
		dev_err(&tj2->client->dev,
			"error clearing interrupt status: %d\n", err);

	return IRQ_HANDLED;
}

static int kxtj2_update_g_range(struct kxtj2_data *tj2, u8 new_g_range)
{
	switch (new_g_range) {
	case KXTJ2_G_2G:
		tj2->shift = 4;
		break;
	case KXTJ2_G_4G:
		tj2->shift = 3;
		break;
	case KXTJ2_G_8G:
		tj2->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	tj2->ctrl_reg1 &= 0xe7;
	tj2->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtj2_update_odr(struct kxtj2_data *tj2, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj2_odr_table); i++) {
		tj2->data_ctrl = kxtj2_odr_table[i].mask;
		if (poll_interval < kxtj2_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(tj2->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj2->client, DATA_CTRL, tj2->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj2->client, CTRL_REG1, tj2->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

static int kxtj2_device_power_on(struct kxtj2_data *tj2)
{
	if (tj2->pdata.power_on)
		return tj2->pdata.power_on();

	return 0;
}

static void kxtj2_device_power_off(struct kxtj2_data *tj2)
{
	int err;

	tj2->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj2->client, CTRL_REG1, tj2->ctrl_reg1);
	if (err < 0)
		dev_err(&tj2->client->dev, "soft power off failed\n");

	if (tj2->pdata.power_off)
		tj2->pdata.power_off();
}

static int kxtj2_enable(struct kxtj2_data *tj2)
{
	int err;

	err = kxtj2_device_power_on(tj2);
	if (err < 0)
		return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj2->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj2->client->irq) {
		err = i2c_smbus_write_byte_data(tj2->client,
						INT_CTRL1, tj2->int_ctrl);
		if (err < 0)
			return err;
	}

	err = kxtj2_update_g_range(tj2, tj2->pdata.g_range);
	if (err < 0)
		return err;

	/* turn on outputs */
	tj2->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj2->client, CTRL_REG1, tj2->ctrl_reg1);
	if (err < 0)
		return err;

	err = kxtj2_update_odr(tj2, tj2->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (tj2->client->irq) {
		err = i2c_smbus_read_byte_data(tj2->client, INT_REL);
		if (err < 0) {
			dev_err(&tj2->client->dev,
				"error clearing interrupt: %d\n", err);
			goto fail;
		}
	}

	return 0;

fail:
	kxtj2_device_power_off(tj2);
	return err;
}

static void kxtj2_disable(struct kxtj2_data *tj2)
{
	kxtj2_device_power_off(tj2);
}

static int kxtj2_input_open(struct input_dev *input)
{
	struct kxtj2_data *tj2 = input_get_drvdata(input);

	return kxtj2_enable(tj2);
}

static void kxtj2_input_close(struct input_dev *dev)
{
	struct kxtj2_data *tj2 = input_get_drvdata(dev);

	kxtj2_disable(tj2);
}

static void kxtj2_init_input_device(struct kxtj2_data *tj2,
					      struct input_dev *input_dev)
{
printk("kxtj2_init_input_device(struct kxtj2_data *tj2,struct input_dev *input_dev)\n");
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "kxtj2_accel";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj2->client->dev;
}

static int kxtj2_setup_input_device(struct kxtj2_data *tj2)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj2->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj2->input_dev = input_dev;

	input_dev->open = kxtj2_input_open;
	input_dev->close = kxtj2_input_close;
	input_set_drvdata(input_dev, tj2);

	kxtj2_init_input_device(tj2, input_dev);

	err = input_register_device(tj2->input_dev);
	if (err) {
		dev_err(&tj2->client->dev,
			"unable to register input polled device %s: %d\n",
			tj2->input_dev->name, err);
		input_free_device(tj2->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj2_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj2_data *tj2 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tj2->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj2_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj2_data *tj2 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj2->input_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	tj2->last_poll_interval = max(interval, tj2->pdata.min_interval);

	kxtj2_update_odr(tj2, tj2->last_poll_interval);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

//=======================================================================
static ssize_t kxtj2_delay_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct kxtj2_data *tj2 = i2c_get_clientdata(client);
        struct input_polled_dev *poll_dev = tj2->poll_dev;

        return sprintf(buf, "%d\n", poll_dev->poll_interval);
}

static ssize_t kxtj2_delay_store(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct kxtj2_data *tj2 = i2c_get_clientdata(client);
        struct input_polled_dev *poll_dev = tj2->poll_dev;
        struct input_dev *input_dev = tj2->input_dev;
        unsigned int interval;
        int error;

        error = kstrtouint(buf, 10, &interval);
        if (error < 0)
                return error;

        /* Lock the device to prevent races with open/close (and itself) */
        mutex_lock(&input_dev->mutex);

        if( interval < poll_dev->poll_interval_min ) {
                interval = poll_dev->poll_interval_min;
        }

        if(interval > poll_dev->poll_interval_max) {
                interval = poll_dev->poll_interval_max;
        }
        if( interval >= poll_dev->poll_interval_min
                &&
                interval <= poll_dev->poll_interval_max)        {

                //we should change the rate both of the hardware and the poll dev
                poll_dev->poll_interval = interval;
                tj2->last_poll_interval = interval;
                kxtj2_update_odr(tj2, interval);
        }

        mutex_unlock(&input_dev->mutex);

        return count;
}

static ssize_t kxtj2_enable_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct kxtj2_data *tj2 = i2c_get_clientdata(client);
        printk("alp : kxtj2_enable_show (%d)\n",atomic_read(&tj2->enable));
        return sprintf(buf, "%d\n", atomic_read(&tj2->enable));
}

static ssize_t kxtj2_enable_store(struct device *dev,
                                        struct device_attribute *attr,
                                                const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct kxtj2_data *tj2 = i2c_get_clientdata(client);
        int val = simple_strtoul(buf, NULL, 10);
        switch(val){
                case 0:
                        //if tj2->enable !=  0, then, tj2->enable -= 1
                        atomic_add_unless(&tj2->enable, -1, 0);

                        if( atomic_read(&tj2->enable) == 0 )
                                kxtj2_disable(tj2);
                        break;
                case 1:
                        if( atomic_inc_return(&tj2->enable) == 1 )
                                kxtj2_enable(tj2);
                        break;
                default:
                        break;
        }
        return count;
}

static ssize_t kxtj2_rawdata_for_mag_show(struct device *dev, struct device_attribute *devattr, char *buf)
{       
        struct i2c_clinet * client = to_i2c_client(dev);
        struct kxtj2_data * tj2 = i2c_get_clientdata(client);
        int ret = 0;
        
        //printk("get_raw_data: begin read raw data\n");        
        spin_lock(&tj2->axis_buf_lock);
        ret = snprintf(buf, 4096/*PAGE_SIZE*/, "%4hx %4hx %4hx\n", tj2->axis_x_buf, tj2->axis_y_buf, tj2->axis_z_buf );
        spin_unlock(&tj2->axis_buf_lock);
                                        
        //printk("kxtj2: buf is %s\n", buf);    
        //printk("kxtj2: get_raw_data : x: 0x%x y: 0x%x z: 0x%x\n", tj2->axis_x_buf, tj2->axis_y_buf, tj2->axis_z_buf);
        //printk("get_raw_data: end read raw data\n");  
        return ret;
}

#ifdef ENABLE_CALIBRATION_INTERFACE
static ssize_t kxtj2_calibration_enable_store(struct device *dev, struct device_attribute *devattr, char *buf, ssize_t count)
{
        struct i2c_clinet * client = to_i2c_client(dev);
        struct kxtj2_data * tj2 = i2c_get_clientdata(client);
        int ret = 0;

        int val = simple_strtoul(buf, NULL, 10) ? 1 : 0 ;

        atomic_set(&tj2->cal_enable, val);
        printk("kxtj2: set calibration enable to %d\n", atomic_read(&tj2->cal_enable));
        return count;
}

static ssize_t kxtj2_calibration_data_store(struct device *dev, struct device_attribute *devattr, char *buf, ssize_t count)
{
        struct i2c_clinet * client = to_i2c_client(dev);
        struct kxtj2_data * tj2 = i2c_get_clientdata(client);
        int cal_data_x;
        int cal_data_y;
        int cal_data_z;

        printk("kxtj2: buf is %s\n", buf);
        sscanf(buf, "%d%d%d", &cal_data_x, &cal_data_y, &cal_data_z);
        spin_lock(&tj2->axis_cal_lock);
        tj2->cal_data_x = cal_data_x;
        tj2->cal_data_y = cal_data_y;
        tj2->cal_data_z = cal_data_z;
        spin_unlock(&tj2->axis_cal_lock);

        printk("kxtj2: set calibration data x=%d, y=%d, z=%d\n", cal_data_x, cal_data_y, cal_data_z);
        return count;
}

static ssize_t kxtj2_rawdata_for_cal_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
        struct i2c_clinet * client = to_i2c_client(dev);
        struct kxtj2_data * tj2 = i2c_get_clientdata(client);
        int ret = 0;

        spin_lock(&tj2->axis_cal_lock);
        ret = snprintf(buf, 4096, "%hd %hd %hd\n", tj2->no_cal_data_x, tj2->no_cal_data_y, tj2->no_cal_data_z);
        spin_unlock(&tj2->axis_cal_lock);
        return ret;
}
#endif

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, kxtj2_get_poll, kxtj2_set_poll);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kxtj2_delay_show, kxtj2_delay_store);
static DEVICE_ATTR(enable, S_IRGRP|S_IWGRP|S_IRUSR|S_IWUSR,kxtj2_enable_show,kxtj2_enable_store);
static DEVICE_ATTR(rawdata_for_mag, S_IRUGO, kxtj2_rawdata_for_mag_show, NULL);

#ifdef ENABLE_CALIBRATION_INTERFACE
static DEVICE_ATTR(cal_data, S_IWUSR | S_IWGRP, NULL, kxtj2_calibration_data_store);
static DEVICE_ATTR(cal_enable, S_IWUSR | S_IWGRP, NULL, kxtj2_calibration_enable_store);
static DEVICE_ATTR(rawdata_for_cal, S_IRUGO, kxtj2_rawdata_for_cal_show, NULL);
#endif

static struct attribute *kxtj2_attributes[] = {
	&dev_attr_poll.attr,
        &dev_attr_enable.attr,
        &dev_attr_rawdata_for_mag.attr,

	#ifdef CONFIG_INPUT_SENSOR_KXTJ2_POLLED_MODE
                &dev_attr_delay.attr,
        #endif
	
	#ifdef ENABLE_CALIBRATION_INTERFACE
        &dev_attr_cal_data.attr,
        &dev_attr_cal_enable.attr,
        &dev_attr_rawdata_for_cal.attr,
        #endif

	NULL
};

static struct attribute_group kxtj2_attribute_group = {
	.attrs = kxtj2_attributes
};


#ifdef CONFIG_INPUT_SENSOR_KXTJ2_POLLED_MODE
static void kxtj2_poll(struct input_polled_dev *dev)
{
	struct kxtj2_data *tj2 = dev->private;
	unsigned int poll_interval = dev->poll_interval;

        if( atomic_read(&tj2->enable) == 0 )
		return 0;

	kxtj2_report_acceleration_data(tj2);

	if (poll_interval != tj2->last_poll_interval) {
		kxtj2_update_odr(tj2, poll_interval);
		tj2->last_poll_interval = poll_interval;
	}
}

static void kxtj2_polled_input_open(struct input_polled_dev *dev)
{
	struct kxtj2_data *tj2 = dev->private;

	kxtj2_enable(tj2);
}

static void kxtj2_polled_input_close(struct input_polled_dev *dev)
{
	struct kxtj2_data *tj2 = dev->private;

	kxtj2_disable(tj2);
}

static int kxtj2_setup_polled_device(struct kxtj2_data *tj2)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	if (!poll_dev) {
		dev_err(&tj2->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	tj2->poll_dev = poll_dev;
	tj2->input_dev = poll_dev->input;

	poll_dev->private = tj2;
	poll_dev->poll = kxtj2_poll;
	poll_dev->open = kxtj2_polled_input_open;
	poll_dev->close = kxtj2_polled_input_close;

	poll_dev->poll_interval = 100 ;
	tj2->last_poll_interval = 100 ;
	kxtj2_update_odr(tj2, 100);

	kxtj2_init_input_device(tj2, poll_dev->input);

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&tj2->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	return 0;
}

static void kxtj2_teardown_polled_device(struct kxtj2_data *tj2)
{
	input_unregister_polled_device(tj2->poll_dev);
	input_free_polled_device(tj2->poll_dev);
}

#else

static inline int kxtj2_setup_polled_device(struct kxtj2_data *tj2)
{
	return -ENOSYS;
}

static inline void kxtj2_teardown_polled_device(struct kxtj2_data *tj2)
{
}

#endif

static int kxtj2_verify(struct kxtj2_data *tj2)
{
	int retval;

	retval = kxtj2_device_power_on(tj2);
	if (retval < 0)
		return retval;

	retval = i2c_smbus_read_byte_data(tj2->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&tj2->client->dev, "read err int source\n");
		goto out;
	}

	retval = (retval != 0x09) ? -EIO : 0;

out:
	kxtj2_device_power_off(tj2);
	return retval;
}

#ifdef SENSOR_FACTORY
static const struct file_operations kxtj2_proc_ops;
#endif

static int kxtj2_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct KXTJ2_platform_data *pdata = &kxtj2_pdata;//client->dev.platform_data;
	struct kxtj2_data *tj2;
	int err;


        client->dev.platform_data = &kxtj2_pdata;
        if (client->irq == -1) {
            client->irq = 0;
        }

       /* struct i2c_adapter *adapter = i2c_get_adapter(KXTJ2_I2C_ADAPTER);
        i2c_new_device(adapter, &kxtj2_board_info);
        i2c_put_adapter(adapter);*/
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	tj2 = kzalloc(sizeof(*tj2), GFP_KERNEL);
	if (!tj2) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	tj2->client = client;
	tj2->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}

	err = kxtj2_verify(tj2);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}

	i2c_set_clientdata(client, tj2);

	tj2->ctrl_reg1 = tj2->pdata.res_12bit | tj2->pdata.g_range;
	tj2->last_poll_interval = tj2->pdata.init_interval;

	if (client->irq) {
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tj2->int_ctrl |= kxtj2_IEN | kxtj2_IEA | kxtj2_IEL;
		tj2->ctrl_reg1 |= DRDYE;

		err = kxtj2_setup_input_device(tj2);
		if (err)
			goto err_pdata_exit;

		err = request_threaded_irq(client->irq, NULL, kxtj2_isr,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "kxtj2-irq", tj2);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}

		err = sysfs_create_group(&client->dev.kobj, &kxtj2_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_irq;
		}

	} else {
		err = kxtj2_setup_polled_device(tj2);
		if (err)
			goto err_pdata_exit;

		err = sysfs_create_group(&client->dev.kobj, &kxtj2_attribute_group);
                if (err) {
                        dev_err(&client->dev, "sysfs create failed: %d\n", err);
                        goto err_destroy_polled_dev;

		}
	}

	atomic_set(&tj2->enable, 0);
        tj2->axis_x_buf = 0;
        tj2->axis_y_buf = 0;
        tj2->axis_z_buf = 0;
        spin_lock_init(&tj2->axis_buf_lock);
	
	#ifdef ENABLE_CALIBRATION_INTERFACE
        atomic_set(&tj2->enable, 0);
        tj2->cal_data_x = 0;
        tj2->cal_data_y = 0;
        tj2->cal_data_z = 0;
        tj2->no_cal_data_x = 0;
        tj2->no_cal_data_y = 0;
        tj2->no_cal_data_z = 0;
        spin_lock_init(&tj2->axis_cal_lock);
        #endif

#ifdef SENSOR_FACTORY
        proc_create("kxtj2", 0666, NULL, &kxtj2_proc_ops);
#endif
	return 0;

err_free_irq:
	free_irq(client->irq, tj2);
err_destroy_polled_dev:
        if( ! client->irq )
                kxtj2_teardown_polled_device(tj2);

err_destroy_input:
	input_unregister_device(tj2->input_dev);
err_pdata_exit:
	if (tj2->pdata.exit)
		tj2->pdata.exit();
err_free_mem:
	kfree(tj2);
	return err;
}

static int kxtj2_remove(struct i2c_client *client)
{
	struct kxtj2_data *tj2 = i2c_get_clientdata(client);

	if (client->irq) {
		sysfs_remove_group(&client->dev.kobj, &kxtj2_attribute_group);
		free_irq(client->irq, tj2);
		input_unregister_device(tj2->input_dev);
	} else {
		kxtj2_teardown_polled_device(tj2);
	}

	if (tj2->pdata.exit)
		tj2->pdata.exit();

	kfree(tj2);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj2_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj2_data *tj2 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj2->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		kxtj2_disable(tj2);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int kxtj2_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj2_data *tj2 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj2->input_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		kxtj2_enable(tj2);

	mutex_unlock(&input_dev->mutex);
	return retval;
}
static int __init kxtj2_attach(struct i2c_adapter *adapter)
{

        struct i2c_client *kxtj2_client;



        if(adapter->nr != KXTJ2_I2C_ADAPTER)
        {
            return 0;
        }

	kxtj2_client = i2c_new_device(adapter, &kxtj2_board_info);
	if (!kxtj2_client)
		return -ENODEV;
	/*
	 * We know the driver is already loaded, so the device should be
	 * already bound. If not it means binding failed, and then there
	 * is no point in keeping the device instantiated.
	 */
	if (!kxtj2_client->driver) {
		i2c_unregister_device(kxtj2_client);
		kxtj2_client = NULL;
		return -ENODEV;
	}
	
	/*
	 * Let i2c-core delete that device on driver removal.
	 * This is safe because i2c-core holds the core_lock mutex for us.
	 */
	list_add_tail(&kxtj2_client->detected,
		      &kxtj2_client->driver->clients);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(kxtj2_pm_ops, kxtj2_suspend, kxtj2_resume);

static const struct i2c_device_id kxtj2_id[] = {
	{ NAME, 0 },
	{ },
};

static struct acpi_device_id kxtj2_acpi_match[] = {
	{ "KXTJ2100", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxtj2_id);

static struct i2c_driver kxtj2_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &kxtj2_pm_ops,
                .acpi_match_table = ACPI_PTR(kxtj2_acpi_match),
	},
	.probe		= kxtj2_probe,
	.remove		= kxtj2_remove,
	.id_table	= kxtj2_id,
        //.attach_adapter = kxtj2_attach,
};
static int __init kxtj2_init(void)
{
printk("__init akm_compass_init\n");
/*struct i2c_adapter *adapter = i2c_get_adapter(KXTJ2_I2C_ADAPTER);
        i2c_new_device(adapter, &kxtj2_board_info);
        i2c_put_adapter(adapter);*/
	return i2c_add_driver(&kxtj2_driver);
}

static void __exit kxtj2_exit(void)
{
	pr_info("AKM compass driver: release.");
	i2c_del_driver(&kxtj2_driver);
}
//module_i2c_driver(kxtj2_driver);

module_init(kxtj2_init);
module_exit(kxtj2_exit);

MODULE_DESCRIPTION("kxtj2 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
