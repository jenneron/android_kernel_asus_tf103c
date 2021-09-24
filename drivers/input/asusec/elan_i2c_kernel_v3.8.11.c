/*
 * Elan I2C Touchpad diver
 *
 * Copyright (c) 2012 ELAN Microelectronics Corp.
 *
 * Author: (Duson Lin) <dusonlin@emc.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

#define DRIVER_NAME		"elan_i2c"
#define HID_DESC_LENGTH		30
#define HID_REPORT_ID_OFFSET	2

/* Length of Elan touchpad information */
#define ETP_INF_LENGTH		2
#define ETP_MAX_FINGERS		5
#define ETP_REPORT_DESC_LENGTH	101
#define ETP_REPORT_LENGTH	30
#define ETP_FINGER_DATA_OFFSET	4
#define ETP_FINGER_DATA_LEN	5

#define ETP_REPORT_ID		0x5d

#define HID_CMD_REGISTER	0x0005
#define ETP_CMD_REGISTER	0x0300
#define ETP_CTRL_REGISTER	0x0301
#define ETP_IAP_CTRL_REGISTER	0x0310
#define ETP_IAP_REGISTER	0x0311
#define ETP_PROG_INDEX_REGISTER	0x0312

#define CMD_RESET		0x0100
#define CMD_WAKE_UP		0x0800
#define CMD_SLEEP		0x0801
#define CMD_ENABLE_ABS		0x0001
#define CMD_DISABLE_PROTECTION	0x0000
#define CMD_ENABLE_FW_UPDATE	0xe15a

#define REG_DESC		0x0001
#define REG_REPORT_DESC		0x0002
#define REG_UNIQUE_ID		0x0101
#define REG_FW_VERSION		0x0102
#define REG_XY_TRACE_NUM	0x0105
#define REG_X_AXIS_MAX		0x0106
#define REG_Y_AXIS_MAX		0x0107
#define REG_RESOLUTION		0x0108
#define REG_IAP_VERSION		0x0110

#define ETP_FW_IAP_REG_L	0x01
#define ETP_FW_IAP_REG_H	0x06
#define ETP_FW_UPDATE_TIME	30
#define ETP_FW_IAP_MODE_ON	(1 << 0)
#define ETP_FW_IAP_PAGE_ERR	(1 << 5)
#define ETP_FW_PAGE_SIZE	128

#define IOC_MAGIC 0x15
#define IOCTL_IAP_INITIALIZE _IO(IOC_MAGIC, 0x0)
#define DEV_MAJOR 88

/* The main device structure */
struct elan_i2c_data {
	struct i2c_client	*client;
	struct input_dev	*input;
	unsigned int		max_x;
	unsigned int		max_y;
	unsigned int		width_x;
	unsigned int		width_y;
	unsigned int		irq;
	unsigned int		iap_start_addr;
	bool			updated_fw;
};


static int __elan_i2c_read_reg(struct i2c_client *client, u16 reg,
				u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		return ret;

	return ret != 2 ? -EIO : 0;
}

static int elan_i2c_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int retval;

	retval = __elan_i2c_read_reg(client, reg, val, ETP_INF_LENGTH);
	if (retval < 0) {
		dev_err(&client->dev, "reading register (0x%04x) failed!\n", reg);
		return retval;
	}

	return 0;
}

static int elan_i2c_write_reg_cmd(struct i2c_client *client, u16 reg, u16 cmd)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = cmd & 0xff;
	buf[3] = (cmd >> 8) & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return ret != 1 ? -EIO : 0;
}

static int elan_i2c_reset(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, HID_CMD_REGISTER,
					CMD_RESET);
}

static int elan_i2c_wake_up(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, HID_CMD_REGISTER,
					CMD_WAKE_UP);
}

static int elan_i2c_sleep(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, HID_CMD_REGISTER,
					CMD_SLEEP);
}

static int elan_i2c_disable_protection(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, ETP_CTRL_REGISTER,
					CMD_DISABLE_PROTECTION);
}

static int elan_i2c_enable_absolute_mode(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, ETP_CMD_REGISTER,
					CMD_ENABLE_ABS);
}

static int elan_i2c_get_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(client, REG_DESC, val,
				   HID_DESC_LENGTH);
}

static int elan_i2c_get_report_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(client, REG_REPORT_DESC, val,
				   ETP_REPORT_DESC_LENGTH);
}

static int elan_i2c_get_x_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_X_AXIS_MAX, val);
}

static int elan_i2c_get_y_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_Y_AXIS_MAX, val);
}

static int elan_i2c_get_trace_num(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_XY_TRACE_NUM, val);
}

static int elan_i2c_get_fw_version(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_FW_VERSION, val);
}

static int elan_i2c_get_resolution(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_RESOLUTION, val);
}

/*
 **************************************************************************
 * IAP functions
 **************************************************************************
 */

static bool elan_i2c_iap_page_write_ok(struct i2c_client *client)
{
	u8 val[2];
	u16 constant;
	int retval;

	retval = elan_i2c_read_reg(client, ETP_IAP_CTRL_REGISTER, val);
	if (retval < 0)
		return false;

	constant = le16_to_cpup((__le16 *)val);
	dev_dbg(&client->dev, "IAP_PageError_B: 0x%04x.\n", constant);
	if (constant & ETP_FW_IAP_PAGE_ERR)
		return false;

	return true;
}

static int elan_i2c_enable_fw_update(struct elan_i2c_data *data)
{
	u8 val[2];
	int retval;

	retval = elan_i2c_write_reg_cmd(data->client, ETP_IAP_REGISTER,
					  CMD_ENABLE_FW_UPDATE);
	if (retval < 0)
		return retval;

	/* read back to check we actually enabled successfully. */
	retval = elan_i2c_read_reg(data->client, ETP_IAP_REGISTER, val);
	if (retval < 0)
		return retval;

	if (le16_to_cpup((__le16 *)val) != CMD_ENABLE_FW_UPDATE)
		return -EFAULT;


	retval = elan_i2c_write_reg_cmd(data->client, ETP_PROG_INDEX_REGISTER,
					  data->iap_start_addr);
	if (retval < 0)
		return retval;

	/* read back to check we actually wrote successfully. */
	retval = elan_i2c_read_reg(data->client, ETP_PROG_INDEX_REGISTER, val);
	if (retval < 0)
		return retval;

	if (le16_to_cpup((__le16 *)val) != data->iap_start_addr)
		return -EFAULT;

	return retval;
}

static int elan_i2c_prepare_fw_update(struct elan_i2c_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &data->client->dev;

	/*
	 * some regs used by IAP firmware updater are not writable until
	 * we disable the protection.
	 */
	if (elan_i2c_disable_protection(client)) {
		dev_err(dev, "failed disabling protection.\n");
		return -EFAULT;
	}

	if (elan_i2c_enable_fw_update(data)) {
		dev_err(dev, "failed enabling IAP firmware update.\n");
		return -EFAULT;
	}

	/* Wait 30 ms for F/W IAP initialization */
	msleep(ETP_FW_UPDATE_TIME);
	return 0;
}

static int elan_i2c_write_fw_block(struct elan_i2c_data *data, const char *page)
{
	int ret;
	int repeat = 3;
	u8 page_store[ETP_FW_PAGE_SIZE + 2];

	page_store[0] = ETP_FW_IAP_REG_L;
	page_store[1] = ETP_FW_IAP_REG_H;
	memcpy(&page_store[2], page, ETP_FW_PAGE_SIZE);

	do {
		ret = i2c_master_send(data->client, page_store,
					ETP_FW_PAGE_SIZE + 2);

		/*
		 * Wait 30ms for F/W to update one page ROM data.
		 */
		msleep(ETP_FW_UPDATE_TIME);

		if (elan_i2c_iap_page_write_ok(data->client))
			break;

		repeat--;
	} while (repeat > 0/*repeat == 0*/);


	if (repeat > 0)
		return 0;
	else
		return -EFAULT;

}

static ssize_t elan_sysfs_fw_version(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct elan_i2c_data *data = dev_get_drvdata(dev);
	elan_i2c_get_fw_version(data->client, buf);
	return (ssize_t)2;
}

static ssize_t elan_sysfs_iap_initialize(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct elan_i2c_data *data = dev_get_drvdata(dev);
	int retval;
	data->iap_start_addr = le16_to_cpup((__le16 *)buf);

	retval = elan_i2c_prepare_fw_update(data);
	if (retval)
		dev_err(dev, "IAP Initialize fail\n");
	else
		data->updated_fw = true;

	return (retval == 0) ? count : retval;
}

static ssize_t elan_sysfs_iap_write_page(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct elan_i2c_data *data = dev_get_drvdata(dev);
	int retval;
	if (count != ETP_FW_PAGE_SIZE)
		return -EFAULT;

	retval = elan_i2c_write_fw_block(data, buf);
	return (retval == 0) ? count : retval;
}

static DEVICE_ATTR(fw_version, S_IROTH|S_IWOTH, elan_sysfs_fw_version, NULL);
static DEVICE_ATTR(iap_initialize, S_IROTH|S_IWOTH, NULL, elan_sysfs_iap_initialize);
static DEVICE_ATTR(iap_write_page, S_IROTH|S_IWOTH, NULL, elan_sysfs_iap_write_page);

static struct attribute *elan_sysfs_entries[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_iap_initialize.attr,
	&dev_attr_iap_write_page.attr,
	NULL,
};

static const struct attribute_group elan_sysfs_group = {
	.attrs = elan_sysfs_entries,
};

/*
 **************************************************************************
 * isr functions
 **************************************************************************
 */

static void elan_i2c_report_absolute(struct elan_i2c_data *data, u8 *packet)
{
	struct input_dev *input = data->input;
	u8 *finger_data = &packet[ETP_FINGER_DATA_OFFSET];
	bool finger_on;
	int pos_x, pos_y;
	int area_x, area_y, pressure;
	int i;

	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		finger_on = (packet[3] >> (3 + i)) & 0x01;

		if (finger_on) {
			pos_x = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
			pos_y = data->max_y -
				(((finger_data[0] & 0x0f) << 8) |
				   finger_data[2]);
			area_x = (finger_data[3] & 0x0f) * data->width_x;
			area_y = (finger_data[3] >> 4) * data->width_y;
			pressure = finger_data[4];

			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
			input_report_abs(input, ABS_MT_TRACKING_ID, i);
			input_report_abs(input, ABS_MT_POSITION_X, pos_x);
			input_report_abs(input, ABS_MT_POSITION_Y, pos_y);
			input_report_abs(input, ABS_MT_PRESSURE, pressure);
			/* use x-axis value as TOOL_WIDTH */
			input_report_abs(input, ABS_TOOL_WIDTH,
					 finger_data[3] & 0x0f);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR,
					 max(area_x, area_y));
			input_report_abs(input, ABS_MT_TOUCH_MINOR,
					 min(area_x, area_y));
			finger_data += ETP_FINGER_DATA_LEN;
		} else {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(input, BTN_LEFT, ((packet[3] & 0x01) == 1));
//	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
}

static int elan_i2c_check_packet(u8 *packet)
{
	u16 length = le16_to_cpu(packet[0]);
	u8 report_id = packet[HID_REPORT_ID_OFFSET];

	if (length != ETP_REPORT_LENGTH ||
	    report_id != ETP_REPORT_ID)
		return -1;

	return 0;
}

static int elan_i2c_initialize(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	u8 val[ETP_REPORT_DESC_LENGTH];
	int rc;

	rc = elan_i2c_reset(client);
	if (rc < 0) {
		dev_err(dev, "device reset failed.\n");
		return -1;
	}

	msleep(100);

	rc = i2c_master_recv(client, val, ETP_INF_LENGTH);	
	if(rc < 0) {
		dev_err(dev, "get device reset return value failed.\n");
		return -1;
	}


	rc = elan_i2c_get_desc(client, val);
	if (rc < 0) {
		dev_err(dev, "couldn't get device descriptor.\n");
		return -1;
	}

	rc = elan_i2c_get_report_desc(client, val);
	if (rc < 0) {
		dev_err(dev, "fetching report descriptor failed.\n");
		return -1;
	}

	return 0;
}

static irqreturn_t elan_i2c_isr(int irq, void *dev_id)
{
	struct elan_i2c_data *data = dev_id;
	u8 packet[ETP_REPORT_LENGTH];
	int retval;

	/*
	Only in I2C protocol, when IAP all page wrote finish, driver will
	get one INT signal from high to low, and driver must get 0000
	to confirm IAP is finished.
	*/
	if (data->updated_fw) {

		retval = i2c_master_recv(data->client, packet,
						ETP_INF_LENGTH);
		if (retval == 2 && !le16_to_cpup((__le16 *)packet)) {
			dev_dbg(&data->client->dev, "reinitializing after F/W update...");
			elan_i2c_initialize(data->client);

			retval = elan_i2c_enable_absolute_mode(data->client);
			if (retval < 0) {
				dev_err(&data->client->dev, "can't switch to absolute mode.\n");
				goto elan_isr_end;
			}

			retval = elan_i2c_wake_up(data->client);
			if (retval < 0) {
				dev_err(&data->client->dev, "device wake up failed.\n");
				goto elan_isr_end;
			}
			
			msleep(20);
			retval = elan_i2c_enable_absolute_mode(data->client);
			if (retval < 0) {
				dev_err(&data->client->dev, "can't switch to absolute mode.\n");
				goto elan_isr_end;
			}
		}
		data->updated_fw = false;
		goto elan_isr_end;
	}

	retval = i2c_master_recv(data->client, packet, ETP_REPORT_LENGTH);
	if (retval != ETP_REPORT_LENGTH || elan_i2c_check_packet(packet)) {
		dev_dbg(&data->client->dev, "wrong packet data.");
		goto elan_isr_end;
	}

	elan_i2c_report_absolute(data, packet);

elan_isr_end:
	return IRQ_HANDLED;
}

/*
 **************************************************************************
 * initialize functions
 **************************************************************************
 */

/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int elan_i2c_convert_res(unsigned int val)
{
	return (val * 10 + 790) * 10 / 254;
}

static int elan_i2c_input_dev_create(struct elan_i2c_data *data)
{
	struct i2c_client *client = data->client;
	struct input_dev *input;
	unsigned int x_res, y_res;
	u8 val[3];
	int ret;

	data->input = input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input->name = "Elan I2C Touchpad";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;

	__set_bit(INPUT_PROP_POINTER, input->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);

	__set_bit(BTN_LEFT, input->keybit);

	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, input->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, input->keybit);
	__set_bit(BTN_TOOL_QUADTAP, input->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, input->keybit);

	elan_i2c_get_x_max(client, val);
	data->max_x = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_y_max(client, val);
	data->max_y = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_trace_num(client, val);
	data->width_x = data->max_x / (val[0] - 1);
	data->width_y = data->max_y / (val[1] - 1);

	elan_i2c_get_resolution(client, val);
	x_res = elan_i2c_convert_res(val[0]);
	y_res = elan_i2c_convert_res(val[1]);

	input_set_drvdata(input, data);

	elan_i2c_get_fw_version(client, val);
	dev_dbg(&client->dev,
		"Elan I2C Trackpad Information:\n" \
		"    Firmware Version:  0x%02x%02x\n" \
		"    Max ABS X,Y:   %d,%d\n" \
		"    Width X,Y:   %d,%d\n" \
		"    Resolution X,Y:   %d,%d (dots/mm)\n",
		val[1], val[0],
		data->max_x, data->max_y, data->width_x,
		data->width_y, x_res, y_res);

	input_set_abs_params(input, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_X, x_res);
	input_abs_set_res(input, ABS_Y, y_res);
	input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	/* handle pointer emulation and unused slots in core */
	ret = input_mt_init_slots(input, ETP_MAX_FINGERS, INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(&client->dev, "allocate MT slots failed, %d\n", ret);
		goto err_free_device;
	}
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, ETP_MAX_FINGERS, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, y_res);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0,
			     15 * max(data->width_x, data->width_y), 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0,
			     15 * min(data->width_x, data->width_y), 0, 0);

	/* Register the device in input subsystem */
	ret = input_register_device(input);
	if (ret) {
		dev_err(&client->dev, "input device register failed, %d\n",
			ret);
		goto err_free_device;
	}

	return 0;

err_free_device:
	input_free_device(input);
	return ret;
}

static int elan_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *dev_id)
{
	struct elan_i2c_data *data;
	int ret;

	data = kzalloc(sizeof(struct elan_i2c_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->irq = client->irq;
	data->updated_fw = false;

	ret = elan_i2c_initialize(client);
	if (ret < 0)
		goto err_init;

	ret = elan_i2c_input_dev_create(data);
	if (ret < 0)
		goto err_input_dev;

	ret = request_threaded_irq(client->irq, NULL, elan_i2c_isr,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   client->name, data);
	if (ret < 0) {
		dev_dbg(&client->dev, "Could not register interrupt for "
			DRIVER_NAME ", irq=%d, ret=%d\n", client->irq, ret);
		goto err_irq;
	}

	ret = elan_i2c_enable_absolute_mode(client);
	if (ret < 0) {
		dev_err(&client->dev, "can't switch to absolute mode.\n");
		goto err_switch_mode;
	}

	ret = elan_i2c_wake_up(client);
	if (ret < 0) {
		dev_err(&client->dev, "device wake up failed.\n");
		goto err_switch_mode;
	}

	i2c_set_clientdata(client, data);
	device_init_wakeup(&client->dev, 1);

	msleep(100);
	ret = elan_i2c_enable_absolute_mode(client);
	if (ret < 0) {
		dev_err(&client->dev, "can't switch to absolute mode.\n");
		goto err_switch_mode;
	}

	ret = sysfs_create_group(&data->input->dev.kobj, &elan_sysfs_group);
	if (ret < 0) {
		dev_err(&client->dev, "cannot register dev attribute %d", ret);
		goto err_create_group;
	}

	return 0;

err_create_group:
err_switch_mode:
	free_irq(data->irq, data);
err_irq:
	input_free_device(data->input);
err_input_dev:
	kfree(data);
err_init:
	return ret;
}

static int elan_i2c_remove(struct i2c_client *client)
{
	struct elan_i2c_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&data->input->dev.kobj, &elan_sysfs_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elan_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	elan_i2c_sleep(client);

	return 0;
}

static int elan_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = elan_i2c_wake_up(client);
	if (ret < 0)
		return ret;

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(elan_i2c_pm_ops,
			 elan_i2c_suspend, elan_i2c_resume);

static const struct i2c_device_id elan_i2c_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, elan_i2c_id);

static struct i2c_driver elan_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &elan_i2c_pm_ops,
	},
	.probe		= elan_i2c_probe,
	.remove		= elan_i2c_remove,
	.id_table	= elan_i2c_id,
};

static int __init elan_init(void)
{
	return i2c_add_driver(&elan_i2c_driver);
}

static void __exit elan_exit(void)
{
	i2c_del_driver(&elan_i2c_driver);
}

module_init(elan_init);
module_exit(elan_exit);

MODULE_AUTHOR("Duson lin <dusonlin@emc.com.tw>");
MODULE_DESCRIPTION("Elan I2C Touchpad driver");
MODULE_LICENSE("GPL");
