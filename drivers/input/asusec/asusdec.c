/*
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <asm/gpio.h>
#include <linux/statfs.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/acpi.h>

#include <linux/acpi_gpio.h>
#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>

#include "asusdec.h"
#include "elan_i2c_asus.h"
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * functions declaration
 */
static void asusdec_send_ec_req(void);
static int asusdec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int asusdec_remove(struct i2c_client *client);
static int asusdec_suspend(struct i2c_client *client, pm_message_t mesg);
static int asusdec_resume(struct i2c_client *client);
static int asusdec_open(struct inode *inode, struct file *flip);
static int asusdec_release(struct inode *inode, struct file *flip);
static long asusdec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static void asusdec_switch_apower_state(int state);
static void asusdec_win_shutdown(void);
static void asusdec_enter_factory_mode(void);
static void asusdec_enter_normal_mode(void);
static int asusdec_set_wakeup_cmd(void);
static void asusdec_reset_dock(void);
static void asusdec_smi(void);
/*
* extern variable
*/
extern unsigned int factory_mode;

extern int intel_mid_pmic_readb(int reg);
extern int intel_mid_pmic_writeb(int reg, u8 val);
/*
 * global variable
 */
char* switch_value[]={"0", "10", "11", "12"}; //0: no dock, 1:mobile dock, 2:audio dock, 3: audio st

enum firmware_update_type {
	UPDATE_BYTE_MODE = 0,
	UPDATE_BLOCK_MODE,
};

static int asusdec_kp_sci_table[]={0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH,
		ASUSDEC_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, ASUSDEC_KEY_AUTOBRIGHT,
		KEY_CAMERA, -9, -10, -11,
		-12, -13, -14, -15,
		KEY_WWW, ASUSDEC_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE,
		KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP,
		-24,ASUSDEC_KEYPAD_READINGMODE};
static bool EC_RESET = 0;
static bool insert_wakeup = 0;
static int BOARD_ER2 = 0;
static unsigned int asusdec_dock_in_gpio = 159;//GPIO_S5_29;
static unsigned int asusdec_dock_in_gpio_irq = 0;
static unsigned int asusdec_pwr_en_gpio = 55;//GPIO_S5_22;
//jjt hall sensor static unsigned int asusdec_hall_sensor_gpio = 0;
//jjt hall sensor static unsigned int asusdec_hall_sensor_gpio_irq = 0;
static unsigned int asusdec_apwake_gpio = 131;//GPIO_S5_1;
static unsigned int asusdec_apwake_gpio_irq = 0;
static unsigned int asusdec_ecreq_gpio = 2;//GPIO_S0_2
static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data
int reg_addr = -1;
static int fu_type = 0;
static int fu_block_mode = 0;
static struct i2c_client kb_client;
static struct i2c_client tp_client;
static struct i2c_client intr_client;
struct i2c_client dockram_client;

static struct class *asusdec_class;
static struct device *asusdec_device ;
static struct asusdec_chip *ec_chip;
static struct elan_i2c_data *elan_data;

struct cdev *asusdec_cdev ;
static dev_t asusdec_dev ;
static int asusdec_major = 0 ;
static int asusdec_minor = 0 ;

static struct workqueue_struct *asusdec_wq;
struct delayed_work asusdec_stress_work;

/*
 * functions definition
 */

void ec_irq_disable(void)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ec_chip->irq_lock, irqflags);
	if (!ec_chip->apwake_disabled)
	{
		ec_chip->apwake_disabled = 1;
		disable_irq_nosync(gpio_to_irq(asusdec_apwake_gpio));
//		disable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
	}
	spin_unlock_irqrestore(&ec_chip->irq_lock, irqflags);
}

void ec_irq_enable(void)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ec_chip->irq_lock, irqflags);
	if (ec_chip->apwake_disabled) 
	{
		enable_irq(gpio_to_irq(asusdec_apwake_gpio));
//		enable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
		ec_chip->apwake_disabled = 0;
	}
	spin_unlock_irqrestore(&ec_chip->irq_lock, irqflags);
}

static int asusdec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSDEC_MAGIC_NUM){
		ASUSDEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to write dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asusdec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSDEC_MAGIC_NUM){
		ASUSDEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to read dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asusdec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to write data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asusdec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		ec_irq_disable();
		disable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
		ASUSDEC_ERR("Disable pad apwake\n");
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to read data, status %d\n", ret);
		ec_chip->i2c_err_count++;
	} else {
		ec_chip->i2c_err_count = 0;
	}
	ASUSDEC_I2C_DATA(ec_chip->i2c_data, ec_chip->i2c_err_count);
	return ret;
}

static int asusdec_intr_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->intr_i2c_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to read data, status %d\n", ret);
	}
	return ret;
}
//add for HID over i2c
static int asus_keyboard_i2c_read(struct i2c_client *client, u16 reg,
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

static int BuffDataSize(void)
{
	int in = buff_in_ptr;
	int out = buff_out_ptr;

	if (in >= out){
		return (in - out);
	} else {
		return ((EC_BUFF_LEN - out) + in);
	}
}

static void BuffPush(char data)
{

	if (BuffDataSize() >= (EC_BUFF_LEN -1)){
		ASUSDEC_ERR("Error: EC work-buf overflow \n");
		return;
	}

	ec_to_host_buffer[buff_in_ptr] = data;
	buff_in_ptr++;
	if (buff_in_ptr >= EC_BUFF_LEN){
		buff_in_ptr = 0;
	}
}

static char BuffGet(void)
{
	char c = (char)0;

	if (BuffDataSize() != 0){
		c = (char) ec_to_host_buffer[buff_out_ptr];
		buff_out_ptr++;
		 if (buff_out_ptr >= EC_BUFF_LEN){
			 buff_out_ptr = 0;
		 }
	}
	return c;
}

static ssize_t dock_ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int i = 0;
	int ret;
	char tmp_buf[EC_BUFF_LEN];
	static int f_counter = 0;
	static int total_buf = 0;
	
	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);

	while ((BuffDataSize() > 0) && count)
	{
		tmp_buf[i] = BuffGet();
//		ASUSDEC_INFO("tmp_buf[%d] = 0x%x, total_buf = %d\n", i, tmp_buf[i], total_buf);
		count--;
		i++;
		f_counter = 0;
		total_buf++;
	}

	ret = copy_to_user(buf, tmp_buf, i);
	if (ret == 0)
	{
		ret = i;
	}

	return ret;
}

static ssize_t dock_ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int err;
	int i;

	if (h2ec_count > 0)
	{				   /* There is still data in the buffer that */
		return -EBUSY;  /* was not sent to the EC */
	}
	if (count > EC_BUFF_LEN)
	{
		return -EINVAL; /* data size is too big */
	}
	
	err = copy_from_user(host_to_ec_buffer, buf, count);
	if (err)
	{
		ASUSDEC_ERR("ec_write copy error\n");
		return err;
	}

	h2ec_count = count;
	switch (fu_type){
	case UPDATE_BYTE_MODE:
	for (i = 0; i < count ; i++){
		i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
	}
	break;
	case UPDATE_BLOCK_MODE:
	for (i = 0; i < count ; i++){
	 	ec_chip->i2c_fu_data[i] = host_to_ec_buffer[i];
	}
	i2c_smbus_write_block_data(&dockram_client, 0x41, count, ec_chip->i2c_fu_data);
	break;
	
	default:
	break;
	}
	
	h2ec_count = 0;
	return count;

}

//***********************SYSFS DOCK**************************************************************
static ssize_t asusdec_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->status);
}

static ssize_t asusdec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusdec_version_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusdec_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asusdec_dockram_read_data(0x0A);
	if (ret_val < 0)
		return sprintf(buf, "fail to get pad ec control-flag info\n");
	else{
		sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
		strcpy(buf, temp_buf);
		for (i = 1; i < 9; i++){
			sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
			strcat(buf, temp_buf);
		}
		return strlen(buf);
	}
}
//[Brook- Docking charging porting]>>
static ssize_t asusdec_dock_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;

	ret_val = asusdec_dockram_read_data(0x23);
	if (ret_val < 0)
		return sprintf(buf, "fail to get control-flag info\n");
	else{
		sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
		strcpy(buf, temp_buf);
		for (i = 1; i < 9; i++){
			sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
			strcat(buf, temp_buf);
		}
		return strlen(buf);
	}
	return sprintf(buf, "fail to get control-flag info\n");
}
//[Brook- Docking charging porting]<<

static ssize_t asusdec_show_lid_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", 1/*//jjt_t100_test_dock gpio_get_value(asusdec_hall_sensor_gpio)*/);
}

static ssize_t asusdec_send_ec_req_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_send_ec_req();
	return sprintf(buf, "EC_REQ is sent\n");
}

static ssize_t asusdec_enter_factory_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_enter_factory_mode();
	return sprintf(buf, "Entering factory mode\n");
}

static ssize_t asusdec_enter_normal_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_enter_normal_mode();
	return sprintf(buf, "Entering normal mode\n");
}

static ssize_t asusdec_win_shutdown_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_win_shutdown();
	return sprintf(buf, "Win shutdown\n");
}
static ssize_t asusdec_cmd_data_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int buf_len = strlen(buf);
	int data_len = (buf_len -1)/2;
	char chr[2], data_num[data_len];
	int i=0, j=0, idx=0, ret, data_cnt;
	int cmd;
	chr[2] = '\0';

	if (ec_chip->ec_in_s3){
		asusdec_send_ec_req();
		msleep(200);
	}
	memset(&ec_chip->i2c_dm_data, 0, 32);
	printk("buf_len=%d, data_len=%d \n",buf_len, data_len);
	if(!(buf_len&0x01) || !data_len){
		return -1;
	} else if(buf_len==3){
		reg_addr = (u8) simple_strtoul (buf,NULL,16);
		return EAGAIN;
	}
	for(i=0;i<buf_len-1;i++){
		chr[j] = *(buf+i);
		if(j==1){
			if (i == 1) {
				cmd = (int) simple_strtoul (chr,NULL,16);
			} else
				data_num[idx++] = (u8) simple_strtoul (chr,NULL,16);
		}
		j++;
		if(j>1){
			j=0;
		}
	}
	data_num[idx] = '\0';
	data_cnt = data_len - 1;

	if(data_cnt > 32) {
		printk("Input data count is over length\n");
		return -1;
	}
	memcpy(&ec_chip->i2c_dm_data[1], data_num, data_cnt);
	ec_chip->i2c_dm_data[0] = data_len - 1;

	for(i=0; i<data_len; i++){
		if (i ==0) printk("I2c cmd=0x%x\n", cmd);
		printk("I2c_dm_data[%d]=0x%x\n",i, ec_chip->i2c_dm_data[i]);
	}
	ret = asusdec_dockram_write_data(cmd, data_len);
	if(ret <0)
		ASUSDEC_NOTICE("Fail to write data\n");
	return count;
}

static ssize_t asusdec_return_data_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i, ret_val;
	char temp_buf[64];

	if (reg_addr != -1) {
		if (ec_chip->ec_in_s3){
			asusdec_send_ec_req();
			msleep(200);
		}
		printk("Smbus read EC command=0x%02x\n", reg_addr);
		ret_val = asusdec_dockram_read_data(reg_addr);
		reg_addr = -1;

		if (ret_val < 0)
			return sprintf(buf, "Fail to read ec data\n");
		else{
			if (ec_chip->i2c_dm_data[0]> 32)
				return sprintf(buf, "EC return data length error\n");
			for (i = 1; i <= ec_chip->i2c_dm_data[0] ; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}
	return 0;
}

static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, asusdec_status_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, asusdec_info_show,NULL);
static DEVICE_ATTR(ec_version, S_IWUSR | S_IRUGO, asusdec_version_show,NULL);
static DEVICE_ATTR(ec_control_flag, S_IWUSR | S_IRUGO, asusdec_control_flag_show,NULL);
//[Brook- Docking charging porting]>>
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, asusdec_dock_control_flag_show,NULL);
static DEVICE_ATTR(ec_lid, S_IWUSR | S_IRUGO, asusdec_show_lid_status,NULL);
//[Brook- Docking charging porting]<<
static DEVICE_ATTR(ec_request, S_IWUSR | S_IRUGO, asusdec_send_ec_req_show,NULL);
static DEVICE_ATTR(ec_factory_mode, S_IWUSR | S_IRUGO, asusdec_enter_factory_mode_show,NULL);
static DEVICE_ATTR(ec_normal_mode, S_IWUSR | S_IRUGO, asusdec_enter_normal_mode_show,NULL);
static DEVICE_ATTR(ec_win_shutdown, S_IWUSR | S_IRUGO, asusdec_win_shutdown_show,NULL);
static DEVICE_ATTR(ec_cmd_data_send, S_IWUSR | S_IRUGO, NULL, asusdec_cmd_data_store);
static DEVICE_ATTR(ec_data_read, S_IWUSR | S_IRUGO,  asusdec_return_data_show, NULL);

static struct attribute *asusdec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_ec_control_flag.attr,
	//[Brook- Docking charging porting]>>
	&dev_attr_ec_dock_control_flag.attr,
	&dev_attr_ec_lid.attr,
	//[Brook- Docking charging porting]<<
	&dev_attr_ec_request.attr,
	&dev_attr_ec_factory_mode.attr,
	&dev_attr_ec_normal_mode.attr,
	&dev_attr_ec_win_shutdown.attr,
	&dev_attr_ec_cmd_data_send.attr,
	&dev_attr_ec_data_read.attr,
NULL
};

static const struct attribute_group asusdec_smbus_group = {
	.attrs = asusdec_smbus_attributes,
};

static void asusdec_kb_init(struct i2c_client *client){
	kb_client.adapter = client->adapter;
	kb_client.addr = 0x16;
	kb_client.detected = client->detected;
	kb_client.dev = client->dev;
	kb_client.driver = client->driver;
	kb_client.flags = client->flags;
	strcpy(kb_client.name,client->name);
}

static void asusdec_tp_init(struct i2c_client *client){
	tp_client.adapter = client->adapter;
	tp_client.addr = 0x15;
	tp_client.detected = client->detected;
	tp_client.dev = client->dev;
	tp_client.driver = client->driver;
	tp_client.flags = client->flags;
	strcpy(tp_client.name,client->name);
}

static void asusdec_intr_init(struct i2c_client *client){
	intr_client.adapter = client->adapter;
	intr_client.addr = 0x19;
	intr_client.detected = client->detected;
	intr_client.dev = client->dev;
	intr_client.driver = client->driver;
	intr_client.flags = client->flags;
	strcpy(intr_client.name,client->name);
}
//[Brook- Bug 282345 + No charging icon when insert AC]>>
static void asusdec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x1b;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
	ec_chip->ec_ram_init = ASUSDEC_MAGIC_NUM;
}

static int asusdec_i2c_test(struct i2c_client *client){
	return asusdec_i2c_write_data(client, 0x0000);
}
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
static int asusdec_set_wakeup_cmd(void){
	int ret_val = 0;
	ASUSDEC_NOTICE("send command \n");
	if (ec_chip->dock_in){
		ret_val = asusdec_i2c_test(ec_chip->client);
		if(ret_val >= 0){
			asusdec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			if (ec_chip->dec_wakeup){
				ec_chip->i2c_dm_data[1] = 0x00;
				ec_chip->i2c_dm_data[2] = 0x00;
				ec_chip->i2c_dm_data[3] = 0x00;
				ec_chip->i2c_dm_data[4] = 0x00;
				ec_chip->i2c_dm_data[5] = 0x80;
			} else {
				ec_chip->i2c_dm_data[1] = 0x80;
			}
			asusdec_dockram_write_data(0x0A,9);
		}
	}
	return 0;
}
static void asusdec_reset_dock(void){
//	ec_chip->dock_init = 0;
	ASUSDEC_NOTICE("send EC_Request \n");
	gpio_set_value(asusdec_ecreq_gpio, 0);
	msleep(20);
	gpio_set_value(asusdec_ecreq_gpio, 1);
}
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
static void asusdec_enable_usb_power(void){

	int ret_val = 0;
	int i = 0;
	
	ec_chip->i2c_dm_data[0] = 0x08;
	ec_chip->i2c_dm_data[1] = 0;
	ec_chip->i2c_dm_data[2] = 0;
	ec_chip->i2c_dm_data[3] = 0;
	ec_chip->i2c_dm_data[4] = 0;
	ec_chip->i2c_dm_data[5] = 0;
	ec_chip->i2c_dm_data[6] = 0;
	ec_chip->i2c_dm_data[7] = 0x40;
	ec_chip->i2c_dm_data[8] = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("asusdec_SUSB_DOWN set to 0 fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("asusdec_SUSB_DOWN set to 0 ok\n");
			break;
		}
	}
	
}


static void asusdec_additonal_porting(void){

	int ret_val = 0;
	int i = 0;

	ret_val = asusdec_dockram_read_data(0x0A);
	if (ret_val < 0){
		ASUSDEC_ERR("fail to get control flag\n");
		return;
	}
	ASUSDEC_INFO("EC RAM 1:%2x %2x %2x %2x %2x %2x %2x %2x %2x \n",ec_chip->i2c_dm_data[0],ec_chip->i2c_dm_data[1],ec_chip->i2c_dm_data[2],ec_chip->i2c_dm_data[3]
	,ec_chip->i2c_dm_data[4],ec_chip->i2c_dm_data[5],ec_chip->i2c_dm_data[6],ec_chip->i2c_dm_data[7],ec_chip->i2c_dm_data[8]);
	
	ec_chip->i2c_dm_data[0] = 0x08;
	ec_chip->i2c_dm_data[1] = 0x20;
	ec_chip->i2c_dm_data[2] = 0;
	ec_chip->i2c_dm_data[3] = 0;
	ec_chip->i2c_dm_data[4] = 0;
	ec_chip->i2c_dm_data[5] = 0;
	ec_chip->i2c_dm_data[6] = 0x20;
	ec_chip->i2c_dm_data[7] = 0;
	ec_chip->i2c_dm_data[8] = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("asusdec touchpad power set to 0 fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("asusdec touchpad power set to 0 ok\n");
			break;
		}
	}
	ret_val = asusdec_dockram_read_data(0x0A);
	if (ret_val < 0){
		ASUSDEC_ERR("fail to get control flag\n");
		return;
	}
	ASUSDEC_INFO("EC RAM 2:%2x %2x %2x %2x %2x %2x %2x %2x %2x \n",ec_chip->i2c_dm_data[0],ec_chip->i2c_dm_data[1],ec_chip->i2c_dm_data[2],ec_chip->i2c_dm_data[3]
	,ec_chip->i2c_dm_data[4],ec_chip->i2c_dm_data[5],ec_chip->i2c_dm_data[6],ec_chip->i2c_dm_data[7],ec_chip->i2c_dm_data[8]);
}

static int asusdec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i = 0;

	ec_chip->op_mode = 0;

	for ( i = 0; i < 10; i++){
		ret_val = asusdec_i2c_test(client);
		if (ret_val < 0)
			msleep(300);
		else
			break;
	}

	if(ret_val < 0){
		ASUSDEC_INFO("asusdec_i2c_test error %x\n", ret_val);
		goto fail_to_access_ec;
	}

	if (asusdec_dockram_read_data(0x01) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	ASUSDEC_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);

	if (asusdec_dockram_read_data(0x02) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	ASUSDEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

	if (asusdec_dockram_read_data(0x03) < 0){
		goto fail_to_access_ec;
	}
	ASUSDEC_INFO("EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (asusdec_dockram_read_data(0x04) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_pcba, &ec_chip->i2c_dm_data[1]);
	ASUSDEC_NOTICE("PCBA Version: %s\n", ec_chip->ec_pcba);

	asusdec_additonal_porting();
	asusdec_enable_usb_power();

	ec_chip->status = 1;
//	switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
fail_to_access_ec:
	return 0;

}

static irqreturn_t asusdec_interrupt_handler(int irq, void *dev_id){
	int gpio = asusdec_apwake_gpio;
	
	//ASUSDEC_INFO("irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
	if (irq == asusdec_dock_in_gpio_irq){
		insert_wakeup = 1;
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	}
	else if (irq == asusdec_apwake_gpio_irq){
		ec_irq_disable();
		if (ec_chip->op_mode){
			queue_delayed_work(asusdec_wq, &ec_chip->asusdec_fw_update_work, 0);
		} else {
			queue_delayed_work(asusdec_wq, &ec_chip->asusdec_work, 0);
		}
	}
	else if (0){//jjt hall sensor (gpio == asusdec_hall_sensor_gpio){
//jjt hall sensor			queue_delayed_work(asusdec_wq, &ec_chip->asusdec_hall_sensor_work, 0);
		ASUSDEC_NOTICE("lid irq = %d", irq);
	}
	return IRQ_HANDLED;
}


static void asusdec_kp_sci(void){
	int ec_signal = ec_chip->intr_i2c_data[2];

	if(ec_chip->dock_status == 0){
		return;
	}

	ec_chip->keypad_data.input_keycode = asusdec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		ASUSDEC_INFO("input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);

		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev);

	}else{
		ASUSDEC_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
	}
}

//consumer key++

static int asusdec_kp_consumer_key_mapping(int x)
{
	switch (x){
		case ASUSDEC_KEYPAD_CONSUMER_BRIGHTNESSDOWN:
			return KEY_BRIGHTNESSDOWN;
		case ASUSDEC_KEYPAD_CONSUMER_BRIGHTNESSUP:
			return KEY_BRIGHTNESSUP;
		case ASUSDEC_KEYPAD_CONSUMER_EXPLORER:
			return KEY_WWW;
		case ASUSDEC_KEYPAD_CONSUMER_PREVIOUSTRACK:
			return KEY_PREVIOUSSONG;
		case ASUSDEC_KEYPAD_CONSUMER_PLAYPAUSE:
			return KEY_PLAYPAUSE;
		case ASUSDEC_KEYPAD_CONSUMER_NEXTTRACK:
			return KEY_NEXTSONG;
		case ASUSDEC_KEYPAD_CONSUMER_MUTE:
			return KEY_MUTE;
		case ASUSDEC_KEYPAD_CONSUMER_VOLUMEDOWN:
			return KEY_VOLUMEDOWN;
		case ASUSDEC_KEYPAD_CONSUMER_VOLUMEUP:
			return KEY_VOLUMEUP;
		default:
			printk("No consumer mapping string\n");
			return -1;
	}
}
//consumer key++

static int asusdec_kp_key_mapping(int x)
{
	switch (x){
		case ASUSDEC_KEYPAD_ESC:
			return KEY_BACK;

		case ASUSDEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSDEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSDEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSDEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSDEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSDEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSDEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSDEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSDEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSDEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSDEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSDEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSDEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSDEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSDEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSDEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSDEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSDEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSDEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSDEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSDEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSDEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSDEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSDEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSDEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSDEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSDEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSDEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSDEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSDEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSDEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSDEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSDEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSDEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSDEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSDEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSDEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSDEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSDEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSDEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSDEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSDEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSDEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSDEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSDEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSDEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSDEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSDEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSDEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSDEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSDEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSDEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSDEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSDEC_KEYPAD_HOME:
			return KEY_HOME;

		case ASUSDEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSDEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSDEC_KEYPAD_END:
			return KEY_END;

		case ASUSDEC_KEYPAD_SCRLK:
			return KEY_SCROLLLOCK;

		case ASUSDEC_KEYPAD_NUMLK:
						return KEY_NUMLOCK;	
	
		case ASUSDEC_KEYPAD_TPONOFF:
			return KEY_F2;

		case ASUSDEC_KEYPAD_MUTE:
			return KEY_MUTE;
 
		case ASUSDEC_KEYPAD_VOLUMEDOWN:
			return KEY_VOLUMEDOWN;
 
		case ASUSDEC_KEYPAD_VOLUMEUP:
			return KEY_VOLUMEUP;

		case ASUSDEC_KEYPAD_DELETE:
			return KEY_DELETE;

		case ASUSDEC_KEYPAD_BRIGHTNESSDOWN:
			return KEY_BRIGHTNESSDOWN;

		case ASUSDEC_KEYPAD_BRIGHTNESSUP:
			return KEY_BRIGHTNESSUP;

		case ASUSDEC_KEYPAD_FLYMODE:
			return KEY_F22;

		case ASUSDEC_KEYPAD_SUSPEND:
			return KEY_SLEEP;

		case ASUSDEC_KEYPAD_PAUSE:
			return KEY_PAUSE;

		case ASUSDEC_KEYPAD_PRINTSCREEN:
			return KEY_PRINT;

		case ASUSDEC_KEYPAD_INSERT:
			return KEY_INSERT;
		/*Chris 0812 end*/
		//--- JP keys
		case ASUSDEC_YEN:
			return KEY_YEN;

		case ASUSDEC_RO:
			return KEY_RO;

		case ASUSDEC_MUHENKAN:
			return KEY_MUHENKAN;

		case ASUSDEC_HENKAN:
			return KEY_HENKAN;

		case ASUSDEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;

		//--- UK keys
		case ASUSDEC_EUROPE_2:
			return KEY_102ND;

		default:
			printk("No mapping string\n");
			return -1;
	}
}

static void asusdec_kb_report_work_function(struct work_struct *dat)
{
	int gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;
	int i = 0;
	int j = 0;
	int the_same_key = 0;
	int scancode = 0;
		
	memset(&ec_chip->i2c_kb_data, 0, 32);
		
	//ret_val = i2c_smbus_read_i2c_block_data(&kb_client, 0x73, 11, ec_chip->i2c_kb_data);
	ret_val = asus_keyboard_i2c_read(&kb_client, 0x73, ec_chip->i2c_kb_data,11);
	ec_irq_enable();

	ASUSDEC_INFO("kb irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
	if(ec_chip->dock_status == 0){
			return;
	}

	if(ec_chip->i2c_kb_data[0] == 0 && ec_chip->i2c_kb_data[1] == 0){//not press key
		return;
	}

	ASUSDEC_NOTICE("key code : 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",ec_chip->i2c_kb_data[0],ec_chip->i2c_kb_data[1],ec_chip->i2c_kb_data[2],ec_chip->i2c_kb_data[3],ec_chip->i2c_kb_data[4],ec_chip->i2c_kb_data[5],ec_chip->i2c_kb_data[6],ec_chip->i2c_kb_data[7],ec_chip->i2c_kb_data[8],ec_chip->i2c_kb_data[9],ec_chip->i2c_kb_data[10]);
	//ASUSDEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[1]);
	//ASUSDEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[2]);
	//ASUSDEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[3]);
	//ASUSDEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[4]);
	//ASUSDEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[5]);

	ec_chip->keypad_data.extend = 0;
//consumer key++
	if(ec_chip->i2c_kb_data[0] == 0x5 && ec_chip->i2c_kb_data[2] == 0x13){//not press key
		ASUSDEC_NOTICE("consumer key\n");
		if(ec_chip->i2c_kb_data[3]){
			input_report_key(ec_chip->indev, asusdec_kp_consumer_key_mapping(ec_chip->i2c_kb_data[3]), 1);
		}else{
			input_report_key(ec_chip->indev, asusdec_kp_consumer_key_mapping(ec_chip->i2c_old_kb_data[3]), 0);
		}
	}else if(ec_chip->i2c_kb_data[0] == 0x4 && ec_chip->i2c_kb_data[2] == 0x14){
		ASUSDEC_NOTICE("power key !\n");
		if(ec_chip->i2c_old_kb_data[3] == 1 && ec_chip->i2c_kb_data[3] == 0){
			input_report_key(ec_chip->indev, KEY_POWER, 1);
			input_sync(ec_chip->indev);
			input_report_key(ec_chip->indev, KEY_POWER, 0);
		}
	}else{
//consumer key++
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
			input_report_key(ec_chip->indev, KEY_LEFTCTRL, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
			input_report_key(ec_chip->indev, KEY_LEFTCTRL, 0);
		}
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
			input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
			input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 0);
		}
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
			input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
			input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 0);
		}
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
			input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
			input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 0);
		}
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
			input_report_key(ec_chip->indev, KEY_RIGHTALT, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
			input_report_key(ec_chip->indev, KEY_RIGHTALT, 0);
		}
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
			input_report_key(ec_chip->indev, KEY_HOMEPAGE, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
			input_report_key(ec_chip->indev, KEY_HOMEPAGE, 0);
		}	
		if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
			input_report_key(ec_chip->indev, KEY_SEARCH, 1);
		}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
			input_report_key(ec_chip->indev, KEY_SEARCH, 0);
		}
	}//consumer key++

	for(i = 0;i < 6;i++)//normal keys
	{
		if(ec_chip->i2c_kb_data[i+5] > 0){//press key
			ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_kb_data[i+5]);
			ec_chip->keypad_data.value = 1;
			ASUSDEC_INFO("keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
			input_report_key(ec_chip->indev,
				ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		}else if(ec_chip->i2c_kb_data[i+5] == 0){
			break;
		}else{
			ASUSDEC_INFO("Unknown scancode = 0x%x\n", scancode);
		}
	}
	for(i = 0;i < 6;i++)
	{
		if(ec_chip->i2c_old_kb_data[i+5] > 0){
			for(j = 0;j < 6;j++)//check key break
			{
				if(ec_chip->i2c_kb_data[j+5] == ec_chip->i2c_old_kb_data[i+5]){
					the_same_key = 1;
					break;
				}else
				{
					the_same_key = 0;
				}
			}
			if(the_same_key == 0){
				ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_old_kb_data[i+5]);
				input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
			}
		}else{
			break;
		}
	}
	for(i = 0;i < 8;i++)
	{
		ec_chip->i2c_old_kb_data[i+3] = ec_chip->i2c_kb_data[i+3];
	}
	input_sync(ec_chip->indev);

#if 1
	if(ec_chip->suspend_state){
		printk("jjt wake system\n");
		input_report_key(ec_chip->indev, KEY_WAKEUP, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, KEY_WAKEUP, 0);
		input_sync(ec_chip->indev);
	}
#endif

}

static void asusdec_tp_report_work_function(struct work_struct *dat) {
	int gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	u8 packet[ETP_REPORT_LENGTH];
	int retval;
	int i=0,tp_mode=1;

	retval = i2c_master_recv(&tp_client, packet, ETP_REPORT_LENGTH);
	ec_irq_enable();
	if(!gpio_get_value(acpi_get_gpio("\\_SB.GPO2", 12)))
		return;
	ASUSDEC_INFO("tp irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
	tp_mode = elan_i2c_check_packet(packet);
	if(tp_mode == 1){
		elan_i2c_report_standard(ec_chip, packet);
	}else if(tp_mode == 0){
		elan_i2c_report_absolute(ec_chip->private, packet);
	}else{
		ASUSDEC_ERR("wrong packet data \n");
		for(i=0;i<retval;i++)
			printk("0x%x ",packet[i]);
	}
	return;
}

static int asusdec_tp_control(int arg){

	int ret_val = 0;
	u8 packet[ETP_REPORT_LENGTH];

	if(arg == ASUSDEC_TP_ON){
		if (ec_chip->tp_enable == 0){
			ec_chip->tp_enable = 1;
			elan_i2c_enable(&tp_client);
		}
		if (ec_chip->touchpad_member == -1){
			ec_chip->init_success = -1;
			queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
		}
		ret_val = 0;
	} else if (arg == ASUSDEC_TP_OFF){
		elan_i2c_disable(&tp_client);
		msleep(20);
		elan_i2c_disable(&tp_client);
		memset(packet,0,sizeof(packet[ETP_REPORT_LENGTH]));
		elan_i2c_report_absolute(ec_chip->private, packet);
		ec_chip->tp_enable = 0;
		ret_val = 0;
	} else
		ret_val = -ENOTTY;

	return ret_val;
}
//******************

static int asusdec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asusdec_ecreq_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asusdec_request" ;

	gpio = asusdec_ecreq_gpio = acpi_get_gpio("\\_SB.GPO0", 2);//GPIO_S0_2
	irq = gpio_to_irq(gpio);


	ASUSDEC_INFO("gpio = %d, irq = %d\n", gpio,irq);
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}

	rc = gpio_direction_output(gpio, 1) ;
	if (rc) {
		ASUSDEC_ERR("gpio_direction_output failed for output %d\n", gpio);
		goto err_exit;
	}
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	return 0 ;

err_exit:
	return rc;
}

static int asusdec_irq_dock_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio;
	unsigned irq;
	unsigned pwr_gpio;
	const char* label = "asusdec_dock_insert" ;

	gpio = asusdec_dock_in_gpio = acpi_get_gpio("\\_SB.GPO2", 29);//GPIO_S5_29
	irq = asusdec_dock_in_gpio_irq = gpio_to_irq(gpio);

	pwr_gpio = asusdec_pwr_en_gpio = acpi_get_gpio("\\_SB.GPO0", 55);//GPIO_S5_22

	ASUSDEC_INFO("gpio = %d, irq = %d\n", gpio, irq);
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_request(pwr_gpio, "DOCK_PWR_EN");
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for ouput %d\n", pwr_gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSDEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	if(BOARD_ER2){
		rc = gpio_direction_output(pwr_gpio, 1) ;
		if (rc) {
			ASUSDEC_ERR("gpio_direction_output failed for output %d\n", pwr_gpio);
			goto err_gpio_direction_input_failed;
		}
		ASUSDEC_INFO("pwr_gpio = %d , state = %d\n", pwr_gpio, gpio_get_value(pwr_gpio));
	}

	rc = request_irq(irq, asusdec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	enable_irq_wake(irq);

	ASUSDEC_INFO("Dock in irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static int asusdec_irq_ec_apwake(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asusdec_apwake" ;

	gpio = asusdec_apwake_gpio = acpi_get_gpio("\\_SB.GPO2", 1);//GPIO_S5_1
	asusdec_apwake_gpio_irq = irq = gpio_to_irq(gpio);

	ASUSDEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSDEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	if(gpio_get_value(asusdec_dock_in_gpio)){
		rc = request_irq(irq, asusdec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
		if (rc < 0) {
			ec_chip->apwake_irq_req = 0;
			ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
			rc = -EIO;
			goto err_gpio_request_irq_fail ;
		}
		//ec_irq_disable();
		ec_chip->apwake_irq_req = 1;
		ASUSDEC_INFO("request irq = %d, rc = %d\n", irq, rc);
	}

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}

//static int asusdec_irq_hall_sensor(struct i2c_client *client)
//{
//	int rc = 0 ;
//	unsigned gpio = asusdec_hall_sensor_gpio;
//	unsigned irq = gpio_to_irq(asusdec_hall_sensor_gpio);
//	const char* label = "asusdec_hall_sensor" ;

//	ASUSDEC_INFO("gpio = %d, irq = %d\n", gpio, irq);
//	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

//	rc = gpio_request(gpio, label);
//	if (rc) {
//		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
//	}

//	rc = gpio_direction_input(gpio) ;
//	if (rc) {
//		ASUSDEC_ERR("gpio_direction_input failed for input %d\n", gpio);
//		goto err_gpio_direction_input_failed;
//	}
//	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

//	rc = request_irq(irq, asusdec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
//	if (rc < 0) {
//		ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
//		rc = -EIO;
//		goto err_gpio_request_irq_fail ;
//	}
//	enable_irq_wake(irq);

//	ASUSDEC_INFO("LID irq = %d, rc = %d\n", irq, rc);

//	if (gpio_get_value(gpio)){
//		ASUSDEC_NOTICE("LID open\n");
//	} else{
//		ASUSDEC_NOTICE("LID close\n");
//	}

//	return 0 ;

//err_gpio_request_irq_fail :
//	gpio_free(gpio);
//err_gpio_direction_input_failed:
//	return rc;
//}

static void asusdec_enter_s3_timer(unsigned long data){
	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_enter_s3_work, 0);
}

static void asusdec_send_ec_req(void){
	ASUSDEC_NOTICE("send EC_Request\n");
	//Shouchung add to solve the touch pad issue when dock in
//jj-	asusdec_tp_enable_function(0xf5);
//	elan_i2c_disable(&tp_client);
	//Shouchung end
	gpio_set_value(asusdec_ecreq_gpio, 0);
	ASUSDEC_INFO("jjt 1 GPIO = %d , state = %d\n", asusdec_ecreq_gpio, gpio_get_value(asusdec_ecreq_gpio));
	msleep(DELAY_TIME_MS);
	gpio_set_value(asusdec_ecreq_gpio, 1);
	ASUSDEC_INFO("jjt 2 GPIO = %d , state = %d\n", asusdec_ecreq_gpio, gpio_get_value(asusdec_ecreq_gpio));
	//Shouchung add to solve the touch pad issue when dock in
	msleep(100);
//jj-	asusdec_tp_enable_function(0xf4);
//	elan_i2c_enable(&tp_client);
	//Shouchung end
}

static void asusdec_enter_s3_work_function(struct work_struct *dat)
{
	int ret_val = 0;
	int i = 0;

	mutex_lock(&ec_chip->state_change_lock);

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		mutex_unlock(&ec_chip->state_change_lock);
		return ;
	}

	ec_chip->ec_in_s3 = 1;
	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSDEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("Send s3 command fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("EC in S3\n");
			break;
		}
	}
	mutex_unlock(&ec_chip->state_change_lock);
}

/*static void asusdec_init_work_function(struct work_struct *dat)
{
	asusdec_send_ec_req();
	msleep(200);
	asusdec_chip_init(ec_chip->client);
}*/

static void asusdec_stresstest_work_function(struct work_struct *dat)
{
	asusdec_i2c_read_data(ec_chip->client);
	queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
}

//[Brook- Docking charging porting]>>
static void asusdec_dock_status_report(void){
	ASUSDEC_NOTICE("dock_in = %d, ec_chip->dock_type = %d \n", ec_chip->dock_in, ec_chip->dock_type);
	switch_set_state(&ec_chip->dock_sdev, switch_value[ec_chip->dock_type]);
}

static void asusdec_keypad_set_input_params(struct input_dev *dev)
{
	int i = 0;
	set_bit(EV_KEY, dev->evbit);
	for ( i = 0; i < MAX_keybit; i++)
		set_bit(i,dev->keybit);
	//Shouchung add for touch pad
	set_bit(EV_REL, dev->evbit);
	set_bit(REL_X, dev->relbit);
	set_bit(REL_Y, dev->relbit);
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);
	set_bit(EV_SYN, dev->evbit);
#if 1
	input_set_capability(dev, EV_KEY, KEY_WAKEUP);
#endif
	//Shouchung end
}

static int asusdec_input_device_create(struct i2c_client *client){
	int err = 0;
	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		ASUSDEC_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->indev->name = "asuspec";
	ec_chip->indev->phys = "/dev/input/asuspec";
//	ec_chip->indev->dev.parent = &client->dev;
//jjt-	ec_chip->indev->event = asusdec_event;
	asusdec_keypad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
		ASUSDEC_ERR("input registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;
}

static void asusdec_dock_init_work_function(struct work_struct *dat)
{
 	int gpio = asusdec_dock_in_gpio;
	int gpio_state = 0;
	int rc = 0 ;
	const char* label = "asusdec_apwake" ;

	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	wake_lock(&ec_chip->wake_lock_init);
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
 	gpio_state = gpio_get_value(gpio);
	if ((!gpio_state) || (EC_RESET == 1)){
		ASUSDEC_NOTICE("No dock detected\n");
		if(ec_chip->apwake_irq_req){
			free_irq(asusdec_apwake_gpio_irq, ec_chip->client);
			ec_chip->apwake_irq_req = 0;
		}
		ec_chip->status = 0;
		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->dock_status = 0;
		ec_chip->dock_type = DOCK_UNKNOWN;
		ec_chip->touchpad_member = -1;
		/*Chris start*/
		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}
		/*Chris end*/
		if (ec_chip->private->input){
			input_unregister_device(ec_chip->private->input);
			ec_chip->private->input = NULL;
		}
		ec_chip->tp_enable = 0;
		if(!BOARD_ER2){
			rc = gpio_direction_output(asusdec_pwr_en_gpio, 0) ;
			if (rc) {
				ASUSDEC_ERR("gpio_direction_output failed for output %d\n", asusdec_pwr_en_gpio);
			}
			ASUSDEC_INFO("pwr_gpio = %d , state = %d\n", asusdec_pwr_en_gpio, gpio_get_value(asusdec_pwr_en_gpio));
		}
		asusdec_dock_status_report();
		if(EC_RESET){
			EC_RESET = 0;
			queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, msecs_to_jiffies(100));//msecs_to_jiffies(500)
		}
	}else{
		ASUSDEC_NOTICE("Dock-in detected\n");
		if(!BOARD_ER2){
			msleep(10);
			rc = gpio_direction_output(asusdec_pwr_en_gpio, 1) ;
			if (rc) {
				ASUSDEC_ERR("gpio_direction_output failed for output %d\n", asusdec_pwr_en_gpio);
			}
			ASUSDEC_INFO("pwr_gpio = %d , state = %d\n", asusdec_pwr_en_gpio, gpio_get_value(asusdec_pwr_en_gpio));
		}
		//asusdec_send_ec_req(); //---remove reset EC i2c module----
		msleep(20);
		if(ec_chip->apwake_irq_req == 0){
			rc = request_irq(asusdec_apwake_gpio_irq, asusdec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, ec_chip->client);
			if (rc < 0) {
				ec_chip->apwake_irq_req = 0;
				ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, asusdec_apwake_gpio_irq, rc);
			}
			ec_chip->apwake_irq_req = 1;
			ASUSDEC_INFO("request irq = %d, rc = %d\n", asusdec_apwake_gpio_irq, rc);
		}	
		ec_chip->dock_type = MOBILE_DOCK;
		ec_chip->dock_in = 1;
	}
exit:
	wake_unlock(&ec_chip->wake_lock_init);
	ASUSDEC_NOTICE("exit! \n");
	return ;

fail_to_access_ec:
	if (!gpio_state){
		ASUSDEC_NOTICE("No EC detected\n");
		ec_chip->dock_in = 0;
	} else {
		ASUSDEC_NOTICE("Need EC FW update\n");
	}
	goto exit;
}

static void asusdec_chip_init_work_function(struct work_struct *dat)
{
	ASUSDEC_INFO("EC keyboard init !\n");

	if (&kb_client != NULL){
		asusdec_input_device_create(&kb_client);
		printk("asusdec_input_device_create in\n");
	}
	printk("asusdec_input_device_create out\n");
	
	if(ec_chip->status == 0)asusdec_chip_init(ec_chip->client);
	
	memset(&ec_chip->i2c_kb_data, 0, 32);
	ec_chip->i2c_kb_data[0] = 0x00;
	ec_chip->i2c_kb_data[1] = 0x00;
	ec_chip->i2c_kb_data[2] = 0x08;
	i2c_smbus_write_i2c_block_data(&kb_client, 0x75, 3, ec_chip->i2c_kb_data);
	msleep(50);//FIXME:will use retry
	asus_keyboard_i2c_read(&kb_client, 0x73, ec_chip->i2c_kb_data,11);	
	return ;
}

static void asusdec_touchpad_init_work_function(struct work_struct *dat)
{
	ASUSDEC_INFO("touchpad init ++++!\n");
	int ret_val = 0;
	wake_lock(&ec_chip->wake_lock_init);
	ret_val = asusdec_dockram_read_data(0x0A);
	if ((ret_val > 0) && (ec_chip->i2c_dm_data[3] & ASUSDEC_HID_STATUS) && ( gpio_get_value(asusdec_dock_in_gpio))){
		if(ec_chip->tp_enable == 1)return;
		if(!elan_i2c_initialize(&tp_client)){
			ec_chip->tp_enable = 1;
			ec_chip->touchpad_member = ELANTOUCHPAD;
			ec_chip->private->client = &tp_client;
			ret_val = elan_i2c_input_dev_create(ec_chip->private);
				if (ret_val < 0) ASUSDEC_NOTICE("fail to creat input for touchpad\n");
		}else{
			ec_chip->tp_enable = 0;
			ASUSDEC_NOTICE("fail to init touchpad\n");
		}
	}
	else{
		ec_chip->tp_enable = 0;
		ASUSDEC_NOTICE("HID is not ready yet\n");
	}
	
	ec_chip->init_success = 1;
	ec_chip->dock_status = 1;
	asusdec_dock_status_report();
	wake_unlock(&ec_chip->wake_lock_init);
	ASUSDEC_INFO("touchpad init ------!\n");
	return ;
}

static void asusdec_lid_report_function(struct work_struct *dat)
{
	int value = 0;

	if (ec_chip->lid_indev == NULL){
		ASUSDEC_ERR("LID input device doesn't exist\n");
		return;
	}
	msleep(CONVERSION_TIME_MS);
	value = 1;//jjt hall sensor  gpio_get_value(asusdec_hall_sensor_gpio);
	input_report_switch(ec_chip->lid_indev, SW_LID, !value);
	input_sync(ec_chip->lid_indev);
	ASUSDEC_NOTICE("SW_LID report value = %d\n", !value);
}
//[Brook- Docking charging porting]<<

static void asusdec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;
	int i = 0;
	int ret = 0;

	mutex_lock(&ec_chip->lock);

		switch (fu_type){
		case UPDATE_BYTE_MODE:
			smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
			ec_irq_enable();
			BuffPush(smbus_data);
			break;
		case UPDATE_BLOCK_MODE:
			ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x6a, 8/*32*/, ec_chip->i2c_fu_data);
			if (ret < 0)
				ASUSDEC_ERR("fu work Fail to read data, status %02x\n", ret);
			for (i = 0; i < ec_chip->i2c_fu_data[0] + 2 ; i++){
				BuffPush(ec_chip->i2c_fu_data[i]);//FIXME:only need push fa
			}
			ec_irq_enable();
			break;

		default:
			break;
		}
	mutex_unlock(&ec_chip->lock);
}

static void asusdec_work_function(struct work_struct *dat)
{
	int gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;

	ret_val = asusdec_intr_i2c_read_data(&intr_client);
	if (ret_val < 0){
		ec_irq_enable();
		return ;
	}

	ASUSDEC_NOTICE("0x%x 0x%x 0x%x 0x%x\n", ec_chip->intr_i2c_data[0],
		ec_chip->intr_i2c_data[1], ec_chip->intr_i2c_data[2], ec_chip->intr_i2c_data[3]);
		
//jjt
	if((ec_chip->intr_i2c_data[0] == 0x3)&&(ec_chip->intr_i2c_data[1] == 0xc1)&&(ec_chip->intr_i2c_data[2] == ASUSDEC_KEY_MASK)){
		if(ec_chip->intr_i2c_data[3] == 0x11 || ec_chip->intr_i2c_data[3] == 0x13 || ec_chip->intr_i2c_data[3] == 0x14/*consumer */){
				queue_delayed_work(asusdec_wq, &ec_chip->asusdec_kb_report_work, 0);
		}else if(ec_chip->intr_i2c_data[3] == 0x1){
				queue_delayed_work(asusdec_wq, &ec_chip->asusdec_tp_report_work, 0);
		}
		return ;
	}
//jjt
	ec_irq_enable();
	if (ec_chip->intr_i2c_data[1] & ASUSDEC_OBF_MASK){
		if (ec_chip->intr_i2c_data[1] & ASUSDEC_SMI_MASK){
			asusdec_smi();
			ASUSDEC_INFO("asusdec_smi irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
			return ;
		}else if(ec_chip->intr_i2c_data[1] & ASUSDEC_SCI_MASK){
			if(ec_chip->intr_i2c_data[2] >= 0 && ec_chip->intr_i2c_data[2] <= 25)
				asusdec_kp_sci();
			ASUSDEC_INFO("asusdec_sci irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
			return ;
		}
	}
}

static void asusdec_smi(void){
	if (ec_chip->intr_i2c_data[2] == ASUSDEC_SMI_HANDSHAKING){
		ASUSDEC_NOTICE("ASUSDEC_SMI_HANDSHAKING\n");
		/*
		if(ec_chip->status == 0){
			asusdec_chip_init(ec_chip->client);
		}*/
		ec_chip->ec_in_s3 = 0;
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SMI_RESET){
		ASUSDEC_NOTICE("ASUSDEC_SMI_RESET\n");
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SMI_WAKE){
		ASUSDEC_NOTICE("ASUSDEC_SMI_WAKE\n");
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_chip_init_work, 0);
	} else if (ec_chip->intr_i2c_data[2] == APOWER_SMI_S5){
		ASUSDEC_NOTICE("APOWER_POWEROFF\n");
		asusdec_switch_apower_state(APOWER_POWEROFF);
	} else if (ec_chip->intr_i2c_data[2] == APOWER_SMI_NOTIFY_SHUTDOWN){
		ASUSDEC_NOTICE("APOWER_NOTIFY_SHUTDOWN\n");
		asusdec_switch_apower_state(APOWER_NOTIFY_SHUTDOWN);
	} else if (ec_chip->intr_i2c_data[2] == APOWER_SMI_RESUME){
		ASUSDEC_NOTICE("APOWER_SMI_RESUME\n");
		asusdec_switch_apower_state(APOWER_RESUME);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_BOOTBLOCK_RESET){ 
		ASUSDEC_NOTICE("ASUSDEC_SxI_BOOTBLOCK_RESET \n");
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	}else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_ADAPTER_CHANGE){
		ASUSDEC_NOTICE("ASUSDEC_SxI_ADAPTER_CHANGE \n");
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_DOCK_INSERT){	
		ASUSDEC_NOTICE("ASUSDEC_SxI_DOCK_INSERT\n");
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	}else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_DOCK_REMOVE){	 
		ASUSDEC_NOTICE("ASUSDEC_SxI_DOCK_REMOVE\n");
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_PAD_BL_CHANGE){ 
		ASUSDEC_NOTICE("ASUSDEC_SxI_PAD_BL_CHANGE \n");
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_HID_Status_Changed){
		ASUSDEC_NOTICE("ASUSDEC_SxI_HID_Status_Changed \n");
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_touchpad_init_work, 0);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_HID_WakeUp){	  
		ASUSDEC_NOTICE("ASUSDEC_SxI_HID_WakeUp \n");
	}

}

static ssize_t apower_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", APOWER_SDEV_NAME);
}

static ssize_t apower_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->apower_state);
}

static int asusdec_open(struct inode *inode, struct file *flip){
	ASUSDEC_NOTICE("\n");
	return 0;
}
static int asusdec_release(struct inode *inode, struct file *flip){
	ASUSDEC_NOTICE("\n");
	return 0;
}
static long asusdec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;
	char name_buf[64];
	int length = 0;
	char *envp[3];
	int env_offset = 0;

	if (_IOC_TYPE(cmd) != ASUSDEC_IOC_MAGIC)
	 return -ENOTTY;
	if (_IOC_NR(cmd) > ASUSDEC_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd) {
		case ASUSDEC_POLLING_DATA:
			if (arg == ASUSDEC_IOCTL_HEAVY){
				ASUSDEC_NOTICE("heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if (arg == ASUSDEC_IOCTL_NORMAL){
				ASUSDEC_NOTICE("normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == ASUSDEC_IOCTL_END){
				ASUSDEC_NOTICE("polling end\n");
				cancel_delayed_work_sync(&asusdec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case ASUSDEC_FW_UPDATE:
			ASUSDEC_NOTICE("ASUSDEC_FW_UPDATE\n");
			mutex_lock(&ec_chip->state_change_lock);
			//asusdec_send_ec_req();
			msleep(200);
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			h2ec_count = 0;
			memset(host_to_ec_buffer, 0, EC_BUFF_LEN);
			memset(ec_to_host_buffer, 0, EC_BUFF_LEN);
			memset(&ec_chip->i2c_dm_data, 0, 32);
			ec_chip->status = 0;
			ec_chip->op_mode = 1;
			wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			msleep(2400);
			switch(arg){
			case 0:
				ASUSDEC_ERR("ASUSDEC_FW_UPDATE:forbid byte mode update to prevent update fail!\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
				switch_set_state(&ec_chip->dock_sdev, !ec_chip->dock_sdev.state);
				msleep(2500);
				return -ENOTTY;
				break;
			case 1:
				if (ec_chip->dock_in){
				ASUSDEC_NOTICE("ASUSDEC_FW_UPDATE use block mode\n");
				fu_block_mode = 1;
				fu_type = UPDATE_BLOCK_MODE;
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x41, 3, ec_chip->i2c_dm_data);
				dockram_client.flags = I2C_CLIENT_PEC;
				msleep(2500);
				} else {
					ASUSDEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;

			default:
				ASUSDEC_ERR("error fu type!\n");
				break;
			}
			msleep(1000);
			mutex_unlock(&ec_chip->state_change_lock);
			break;
		case ASUSDEC_INIT:
			ASUSDEC_NOTICE("ASUSDEC_INIT\n");
			msleep(500);
			ec_chip->status = 0;
			ec_chip->op_mode = 0;
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			switch(fu_type){
			case UPDATE_BYTE_MODE:
			case UPDATE_BLOCK_MODE:
				EC_RESET = 1;
				queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
				switch_set_state(&ec_chip->dock_sdev, !ec_chip->dock_sdev.state);
				msleep(2500);
				ASUSDEC_NOTICE("ASUSDEC_INIT - EC version: %s\n", ec_chip->ec_version);
				length = strlen(ec_chip->ec_version);
				ec_chip->ec_version[length] = NULL;
				snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
				envp[env_offset++] = name_buf;
				envp[env_offset] = NULL;
				kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
				break;
			default:
				ASUSDEC_ERR("ASUSDEC_INIT unknow case!\n");
				break;
			}
			msleep(2500);
			break;
		case ASUSDEC_TP_CONTROL:
			   ASUSDEC_NOTICE("ASUSDEC_TP_CONTROL\n");
					 if ((ec_chip->op_mode == 0) && ec_chip->dock_in){
						 //err = asusdec_tp_control(arg);
						 //err = asusdec_tp_control2(arg);
						 asusdec_tp_control(arg);//jj
						 return err;
						}
			   else
				return -ENOTTY;
		case ASUSDEC_EC_WAKEUP:
			msleep(500);
			ASUSDEC_NOTICE("ASUSDEC_EC_WAKEUP, arg = %s \n", arg?"ASUSDEC_EC_ON":"ASUSDEC_EC_OFF");
			if (arg == ASUSDEC_EC_OFF){
				ec_chip->dec_wakeup = 0;
				ASUSDEC_NOTICE("Set EC shutdown when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}
			else if (arg == ASUSDEC_EC_ON){
				ec_chip->dec_wakeup = 1;
				ASUSDEC_NOTICE("Keep EC active when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}else{
				ASUSDEC_ERR("Unknown argument");
				return -ENOTTY;
			}
		case ASUSDEC_FW_DUMMY:
			ASUSDEC_NOTICE("ASUSDEC_FW_DUMMY\n");
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;

			switch(fu_type){
			case UPDATE_BYTE_MODE:
				ASUSDEC_ERR("dont support byte mode\n");
				break;
			case UPDATE_BLOCK_MODE:
				if (ec_chip->dock_in){
				ASUSDEC_NOTICE("ASUSDEC_dock FW_UPDATE use block mode\n");
				fu_block_mode = 1;
				ASUSDEC_NOTICE("ASUSDEC_FW_DUMMY pad block mode\n");
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x41, 3, ec_chip->i2c_dm_data);
				dockram_client.flags = I2C_CLIENT_PEC;
				msleep(500);
				} else {
					ASUSDEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;
			default:
				ASUSDEC_ERR("error fu type!\n");
				break;
			}
			if(arg == 1){
			}
			else{
				ASUSDEC_ERR("ASUSDEC_FW_UPDATE:forbid byte mode update to prevent update fail!\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
				switch_set_state(&ec_chip->dock_sdev, !ec_chip->dock_sdev.state);
				msleep(2500);
				return -ENOTTY;
			}
			break;
		case ASUSDEC_WIN_SHUTDOWN:
			ASUSDEC_NOTICE("ASUSDEC_WIN_SHUTDOWN\n", arg);
			asusdec_win_shutdown();
			break;
		default: /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return 0;
}

static void asusdec_switch_apower_state(int state){
	ec_chip->apower_state = state;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	ec_chip->apower_state = APOWER_IDLE;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
}

static void asusdec_win_shutdown(void){
	int ret_val = 0;
	int i = 0;

	if (ec_chip->ec_in_s3){
		asusdec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSDEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[8] = ec_chip->i2c_dm_data[8] | 0x40;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("Win shutdown command fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("Win shutdown\n");
			break;
		}
	}
}

static void asusdec_enter_factory_mode(void){

	ASUSDEC_NOTICE("Entering factory mode\n");
	asusdec_dockram_read_data(0x0A);
	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
	asusdec_dockram_write_data(0x0A,9);
}

static void asusdec_enter_normal_mode(void){

	int ret_val = 0;
	int i = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSDEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("Entering normal mode fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("Entering normal mode\n");
			break;
		}
	}
}

static ssize_t asusdec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", switch_value[ec_chip->dock_type]);
}

//[Panda-Hall sensor]>>
static void asusdec_lid_set_input_params(struct input_dev *dev)
{
	set_bit(EV_SW, dev->evbit);
	set_bit(SW_LID, dev->swbit);
}

static int asusdec_lid_input_device_create(struct i2c_client *client){
	int err = 0;

	ec_chip->lid_indev = input_allocate_device();
	if (!ec_chip->lid_indev) {
		ASUSDEC_ERR("lid_indev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->lid_indev->name = "lid_input";
	ec_chip->lid_indev->phys = "/dev/input/lid_indev";
	ec_chip->lid_indev->dev.parent = &client->dev;

	asusdec_lid_set_input_params(ec_chip->lid_indev);
	err = input_register_device(ec_chip->lid_indev);
	if (err) {
		ASUSDEC_ERR("lid_indev registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->lid_indev);
	ec_chip->lid_indev = NULL;
exit:
	return err;

}
//[Panda-Hall sensor]<<

struct file_operations asusdec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asusdec_ioctl,
	.open = asusdec_open,
	.write = dock_ec_write,
	.read = dock_ec_read,
	.release = asusdec_release,
};

static const struct i2c_device_id asusdec_id[] = {
	{"asuspec", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, asusdec_id);

static struct acpi_device_id mxt_acpi_match[] = {
		{ "NPCE69A", 0 },
		{ },
};
MODULE_DEVICE_TABLE(acpi, mxt_acpi_match);

static struct i2c_driver asusdec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "asuspec",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(mxt_acpi_match),
	},
	.probe	 = asusdec_probe,
	.remove	 = asusdec_remove,
	.suspend = asusdec_suspend,
	.resume = asusdec_resume,
	.id_table = asusdec_id,
};
static int is_COS()
{
	char *start;
	static char COS_CMDLINE[] = "androidboot.mode=charger";

	ASUSDEC_INFO("command line : %s\n",saved_command_line);

	start = strstr(saved_command_line,COS_CMDLINE);
	if(start != NULL){
		return 1;
	}else{
		return 0;
	}
	
	
}
static void is_ER2()
{
	int GP0P2 = 0,temp=0;

	temp = intel_mid_pmic_readb(0x2d);
	temp &= ~0x2;
	temp |= 0xc;
	intel_mid_pmic_writeb(0x2d,(u8)temp);
	GP0P2 = intel_mid_pmic_readb(0x35);
	GP0P2 &= 0x01;
	ASUSDEC_INFO("is_ER2 GP0P2 = %d %d\n",GP0P2,intel_mid_pmic_readb(0x2d));
	if(GP0P2 == 1){
		BOARD_ER2 = 1;
	}else{
		BOARD_ER2 = 0;
	}		
}

static int asusdec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ASUSDEC_INFO("asusdec probe\n");
	is_ER2();
	if(is_COS()){
		ASUSDEC_INFO("android COS\n");
		int rc = 0,pwr_gpio;
		pwr_gpio = acpi_get_gpio("\\_SB.GPO0", 55);//GPIO_S5_22
		rc = gpio_request(pwr_gpio, "DOCK_PWR_EN");
		if (rc) {
			ASUSDEC_ERR("gpio_request failed for ouput %d\n", pwr_gpio);
		}
		rc = gpio_direction_output(pwr_gpio, 0) ;
		if (rc) {
			ASUSDEC_ERR("gpio_direction_output failed for output %d\n", pwr_gpio);
		}
		ASUSDEC_INFO("pwr_gpio = %d , state = %d\n", pwr_gpio, gpio_get_value(pwr_gpio));
		return 0;
	}

	err = sysfs_create_group(&client->dev.kobj, &asusdec_smbus_group);
	if (err) {
		ASUSDEC_ERR("Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof (struct asusdec_chip), GFP_KERNEL);
	if (!ec_chip) {
		ASUSDEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->private = kzalloc(sizeof(struct elan_i2c_data), GFP_KERNEL);
	if (!ec_chip->private) {
		ASUSDEC_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, ec_chip);
#if 1
	device_init_wakeup(&client->dev, 1);
#endif
	
	ec_chip->client = client;
	ec_chip->client->driver = &asusdec_driver;
	ec_chip->client->flags = 1;
	init_timer(&ec_chip->asusdec_timer);
	asusdec_kb_init(client);
	asusdec_tp_init(client);
	asusdec_intr_init(client);
	ec_chip->asusdec_timer.function = asusdec_enter_s3_timer;
	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asusdec_wake");
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "asusdec_wake_init");
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
	mutex_init(&ec_chip->lock);
//	mutex_init(&ec_chip->irq_lock);
	spin_lock_init(&ec_chip->irq_lock); 
	mutex_init(&ec_chip->state_change_lock);
	//Shouchung add to solve the touch pad issue when dock in
	mutex_init(&ec_chip->tp_lock);
	//Shouchung end
	mutex_init(&ec_chip->dock_init_lock);
	ec_chip->indev = NULL;
	ec_chip->lid_indev = NULL;
	ec_chip->private->input = NULL;
	ec_chip->dock_status = 0;
//	ec_chip->dock_init = 0;	
	ec_chip->ec_ram_init = 0;
	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->apwake_disabled = 0;
	ec_chip->apwake_irq_req = 0;
	ec_chip->dock_type = DOCK_UNKNOWN;
//	ec_chip->kb_and_ps2_enable = 0;
	//Shouchung modify for enable touchpad when dock-in
	ec_chip->tp_enable = 0;
	//Shouchung end
	asusdec_dockram_init(client);
	cdev_add(asusdec_cdev,asusdec_dev,1) ;

	ec_chip->apower_sdev.name = APOWER_SDEV_NAME;
	ec_chip->apower_sdev.print_name = apower_switch_name;
	ec_chip->apower_sdev.print_state = apower_switch_state;
	ec_chip->apower_state = 0;
	if(switch_dev_register(&ec_chip->apower_sdev) < 0){
		ASUSDEC_ERR("switch_dev_register for apower failed!\n");
	}
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = asusdec_switch_name;
	ec_chip->dock_sdev.print_state = asusdec_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		ASUSDEC_ERR("switch_dev_register for dock failed!\n");
		goto exit;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);
	asusdec_lid_input_device_create(ec_chip->client);
	asusdec_wq = create_singlethread_workqueue("asusdec_wq");
	INIT_DELAYED_WORK(&ec_chip->asusdec_chip_init_work, asusdec_chip_init_work_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_dock_init_work, asusdec_dock_init_work_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_touchpad_init_work, asusdec_touchpad_init_work_function);
//jjt hall sensor  	INIT_DELAYED_WORK(&ec_chip->asusdec_hall_sensor_work, asusdec_lid_report_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_work, asusdec_work_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_fw_update_work, asusdec_fw_update_work_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_enter_s3_work, asusdec_enter_s3_work_function);
	INIT_DELAYED_WORK(&asusdec_stress_work, asusdec_stresstest_work_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_kb_report_work, asusdec_kb_report_work_function);
	INIT_DELAYED_WORK(&ec_chip->asusdec_tp_report_work, asusdec_tp_report_work_function);

 	asusdec_irq_dock_in(client);//dock in irq request

	asusdec_irq_ec_request(client);//why ???
	asusdec_irq_ec_apwake(client);
//jjt hall sensor  	asusdec_irq_hall_sensor(client);

//	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_init_work, 0);
	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 5*HZ);
//jjt hall sensor  	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_hall_sensor_work, 0);

	return 0;

exit:
	return err;
}

static int asusdec_remove(struct i2c_client *client)
{
	int rc = 0;
	struct asusdec_chip *chip = i2c_get_clientdata(client);

	rc = gpio_direction_output(asusdec_pwr_en_gpio, 0) ;
	if (rc) {
		ASUSDEC_ERR("gpio_direction_output failed for output %d\n", asusdec_pwr_en_gpio);
	}
	ASUSDEC_INFO("pwr_gpio = %d , state = %d\n", asusdec_pwr_en_gpio, gpio_get_value(asusdec_pwr_en_gpio));

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static int asusdec_suspend(struct i2c_client *client, pm_message_t mesg){
	int ret_val;
	printk("asusdec_suspend+\n");
	if (ec_chip->dock_in && (ec_chip->ec_in_s3 == 0)){
		ret_val = asusdec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		//jj SUSB-Down = 1, Device-on = 0  
		asusdec_dockram_read_data(0x0A);
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[1] = 0;   //Write-1-Clear Flag set i2c_dm_data[1] BIT5 to 1 i2c_dm_data[5] BIT5 will clear to 0.
		ec_chip->i2c_dm_data[2] = 0x20;
		ec_chip->i2c_dm_data[3] = 0;
		ec_chip->i2c_dm_data[4] = 0;
		ec_chip->i2c_dm_data[5] = 0x22;   //Write-1-Set Flag set i2c_dm_data[5] BIT1 and BIT5 to 1
		ec_chip->i2c_dm_data[6] = 0;
		ec_chip->i2c_dm_data[7] = 0;
		ec_chip->i2c_dm_data[8] = 0;
		asusdec_dockram_write_data(0x0A,9);
/*
		//jj set HOST in s3 to force EC into power saving mode!
		asusdec_dockram_read_data(0x0A);
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[1] = 0;   //Write-1-Clear Flag set i2c_dm_data[1] BIT5 to 1 i2c_dm_data[5] BIT5 will clear to 0.
		ec_chip->i2c_dm_data[2] = 0;
		ec_chip->i2c_dm_data[3] = 0;
		ec_chip->i2c_dm_data[4] = 0;
		ec_chip->i2c_dm_data[5] = 0x02;   //Write-1-Set Flag set i2c_dm_data[5] BIT1 and BIT5 to 1
		ec_chip->i2c_dm_data[6] = 0;
		ec_chip->i2c_dm_data[7] = 0;
		ec_chip->i2c_dm_data[8] = 0;
		asusdec_dockram_write_data(0x0A,9);	*/	
	}
	
	if (device_may_wakeup(&client->dev)){
		printk("jjt dec suspend");
		ec_irq_enable();
		enable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
	}
	insert_wakeup = 0;
fail_to_access_ec:
	flush_workqueue(asusdec_wq);
	ec_chip->suspend_state = 1;
//	ec_chip->dock_det = 0;
//	ec_chip->dock_init = 0;
	ec_chip->init_success = 0;
	ec_chip->touchpad_member = -1;
	//[Brook- Docking charging porting]<<
	ec_chip->ec_in_s3 = 1;
	printk("asusdec_suspend-\n");
	return 0;
}

static int asusdec_resume(struct i2c_client *client){
	printk("asusdec_resume+\n");
	/*
	if (ec_chip->dock_in && gpio_get_value(asusdec_apwake_gpio)) {
		ec_chip->dock_type = DOCK_UNKNOWN;
		asusdec_reset_dock();
	}*/
	if (device_may_wakeup(&client->dev)){
		printk("jjt dec resume");
		ec_irq_enable();
		disable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
	}
	ec_chip->suspend_state = 0;
//	ec_chip->dock_det = 0;
//	ec_chip->init_success = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->tp_enable = 0;
//	ec_chip->touchpad_member = -1;
	//asusdec_lid_report_function(NULL);
	if(!insert_wakeup)
		queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
	else insert_wakeup = 0;
	if(ec_chip->dock_in){
		asusdec_additonal_porting();
	}
	ec_chip->i2c_err_count = 0;
	printk("asusdec_resume-\n");
	return 0;
}

static int __init asusdec_init(void)
{
	int err_code = 0;

	printk(KERN_INFO "%s+ #####\n", __func__);

	if (asusdec_major) {
		asusdec_dev = MKDEV(asusdec_major, asusdec_minor);
		err_code = register_chrdev_region(asusdec_dev, 1, "asuspec");
	} else {
		err_code = alloc_chrdev_region(&asusdec_dev, asusdec_minor, 1,"asuspec");
		asusdec_major = MAJOR(asusdec_dev);
	}

	ASUSDEC_NOTICE("cdev_alloc\n") ;
	asusdec_cdev = cdev_alloc() ;
	asusdec_cdev->owner = THIS_MODULE ;
	asusdec_cdev->ops = &asusdec_fops ;

	err_code=i2c_add_driver(&asusdec_driver);
	if(err_code){
		ASUSDEC_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	asusdec_class = class_create(THIS_MODULE, "asuspec");
	if(asusdec_class <= 0){
		ASUSDEC_ERR("asusdec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	asusdec_device = device_create(asusdec_class, NULL, MKDEV(asusdec_major, asusdec_minor), NULL, "asuspec" );
	if(asusdec_device <= 0){
		ASUSDEC_ERR("asusdec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	ASUSDEC_INFO("return value %d\n", err_code) ;
	printk(KERN_INFO "%s- #####\n", __func__);

	return 0;

device_create_fail :
	class_destroy(asusdec_class) ;
class_create_fail :
	i2c_del_driver(&asusdec_driver);
i2c_add_driver_fail :
	printk(KERN_INFO "%s- #####\n", __func__);
	return err_code;
}

static void __exit asusdec_exit(void)
{
	device_destroy(asusdec_class,MKDEV(asusdec_major, asusdec_minor)) ;
	class_destroy(asusdec_class) ;
	i2c_del_driver(&asusdec_driver);
	unregister_chrdev_region(asusdec_dev, 1);
	switch_dev_unregister(&ec_chip->apower_sdev);
	switch_dev_unregister(&ec_chip->dock_sdev);		
}

late_initcall(asusdec_init);
module_exit(asusdec_exit);
