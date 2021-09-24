#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <linux/earlysuspend.h>
#include <linux/timer.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_vlv2.h>
#include <linux/acpi_gpio.h>
#include <linux/semaphore.h>
#define MIRQLVL1 0x0e
#define GPIO0CTLO_BASE 0x2b
#define GPIO1CTLO_BASE 0x3b
#define GPIO0CTLI_BASE 0x33
#define GPIO1CTLI_BASE 0x43
#define GPIO1P0CTLO (GPIO1CTLO_BASE+0)
#define GPIO1P0CTLI (GPIO1CTLI_BASE+0)
#define GPIO0P2CTLO (GPIO0CTLO_BASE+2)
#define GPIO0P2CTLI (GPIO0CTLI_BASE+2)
#define GPIO0P5CTLO (GPIO0CTLO_BASE+5)
#define GPIO0P5CTLI (GPIO0CTLI_BASE+5)
#define BOARD_ID_CTLO GPIO0P2CTLO
#define BOARD_ID_CTLI GPIO0P2CTLI
#define BOARD_ID_5_CTLO GPIO0P5CTLO
#define BOARD_ID_5_CTLI GPIO0P5CTLI
//bit[0]<-->hall,
#define MGPIO1P0IRQS0 0x1A
//bit[0]<-->hall,
#define MGPIO1P0IRQSX 0x1C
//bit[2]<-->id_ER,
#define MGPIO0P2IRQS0 0x19
//bit[2]<-->id_ER,
#define MGPIO0P2IRQSX 0x1b
//bit[2]<-->id_ER,
#define M_ID_ER_IRQS0 0x19
//bit[2]<-->id_ER,
#define M_ID_ER_IRQSX 0x1b
#define MASK 0
#define UNMASK 1
#define HALL_GPIO 1
#define HALL_PMIC 2
//#define HALL_DEBUG
#ifdef HALL_DEBUG
#define p_debug(format, ...) printk(format, ## __VA_ARGS__)
#else
#define p_debug(format, ...) do {} while (0)
#endif


#define DRIVER_NAME "hall_sensor"
enum {
	GPIO0P0 = 0,
		GPIO0P1,
		GPIO0P2,
		GPIO0P3,
		GPIO0P4,
		GPIO0P5,
		GPIO0P6,
		GPIO0P7,
		GPIO_HALL,
		GPIO1P1,
		GPIO1P2,
		GPIO1P3,
		GPIO1P4,
		GPIO1P5,
		GPIO1P6,
		GPIO1P7
};

static struct kobject *hall_sensor_kobj;
static struct platform_device *pdev;
/*
static struct input_device_id mID[] = {
        { .driver_info = 1 },		//scan all device to match hall sensor
        { },
};
*/
static int lid_probe(struct platform_device *pdev);
static int lid_suspend(struct platform_device *pdev, pm_message_t state) ;

static struct hall_sensor_str {
 	int irq_pmic ;
	int irq_gpio ;
	int status ;
	int gpio ;
	int enable ; 
	struct semaphore sema ;
	struct input_dev *lid_indev ;
}* hall_sensor_dev ;
static struct resource hall_resources[] = {
	{
		.name  = "HALL",
		.start = VV_PMIC_GPIO_IRQBASE+GPIO_HALL,
		.end   = VV_PMIC_GPIO_IRQBASE+GPIO_HALL,
		.flags = IORESOURCE_IRQ,
	},
};
static int board_id ;

#define ER1 0
#define ER2 2
#define ER3 3
#define PR 1
static int get_board_id()
{
	switch( ((intel_mid_pmic_readb(BOARD_ID_CTLI)&0x01)<<1)|(intel_mid_pmic_readb(BOARD_ID_5_CTLI)&0x01) )
	{
	case 0:return board_id = ER1;break;
	case 1:return board_id = PR;break;
	case 2:return board_id = ER2;break;
	case 3:return board_id = ER3;break;
	}
	return -1 ;

}
int lid_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id){
	p_debug("\n[%s]lid_connect!\n",DRIVER_NAME);
	return 0;
}
/*
//used when reporting
void lid_event(struct input_handle *handle, unsigned int type, unsigned int code, int value){
	p_debug("\n[%s]lid_event!type=%d,code=%d,value=%d\n",
		DRIVER_NAME,type,code,value);
	if(type == EV_SW && code == SW_LID)
	{
		p_debug("\n[%s]lid_event!type== ev_sw && code ==sw_lid\n",
			DRIVER_NAME);
		//if sw ==1 && status==1 OR sw==0 && status == 0
	if(!!test_bit(code, hall_sensor_dev->lid_indev->sw) != !hall_sensor_dev->status){
       __change_bit(code,  hall_sensor_dev->lid_indev->sw);
   	p_debug("[%s] reset dev->sw(!hall_sensor_dev->status)=%d \n", DRIVER_NAME,!hall_sensor_dev->status);
	}//end if
	}//end if
}
*/
/*
bool lid_match(struct input_handler *handler, struct input_dev *dev){
	p_debug("\n[%s]lid_match!\n",DRIVER_NAME);
	if(dev->name && handler->name)
		if(!strcmp(dev->name,"lid_input") && !strcmp(handler->name,"lid_input_handler"))
		        return true;
		
	return false;
}
*/
//@return DIN ,1(high) or 0(low)
int hall_get_din()
{
	int result = intel_mid_pmic_readb(GPIO1P0CTLI)&0x01 ;
	return result ;
}


void _set_pmic_hall_ioctlreg()
{
//	GPIO1P0CTLO WR
//	GPIO1P0CTLI WR
	intel_mid_pmic_writeb(BOARD_ID_CTLO,0x1c) ;
	intel_mid_pmic_writeb(GPIO1P0CTLO,0x54) ;
	intel_mid_pmic_writeb(GPIO1P0CTLI,0x0e) ;
//	intel_mid_pmic_writeb(GPIO1P0CTLI,0x0f) ;
}
void _set_irq_mask_gpio(int state)
{
	int mask =0x20 ;
 	switch(state){
	case MASK :
		intel_scu_ipc_update_register(MIRQLVL1,mask,0);
		break;
	case UNMASK :
		intel_scu_ipc_update_register(MIRQLVL1,0,mask);
		break;
 	}
}
void _set_irq_mask_hall(int state)
{
	int mask =0x01 ;
	//	MGPIO1IRQS0  WR
 	switch(state){
	case MASK :
		intel_scu_ipc_update_register(MGPIO1P0IRQS0,mask,0);
		intel_scu_ipc_update_register(MGPIO1P0IRQSX,mask,0);
		break;
	case UNMASK :
		intel_scu_ipc_update_register(MGPIO1P0IRQS0,0,mask);
		intel_scu_ipc_update_register(MGPIO1P0IRQSX,0,mask);
		break;
 	}
}

//set first level irq mask,gpio1p0ctlo ,gpio1p0ctli,2nd level irq mask
void hall_set_pmic_reg()
{
	_set_irq_mask_gpio(UNMASK) ;
	_set_pmic_hall_ioctlreg();
	_set_irq_mask_hall(UNMASK);
}

static ssize_t show_action_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	p_debug("\n[%s]show_pmic_action_status!\n",DRIVER_NAME);
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	switch(board_id)
	{
	case ER1 :
		return sprintf(buf, "%d\n", (hall_get_din()>0)?1:0);
		break ;
	case ER2 :
	case PR :
		return sprintf(buf, "%d\n", (gpio_get_value(hall_sensor_dev->gpio)>0)?1:0);
		break ;
	}
}
static ssize_t show_hall_sensor_enable(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	p_debug("\n[%s]show_hall_sensor_enable!\n",DRIVER_NAME);
	if(!hall_sensor_dev)
			 return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	p_debug("\n[%s]store_hall_sensor_enable!\n",DRIVER_NAME);
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	sscanf(buf, "%du", &request);
	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		down(&hall_sensor_dev->sema) ;
		if (hall_sensor_dev->enable==0){
			switch(board_id){	
			case ER1:enable_irq(hall_sensor_dev->irq_pmic);break;
			case ER2:case PR:
				enable_irq(hall_sensor_dev->irq_gpio);break;
			}
			hall_sensor_dev->enable=1;
			p_debug("\n[%s]store_hall_sensor_enable!hall_sensor_Dev->enable=%d\n",
				DRIVER_NAME,hall_sensor_dev->enable);
		}
		else if (hall_sensor_dev->enable==1){	
			switch(board_id){	
			case ER1:disable_irq(hall_sensor_dev->irq_pmic);break;
			case ER2:case PR:
				disable_irq(hall_sensor_dev->irq_gpio);break;
			}	
			hall_sensor_dev->enable=0;
			p_debug("\n[%s]store_hall_sensor_enable!hall_sensor_Dev->enable=%d\n",
				DRIVER_NAME,hall_sensor_dev->enable);
		}
		up(&hall_sensor_dev->sema) ;
	}
	return count;
}
static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO, 
	show_action_status, NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR|S_IWGRP,
	show_hall_sensor_enable, store_hall_sensor_enable, 0, 0);


static struct attribute *hall_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
	NULL
	//*need to NULL terminate the list of attributes 
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};
static int lid_input_device_create(void)
{
	int err = 0;
	p_debug("\n[%s]lid_input_device_create!\n",DRIVER_NAME);

	hall_sensor_dev->lid_indev = input_allocate_device();     
	if(!hall_sensor_dev->lid_indev){
		p_debug("[%s] lid_indev allocation fails\n", DRIVER_NAME);
		err = -ENOMEM;
		goto exit;
	}

	hall_sensor_dev->lid_indev->name = "hall_sensor";
	hall_sensor_dev->lid_indev->phys= "/dev/input/hall_dev";
	hall_sensor_dev->lid_indev->dev.parent= NULL;

	//mark device as capable of a certain event
//	input_set_capability(hall_sensor_dev->lid_indev, EV_KEY, KEY_POWER);
	input_set_capability(hall_sensor_dev->lid_indev, EV_SW, SW_LID);
	err = input_register_device(hall_sensor_dev->lid_indev);
	if (err) {
		p_debug("[%s] input registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_input_free;
	}
	return 0;
/*
	input_unregister_device(hall_sensor_dev->lid_indev) ;
*/
exit_input_free:
    	input_free_device(hall_sensor_dev->lid_indev);
      	hall_sensor_dev->lid_indev = NULL;
exit:
       return err;
}

static void lid_report_function(int type)
{
	p_debug("[%s] lid_report_function\n", DRIVER_NAME);
//	msleep(50);
	switch(type)
	{
	case HALL_PMIC :
		if(hall_get_din()>0)	
		{
			hall_sensor_dev->status = 1;
		}
		else{
			hall_sensor_dev->status = 0;
		}
		break ;
	case HALL_GPIO :
		if (gpio_get_value(hall_sensor_dev->gpio) > 0)
		{
			hall_sensor_dev->status = 1;
		}
		else{
			hall_sensor_dev->status = 0;
		}
		break ;

	}
	input_report_switch(hall_sensor_dev->lid_indev,
		SW_LID,!(hall_sensor_dev->status)) ;
	input_sync(hall_sensor_dev->lid_indev) ;
}
static irqreturn_t hall_pmic_interrupt_handler(int irq, void *dev_id)
{
	p_debug("[%s] hal_pmic_interrupt_handler->GPIO report value = %d\n", DRIVER_NAME, hall_get_din());

	udelay(1);

	lid_report_function(HALL_PMIC);
	return IRQ_HANDLED;
}
static irqreturn_t hall_gpio_interrupt_handler(int irq, void *dev_id)
{
	p_debug("[%s] hal_sensor_interrupt_handler-> GPIO report value = %d\n", DRIVER_NAME, gpio_get_value(hall_sensor_dev->gpio));

	udelay(1);

	lid_report_function(HALL_GPIO);
	return IRQ_HANDLED;
}
static int set_irq_hall_sensor(void)
{
	int rc = 0 ;	
	p_debug("[%s] set_irq_hall_sensor\n", DRIVER_NAME);
	switch(board_id)
	{
	case ER1:
		hall_sensor_dev->irq_pmic = platform_get_irq(pdev,0);
		p_debug("\n[%s]hall_sensor_probe!hall_sensor_dev->irq_pmic:%d\n",
			DRIVER_NAME,hall_sensor_dev->irq_pmic);	
//		disable_irq(hall_sensor_dev->irq_pmic) ;
		rc = request_threaded_irq(hall_sensor_dev->irq_pmic,
		NULL,
		hall_pmic_interrupt_handler,
		IRQF_ONESHOT,
		"hall_pmic_irq",
		hall_sensor_dev);
		if(rc){
			p_debug("[%s] Could not register for hall sensor interrupt, irq_pmic = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq_pmic,rc);
			rc = -EIO;
			goto err_pmic_request_irq_fail ;
		}
		enable_irq_wake(hall_sensor_dev->irq_pmic);
//		enable_irq(hall_sensor_dev->irq_pmic) ;
		break;
	case ER2:case PR:
		//gpio = acpi_get_gpio(\\_SB.GPO0, 5);                           
		// 若是GPIO_S5_5則為 (\\_SB.GPO2, 5)
		hall_sensor_dev->gpio = acpi_get_gpio("\\_SB.GPO2", 12); 
		p_debug("\n[%s]hall_sensor_probe!hall_sensor_dev->gpio:%d\n",
			DRIVER_NAME,hall_sensor_dev->gpio);
		if (!gpio_is_valid(hall_sensor_dev->gpio))
		{
			p_debug("[%s] GPIO for hall sensor does not exist.\n", DRIVER_NAME);
			rc= -1;
			goto fail_for_set_gpio_hall_sensor;
		}
		//para0:pin no,para1:name set for the pin
		gpio_request(hall_sensor_dev->gpio,"hall_sensor_gpio");
		//set direction
		gpio_direction_input(hall_sensor_dev->gpio);
		hall_sensor_dev->irq_gpio = gpio_to_irq(hall_sensor_dev->gpio);
		p_debug("\n[%s]hall_sensor_probe!hall_sensor_dev->irq_gpio:%d\n",
			DRIVER_NAME,hall_sensor_dev->irq_gpio);	
//		disable_irq(hall_sensor_dev->irq_gpio) ;
		rc = request_threaded_irq(hall_sensor_dev->irq_gpio,
		NULL,
		hall_gpio_interrupt_handler,
		IRQF_ONESHOT|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
		"hall_gpio_irq",
		hall_sensor_dev);
		if(rc){
			p_debug("[%s] Could not register for hall sensor interrupt, irq_gpio = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq_gpio,rc);
			rc = -EIO;
			goto err_gpio_request_irq_fail ;
		}
/*
	rc = request_irq(hall_sensor_dev->irq_gpio,
	hall_gpio_interrupt_handler,
	IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
	"hall_gpio_irq",
	hall_sensor_dev);
	if(rc<0){
		p_debug("[%s] Could not register for hall sensor interrupt, irq_gpio = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq_gpio,rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
*/
		enable_irq_wake(hall_sensor_dev->irq_gpio);
//		enable_irq(hall_sensor_dev->irq_gpio) ;
		break;
	}
	return 0 ;
/*
		free_irq(hall_sensor_dev->irq_pmic, hall_sensor_dev);	
		free_irq(hall_sensor_dev->irq_gpio, hall_sensor_dev);	
*/
err_gpio_request_irq_fail:
		gpio_free(hall_sensor_dev->gpio) ;	
fail_for_set_gpio_hall_sensor :
err_pmic_request_irq_fail:
	return rc;
}



static int lid_suspend_noirq(struct device *dev){
	p_debug("[%s] lid_suspend_noirq\n", DRIVER_NAME);
	return 0;
}
static int lid_suspend_prepare(struct device *dev){
	p_debug("[%s]lid_suspend_prepare\n", DRIVER_NAME);
      return 0;
}
static int lid_suspend_suspend(struct device *dev){
	p_debug("[%s]lid_suspend_suspend\n", DRIVER_NAME);
      return 0;
}
static void lid_resume_complete(struct device *dev){
	p_debug("[%s]lid_resume_complete\n", DRIVER_NAME);
}

static struct dev_pm_ops lid_dev_pm_ops ={
	.prepare = lid_suspend_prepare ,
	.complete = lid_resume_complete,
	.suspend = lid_suspend_suspend,
	.suspend_noirq = lid_suspend_noirq ,
};


//+++++++++++++for pm_ops callback+++++++++++++++

static const struct platform_device_id lid_id_table[] = {
        {DRIVER_NAME, 1},
};

static struct platform_driver lid_platform_driver = {
	// for non-;interrupt control descade mode'
	.driver.name    = DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.driver.pm      = &lid_dev_pm_ops,
	.probe          = lid_probe ,
	.suspend  		=lid_suspend ,
	.id_table	= lid_id_table,
};

static int lid_suspend(struct platform_device *pdev, pm_message_t state)
{
	p_debug("\n[%s]lid_suspend!\n",DRIVER_NAME);
	return 0 ;
}
static int lid_probe(struct platform_device *pdev){
	int ret =0;
	p_debug("\n[%s]lid_probe!\n",DRIVER_NAME);
	hall_set_pmic_reg();
	get_board_id() ;
	p_debug("[%s] lid_probe!get_board_id():%d \n", DRIVER_NAME,board_id);

	printk("[%s] lid_probe!get_board_id():%d \n", DRIVER_NAME,board_id);

	//set file node
	//kernel_kobj ( /sys/kernel/hall_sensor_kobject) 
	hall_sensor_kobj = kobject_create_and_add("hall_sensor_kobject", kernel_kobj);
	if (!hall_sensor_kobj){
		p_debug("[%s] hall_sensor_kobject fails for hall sensor\n", DRIVER_NAME);
		platform_device_unregister(pdev);
		ret = -ENOMEM ;
		goto fail_for_add_kobj ;
	}
	else{
		p_debug("\n[%s]hall_sensor_probe!hall_sensor_kobj create successfully\n",DRIVER_NAME);
	}
	//create file 'action_status' and 'activity' and 'delay' in /sys/kernel/hall_sensor_kobject
	ret = sysfs_create_group(hall_sensor_kobj, &hall_sensor_group);
	if (ret){
		p_debug("\n[%s]hall_sensor_probe!sysfs_create_group() failed\n",DRIVER_NAME);
		goto fail_for_hall_sensor;
	}
	else{
		p_debug("\n[%s]hall_sensor_probe!sys_create_group() succeed\n",DRIVER_NAME);
	}
	
	//Memory allocation
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		p_debug("[%s] Memory allocation fails for hall sensor\n", DRIVER_NAME);
		ret = -ENOMEM;
		goto fail_for_hall_sensor;
	}
	else{
		p_debug("\n[%s]hall_sensor_init!kzalloc succeed\n",DRIVER_NAME);
	}
//	init_MUTEX(&hall_sensor_dev->sema) ;
	sema_init(&hall_sensor_dev->sema,1) ;
	hall_sensor_dev->enable = 1;	
	//create input_dev
	hall_sensor_dev->lid_indev = NULL;
	ret = lid_input_device_create();
	if (ret < 0)	
		goto fail_for_create_input_dev;

	//set irq
	ret = set_irq_hall_sensor();
	if (ret < 0){
		p_debug("[%s]set_irq_hall_sensor->failed\n", DRIVER_NAME);
		goto fail_for_irq_hall_sensor;
	}
	else{
		p_debug("[%s]set_irq_hall_sensor->successfully.\n", DRIVER_NAME);
	}

	return 0;
/*
		free_irq(hall_sensor_dev->irq_pmic, hall_sensor_dev);	
		free_irq(hall_sensor_dev->irq_gpio, hall_sensor_dev);	
		gpio_free(hall_sensor_dev->gpio) ;
*/
	fail_for_irq_hall_sensor:
		input_unregister_device(hall_sensor_dev->lid_indev);	
		input_free_device(hall_sensor_dev->lid_indev) ;
	fail_for_create_input_dev:		
		kfree(hall_sensor_dev);
		hall_sensor_dev=NULL;
	fail_for_hall_sensor:
		kobject_put(hall_sensor_kobj);
	fail_for_add_kobj :
	return ret;
}

//----------------for pm_ops callback----------------

static int __init hall_sensor_init(void)
{	
	int ret =0 ;
	p_debug("\n[%s]hall_sensor_init!\n",DRIVER_NAME);
	//insert pm_ops
	pdev= platform_device_alloc(DRIVER_NAME,-1);
	
	if (!pdev)
		return -1;
	ret = platform_device_add_resources(pdev,
		hall_resources,ARRAY_SIZE(hall_resources));
	ret = platform_device_add(pdev);
      if (ret) 
	  	return -1 ;
	return  platform_driver_register(&lid_platform_driver);
}

static void __exit hall_sensor_exit(void)
{
	p_debug("[%s]hall_sensor_exit\n", DRIVER_NAME);

	if(hall_sensor_dev){
		if(hall_sensor_dev->irq_pmic)
			free_irq(hall_sensor_dev->irq_pmic, hall_sensor_dev);
		if(hall_sensor_dev->irq_gpio)
		{	free_irq(hall_sensor_dev->irq_gpio, hall_sensor_dev);
			gpio_free(hall_sensor_dev->gpio) ;
		}

		input_unregister_device(hall_sensor_dev->lid_indev);
		input_free_device(hall_sensor_dev->lid_indev);
		hall_sensor_dev->lid_indev=NULL;
		kfree(hall_sensor_dev);
		hall_sensor_dev=NULL;
	}
	if(hall_sensor_kobj){
		kobject_put(hall_sensor_kobj);
	}

	platform_driver_unregister(&lid_platform_driver);
	platform_device_unregister(pdev);
}
late_initcall(hall_sensor_init);
//module_init(hall_sensor_init);
module_exit(hall_sensor_exit);


MODULE_DESCRIPTION("Intel Hall sensor Driver");
MODULE_LICENSE("GPL v2");
