#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/input.h>
//#include <cust_eint.h>
#include<mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>

#define CUST_EINT_SWITCH_HAL_NUM 2
#define SWITCH_HAL_GPIO_ENABLE_LEVEL 1
#define GPIO_SWITCH_HAL_EINT_PIN         (GPIO2 | 0x80000000)

#define GPIO_SWITCH_HAL_EINT_PIN_M_EINT  GPIO_MODE_00
extern int i2c_addr;

static struct workqueue_struct * switch_hal_eint_workqueue = NULL;
static struct work_struct switch_hal_eint_work;
static struct input_dev *kpd_switch_hal_dev;
struct delayed_work  turn_on_backlight_work;

/*
if DEBUG_POWERKEY is defined,we can debug the power key
enable switch_hal: "echo 1 > /sys/bus/platform/drivers/Smartcover_Driver/simulate_key"
disable switch_hal: "echo 0 > /sys/bus/platform/drivers/Smartcover_Driver/simulate_key"
*/
#define DEBUG_SWITCH_HAL

typedef enum
{ 
	SWITCH_HAL_STATUS_UNKNOWN = -1,
	SWITCH_HAL_STATUS_DISABLE = 0,
	SWITCH_HAL_STATUS_ENABLE,
} SWITCH_HAL_STATUS;

typedef enum
{
	VER_PR2 = 0,
	VER_PIR,
} HARDWARE_VERSION;


static struct platform_driver switch_hal_driver;

static int log_enable = 1;
static bool first_read = true;
//int key_value = 0;
static int sf_first_time = 0;

extern void lcm_init_a(void);
extern void lcm_init_b(void);
extern void lcm_suspend_a(void);
extern void lcm_suspend_b(void);
extern void hall_switch_for_kpd(int state);
extern void tpd_switch_a(void);
extern void tpd_switch_b(void);
int lcd_b = 0;
int backlight_lock = 0;
//---- Kevin Pi add to handle fliping action soon after boot ------
//extern char* saved_command_line;
//-------------------------------------------------------
extern int mt65xx_leds_brightness_set_test(int type, int level);
extern unsigned int mt_get_bl_brightness(void);

#if 0
extern void lcm_init_a(void);
extern void lcm_init_b(void);
extern void lcm_suspend_a(void);
extern void lcm_suspend_b(void);
extern void set_gpio_cs(int value);
extern int primary_display_resume_test(void);
extern int primary_display_suspend_test(void);

extern int primary_display_resume(void);
extern int primary_display_suspend(void);

int lcd_b = 0;
extern int mt65xx_leds_brightness_set_test(int type, int level);
extern void lcm_resume2(void);
#endif
static unsigned int pol;
static int switch_hal_gpio_level;
unsigned int keyenable = 0;

#define SWITCH_HAL_DEBUG(format, args...) do{ \
	if(log_enable) \
	{\
		printk(KERN_ERR "[SWITCH_HAL]" format,##args);\
	}\
}while(0)

extern int tpd_is_suspend;
static void eint_switch_hal_handler(void);
extern int primary_display_esd_recovery_test(void);
extern int g_ro_level;
extern int g_elan_ic_init_work_finish;

static void turn_on_backlight(struct work_struct *work)
{
    SWITCH_HAL_DEBUG("turn_on_backlight\n");
    backlight_lock = 0;
    mt65xx_leds_brightness_set_test(6,g_ro_level);
}

static void turn_on_backlight_and_unlock()
{
    SWITCH_HAL_DEBUG("turn_on_backlight_and_unlock\n");
    schedule_delayed_work(&turn_on_backlight_work,msecs_to_jiffies(200));
}

static void turn_off_backlight_and_lock()
{
    SWITCH_HAL_DEBUG("turn_off_backlight_and_lock\n");
    cancel_delayed_work_sync(&turn_on_backlight_work);
    backlight_lock = 1;
    mt65xx_leds_brightness_set_test(6,0);
}

SWITCH_HAL_STATUS newStatus=0;
static void enable_switch_hal( SWITCH_HAL_STATUS status )
{
    if(backlight_lock == 1)
    {
        newStatus = status; 
        SWITCH_HAL_DEBUG("backlight_lock is 1, return\n");
        return;
    }

    newStatus = status; 
AAA:
    //if(!g_elan_ic_init_work_finish)
    {
     //   turn_off_backlight_and_lock();
    }
    //else
    {
       SWITCH_HAL_DEBUG("set backlight_lock 1\n");
       backlight_lock = 1;
       SWITCH_HAL_DEBUG("set backlight %d\n",mt_get_bl_brightness());
       mt65xx_leds_brightness_set_test(6,mt_get_bl_brightness());
    }
        

    if(status)
    {
        lcd_b = 1;

        //input_report_switch(kpd_switch_hal_dev, SW_LID, 1);
        //input_sync(kpd_switch_hal_dev);
        keyenable = 1;

        // close kpd blacklight
        hall_switch_for_kpd(status);

        printk(KERN_ERR"--enable_switch_hal--lcd_b = 1 g_ro_level=%d\n",g_ro_level);

        if(tpd_is_suspend==0)//by Colin
        {
           tpd_switch_b();
           lcm_suspend_a();
        } 
        primary_display_esd_recovery_test();

    }
    else
    {
        lcd_b = 0;

        //input_report_switch(kpd_switch_hal_dev, SW_LID, 0);
       //input_sync(kpd_switch_hal_dev);	
        keyenable = 0;

        printk(KERN_ERR"--enable_switch_hal--lcd_b = 0 g_ro_level=%d\n",g_ro_level);

        if(tpd_is_suspend==0)//by Colin
        {
           tpd_switch_a();
           lcm_suspend_b();
        } 	
        primary_display_esd_recovery_test();

    }
    //if(!g_elan_ic_init_work_finish)
    {
    //   turn_on_backlight_and_unlock();
    }
    //else
    {
       msleep(100);
       SWITCH_HAL_DEBUG("set backlight_lock 0\n");
       backlight_lock = 0;
       SWITCH_HAL_DEBUG("set backlight %d\n",g_ro_level);
       mt65xx_leds_brightness_set_test(6,g_ro_level);
    }
    
    if(newStatus != status)
    {
        status = newStatus;
        SWITCH_HAL_DEBUG("newStatus != status, goto AAA\n");
        goto AAA;
    }
    
    SWITCH_HAL_DEBUG("switch_hal process finish\n");

    if(status)
    {
        msleep(200);
        input_report_switch(kpd_switch_hal_dev, SW_LID, 1);
        input_sync(kpd_switch_hal_dev);
    }
    else
    {
        msleep(100);
        input_report_switch(kpd_switch_hal_dev, SW_LID, 0);
        input_sync(kpd_switch_hal_dev);	
    }
}


static ssize_t show_switch_hal_simulate_key(struct device_driver *ddri, const char *buf)
{
	return sprintf(buf, "%d\n", keyenable);
}

static DRIVER_ATTR(simulate_key,      S_IWUSR | S_IRUGO, show_switch_hal_simulate_key, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *switch_hal_attr_list[] = {
	&driver_attr_simulate_key,        
};

static int switch_hal_create_debug_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(switch_hal_attr_list)/sizeof(switch_hal_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, switch_hal_attr_list[idx])))
		{            
			SWITCH_HAL_DEBUG("driver_create_file (%s) = %d\n", switch_hal_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int switch_hal_delete_debug_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(switch_hal_attr_list)/sizeof(switch_hal_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, switch_hal_attr_list[idx]);
	}

	return err;
}

//---- Kevin Pi add to handle fliping action soon after boot ------
    int status_lk=9;
    int status_first=9;	
    int status_after=9;
    int status_matched=0;
	char * lcm_status_p;
//-------------------------------------------------------
static SWITCH_HAL_STATUS get_current_switch_hal_status(void)
{
    //read GPIO level,low level enable
    int nLevel;

	mt_set_gpio_mode(GPIO_SWITCH_HAL_EINT_PIN, GPIO_SWITCH_HAL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_SWITCH_HAL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_SWITCH_HAL_EINT_PIN, GPIO_PULL_DISABLE);
	
	if(first_read) {
    	nLevel = mt_get_gpio_in(GPIO_SWITCH_HAL_EINT_PIN);
		first_read = false;
		
//---- Kevin Pi add to handle fliping action soon after boot ------
		status_first = nLevel;
		//SWITCH_HAL_DEBUG("saved_command_line = %s\n",saved_command_line);

		//lcm_status_p = strstr(saved_command_line, "lcd_lk_b=");
		//if(lcm_status_p == NULL)
		//	SWITCH_HAL_DEBUG("switch_hal p==NULL \n");
		//else
		//	SWITCH_HAL_DEBUG("switch_hal p==%s\n",lcm_status_p);
		//if(strncmp(lcm_status_p,"lcd_lk_b=1", 10)==0)
		//	status_lk = 0;
		//else
		//	status_lk = 1;

		//////if(status_first!=status_lk)
		//////	status_matched = 0;
//-------------------------------------------------------
		
	}
	else
	{
		if(pol == SWITCH_HAL_GPIO_ENABLE_LEVEL)
		{
			mdelay(30);
			nLevel = mt_get_gpio_in(GPIO_SWITCH_HAL_EINT_PIN);
		}
		else
		{
			mdelay(20);
			nLevel = mt_get_gpio_in(GPIO_SWITCH_HAL_EINT_PIN);
		}
//---- Kevin Pi add to handle fliping action soon after boot ------
		status_after = nLevel;
//-------------------------------------------------------
	}

	switch_hal_gpio_level = nLevel;

    return (nLevel == SWITCH_HAL_GPIO_ENABLE_LEVEL) ? SWITCH_HAL_STATUS_DISABLE : SWITCH_HAL_STATUS_ENABLE;
}

void report_current_switch_hal_status( int nSetupInt )
{
	SWITCH_HAL_STATUS status = get_current_switch_hal_status();
	if(sf_first_time)
        {
	    enable_switch_hal(status);
            printk(KERN_ERR"---------------------------------Colin--------status = %d ----------\n", status);
	}
        else
        {
            if(status)
            {
                input_report_switch(kpd_switch_hal_dev, SW_LID, 1);
                input_sync(kpd_switch_hal_dev);
                keyenable=1;
            }
	    else
	    {
                input_report_switch(kpd_switch_hal_dev, SW_LID, 0);
                input_sync(kpd_switch_hal_dev);	
	        keyenable=0;
	    }
        }
	sf_first_time = 1;

        if ( 1 == nSetupInt )
        {
	    mt_set_gpio_mode(GPIO_SWITCH_HAL_EINT_PIN, GPIO_SWITCH_HAL_EINT_PIN_M_EINT);
	    mt_set_gpio_dir(GPIO_SWITCH_HAL_EINT_PIN, GPIO_DIR_IN);
	    mt_set_gpio_pull_enable(GPIO_SWITCH_HAL_EINT_PIN, GPIO_PULL_DISABLE);
	  
	    if ( switch_hal_gpio_level == 0 )
	    {
	        pol = SWITCH_HAL_GPIO_ENABLE_LEVEL;
	    }
	    else
		{
	        pol = 1 - SWITCH_HAL_GPIO_ENABLE_LEVEL;
	    }

		mt_eint_registration(CUST_EINT_SWITCH_HAL_NUM, pol ? EINTF_TRIGGER_HIGH : EINTF_TRIGGER_LOW, eint_switch_hal_handler, 0);

//---- Kevin Pi add to handle fliping action soon after boot ------
		if(status_matched==0)
			mt_eint_mask(CUST_EINT_SWITCH_HAL_NUM);
//-------------------------------------------------------

    }
}

//---- Kevin Pi add to handle fliping action soon after boot ------
void unmask_hal_eint(void)
{
	mt_eint_unmask(CUST_EINT_SWITCH_HAL_NUM);
}
//-------------------------------------------------------

static void eint_switch_hal_handler(void)
{
    SWITCH_HAL_DEBUG("eint_switch_hal_handler\n");
    mt_eint_mask(CUST_EINT_SWITCH_HAL_NUM);
    queue_work(switch_hal_eint_workqueue, &switch_hal_eint_work);
}

extern void send_fake_touch_up_event();

void switch_hal_eint_work_callback(struct work_struct *work)
{
    report_current_switch_hal_status( 1 );
    send_fake_touch_up_event();
    mt_eint_unmask(CUST_EINT_SWITCH_HAL_NUM);
}
static int switch_hal_probe(struct platform_device *dev)	
{
	int ret = 0;

	SWITCH_HAL_DEBUG("switch_hal_probe\n");
	
	kpd_switch_hal_dev = input_allocate_device();
	if (!kpd_switch_hal_dev) 
	{
		printk("[Smartcover]kpd_switch_hal_dev : fail!\n");
		return -ENOMEM;
	}

	__set_bit(EV_SW, kpd_switch_hal_dev->evbit);
	__set_bit(SW_LID, kpd_switch_hal_dev->swbit);
	
	kpd_switch_hal_dev->id.bustype = BUS_HOST;
	kpd_switch_hal_dev->name = "SWITCH_HAL";
	if(input_register_device(kpd_switch_hal_dev))
	{
		printk("[Smartcover]kpd_switch_hal_dev register : fail!\n");
	}else
	{
		printk("[Smartcover]kpd_switch_hal_dev register : success!!\n");
	} 
    INIT_DELAYED_WORK(&turn_on_backlight_work, turn_on_backlight);
    report_current_switch_hal_status( 1 );
    switch_hal_eint_workqueue = create_singlethread_workqueue("switch_hal_eint");
    INIT_WORK(&switch_hal_eint_work, switch_hal_eint_work_callback);
    
	if((ret = switch_hal_create_debug_attr(&switch_hal_driver.driver)))
	{
		SWITCH_HAL_DEBUG("create attribute err = %d\n", ret);		
	}

	return 0;
}


static int switch_hal_remove(struct platform_device *dev)	
{
	int ret = 0;

	SWITCH_HAL_DEBUG("switch_hal_remove\n");

	if((ret = switch_hal_delete_debug_attr(&switch_hal_driver.driver)))
	{
		SWITCH_HAL_DEBUG("delete attribute err = %d\n", ret);
	}

	input_unregister_device(kpd_switch_hal_dev);
	return 0;
}

static struct platform_driver switch_hal_driver = {
	.probe		= switch_hal_probe,	
	.remove     = switch_hal_remove,
	.driver     = {
	.name       = "switch_hal_Driver",
	},
};

struct platform_device switch_hal_device = {
	.name	  ="switch_hal_Driver",
	.id		  = -1,	
};


static int __init switch_hal_mod_init(void)
{
	int ret = 0;

	SWITCH_HAL_DEBUG("switch_hal_mod_init\n");
	
	ret = platform_driver_register(&switch_hal_driver);
	if (ret) {
		SWITCH_HAL_DEBUG("platform_driver_register error:(%d)\n", ret);
		return ret;
	}
	else
	{
		SWITCH_HAL_DEBUG("platform_driver_register done!\n");
	}

	ret = platform_device_register(&switch_hal_device);
	printk("register switch_hal device\n");

	if (ret != 0) 
	{
		printk("platform_device_switch_hal_register error:(%d)\n", ret);	
		return ret;
	}
	else
	{
		printk("platform_device_switch_hal_register done!\n");
	}

	SWITCH_HAL_DEBUG("switch_hal_mod_init done!\n");

/************by Colin*************/
        if(switch_hal_gpio_level == 1) 
        { 
            printk(KERN_ERR"----Colin--switch_hal_mod_init-- lcd_b == 0 \n");
            lcd_b = 0;
        }
        else
        {
            printk(KERN_ERR"----Colin--switch_hal_mod_init -- lcd_b == 1 \n");
            lcd_b = 1;
        } 
/************by Colin*************/


	return 0;
}

static void  __exit switch_hal_mod_exit(void)
{
	SWITCH_HAL_DEBUG("switch_hal_mod_exit\n");

	platform_driver_unregister(&switch_hal_driver);
	
	SWITCH_HAL_DEBUG("switch_hal_mod_exit done\n");
}
/*----------------------------------------------------------------------------*/

module_init(switch_hal_mod_init);
module_exit(switch_hal_mod_exit);

