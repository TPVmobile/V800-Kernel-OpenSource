#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <cust_eint.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>

#define CUST_EINT_PSKEY_NUM 3
#define PSKEY_GPIO_ENABLE_LEVEL 1
#define GPIO_PSKEY_EINT_PIN         (GPIO3 | 0x80000000)

#define GPIO_PSKEY_EINT_PIN_M_EINT  GPIO_MODE_00


/*
if DEBUG_POWERKEY is defined,we can debug the power key
enable powerkey: "echo 1 > /sys/bus/platform/drivers/Pskey_Driver/simulate_key"
disable powerkey: "echo 0 > /sys/bus/platform/drivers/Pskey_Driver/simulate_key"
*/
#define DEBUG_POWERKEY

typedef enum
{ 
	PSKEY_STATUS_UNKNOWN = -1,
	PSKEY_STATUS_DISABLE = 0,
	PSKEY_STATUS_ENABLE,
} PSKEY_STATUS;

typedef enum
{
	VER_PR2 = 0,
	VER_PIR,
} HARDWARE_VERSION;

static void tpd_eint_pskey_handler(void);
static void enable_power_save_key(PSKEY_STATUS);
static PSKEY_STATUS get_current_pskey_status(void);

static struct workqueue_struct * psk_eint_workqueue = NULL;
static struct work_struct psk_eint_work;

static struct platform_driver pskey_driver;
static struct input_dev *kpd_power_save_dev;

static int log_enable = 1;
static bool first_read = true;

static unsigned int pol;
static int pskey_gpio_level;

static PSKEY_STATUS status = PSKEY_STATUS_DISABLE;

#define PSKEY_DEBUG(format, args...) do{ \
	if(log_enable) \
	{\
		printk(KERN_ERR format,##args);\
	}\
}while(0)



#ifdef DEBUG_POWERKEY
static ssize_t show_pskey_simulate_key(struct device_driver *ddri, char *buf)
{   
    ssize_t res;
    res = snprintf(buf, PAGE_SIZE, "%d\n", status);
	return res;
}

static DRIVER_ATTR(simulate_key,      S_IWUSR | S_IRUGO, show_pskey_simulate_key, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *pskey_attr_list[] = {
	&driver_attr_simulate_key,        
};

static int pskey_create_debug_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(pskey_attr_list)/sizeof(pskey_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, pskey_attr_list[idx])))
		{            
			PSKEY_DEBUG("driver_create_file (%s) = %d\n", pskey_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int pskey_delete_debug_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(pskey_attr_list)/sizeof(pskey_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, pskey_attr_list[idx]);
	}

	return err;
}
#endif

static void enable_power_save_key( PSKEY_STATUS status )
{
    input_report_switch(kpd_power_save_dev, SW_POWER_SAVE, status);
    input_sync(kpd_power_save_dev);
}

static PSKEY_STATUS get_current_pskey_status(void)
{
    //read GPIO level,low level enable
    int nLevel;
    mt_set_gpio_mode(GPIO_PSKEY_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_PSKEY_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_PSKEY_EINT_PIN, GPIO_PULL_DISABLE);
	
	if(first_read) {
    	nLevel = mt_get_gpio_in(GPIO_PSKEY_EINT_PIN);
		first_read = false;
	}
	else
	{
		if(pol == PSKEY_GPIO_ENABLE_LEVEL)
		{
			mdelay(30);
			nLevel = mt_get_gpio_in(GPIO_PSKEY_EINT_PIN);
		}
		else
		{
			mdelay(20);
			nLevel = mt_get_gpio_in(GPIO_PSKEY_EINT_PIN);
		}
	}

	pskey_gpio_level = nLevel;

    return (nLevel == PSKEY_GPIO_ENABLE_LEVEL) ? PSKEY_STATUS_ENABLE : PSKEY_STATUS_DISABLE;

}

static void report_current_pskey_status( int nSetupInt )
{
	status = get_current_pskey_status();

	enable_power_save_key(status);
    
    if ( 1 == nSetupInt )
    {
	    mt_set_gpio_mode(GPIO_PSKEY_EINT_PIN, GPIO_PSKEY_EINT_PIN_M_EINT);
	    mt_set_gpio_dir(GPIO_PSKEY_EINT_PIN, GPIO_DIR_IN);
	    //mt_set_gpio_pull_enable(GPIO_PSKEY_EINT_PIN, TRUE);
	    //mt_set_gpio_pull_select(GPIO_PSKEY_EINT_PIN, GPIO_PULL_UP);
        mt_set_gpio_pull_enable(GPIO_PSKEY_EINT_PIN, GPIO_PULL_DISABLE);

	    if ( pskey_gpio_level == 0 )
	    {
	        pol = PSKEY_GPIO_ENABLE_LEVEL;
	    }
	    else
		{
	        pol = 1 - PSKEY_GPIO_ENABLE_LEVEL;
	    }
        
		mt_eint_registration(CUST_EINT_PSKEY_NUM, pol ? EINTF_TRIGGER_HIGH : EINTF_TRIGGER_LOW, tpd_eint_pskey_handler, 0);
    }
}

static void tpd_eint_pskey_handler(void)
{
    PSKEY_DEBUG("tpd_eint_pskey_handler");
    queue_work(psk_eint_workqueue, &psk_eint_work);
}

void psk_eint_work_callback(struct work_struct *work)
{
	mt_eint_mask(CUST_EINT_PSKEY_NUM);
    report_current_pskey_status( 1 );
	mt_eint_unmask(CUST_EINT_PSKEY_NUM);
}
static int pskey_probe(struct platform_device *dev)	
{
	int ret = 0;

	PSKEY_DEBUG("pskey_probe\n");

    kpd_power_save_dev = input_allocate_device();
	if (!kpd_power_save_dev) 
	{
		PSKEY_DEBUG("[PowerSaveKey]kpd_power_save_dev : fail!\n");
		return -ENOMEM;
	}

	__set_bit(EV_SW, kpd_power_save_dev->evbit);
	__set_bit(SW_POWER_SAVE, kpd_power_save_dev->swbit);
	
	kpd_power_save_dev->id.bustype = BUS_HOST;
	kpd_power_save_dev->name = "SWITCH_POWERSAVE";
	if(input_register_device(kpd_power_save_dev))
	{
		PSKEY_DEBUG("[PowerSaveKey]kpd_power_save_dev register : fail!\n");
	}else
	{
		PSKEY_DEBUG("[PowerSaveKey]kpd_power_save_dev register : success!!\n");
	}

    psk_eint_workqueue = create_singlethread_workqueue("psk_eint");
    INIT_WORK(&psk_eint_work, psk_eint_work_callback);
    report_current_pskey_status(1);

#ifdef DEBUG_POWERKEY
	if((ret = pskey_create_debug_attr(&pskey_driver.driver)))
	{
		PSKEY_DEBUG("create attribute err = %d\n", ret);
		
	}
#endif

    PSKEY_DEBUG("pskey_probe done\n");
	return 0;
}


static int pskey_remove(struct platform_device *dev)	
{
	int ret = 0;

	PSKEY_DEBUG("pskey_remove\n");

    input_unregister_device(kpd_power_save_dev);

#ifdef DEBUG_POWERKEY
	if((ret = pskey_delete_debug_attr(&pskey_driver.driver)))
	{
		PSKEY_DEBUG("delete attribute err = %d\n", ret);
	}
#endif

	return 0;
}


static struct platform_driver pskey_driver = {
	.probe		= pskey_probe,	
	.remove     = pskey_remove,
	.driver     = {
	    .name       = "Pskey_Driver",
	},
};

struct platform_device pskey_device = {
	.name	  ="Pskey_Driver",
	.id		  = -1,	
};
static int __init pskey_mod_init(void)
{
	int ret = 0;

	PSKEY_DEBUG("pskey_mod_init\n");
	
	ret = platform_driver_register(&pskey_driver);
	if (ret) {
		PSKEY_DEBUG("platform_driver_register error:(%d)\n", ret);
		return ret;
	}
	else
	{
		PSKEY_DEBUG("platform_driver_register done!\n");
	}

    ret = platform_device_register(&pskey_device);
    PSKEY_DEBUG("register pskey device\n");
    if (ret != 0) 
    {
        PSKEY_DEBUG("platform_device_register pskey_device error:(%d)\n", ret);	
        return ret;
    }
    else
    {
        PSKEY_DEBUG("platform_device_register pskey_device done!\n");
    }

	PSKEY_DEBUG("pskey_mod_init done!\n");

	return 0;
}

static void  __exit pskey_mod_exit(void)
{
	PSKEY_DEBUG("pskey_mod_exit\n");

	platform_driver_unregister(&pskey_driver);
	
	PSKEY_DEBUG("pskey_mod_exit done\n");
}
/*----------------------------------------------------------------------------*/

module_init(pskey_mod_init);
module_exit(pskey_mod_exit);

