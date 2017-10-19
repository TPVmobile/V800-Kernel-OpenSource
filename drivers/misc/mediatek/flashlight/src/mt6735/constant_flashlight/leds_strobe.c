
//#define FLASH_PMIC
#define FLASH_LM3642_SKY


int Flashlight_Switch=0;//aeon add for factory mode  flashlight test
int flag1 = 1;

#ifdef FLASH_PMIC

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif
// add for pwm
#include<mach/mt_pwm.h>
#include<mach/mt_clkmgr.h>
#include<mach/mt_gpio.h>
#include<mach/mt_typedefs.h>
#include<mach/upmu_common.h>
//end


/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
//#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
//#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
//#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
//#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
//	#define PK_DBG PK_DBG_FUNC
//	#define PK_VER PK_TRC_VERBOSE
//	#define PK_ERR PK_ERROR

	#define PK_DBG printk
	#define PK_VER printk
	#define PK_ERR printk


#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif




static struct work_struct workTimeOut;

#define FLASH_GPIO_ENF GPIO_PWM_BACK_FLASH_EN_PIN
//#define FLASH_GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static int gpio_pwm_flash_15(void)
{
	struct pwm_spec_config   pwm_setting ;
	printk("func:%s Enter\n",__func__);
	mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_05);
	pwm_setting.pwm_no  = PWM4;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = true;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xfe000000;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0;
	pwm_set_spec_config(&pwm_setting);
}
static int gpio_pwm_flash_50(void)

{
	struct pwm_spec_config   pwm_setting ;
	printk("func:%s Enter\n",__func__);
	mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_05);
	pwm_setting.pwm_no  = PWM4;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = true;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0x00ffffff;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x00000000;
	pwm_set_spec_config(&pwm_setting);
}

static void gpio_flash_open(void)
{
	printk("func:%s Enter\n",__func__);
	mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
  	mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT);
	mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ONE);
}
static void gpio_flash_close(void)
{
	printk("func:%s Enter\n",__func__);
	mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
  	mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT);
	mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
}
static int FL_Enable(void)
{
	printk("func:%s,g_duty = %d\n",__func__,g_duty);
	if(g_duty > 4)//flashlight
		gpio_pwm_flash_50();
	else//torch
	{
		gpio_pwm_flash_15();
	}
	PK_DBG(" FL_Enable line=%d\n",__LINE__);
	Flashlight_Switch= 1 ;	//liuluan add for ATA
    return 0;
}



static int FL_Disable(void)
{
	printk("func:%s,Enter\n",__func__);
	gpio_flash_close();

	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	Flashlight_Switch=0;	//liuluan add for ATA
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
    return 0;
}




static int FL_Init(void)
{


    //PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n",regVal0, g_bLtVersion);
    printk("func:%s:Enter\n",__func__);

    if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}



    gpio_flash_close();
    PK_DBG(" FL_Init line=%d\n",__LINE__);
	Flashlight_Switch=0; //liuluan add for ATA
    return 0;
}


static int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


static int __init flash_pmic_init(void)
{
	printk("flash_pmic_init\n");

//	g_duty = 3;
//	FL_Enable();
//	gpio_flash_open();
	gpio_flash_close();
	return 0;
}

static void __exit flash_pmic_exit(void)
{
	printk("flash_pmic_exit\n");
}


//module_init(flash_pmic_init);
late_initcall(flash_pmic_init);
module_exit(flash_pmic_exit);

MODULE_DESCRIPTION("Flash driver for PMIC");
MODULE_AUTHOR("feng.guangyue");
MODULE_LICENSE("GPL v2");

/*************aeon add for ATA flashlight test*********/
void Flashlight_ON(void)
{
	//hrtimer_cancel( &g_timeOutTimer );
	FL_dim_duty(1);
	if(0 == strobe_Res)
	{	
		FL_Init();	
	}
	if(flag1==1)
	{
		FL_Enable();
	}
}
void Flashlight_OFF(void)
{	
	FL_Uninit();
}

EXPORT_SYMBOL(Flashlight_ON);
EXPORT_SYMBOL(Flashlight_OFF);
EXPORT_SYMBOL(Flashlight_Switch);
/**************************end**********************/

#endif


#ifdef FLASH_LM3642_SKY

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>
#include <cust_i2c.h>


/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
//#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
//#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
//#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
//#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
//#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
//	#define PK_DBG PK_DBG_FUNC
//	#define PK_VER PK_TRC_VERBOSE
//	#define PK_ERR PK_ERROR

	#define PK_DBG printk
	#define PK_VER printk
	#define PK_ERR printk


#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static int lm3642_sky_flag=1;  // 0--lm3642  1--sky
static int flash_first_in=0;


static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

#define STROBE_DEVICE_ID_LM3642 0xC6
#define STROBE_DEVICE_ID_SKY 0x6e
//#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO_FLASH_EN_PIN
//#define FLASH_GPIO_ENT GPIO_FLASH_STROBE_PIN

static int g_bLtVersion=0;

static struct mutex lock;
static struct i2c_client *SKY81294_i2c_client = NULL;


/*****************************************************************************
Functions
*****************************************************************************/
//extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

static int LM3642_write_reg(u8 reg, u8 val)
{
	int ret=0;
	u8 buf[2];
	int retry = 3;
	
	buf[0]=reg;
	buf[1]=val;
	
	mutex_lock(&lock);
//	ret =  i2c_smbus_write_byte_data(LM3642_i2c_client, reg, val);
	SKY81294_i2c_client->addr = (STROBE_DEVICE_ID_LM3642>> 1);
	SKY81294_i2c_client->ext_flag = (SKY81294_i2c_client->ext_flag)&(~I2C_DMA_FLAG);
	mutex_unlock(&lock);

	do {
		ret = i2c_master_send(SKY81294_i2c_client, buf, 2);
		if (ret < 0)
			printk("failed writting at 0x%02x\n", reg);
		else
			break;
		
    	udelay(50);
		
    } while ((retry--) > 0);

	
	
	return ret;
}

static int LM3642_read_reg(u8 reg)
{
	int val=0,ret;
	u8 tx_buf[1],rx_buf[1];
	
	tx_buf[0]= reg;
	
	mutex_lock(&lock);
	SKY81294_i2c_client->addr = (STROBE_DEVICE_ID_LM3642>> 1);
	SKY81294_i2c_client->ext_flag = (SKY81294_i2c_client->ext_flag)&(~I2C_DMA_FLAG);
	mutex_unlock(&lock);

    ret = i2c_master_send(SKY81294_i2c_client, tx_buf, 1);
    if (ret != 1) {
        printk("LM3642_read_reg I2C send failed, addr = 0x%x \n", tx_buf[0]);
        return -1;
    }
    //
    ret = i2c_master_recv(SKY81294_i2c_client, (char *)rx_buf, 1);
    if (ret != 1) {
        printk("LM3642_read_reg I2C read failed!! \n");
        return -1;
    }	

	val = (int)rx_buf[0];
	
//	return val;
	return 1;
}


static int SKY81294_write_reg(u8 reg, u8 val)
{
	int ret=0;
	u8 buf[2];
	int retry = 3;
	
	buf[0]=reg;
	buf[1]=val;
	
	mutex_lock(&lock);
//	ret =  i2c_smbus_write_byte_data(SKY81294_i2c_client, reg, val);
	SKY81294_i2c_client->addr = (STROBE_DEVICE_ID_SKY>> 1);
	SKY81294_i2c_client->ext_flag = (SKY81294_i2c_client->ext_flag)&(~I2C_DMA_FLAG);
	mutex_unlock(&lock);

	do {
		ret = i2c_master_send(SKY81294_i2c_client, buf, 2);
		if (ret < 0)
			printk("failed writting at 0x%02x\n", reg);
		else
			break;
		
    	udelay(50);
		
    } while ((retry--) > 0);

	
	
	return ret;
}

static int SKY81294_read_reg(u8 reg)
{
	int val=0,ret;
	u8 tx_buf[1],rx_buf[1];
	
	tx_buf[0]= reg;
	
	mutex_lock(&lock);
	SKY81294_i2c_client->addr = (STROBE_DEVICE_ID_SKY>> 1);
	SKY81294_i2c_client->ext_flag = (SKY81294_i2c_client->ext_flag)&(~I2C_DMA_FLAG);
	mutex_unlock(&lock);

    ret = i2c_master_send(SKY81294_i2c_client, tx_buf, 1);
    if (ret != 1) {
        printk("SKY81294_read_reg I2C send failed, addr = 0x%x \n", tx_buf[0]);
        return -1;
    }
    //
    ret = i2c_master_recv(SKY81294_i2c_client, (char *)rx_buf, 1);
    if (ret != 1) {
        printk("SKY81294_read_reg I2C read failed!! \n");
        return -1;
    }	

	val = (int)rx_buf[0];
	
//	return val;
	return 1;
}



static void work_timeOutFunc(struct work_struct *data);



//int readReg(int reg)
//{
//    char buf[2];
//    char bufR[2];
//    buf[0]=reg;
//    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
//    //PK_DBG("qq reg=%d val=%d qq\n", buf[0],bufR[0]);
//    return (int)bufR[0];
//}

int FL_Enable(void)
{
	if(lm3642_sky_flag == 0)
	{
		char buf[2];
		//	char bufR[2];
		if(g_duty<0)
			g_duty=0;
		else if(g_duty>16)
			g_duty=16;
		if(g_duty<=2)
		{
			int val;
			if(g_bLtVersion==1)
			{
				if(g_duty==0)
					val=3;
				else if(g_duty==1)
					val=5;
				else //if(g_duty==2)
					val=7;
			}
			else
			{
				if(g_duty==0)
                val=3;//1
				else if(g_duty==1)
                val=4;//2
				else //if(g_duty==2)
                val=5;//3
			}
			//        buf[0]=9;
			//        buf[1]=0x40;
			//        iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			LM3642_write_reg(9,0x40);
			LM3642_read_reg(9);
			//=====fenggy add========
			//        buf[0]=10;
			//        buf[1]=0x03;
			//        iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			LM3642_write_reg(10,0x03);

			LM3642_read_reg(10);
			//=====fenggy add end=====
			//        buf[0]=10;
			//        buf[1]=0x02;
			//        iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			LM3642_write_reg(10,0x02);

			LM3642_read_reg(10);
		}
		else
		{
			int val;
			val = (g_duty-2);//1
			//	  buf[0]=9;
			//	  buf[1]=;
			//	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			LM3642_write_reg(9,val);
			LM3642_read_reg(9);

			//	  buf[0]=10;
			//	  buf[1]=0x03;
			//	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			LM3642_write_reg(10,0x03);
			LM3642_read_reg(10);
		}
		PK_DBG(" FL_Enable line=%d\n",__LINE__);

		LM3642_read_reg(0);
		LM3642_read_reg(1);
		LM3642_read_reg(6);
		LM3642_read_reg(8);
		LM3642_read_reg(9);
		LM3642_read_reg(0xa);
		LM3642_read_reg(0xb);

		return 0;
	}
	else
	{
		char buf[2];
		//	char bufR[2];
		if(g_duty<0)
			g_duty=0;
		else if(g_duty>20)
			g_duty=20;
		if(g_duty<=2)
		{
			int val;

			if(g_duty==0)
				val=6;
			else if(g_duty==1)
				val=8;
			else //if(g_duty==2)
				val=9;


			SKY81294_write_reg(2,val);
			//		SKY81294_read_reg(2);


			SKY81294_write_reg(3,0x09);
			//		SKY81294_read_reg(10);
		}
		else
		{
			int val;
			val = (g_duty-1);
			//	  buf[0]=9;
			//	  buf[1]=;
			//	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			SKY81294_write_reg(0,val);
			////	  SKY81294_read_reg(0);

			//	  buf[0]=10;
			//	  buf[1]=0x03;
			//	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
			SKY81294_write_reg(3,0x0a);
			//	  SKY81294_read_reg(10);
		}
		PK_DBG(" FL_Enable line=%d\n",__LINE__);

		//    SKY81294_read_reg(0);
		//	SKY81294_read_reg(1);
		//	SKY81294_read_reg(6);
		//	SKY81294_read_reg(8);
		//	SKY81294_read_reg(0);
		//	SKY81294_read_reg(0xa);
		//	SKY81294_read_reg(0xb);

		return 0;
	}
	Flashlight_Switch=1;	//liuluan add for ATA
}


int FL_Disable(void)
{
	if(lm3642_sky_flag == 0)
	{
		char buf[2];
	
	///////////////////////
	//	buf[0]=10;
	//	buf[1]=0x00;
	//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3642_write_reg(10,0x00);
		LM3642_read_reg(10);
		PK_DBG(" FL_Disable line=%d\n",__LINE__);
		return 0;
	}
	else
	{
		char buf[2];

	///////////////////////
	//	buf[0]=10;
	//	buf[1]=0x00;
	//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		SKY81294_write_reg(3,0x08);
	//	SKY81294_read_reg(10);
		PK_DBG(" FL_Disable line=%d\n",__LINE__);
	    return 0;
	}
	Flashlight_Switch=0;	//liuluan add for ATA
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
    return 0;
}




int FL_Init(void)
{
	if(lm3642_sky_flag == 0)
	{
		//int regVal0;
		char buf[2];

		//  buf[0]=0xa;
		//	buf[1]=0x0;
		//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3642_write_reg(0xa,0x0);
		LM3642_read_reg(0xa);

		//	buf[0]=0x8;
		//	buf[1]=0x47;
		//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3642_write_reg(0x8,0x47);
		LM3642_read_reg(8);

		//	buf[0]=9;
		//	buf[1]=0x35;
		//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3642_write_reg(9,0x35);
		LM3642_read_reg(9);




		//regVal0 = readReg(0);

		//if(regVal0==1)
		//    g_bLtVersion=1;
		//else
		//    g_bLtVersion=0;
		g_bLtVersion=0;


		//PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n",regVal0, g_bLtVersion);



//		if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
//		if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
//		if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

//			if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
//		if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
//		if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
#if 0
		mt_set_gpio_mode(FLASH_GPIO_ENT, GPIO_MODE_00);
		mt_set_gpio_dir(FLASH_GPIO_ENT, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(FLASH_GPIO_ENT, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(FLASH_GPIO_ENT, GPIO_PULL_DOWN);

		mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
		mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(FLASH_GPIO_ENF, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(FLASH_GPIO_ENF, GPIO_PULL_DOWN);
#endif

		PK_DBG(" FL_Init line=%d\n",__LINE__);
		return 0;
	}		
	else
	{
	   //int regVal0;
	  char buf[2];

	//  buf[0]=0xa;
	//	buf[1]=0x0;
	//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		SKY81294_write_reg(0x3,0x8);
	//	SKY81294_read_reg(0xa);
		
	//	buf[0]=0x8;
	//	buf[1]=0x47;
	//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
	//	SKY81294_write_reg(0x8,0x47);
	//	SKY81294_read_reg(8);
		
	//	buf[0]=9;
	//	buf[1]=0x35;
	//	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
	//	SKY81294_write_reg(0,0x35);
	//	SKY81294_read_reg(0);




		//regVal0 = readReg(0);

		//if(regVal0==1)
		//    g_bLtVersion=1;
		//else
		//    g_bLtVersion=0;
		g_bLtVersion=0;


	    //PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n",regVal0, g_bLtVersion);



//		if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
//	    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
//	    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

//	    if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
//	    if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
//	    if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
#if 0
		mt_set_gpio_mode(FLASH_GPIO_ENT, GPIO_MODE_00);
		mt_set_gpio_dir(FLASH_GPIO_ENT, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(FLASH_GPIO_ENT, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(FLASH_GPIO_ENT, GPIO_PULL_DOWN);

		mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
		mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(FLASH_GPIO_ENF, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(FLASH_GPIO_ENF, GPIO_PULL_DOWN);
#endif


	    PK_DBG(" FL_Init line=%d\n",__LINE__);
	    return 0;
	}
	Flashlight_Switch=0;	//liuluan add for ATA
}

int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
    static int init_flag;
	if (init_flag==0){
	         init_flag=1;
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;
  }
}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}



//EXPORT_SYMBOL(zsdebug);

static int constant_flashlight_open(void *pArg)
{
	int tmp=-1,i;
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if(flash_first_in == 0)
	{
		flash_first_in = 1;
		
		for (i=0;i<3;i++)
		{
			tmp = SKY81294_read_reg(0x6);
			if(tmp > 0)
			{
				lm3642_sky_flag = 1;
				break;
			}
			else
			{
				tmp = LM3642_read_reg(0x6);
				if(tmp > 0)
				{
					lm3642_sky_flag = 0;
					break;
				}
			}
		}
	}

	
	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


static int SKY81294_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
//	int tmp = -1;
//	struct SKY81294_chip_data *chip;
//	struct SKY81294_platform_data *pdata = client->dev.platform_data;

//	int err = -1;

//	PK_DBG("SKY81294_probe start--->.\n");


//	SKY81294_i2c_client = client;
//	mutex_init(&chip->lock);
//	i2c_set_clientdata(client, chip);

//	if(pdata == NULL){ //values are set to Zero.
//		PK_ERR("SKY81294 Platform data does not exist\n");
//		pdata = kzalloc(sizeof(struct SKY81294_platform_data),GFP_KERNEL);
//		chip->pdata  = pdata;
//		chip->no_pdata = 1;
//	}

//	chip->pdata  = pdata;
//	if(SKY81294_chip_init(chip)<0)
//		goto err_chip_init;

	SKY81294_i2c_client = client;

//	tmp = SKY81294_read_reg(0x3);
//	tmp = 1;
//	if(tmp<0)
//	{
//		lm3642_sky_flag = 0;
////		SKY81294_i2c_client->addr = (STROBE_DEVICE_ID_LM3642>> 1);
//	}
//	else
//	{
//		lm3642_sky_flag = 1;
////		SKY81294_i2c_client->addr = (STROBE_DEVICE_ID_SKY>> 1);
//	}	
	mutex_init(&lock);

//==================test=================
//	lm3642_sky_flag = 1;
//	g_duty = 1;
//	
//	FL_Init();
//	FL_Enable();
//======================================

	PK_DBG("SKY81294 Initializing is done \n");

	return 0;

//err_chip_init:
//	i2c_set_clientdata(client, NULL);
//	kfree(chip);
//	PK_ERR("SKY81294 probe is failed \n");
//	return -ENODEV;
}

static int SKY81294_remove(struct i2c_client *client)
{
//	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

//    if(chip->no_pdata)
//		kfree(chip->pdata);
//	kfree(chip);
	return 0;
}


#define SKY81294_NAME "leds-SKY81294"
static const struct i2c_device_id SKY81294_id[] = {
	{SKY81294_NAME, 0},
	{}
};

static struct i2c_driver SKY81294_i2c_driver = {
	.driver = {
		.name  = SKY81294_NAME,
	},
	.probe	= SKY81294_probe,
	.remove   = SKY81294_remove,
	.id_table = SKY81294_id,
};

//struct SKY81294_platform_data SKY81294_pdata = {0, 0, 0, 0, 0};
//static struct i2c_board_info __initdata i2c_SKY81294={ I2C_BOARD_INFO(SKY81294_NAME, I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR), \
//													.platform_data = &SKY81294_pdata,};
static struct i2c_board_info __initdata i2c_SKY81294={ I2C_BOARD_INFO(SKY81294_NAME, (STROBE_DEVICE_ID_SKY>>1))};

static int __init SKY81294_init(void)
{
	printk("SKY81294_init\n");

//==============test==============
//	mt_set_gpio_mode(GPIO_SPEAKER_EN_PIN,GPIO_MODE_00);  // gpio mode
//	mt_set_gpio_pull_enable(GPIO_SPEAKER_EN_PIN,GPIO_PULL_DISABLE);
//	mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN,GPIO_DIR_OUT); // output
//	mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high 
//==============end===============


	i2c_register_board_info(2, &i2c_SKY81294, 1);
	return i2c_add_driver(&SKY81294_i2c_driver);
}

static void __exit SKY81294_exit(void)
{
	i2c_del_driver(&SKY81294_i2c_driver);
}


module_init(SKY81294_init);
//late_initcall(SKY81294_init);
module_exit(SKY81294_exit);

MODULE_DESCRIPTION("Flash driver for SKY81294");
MODULE_AUTHOR("feng.guangyue");
MODULE_LICENSE("GPL v2");

/*************aeon add for factory mode flashlight test*********/
void Flashlight_ON(void)
{
	//hrtimer_cancel( &g_timeOutTimer );
	FL_dim_duty(1);
	if(0 == strobe_Res)
	{	
		FL_Init();	
	}
	if(flag1==1)
	{
		FL_Enable();
	}
}
void Flashlight_OFF(void)
{	
	FL_Uninit();
}

EXPORT_SYMBOL(Flashlight_ON);
EXPORT_SYMBOL(Flashlight_OFF);
EXPORT_SYMBOL(Flashlight_Switch);
/**************************end**********************/



#endif


