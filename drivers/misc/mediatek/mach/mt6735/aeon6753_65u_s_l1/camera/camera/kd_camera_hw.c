#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/*
#ifndef BOOL
typedef unsigned char BOOL;
#endif
*/

/* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04*/
extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

static u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN, // The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {  GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };

void VCAMA_LDO_EN_fuc(int flag)
{
#if 0
    if(mt_set_gpio_mode(GPIO_VCAM_AVDD_LDO_PIN,GPIO_VCAM_AVDD_LDO_PIN_M_GPIO))
    {
        PK_DBG("[VCAMA_LDO_EN_fuc] set gpio mode failed!! \n");
    }

    if(mt_set_gpio_dir(GPIO_VCAM_AVDD_LDO_PIN,GPIO_DIR_OUT))
    {
        PK_DBG("[VCAMA_LDO_EN_fuc] set gpio dir failed!! \n");
    }

    if(flag==1)
    {
        if(mt_set_gpio_out(GPIO_VCAM_AVDD_LDO_PIN,GPIO_OUT_ONE))
        {
            PK_DBG("[VCAMA_LDO_EN_fuc ] set gpio high failed!! \n");
        }
    }
    else
    {
        if(mt_set_gpio_out(GPIO_VCAM_AVDD_LDO_PIN,GPIO_OUT_ZERO))
        {
            PK_DBG("[VCAMA_LDO_EN_fuc ] set gpio low failed!! \n");
        }
	}
#endif
}

void VCAMD_LDO_EN_fuc(int flag)
{
#if 0  // S502 S504 must set 1
    if(mt_set_gpio_mode(GPIO_VCAMD_LDO_EN_PIN,GPIO_VCAMD_LDO_EN_PIN_M_GPIO))
    {
        PK_DBG("[VCAMA_LDO_EN_fuc] set gpio mode failed!! \n");
    }

    if(mt_set_gpio_dir(GPIO_VCAMD_LDO_EN_PIN,GPIO_DIR_OUT))
    {
        PK_DBG("[VCAMA_LDO_EN_fuc] set gpio dir failed!! \n");
    }

    if(flag==1)
    {
        if(mt_set_gpio_out(GPIO_VCAMD_LDO_EN_PIN,GPIO_OUT_ONE))
        {
            PK_DBG("[VCAMA_LDO_EN_fuc ] set gpio high failed!! \n");
        }
    }
    else
    {
        if(mt_set_gpio_out(GPIO_VCAMD_LDO_EN_PIN,GPIO_OUT_ZERO))
        {
            PK_DBG("[VCAMA_LDO_EN_fuc ] set gpio low failed!! \n");
        }
	}
#endif
}

int MCLK_ON(u32 pinSetIdx)
{	    
if( pinSetIdx ==0)
		ISP_MCLK1_EN(1);
else if( pinSetIdx ==1)
		ISP_MCLK1_EN(1);
}

int MCLK_OFF(u32 pinSetIdx)
{
if( pinSetIdx ==0)
		ISP_MCLK1_EN(0);
else if( pinSetIdx ==1)
		ISP_MCLK1_EN(0);
}
int kdCISModulePowerOn_S5K5E2YA_On(u32 pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 1) return 0;
		MCLK_ON(pinSetIdx);
	printk("SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW power on \r\n");

				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					//reset pin
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
					mdelay(1);
					//PDN pin
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}

//				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(10);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(10);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)) // add lanhai af
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(10);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
				{
					 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					 //return -EIO;
					 goto _kdCISModulePowerOn_exit_;
				}
				// wait power to be stable
				mdelay(20);

				//enable active sensor
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					//PDN pin
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
					mdelay(5);
					//RST pin
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);

	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}

int kdCISModulePowerOn_S5K5E2YA_Off(u32 pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 1) return 0;
		MCLK_OFF(pinSetIdx);
	printk("SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW power off \r\n");
		
						if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
							if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
							if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
							if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
							if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
							if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
							mdelay(1);
							if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
						}
						mdelay(5);
		
		//				VCAMA_LDO_EN_fuc(0);
						if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
							PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
							//return -EIO;
							goto _kdCISModulePowerOn_exit_;
						}
						if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
						{
							PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
							//return -EIO;
							goto _kdCISModulePowerOn_exit_;
						}
						if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
							PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
							//return -EIO;
							goto _kdCISModulePowerOn_exit_;
						}
						if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name))
						{
							PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
							//return -EIO;
							goto _kdCISModulePowerOn_exit_;
						}	
            mdelay(1);
            //ISP_MCLK1_EN(0);
	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}


int kdCISModulePowerOn_OV5648_On(u32 pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 0) return 0;
			
			//ISP_MCLK1_EN(1);
			MCLK_ON(pinSetIdx);
	
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
								if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
				mdelay(1);
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
            
            //VCAM_IO
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
						//VCAM_A
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(50);
            //AF_VCC
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }
            
            mdelay(50);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);
                        //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

						mdelay(10);
						if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
								if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }


            mdelay(20);

	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}

int kdCISModulePowerOn_OV5648_Off(u32 pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 0) return 0;
		MCLK_OFF(pinSetIdx);
	
	            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }


            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //AF_VCC
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            //ISP_MCLK1_EN(0);
	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}

int kdCISModulePowerOn_GC2145_On(u32 pinSetIdx, char* mode_name)
{
	PK_DBG("kdCISModulePowerOn_GC2145_On: pinSetIdx=%d mode_name=%s\n", pinSetIdx, mode_name);
	
	//if(pinSetIdx != 1) return 0;
//		ISP_MCLK2_EN(1);
	//ISP_MCLK1_EN(1);
			MCLK_ON(pinSetIdx);

	            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

						mdelay(1);
						
            //VCAM_IO
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);
#if 0
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
#endif
            mdelay(10);
#if 0
            //AF_VCC
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }

#endif
            mdelay(10);


            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                	
                	if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }


            mdelay(20);


            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                	if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            mdelay(30);
	return 0;
_kdCISModulePowerOn_exit_:
	PK_DBG("kdCISModulePowerOn_GC2145_On: error\n");
	return -EIO;
}

int kdCISModulePowerOn_GC2145_Off(u32 pinSetIdx, char* mode_name)
{
	PK_DBG("kdCISModulePowerOn_GC2145_Off: pinSetIdx=%d mode_name=%s\n", pinSetIdx, mode_name);
	//if(pinSetIdx != 1) return 0;
		
//								ISP_MCLK2_EN(0);
			MCLK_OFF(pinSetIdx);
					
	            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
						mdelay(5);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
						mdelay(5);

#if 0
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
#endif
            //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(20);
#if 0
            //AF_VCC
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#endif
						if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
						mdelay(5);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
						mdelay(5);
						//ISP_MCLK1_EN(0);
	return 0;
_kdCISModulePowerOn_exit_:
	PK_DBG("kdCISModulePowerOn_GC2145_Off: error\n");
	return -EIO;
}

static int kdCISModulePowerOn_AT2250_On(int pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 1) return 0;
				//power ON
			//ISP_MCLK2_EN(1);
				//in case
						MCLK_ON(pinSetIdx);

				printk("SENSOR_DRVNAME_AT2250_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				udelay(10);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}

				udelay(10);	
//				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(10);		  
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			///		goto _kdCISModulePowerOn_exit_;
				//}
				
				//reset pin
				//mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
				//PDN pin

			return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;			
}

static int kdCISModulePowerOn_AT2250_Off(int pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 1) return 0;
				//power ON
	//ISP_MCLK2_EN(0);
			MCLK_OFF(pinSetIdx);

				printk("SENSOR_DRVNAME_AT2250_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
//				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				
		//		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
		//			goto _kdCISModulePowerOn_exit_;
				//}	
				mdelay(1);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
//					goto _kdCISModulePowerOn_exit_;
				}

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
//					goto _kdCISModulePowerOn_exit_;
				}  
	
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMRST]){
					if(mt_set_gpio_out(pinSet[1-pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			
			return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;			
}


static int kdCISModulePowerOn_S5K3L2_On(int pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 0) return 0;
				//power ON
			//ISP_MCLK1_EN(1);
				//in case
						MCLK_ON(pinSetIdx);

				printk("SENSOR_DRVNAME_S5K3L2XX_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(10);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(10);		  
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(10);		  
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				udelay(20);	
//				VCAMA_LDO_EN_fuc(1);
				
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			///		goto _kdCISModulePowerOn_exit_;
				//}
				
				//reset pin
				//mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
				//PDN pin

			return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;			
}

static int kdCISModulePowerOn_S5K3L2_Off(int pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 0) return 0;
				//power ON
			//ISP_MCLK1_EN(0);
					MCLK_OFF(pinSetIdx);

				printk("SENSOR_DRVNAME_S5K3L2XX_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]){
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
//				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				mdelay(1);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
//					goto _kdCISModulePowerOn_exit_;
				}

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
//					goto _kdCISModulePowerOn_exit_;
				}  
			
			return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;			
}


int kdCISModulePowerOn_OV13850_On(u32 pinSetIdx, char* mode_name)
{
	//f(pinSetIdx != 0) return 0;
				MCLK_ON(pinSetIdx);

		//�ر�ǰ��
				if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
			
			//ISP_MCLK1_EN(1);
			
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
            
            //VCAM_IO
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);
            
						//VCAM_A
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            
            mdelay(1);
            
            

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);

            //AF_VCC
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }


            mdelay(1);
						
						if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
								if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            mdelay(20);

	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}

int kdCISModulePowerOn_OV13850_Off(u32 pinSetIdx, char* mode_name)
{
	//if(pinSetIdx != 0) return 0;
				MCLK_OFF(pinSetIdx);

	            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
						
						 if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
            
             //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            
						//VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //AF_VCC
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            //ISP_MCLK1_EN(0);
	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor
u32 ret = 0;

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }
    
    PK_DBG("[PowerONOff]pinSetIdx:%d, currSensorName: %s On=%d\n", pinSetIdx, currSensorName, On);

			if (currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW))) 
                         {
				if(On)
				        return kdCISModulePowerOn_S5K5E2YA_On(pinSetIdx, mode_name);
				else
				        return kdCISModulePowerOn_S5K5E2YA_Off(pinSetIdx, mode_name);
			}
			else if (currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV13850_MIPI_RAW)))
			{
					if(On)
						return kdCISModulePowerOn_OV13850_On(pinSetIdx, mode_name);
					else
						return kdCISModulePowerOn_OV13850_Off(pinSetIdx, mode_name);
			}

			else if (currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_S5K3L2_MIPI_RAW) )) 
                        {
					if(On)
						return kdCISModulePowerOn_S5K3L2_On(pinSetIdx, mode_name);
					else
						return kdCISModulePowerOn_S5K3L2_Off(pinSetIdx, mode_name);
			}


    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//


