/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

 
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
//clj 20131218 SFZZ-160
#include <linux/dma-mapping.h> 
#include <mach/eint.h>

 
#include "tpd_custom_ft5316.h"

//#ifdef MT6577
//#include <mach/mt6577_pm_ldo.h>
//#include <mach/mt6577_typedefs.h>
//#include <mach/mt6577_boot.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"

//zhaoshaopeng add for ft driver firmware update 20120903 start



//gesture add begin
#define FTS_GESTURE_MODE  //open test mode by yangsonglin 20131029 
// SFZZ-53 add by yangsonglin 20131106
#define KEY_GESTURE_C         67 //0x34
#define KEY_GESTURE_E         68 // 0x33
#define KEY_GESTURE_M         87//0x32
#define KEY_GESTURE_UP        88 //0x22

#define OPEN_GESTRUE         0x50
#define CLOSE_GESTRUE       0x51
#define OPEN_GLOVE             0x53
#define CLOSE_GLOVE           0x54

#define OPEN_GESTURE               1
#define CLOSE_GESTURE             0

unsigned int g_gs_status = 0;
char g_gs_value = 0x00;
static unsigned int g_open_status = 0;
//gesture add end
static unsigned int g_glove_status = 1; //add by yangsonglin 20131106
  unsigned int tpd_gesture_ctrl=0 ;   //test by yangsonglin

//charger detect add by yangsonglin20131204 //add by yangsonglin20131205 SFZZ-149
//extern kal_uint32 upmu_get_pchr_chrdet(void);
unsigned int gtp_charger_mode=0;

unsigned int open_gesture_ctrl(void);
unsigned int close_gesture_ctrl(void);
unsigned int open_glove_ctrl(void);
unsigned int close_glove_ctrl(void);
//#define FTS_CTL_IIC

//#ifdef FTS_CTL_IIC
//#include "focaltech_ctl.h"
//extern int ft_rw_iic_drv_init(struct i2c_client *client);
//extern void  ft_rw_iic_drv_exit(void);
//#endif

#define TPD_INFO(fmt, arg...)  printk("[tpd info:5x06]" "[%s]" fmt "\r\n", __FUNCTION__ ,##arg)


 //zhaoshaopeng add start
// #define  TPD_DEBUG printk
extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
 #if 0
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#endif
 
static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int  tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

#define POINT_NUM 5
//register define
struct touch_info {
    int y[POINT_NUM];
    int x[POINT_NUM];
    int p[POINT_NUM];
    s16 touchId[POINT_NUM];
 //   u16 pressure;
    int count;
};
#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5306t
#ifdef VELOCITY_CUSTOM_FT5306t
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************
#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

static int g_v_magnify_x =12;
static int g_v_magnify_y =16;

//clj 20131218 SFZZ-160

//#define FT_FM_UPDATE
//static u8 *CTPI2CDMABuf_va = NULL;
//static u32 CTPI2CDMABuf_pa = NULL;
//typedef enum
//{
//    ERR_OK,
//    ERR_MODE,
//    ERR_READID,
//    ERR_ERASE,
//    ERR_STATUS,
//    ERR_ECC,
//    ERR_DL_ERASE_FAIL,
//    ERR_DL_PROGRAM_FAIL,
//    ERR_DL_VERIFY_FAIL
//}E_UPGRADE_ERR_TYPE;

//typedef unsigned char         FTS_BYTE;     //8 bit
//typedef unsigned short        FTS_WORD;    //16 bit
//typedef unsigned int          FTS_DWRD;    //16 bit
//typedef unsigned char         FTS_BOOL;    //8 bit

//#define FTS_NULL                0x0
//#define FTS_TRUE                0x01
//#define FTS_FALSE               0x0

//#define I2C_CTPM_ADDRESS        0x70

//static int ft5x0x_i2c_rxdata(char *rxdata, int length)
//{
//	int ret;

//	struct i2c_msg msgs[] = {
//		{
//			.addr	= i2c_client->addr,
//			.flags	= 0,
//			.len	= 1,
//			.buf	= rxdata,
//		},
//		{
//			.addr	= i2c_client->addr,
//			.flags	= I2C_M_RD,
//			.len	= length,
//			.buf	= rxdata,
//		},
//	};

//    //msleep(1);
//	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
//	if (ret < 0)
//		pr_err("msg %s i2c read error: %d\n", __func__, ret);
//	
//	return ret;
//}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
//static int ft5x0x_i2c_txdata(char *txdata, int length)
//{
//	int ret;

//	struct i2c_msg msg[] = {
//		{
//			.addr	= i2c_client->addr,
//			.flags	= 0,
//			.len	= length,
//			.buf	= txdata,
//		},
//	};
//   	//msleep(1);
//	ret = i2c_transfer(i2c_client->adapter, msg, 1);
//	if (ret < 0)
//		pr_err("%s i2c write error: %d\n", __func__, ret);

//	return ret;
//}
//static int ft5x0x_write_reg(u8 addr, u8 para)
//{
//    u8 buf[3];
//    int ret = -1;

//    buf[0] = addr;
//    buf[1] = para;
//    ret = ft5x0x_i2c_txdata(buf, 2);
//    if (ret < 0) {
//        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
//        return -1;
//    }
//    
//    return 0;
//}

/***********************************************************************************************
Name	:	ft5x0x_read_reg 

Input	:	addr
                     pdata

Output	:	

function	:	read register of ft5x0x

***********************************************************************************************/
//static int ft5x0x_read_reg(u8 addr, u8 *pdata)
//{
//	int ret;
//	u8 buf[2] = {0};

//	buf[0] = addr;
//	struct i2c_msg msgs[] = {
//		{
//			.addr	= i2c_client->addr,
//			.flags	= 0,
//			.len	= 1,
//			.buf	= buf,
//		},
//		{
//			.addr	= i2c_client->addr,
//			.flags	= I2C_M_RD,
//			.len	= 1,
//			.buf	= buf,
//		},
//	};

//    //msleep(1);
//	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
//	if (ret < 0)
//		pr_err("msg %s i2c read error: %d\n", __func__, ret);

//	*pdata = buf[0];
//	return ret;
//  
//}

//int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
//		    int writelen, char *readbuf, int readlen)
//{
//	int ret;

//	if (writelen > 0) {
//		struct i2c_msg msgs[] = {
//			{
//			 .addr = client->addr,
//			 .flags = 0,
//			 .len = writelen,
//			 .buf = writebuf,
//			 },
//			{
//			 .addr = client->addr,
//			 .flags = I2C_M_RD,
//			 .len = readlen,
//			 .buf = readbuf,
//			 },
//		};
//		ret = i2c_transfer(client->adapter, msgs, 2);
//		if (ret < 0)
//			dev_err(&client->dev, "f%s: i2c read error.\n",
//				__func__);
//	} else {
//		struct i2c_msg msgs[] = {
//			{
//			 .addr = client->addr,
//			 .flags = I2C_M_RD,
//			 .len = readlen,
//			 .buf = readbuf,
//			 },
//		};
//		ret = i2c_transfer(client->adapter, msgs, 1);
//		if (ret < 0)
//			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
//	}
//	return ret;
//}
/*write data by i2c*/
//int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
//{
//	int ret;

//	struct i2c_msg msg[] = {
//		{
//		 .addr = client->addr,
//		 .flags = 0,
//		 .len = writelen,
//		 .buf = writebuf,
//		 },
//	};

//	ret = i2c_transfer(client->adapter, msg, 1);
//	if (ret < 0)
//		dev_err(&client->dev, "%s i2c write error.\n", __func__);

//	return ret;
//}


//void delay_qt_ms(unsigned long  w_ms)
//{
//    unsigned long i;
//    unsigned long j;

//    for (i = 0; i < w_ms; i++)
//    {
//        for (j = 0; j < 1000; j++)
//        {
//            udelay(1);
//        }
//    }
//}

/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
//FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
//{
//    int ret;
//    
//    ret=i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

//    if(ret<=0)
//    {
//        printk("[TSP]i2c_read_interface error\n");
//        return FTS_FALSE;
//    }
//  
//    return FTS_TRUE;
//}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
//FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
//{
//    int ret;
//    ret=i2c_master_send(i2c_client, pbt_buf, dw_lenth);
//    if(ret<=0)
//    {
//        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
//        return FTS_FALSE;
//    }

//    return FTS_TRUE;
//}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
//FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
//{
//    FTS_BYTE write_cmd[4] = {0};

//    write_cmd[0] = btcmd;
//    write_cmd[1] = btPara1;
//    write_cmd[2] = btPara2;
//    write_cmd[3] = btPara3;
//    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
//}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
//FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
//{
//    
//    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
//}

//static int CTPDMA_i2c_write(FTS_BYTE slave,FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
//{
//    
//	int i = 0;
//	for(i = 0 ; i < dw_len; i++)
//	{
//		CTPI2CDMABuf_va[i] = pbt_buf[i];
//	}

//	if(dw_len <= 8)
//	{
//		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
//		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
//		return i2c_master_send(i2c_client, pbt_buf, dw_len);
//	}
//	else
//	{
//		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
//		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
//		return i2c_master_send(i2c_client, CTPI2CDMABuf_pa, dw_len);
//	}    
//}

//static int CTPDMA_i2c_read(FTS_BYTE slave, FTS_BYTE *buf, FTS_DWRD len)
//{
//	int i = 0, err = 0;

//	if(len < 8)
//	{
//		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
//		//MSE_ERR("Sensor non-dma read timing is %x!\r\n", this_client->timing);
//		return i2c_master_recv(i2c_client, buf, len);
//	}
//	else
//	{
//		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
//		//MSE_ERR("Sensor dma read timing is %x!\r\n", this_client->timing);
//		err = i2c_master_recv(i2c_client, CTPI2CDMABuf_pa, len);
//		
//	    if(err < 0)
//	    {
//			return err;
//		}

//		for(i = 0; i < len; i++)
//		{
//			buf[i] = CTPI2CDMABuf_va[i];
//		}
//	}
//}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
//FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
//{
//    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
//}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error. 
*/


//#define FT_UPGRADE_AA	0xAA
//#define FT_UPGRADE_55 	0x55

/*upgrade config of FT6X06*/
//#define FT6X06_UPGRADE_AA_DELAY 	50
//#define FT6X06_UPGRADE_55_DELAY 	30
//#define FT6X06_UPGRADE_ID_1			0x79
//#define FT6X06_UPGRADE_ID_2			0x11//0x08
//#define FT6X06_UPGRADE_READID_DELAY 10
//#define FT6X06_UPGRADE_EARSE_DELAY	2000

//#define FTS_PACKET_LENGTH       128
//#define FTS_SETTING_BUF_LEN     128

//#define FTS_UPGRADE_LOOP	    20

//#define FTS_FACTORYMODE_VALUE	0x40
//#define FTS_WORKMODE_VALUE		0x00

//static unsigned char CTPM_FW[]=/*shengda*/
//{

// #include  "s500_app.i"
//};
/*static unsigned char CTPM_FW_DIJING[]=/*dijing*/
/*{
#include "Konka_V870_6306_0x67_Ver0x13_20130823_app.i"
};
*/
//int ft6x06_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
//{
//	unsigned char buf[2] = {0};
//	buf[0] = regaddr;
//	buf[1] = regvalue;

//	return ft6x06_i2c_Write(client, buf, sizeof(buf));
//}

//int ft6x06_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
//{
//	return ft6x06_i2c_Read(client, &regaddr, 1, regvalue, 1);
//}

//int fts_ctpm_fw_read_app(u8 *pbt_buf, u32 dw_lenth)
//{
//	u32 packet_number;
//	u32 j = 0;
//	u32 temp;
//	u32 lenght = 0;
//	u8 *pReadBuf = NULL;
//	u8 auc_i2c_write_buf[10];
//	int i_ret;

//	dw_lenth = dw_lenth - 2;

//	pReadBuf = kmalloc(dw_lenth + 1, GFP_ATOMIC);
//	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
//	auc_i2c_write_buf[0] = 0x03;
//	auc_i2c_write_buf[1] = 0x00;

//	/*Read flash*/
//	for (j = 0; j < packet_number; j++) {
//		temp = j * FTS_PACKET_LENGTH;
//		auc_i2c_write_buf[2] = (u8) (temp >> 8);
//		auc_i2c_write_buf[3] = (u8) temp;

//              CTPDMA_i2c_write(0x70,auc_i2c_write_buf, 4);
//		
//		i_ret = CTPDMA_i2c_read(0x70, pReadBuf+lenght, FTS_PACKET_LENGTH);
//		if (i_ret < 0)
//			return -EIO;
//		msleep(FTS_PACKET_LENGTH / 6 + 1);
//		lenght += FTS_PACKET_LENGTH;
//	}

//	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
//		temp = packet_number * FTS_PACKET_LENGTH;
//		auc_i2c_write_buf[2] = (u8) (temp >> 8);
//		auc_i2c_write_buf[3] = (u8) temp;
//		temp = (dw_lenth) % FTS_PACKET_LENGTH;

//		CTPDMA_i2c_write(0x70,auc_i2c_write_buf, 4);
//		i_ret = CTPDMA_i2c_read(0x70,	pReadBuf+lenght, temp);
//		if (i_ret < 0)
//			return -EIO;
//		msleep(FTS_PACKET_LENGTH / 6 + 1);
//		lenght += temp;
//	}

//	/*read the last six byte */
//	temp = 0x6ffa + j;
//	auc_i2c_write_buf[2] = (u8) (temp >> 8);
//	auc_i2c_write_buf[3] = (u8) temp;
//	temp = 6;
//	i_ret = CTPDMA_i2c_write( 0x70,auc_i2c_write_buf, 4); 
//	byte_read(pReadBuf+lenght, temp);
//	if (i_ret < 0)
//		return -EIO;
//	msleep(FTS_PACKET_LENGTH / 6 + 1);
//	lenght += temp;

//       for (j=0; j<dw_lenth-2; j++) {
//	   	printk("0x%0x",pReadBuf[j]);
//		if((j+1)%16 == 0)
//			printk("\n");
//       	}
//	/*read app from flash and compart*/
//	for (j=0; j<dw_lenth-2; j++) {
//		if(pReadBuf[j] != pbt_buf[j]) {
//			kfree(pReadBuf);
//			return -EIO;
//		}
//	}

//	kfree(pReadBuf);
//	return 0;
//}

//#define    BL_VERSION_LZ4        0
//#define    BL_VERSION_Z7        1
//#define    BL_VERSION_GZF        2

//#define IC_FT5X06	0
//#define IC_FT5606	1
//#define IC_FT5316	2
//#define IC_FT5X36	3
//#define DEVICE_IC_TYPE	IC_FT5X36

//end clj 20131217
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[256];
	void __user *data;
	unsigned int value = 0;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;
	   case OPEN_GESTRUE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			value = open_gesture_ctrl();
			if(copy_to_user(data, &value, sizeof(value)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case CLOSE_GESTRUE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			value = close_gesture_ctrl();
			if(copy_to_user(data, &value, sizeof(value)))
			{
				err = -EFAULT;
				break;
			}				 
			break;
	   case OPEN_GLOVE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			value = open_glove_ctrl();
			if(copy_to_user(data, &value, sizeof(value)))
			{
				err = -EFAULT;
				break;
			}				 
			break;
	   case CLOSE_GLOVE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			value = close_glove_ctrl();
			if(copy_to_user(data, &value, sizeof(value)))
			{
				err = -EFAULT;
				break;
			}				 
			break;			

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif




#define TPD_X_RES (480)
#define TPD_Y_RES (800)
#define TPD_WARP_Y(y) ( TPD_Y_RES - 1 - y )
#define TPD_WARP_X(x) (x)//( TPD_X_RES - 1 - x )//(x)


//zhaoshaopeng undefine this 20120612
//#define __FT5X06_HASE_KEYS__
#ifdef __FT5X06_HASE_KEYS__
#define KEY_START_Y 824
#define KEY_END_Y 861 //for xl ft 849

#define KEY1_STARTX     0
#define KEY1_ENDX         60

#define KEY2_STARTX     150
#define KEY2_ENDX         190

#define KEY3_STARTX      280
#define KEY3_ENDX          320

#define KEY4_STARTX       410
#define KEY4_ENDX           470

static int ft5x06_touch_keyarry[4] = {
    KEY_MENU,KEY_HOMEPAGE,KEY_BACK,KEY_SEARCH
};

typedef enum
{
    TOUCHKEY1,
    TOUCHKEY2,
    TOUCHKEY3,
    TOUCHKEY4,
    TOUCHKEYNONE,
}TOUCH_KEY;

static TOUCH_KEY cur_touch_key = TOUCHKEYNONE;
static bool v_key_down = FALSE;
static TOUCH_KEY last_key = TOUCHKEYNONE;
#endif
//zhaoshaopeng add end


 static const struct i2c_device_id ft5306t_tpd_id[] = {{"aeon_factory_ctp",0},{}};
// static struct i2c_board_info __initdata ft5306t_i2c_tpd={ I2C_BOARD_INFO("aeon_factory_ctp", (0x70>>1))};
static struct i2c_board_info __initdata ft5306t_i2c_tpd={ I2C_BOARD_INFO("aeon_factory_ctp", (0x7e>>1))}; 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "aeon_factory_ctp",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = (tpd_remove),
  .id_table = ft5306t_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
 };
 

static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //printk("D[%4d %4d %4d] ", x, y, p);
	 input_mt_sync(tpd->dev);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
       tpd_button(x, y, 1);  
     }
	 if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	 {
         msleep(50);
		 printk("D virtual key \n");
	 }
	 TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }
 
static  void tpd_up(int x, int y,int *count) {
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
        tpd_button(x, y, 0); 
     }   		 

 }


unsigned int open_gesture_ctrl(void)
{
	tpd_gesture_ctrl = 1;
	return tpd_gesture_ctrl;
}
unsigned int  close_gesture_ctrl(void)
{
	tpd_gesture_ctrl = 0;
	return tpd_gesture_ctrl;
}

unsigned int open_glove_ctrl(void)
{
	char data1 = 0x01;
	char data2 = 0x00;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xc0, 1, &data1);
	msleep(10);
	i2c_smbus_read_i2c_block_data(i2c_client, 0xc0, 1, &data2);
	if(data2==0x01)
		return 1;
	else
		return 0;
}
unsigned int close_glove_ctrl(void)
{
	char data1 = 0xff;
	char data2 = 0x00;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xc0, 1, &data2);
	msleep(10);
	i2c_smbus_read_i2c_block_data(i2c_client, 0xc0, 1, &data1);
	if(data1==0x00)
		return 1;
	else
		return 0;
}

void gesture_event_pro(void)
{

		//if (data[41]!=0)
		g_gs_status = 0;
		TPD_DEBUG("if(g_gs_status)->gesture g_gs_status = 0x%x;//yangsonglin20131029 \n", g_gs_status);
		TPD_DEBUG("gesture mode enter the KEY_POWER key] //yangsonglin20131105 \n");
		input_report_key(tpd->dev, KEY_POWER, 1);//KEY_POWER
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);

		TPD_DEBUG("gesture mode leaver the KEY_POWER key] //yangsonglin20131105 \n");

		switch(g_gs_value)
		{
		case 0x22:
				TPD_DEBUG("gesture mode enter the desktop] //yangsonglin20131029 \n");
				input_report_key(tpd->dev, KEY_GESTURE_UP, 1);//KEY_POWER
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_UP, 0);
				input_sync(tpd->dev);
				TPD_DEBUG("gesture mode leaver the desktop] //yangsonglin20131029 \n");
				break;
		case 0x34:
				TPD_DEBUG("gesture mode enter the C mode //yangsonglin20131029 \n");
				input_report_key(tpd->dev, KEY_GESTURE_C, 1);//GESTURE_C_MODE
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_C, 0);
				input_sync(tpd->dev);
				TPD_DEBUG("gesture mode leaver the C mode] //yangsonglin20131029 \n");
				break;
		case 0x33:
				TPD_DEBUG("gesture mode enter the E mode] //yangsonglin20131029 \n");
				input_report_key(tpd->dev, KEY_GESTURE_E, 1);//GESTURE_E_MODE
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_E, 0);
				input_sync(tpd->dev);
				TPD_DEBUG("gesture mode leaver the E mode] //yangsonglin20131029 \n");
				break;
		case 0x32:
				TPD_DEBUG("gesture mode enter the M mode] //yangsonglin20131029 \n");
				input_report_key(tpd->dev, KEY_GESTURE_M, 1);//KEY_POWER
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_M, 0);
				input_sync(tpd->dev);
				TPD_DEBUG("gesture mode leaver the M mode] //yangsonglin20131029 \n");
				break;
		default:
			TPD_DEBUG("gesture mode leaver the error mode] //yangsonglin20131029 \n");
		}
		
		g_gs_value = 0;

}

//add by yangsonglin20131205 SFZZ-149
//static void gtp_charger_check(void)
//{
//	int cur_charger_state;
//	char data = 0x00;
//    cur_charger_state = upmu_get_pchr_chrdet();

//    TPD_DEBUG("ft5336 Charger mode = %d; //yangsonglin20131204 \n", cur_charger_state);

//    if (gtp_charger_mode != cur_charger_state)
//    {
//       TPD_DEBUG("Charger state change detected~!\n");
//       // GTP_DEBUG("Charger mode = %d", cur_charger_state);
//        gtp_charger_mode = cur_charger_state;
//	if (1 == gtp_charger_mode)
//	{
//		data = 0x1;
//		i2c_smbus_write_i2c_block_data(i2c_client, 0x8b, 1, &data);
//		 TPD_DEBUG("write tp i2c 0x8b data  = %d, \n", data);
//	}
//	else
//	{
//		data = 0x0;
//		i2c_smbus_write_i2c_block_data(i2c_client, 0x8b, 1, &data);       
//		 TPD_DEBUG(" aa Charger mode = %d", data);
//	}
//    }
//	

//    return;
//}

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

	int i = 0;
	
	//char data[30] = {0};
	char data[50] = {0};

    u16 high_byte,low_byte;
	u8 report_rate =0;
	g_gs_value = 0;
	g_gs_status = 0;
	p_point_num = point_num;
	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	//zhaoshaopeng start
	//i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[24]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x20, 8, &(data[32]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[40]));
	if(g_open_status==1)
		{
	i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, 1, &g_gs_value);
	TPD_DEBUG("iic->gesture g_gs_value = 0x%x;//yangsonglin20131029 \n", g_gs_value);
		}
	
	//TPD_DEBUG("FW version=%x]\n",data[24]);
	TPD_DEBUG("FW version=%x]\n",data[40]);
	TPD_DEBUG("gesture mode =%x] //yangsonglin20131029 \n", g_gs_value);
	if(g_gs_value)
	{
	TPD_DEBUG("if(g_gs_value)->gesture g_gs_status = 0x%x;//yangsonglin20131029 \n", g_gs_status);
		g_gs_status = 1;
	}

	

        /* Device Mode[2:0] == 0 :Normal operating Mode*/
        if(data[0] & 0x70 != 0) return false; 


	 if(report_rate < 8)
	 {
	   report_rate = 0x8;
	   if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	   {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	   }
	 }

	cinfo->count = data[2] & 0x07;	point_num = cinfo->count;

    for(i = 0;i<cinfo->count ;i++)
		{
			cinfo->p[i] = data[3+6*i] >> 6; //event flag 

	       /*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;

				//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
		
			/*get the Y coordinate, 2 bytes*/
			
			high_byte = data[3+6*i+2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

               /*get touch ID*/
			cinfo->touchId[i] = (data[3+6*i+2] & 0xf0)>>4;
           //printk("\r\n zhaoshaopeng cinfo->count =%d, i=%d , cinfo->touchId[%d]=%d \r\n", cinfo->count, i, cinfo->touchId[i]);			   
			     }
	
 
    return true;

 };

//static unsigned int g_gs_status = 0;
 static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo, pinfo;
    //printk("\r\n zhaoshaopeng for k500 touch_event_handler \r\n");
	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
//	gtp_charger_check(); //add by yangsonglin20131205 SFZZ-149
						 
			 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 

		  if (tpd_touchinfo(&cinfo, &pinfo)&&(g_gs_status!=1)) 
		  {
		  TPD_DEBUG("tpd_touchinfo->gesture g_gs_status = 0x%x;//yangsonglin20131029 \n", g_gs_status);
		    //TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
		  
           if(point_num >0) {
                tpd_down(TPD_WARP_X(cinfo.x[0]), cinfo.y[0], cinfo.touchId[0]);
                if(point_num>1)
             	{         tpd_down(TPD_WARP_X(cinfo.x[1]), cinfo.y[1], cinfo.touchId[1]);
				if(point_num >2)
             	{
			   	    tpd_down(TPD_WARP_X(cinfo.x[2]), cinfo.y[2], cinfo.touchId[2]);
				    if(point_num >3)
    			           {
    			   	        tpd_down(TPD_WARP_X(cinfo.x[3]), cinfo.y[3], cinfo.touchId[3]);
					 if(point_num >4)
    			              {
    			   	            tpd_down(TPD_WARP_X(cinfo.x[4]), cinfo.y[4], cinfo.touchId[4]);
    			              }
    			           }
				}
             	}
                input_sync(tpd->dev);
				TPD_DEBUG("press --->\n");
				
            }else{
			    tpd_up(cinfo.x[0], cinfo.y[0], 0);
                //TPD_DEBUG("release --->\n"); 
                //input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }

        }
		  
	if(g_gs_status)
	{
		gesture_event_pro();
	}

 }while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
         //printk("\r\n zhaoshaopeng for k500 tpd_eint_interrupt_handler FT5306t \r\n");
	 TPD_DEBUG("TPD interrupt has been triggered\n");
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }
 //clj 20131218 SFZZ-160
// extern int tpd_firmware_version[2]; 
// static unsigned char ft5x0x_read_fw_ver(void)
//{
//	unsigned char ver;
//	ft5x0x_read_reg(0xa6, &ver);    
//    tpd_firmware_version[0] = ver;    
//	return(ver);
//}
//static unsigned char ft5x0x_read_ID_ver(void)
//{
//	unsigned char ver;
//	ft5x0x_read_reg(0xa8, &ver);
//    
//    tpd_firmware_version[1] = ver;
//    
//	return(ver);
//}

//int fts_ctpm_auto_clb(struct i2c_client *client)
//{
//	unsigned char uc_temp = 0x00;
//	unsigned char i = 0;

//	/*start auto CLB */
//	msleep(200);

//	ft5x0x_write_reg( 0, 0x40);
//	/*make sure already enter factory mode */
//	msleep(100);
//	/*write command to start calibration */
//	ft5x0x_write_reg( 2, 0x4);
//	msleep(300);
//	for (i = 0; i < 100; i++) {
//		ft5x0x_read_reg( 0, &uc_temp);
//		/*return to normal mode, calibration finish */
//		if (0x0 == ((uc_temp & 0x70) >> 4))
//			break;
//	}

//	msleep(200);
//	/*calibration OK */
//	msleep(300);
//	ft5x0x_write_reg( 0, 0x40);	/*goto factory mode for store */
//	msleep(100);	/*make sure already enter factory mode */
//	ft5x0x_write_reg( 2, 0x5);	/*store CLB result */
//	msleep(300);
//	ft5x0x_write_reg(0, 0x0);	/*return to normal mode */
//	msleep(300);

//	/*store CLB result OK */
//	
//	return 0;

//     return 0;
//	
//}


//int fts_ctpm_fw_upgrade(struct i2c_client *client,u8 *pbt_buf,u32 dw_lenth)
//{
//		u8 reg_val[2] = {0};
//		u32 i = 0;
//		u32 packet_number;
//		u32 j;
//		u32 temp;
//		u32 lenght;
//		u8 packet_buf[FTS_PACKET_LENGTH + 6];
//		u8 auc_i2c_write_buf[10];
//		u8 bt_ecc;
//		int i_ret;
//		int k ;
//		int ret;
//		u8 is_5336_new_bootloader = 0;
//		u8 is_5336_fwsize_30 = 0;
//		u32 sDelayadjust;
//		int	fw_filenth = sizeof(CTPM_FW);
//		printk("%s_%d   Enter \n",__func__,__LINE__);
//	if(CTPM_FW[fw_filenth-12] == 30)
//	{
//		is_5336_fwsize_30 = 1;
//	}
//	else 
//	{
//		is_5336_fwsize_30 = 0;
//	}
//	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
//		/*********Step 1:Reset  CTPM *****/
//		/*write 0xaa to register 0xbc */

//		if(i < 10)
//	{
//		sDelayadjust = i * 3;
//	}
//	else 
//	{
//		sDelayadjust = i * (-3);
//	}

//    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
//    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
//    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
//    msleep(3);
//    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
//    msleep(3);  
//    //mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
//    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
//    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
//    msleep(30 + sDelayadjust);

//     printk("[FTS] enter fw_updgrate\n");
//		/*********Step 2:Enter upgrade mode *****/
//		auc_i2c_write_buf[0] = FT_UPGRADE_55;
//		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
//		do {
//			k++;
//			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
//			msleep(5);
//		} while (i_ret <= 0 && k< 5);


//		/*********Step 3:check READ-ID***********************/
//		msleep(200);
//		auc_i2c_write_buf[0] = 0x90;
//		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
//			0x00;
//		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

//		if (reg_val[0] == FT6X06_UPGRADE_ID_1
//			&& reg_val[1] == FT6X06_UPGRADE_ID_2) {
//			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
//				//reg_val[0], reg_val[1]);
//			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
//				reg_val[0], reg_val[1]);
//			break;
//		} else {
//			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
//				reg_val[0], reg_val[1]);
//		}
//	}
//	if (i == FTS_UPGRADE_LOOP)
//		return -EIO;
//	auc_i2c_write_buf[0] = 0xcd;
///*********0705 mshl ********************/
//	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
//       if (reg_val[0] <= 4)
//		is_5336_new_bootloader = BL_VERSION_LZ4 ;
//	else if(reg_val[0] == 7)
//		is_5336_new_bootloader = BL_VERSION_Z7 ;	
//	else if(reg_val[0] >= 0x0f)
//		is_5336_new_bootloader = BL_VERSION_GZF ;
//	
//	pr_info("bootloader version:%d\n", reg_val[0]);

//	/*Step 4:erase app and panel paramenter area*/
//	printk("Step 4:erase app and panel paramenter area\n");
//	auc_i2c_write_buf[0] = 0x61;
//	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
//	msleep(FT6X06_UPGRADE_EARSE_DELAY);
//	/*erase panel parameter area */
//	auc_i2c_write_buf[0] = 0x63;
//	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
//	msleep(100);

//	/*********Step 5:write firmware(FW) to ctpm flash*********/
//	bt_ecc = 0;
//	printk("Step 5:write firmware(FW) to ctpm flash\n");

//	//dw_lenth = dw_lenth - 8;
//	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
//	{
//		dw_lenth = dw_lenth - 8;
//	}
//	else if(is_5336_new_bootloader == BL_VERSION_GZF) dw_lenth = dw_lenth - 14;
//	
//	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
//	packet_buf[0] = 0xbf;
//	packet_buf[1] = 0x00;
//	for (j=0;j<packet_number;j++)
//    {
//    	printk("%s  j = %d \n",__func__,j);
//        temp = j * FTS_PACKET_LENGTH;
//        packet_buf[2] = (FTS_BYTE)(temp>>8);
//        packet_buf[3] = (FTS_BYTE)temp;
//        lenght = FTS_PACKET_LENGTH;
//        packet_buf[4] = (FTS_BYTE)(lenght>>8);
//        packet_buf[5] = (FTS_BYTE)lenght;

//        for (i=0;i<FTS_PACKET_LENGTH;i++)
//        {
//            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
//            bt_ecc ^= packet_buf[6+i];
//        }
//        
//        ret=CTPDMA_i2c_write(0x70, &packet_buf[0],FTS_PACKET_LENGTH + 6);
// //             printk("[TSP] 111 ret 0x%x \n", ret);
//        msleep(20);
//        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
//        {
//     //         printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
//        }
//    }

//    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
//    {
//        temp = packet_number * FTS_PACKET_LENGTH;
//        packet_buf[2] = (FTS_BYTE)(temp>>8);
//        packet_buf[3] = (FTS_BYTE)temp;

//        temp = (dw_lenth) % FTS_PACKET_LENGTH;
//        packet_buf[4] = (FTS_BYTE)(temp>>8);
//        packet_buf[5] = (FTS_BYTE)temp;

//        for (i=0;i<temp;i++)
//        {
//            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
//            bt_ecc ^= packet_buf[6+i];
//        }
////             printk("[TSP]temp 0x%x \n", temp);
//        ret = CTPDMA_i2c_write(0x70, &packet_buf[0],temp+6);    
// //            printk("[TSP] 222 ret 0x%x \n", ret);
//        msleep(20);
//    }

//    //send the last six byte

//	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
//	{
//		for (i = 0; i<6; i++)
//		{
//			if (is_5336_new_bootloader  == BL_VERSION_Z7 && DEVICE_IC_TYPE==IC_FT5X36) 
//			{
//				temp = 0x7bfa + i;
//			}
//			else if(is_5336_new_bootloader == BL_VERSION_LZ4)
//			{
//				temp = 0x6ffa + i;
//			}
//			packet_buf[2] = (u8)(temp>>8);
//			packet_buf[3] = (u8)temp;
//			temp =1;
//			packet_buf[4] = (u8)(temp>>8);
//			packet_buf[5] = (u8)temp;
//			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
//			bt_ecc ^= packet_buf[6];
//  
//			//ft5x0x_i2c_Write(client, packet_buf, 7);
//			CTPDMA_i2c_write(0x70,&packet_buf[0],7);  
//			msleep(10);
//		}
//	}
//	else if(is_5336_new_bootloader == BL_VERSION_GZF)
//	{
//		for (i = 0; i<12; i++)
//		{
//			if (is_5336_fwsize_30 && DEVICE_IC_TYPE==IC_FT5X36) 
//			{
//				temp = 0x7ff4 + i;
//			}
//			else if (DEVICE_IC_TYPE==IC_FT5X36) 
//			{
//				temp = 0x7bf4 + i;
//			}
//			packet_buf[2] = (u8)(temp>>8);
//			packet_buf[3] = (u8)temp;
//			temp =1;
//			packet_buf[4] = (u8)(temp>>8);
//			packet_buf[5] = (u8)temp;
//			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
//			bt_ecc ^= packet_buf[6];
//  
//		//	ft5x0x_i2c_Write(client, packet_buf, 7);
//				CTPDMA_i2c_write(0x70,&packet_buf[0],7);  
//			msleep(10);

//		}
//	}



//	/*********Step 6: read out checksum***********************/
//	/*send the opration head */
//	printk("Step 6: read out checksum\n");
//	auc_i2c_write_buf[0] = 0xcc;
//	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
//	if (reg_val[0] != bt_ecc) {
//		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
//					reg_val[0],
//					bt_ecc);
//		return -EIO;
//	}

//	/*********Step 7: reset the new FW***********************/

// 	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
//    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
//    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
//    msleep(10);  
//    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
//    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
//    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

//	printk("%s_%d   Exit \n",__func__,__LINE__);
//	return 0;
//}

//static int fts_ctpm_fw_upgrade_with_i_file(void)
//{
//    FTS_BYTE*     pbt_buf = FTS_NULL;
//    int i_ret,fw_len;
//    unsigned char version=0;
//    FTS_BYTE flag;
//    FTS_DWRD i = 0;
//    //=========FW upgrade========================*/
// pbt_buf = CTPM_FW;
//  fw_len = sizeof(CTPM_FW);
//  printk( "%s:Enter the fw update %d \n", __func__,__LINE__);

//	if ((pbt_buf[fw_len - 8] ^ pbt_buf[fw_len - 6]) == 0xFF
//		&& (pbt_buf[fw_len - 7] ^ pbt_buf[fw_len - 5]) == 0xFF
//		&& (pbt_buf[fw_len - 3] ^ pbt_buf[fw_len - 4]) == 0xFF) {
//		/*FW upgrade */
//		/*call the upgrade function */
//		   i_ret =  fts_ctpm_fw_upgrade(i2c_client,pbt_buf,fw_len);
//		if (i_ret != 0)
//			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",__func__);
//#ifdef AUTO_CLB
//		else
//			fts_ctpm_auto_clb(client);	/*start auto CLB */
//#endif
//	} else {
//		printk(&i2c_client->dev, "%s:FW format error\n", __func__);
//		return -EBADFD;
//	}

//	msleep(200);  
//    ft5x0x_write_reg(0xfc,0x04);
//	msleep(4000);
//	flag=0;
//	i2c_smbus_read_i2c_block_data(i2c_client, 0xFC, 1, &flag);
//	//printk("flag=%d\n",flag);
//    return i_ret;
//}

// static int touch_firmware_upgrade_thread(void *unused)
//{
//    u8 reg_version = 0, reg_value, lib_version = 0;
//    int err = 0, reg_val[2], fw_len;
//    
//    msleep(200);
//    unsigned char*     pbt_buf = FTS_NULL;

//    pbt_buf = CTPM_FW;
//    fw_len = sizeof(CTPM_FW);

//    reg_version = ft5x0x_read_fw_ver();
//    lib_version = pbt_buf[fw_len-2];
//	
//	printk("version=%x ,pbt_buf[sizeof(CTPM_FW)-2]=%d\n",reg_version,lib_version);
//    printk("read lib file version=0x%x ,read chip register ft5206_version= 0x%x by cheehwa\n", lib_version, reg_version);

//    if((lib_version > reg_version) )
//    {
//        printk("upgrading,the verison not match 0x%2x != 0x%2x\n", lib_version, reg_version);
//        msleep(200);
//        err = fts_ctpm_fw_upgrade_with_i_file();
//        printk("Return fts_ctpm_fw_upgrade_with_i_file_ err=%d by cheehwa\n", err);

//        if(err == 0)
//        {
//            printk("[TSP] ugrade successfuly.\n");
//            msleep(300);
//            //msleep(50);
//            reg_value = ft5x0x_read_fw_ver();
//            printk("FTS_DBG from old_version 0x%2x to new_version = 0x%2x\n", reg_version, reg_value);
//        }
//        else
//        {
//            printk("[TSP]  ugrade fail err=%d, line = %d.\n",
//                   err, __LINE__);
//        }

//       	// msleep(4000);
//        msleep(500);
//    }

//	kthread_should_stop();
// printk( "%s:Exit the fw update:%d\n", __func__,__LINE__);
//}


 //end clj 20131217


//extern int factory_pin_id;

static int factory_pin_id =1;

 
 static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;
	

#if 0
	mt_set_gpio_mode(GPIO_AEON_FACTORY_PIN, GPIO_AEON_FACTORY_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_AEON_FACTORY_PIN, GPIO_DIR_IN);	  
	mt_set_gpio_pull_enable(GPIO_AEON_FACTORY_PIN, 1);
	mt_set_gpio_pull_select(GPIO_AEON_FACTORY_PIN, GPIO_PULL_UP);

	udelay(50);

	factory_pin_id = mt_get_gpio_in(GPIO_AEON_FACTORY_PIN);

	printk("jacob tpd_probe factory_pin_id=%d\n",factory_pin_id);

	if(factory_pin_id == 1)
		return -1;
#endif

reset_proc:   
	i2c_client = client;
	i2c_client->addr = (0x70>>1);

       printk("\r\n test ft_5306t tpd_i2c_probe\r\n ");
#ifdef __FT5X06_HASE_KEYS__
       int key;
       for(key=0;key<TOUCHKEYNONE;key++)
              set_bit(ft5x06_touch_keyarry[key],tpd->dev->keybit); 
#endif

	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_C);//KEY_CAMERA
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP);

	mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(50);



	hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_2800, "TP");
		   msleep(1);
//	 hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	

	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
 
	msleep(100);
 
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		hwPowerDown(MT6328_POWER_LDO_VGP1, "TP");
//		msleep(1);
//		hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
		msleep(10);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
		   return -1; 
	}


	//set report rate 80Hz
	report_rate = 0x8; 
	if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	{
	    if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	    {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	    }
		   
	}

	tpd_load_status = 1;

	#ifdef VELOCITY_CUSTOM_FT5306t
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

//zhaoshaopeng add for ft firmware update
//#ifdef  FT_FM_UPDATE
////clj 20131218 SFZZ-160
//	CTPI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &CTPI2CDMABuf_pa, GFP_KERNEL);
//    	if(!CTPI2CDMABuf_va)
//	{
//    		printk("[TSP] dma_alloc_coherent error\n");
//	}

//    printk("clj %s %s %d\n",__FILE__,__func__,__LINE__);

//    kthread_run(touch_firmware_upgrade_thread, 0, TPD_DEVICE);
//	

//#endif
//zhaoshaopeng end

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}
//	#ifdef FTS_CTL_IIC
//		if (ft_rw_iic_drv_init(client) < 0)
//			dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
//					__func__);
//	#endif


	printk("ft5306t Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   return 0;
   
 }

 static int  tpd_remove(struct i2c_client *client)
 
 {  //clj 20131218 SFZZ-160

// 	if(CTPI2CDMABuf_va)
//	{
//		dma_free_coherent(NULL, 4096, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
//		CTPI2CDMABuf_va = NULL;
//		CTPI2CDMABuf_pa = 0;
//	}
	
//     	#ifdef FTS_CTL_IIC
//	ft_rw_iic_drv_exit();
//	#endif 
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech FT5306t I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("ft5306t unable to add i2c driver.\n");
      	return -1;
    }
   //zhaoshaopeng add
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("add error touch panel driver.======ft5306t\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
//end
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

static int tpd_ioctl(unsigned int cmd)
{
	TPD_DEBUG("TPD tpd_ioctl     enter cmd=%x;//ysl20131102   \n", cmd);
	switch (cmd)
	{
		case OPEN_GESTURE:
			open_gesture_ctrl();	
			break;
			
		case CLOSE_GESTURE:
			
			close_gesture_ctrl();
			break;
		
		default:
			TPD_DEBUG("TPD tpd_ioctl     leave   \n");
				
	}
	
	return 0;

}

 static int tpd_resume(struct i2c_client *client)
{
	int retval = TPD_OK;
	char close_gs_cmd=0x00;
	char data=0xff;

	TPD_DEBUG("TPD wake up\n");

	if(factory_pin_id == 1)
		return retval;
	
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
#else

//#ifdef FTS_GESTURE_MODE
if(tpd_gesture_ctrl == 1)
{
	g_open_status = 0;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &close_gs_cmd);  //TP enter gesture mode
	i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &data);
	TPD_DEBUG("TPD leave gesture mode data = 0x%x; //20131031 \n",   data);
	TPD_DEBUG("TPD leave gesture mode \n");
//#else // FTS_GESTURE_MODE
}else
{
	msleep(100);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(3);	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);  
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(3);
} //#endif // FTS_GESTURE_MODE
	
#endif
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
//	gtp_charger_check(); //add by yangsonglin20131205 SFZZ-149

	return retval;
}
 
 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
 {
	 int retval = TPD_OK;
	 static char data = 0x3;
	 char aa=0;

	 //next add by yangsonglin20131029
	 static char open_gs = 0x1;                         //open gs
	 static char gs_lh_mode = 0x4;                   //low->high mode
	 static char gs_mecd_mode = 0x1c;           //c,e,m mode
 
	 TPD_DEBUG("TPD enter sleep\n");
//	 mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	if(factory_pin_id == 1)
		return retval;


#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");

#else // TPD_CLOSE_POWER_IN_SLEEP

//#ifdef FTS_GESTURE_MODE
if(tpd_gesture_ctrl ==1)
{
	TPD_DEBUG("TPD enter gesture mode \n");
	g_open_status =1;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &open_gs);  //TP enter gesture mode
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd1, 1, &gs_lh_mode);  //TP enter gesture mode
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd2, 1, &gs_mecd_mode);  //TP enter gesture mode
	i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &aa);
	TPD_DEBUG("TPD enter gesture mode aa = 0x%x;\n",   aa);
//#else
}else
{
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
//#endif
}

#ifdef MT6573
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#endif //end of MT6573

#endif // end of TPD_CLOSE_POWER_IN_SLEEP



//#ifdef MT6575
//    //power on, need confirm with SA
//    //hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
//    //hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");  
//#endif    
//#ifdef MT6577
//    //power on, need confirm with SA
//    //hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
//    //hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");  
//#endif    	 
	 return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "aeon_factory_ctp",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	 printk("MediaTek FT5306t touch panel driver init\n");
	   i2c_register_board_info(1, &ft5306t_i2c_tpd, 1);
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add FT5306t driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT5306t touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


