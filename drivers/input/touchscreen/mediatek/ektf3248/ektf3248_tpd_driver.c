//TP driver
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>



#include <mach/mt_pm_ldo.h> 
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include "cust_gpio_usage.h"

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>

//dma
#include <linux/dma-mapping.h>

#include <cust_eint.h>

/////////////////////////////////////////////////////////
#define I2C_NUM 1
#include "tpd_custom_ektf3248.h"

static struct workqueue_struct *init_elan_ic_wq = NULL;
static struct delayed_work init_work;
static unsigned long delay = 2*HZ;

/*********************************custom module********************************************/
#if defined MT6592 || defined MT6582 || defined MT6589 || defined MT6577
#define I2C_BORAD_REGISTER
#endif
#define I2C_BORAD_REGISTER

#ifndef MT6575
#define ELAN_I2C_DMA_MOD
static uint8_t *gpDMABuf_va = NULL;
static uint32_t gpDMABuf_pa = NULL;
#endif

int g_elan_ic_init_work_finish=0;
int g_Touch_Send_finish=1;
int g_Touch_Recv_finish=1; 

//#define ELAN_ESD_CHECK
#ifdef ELAN_ESD_CHECK
    static struct workqueue_struct *esd_wq = NULL;
	static struct delayed_work esd_work;
	static atomic_t have_interrupts = ATOMIC_INIT(0);
	static atomic_t elan_cmd_response = ATOMIC_INIT(0);
#endif

/*********************************platform data********************************************/
//must be init first

static const struct i2c_device_id elan_ts_id[] = {
	{ELAN_TS_NAME, 0 },
	{ }
};

#ifdef I2C_BORAD_REGISTER 
	static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(ELAN_TS_NAME, ELAN_7BITS_ADDR_B)};
#else
	static unsigned short force[] = {0, ELAN_8BITS_ADDR, I2C_CLIENT_END,I2C_CLIENT_END};
	static const unsigned short * const forces[] = { force, NULL };
	static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif
/**********************************elan struct*********************************************/

struct elan_ts_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
//queue or thread handler interrupt	
	struct task_struct *work_thread;
	struct work_struct  work;
#ifdef CONFIG_HAS_EARLYSUSPEND
//used for early_suspend
	struct early_suspend early_suspend;
#endif
	
//Firmware Information
	int fw_ver;
	int fw_id;
	int fw_bcd;
	int x_resolution;
	int y_resolution;
	int recover;//for iap mod
//for suspend or resum lock
 	int power_lock;
 	int circuit_ver;
//for button state
 	int button_state;
//For Firmare Update 
	struct miscdevice firmware;
	struct miscdevice firmware_15;
};


/************************************global elan data*************************************/
static int tpd_flag = 0;
static int boot_normal_flag = 1;
int tpd_is_suspend = 0;
static int file_fops_addr = ELAN_7BITS_ADDR;
extern int lcd_b;
int i2c_addr = 0x10;
int pre_addr;
static struct elan_ts_data *private_ts;
extern struct tpd_device *tpd;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

/*********************************global elan function*************************************/
#if defined MT6592 || defined MT6582
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eintno, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eintno, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, 
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
#else
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
 #endif
static int __hello_packet_handler(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ts_rough_calibrate(struct i2c_client *client);

bool g_fw_update_LCD_a = TRUE;
bool g_fw_update = FALSE;

#define GPIO_CTP_B_RST_PIN (GPIO64 | 0x80000000)
#define GPIO_CTP_B_RST_PIN_M_GPIO   GPIO_MODE_00  //Add By Shibo

#define GPIO_CTP_2V8_EN_PIN (GPIO81 | 0x80000000)
#define GPIO_CTP_2V8_EN_PIN_M_GPIO   GPIO_MODE_00  //Add By Shibo

#define GPIO_CTP_B_EINT_PIN (GPIO11 | 0x80000000)
#define GPIO_CTP_B_EINT_PIN_M_GPIO   GPIO_MODE_00  //Add By Shibo
#define CUST_EINT_TOUCH_PANEL_B_NUM 11

#define GPIO_SWITCH_HAL_EINT_PIN_M_EINT  GPIO_MODE_00
#define GPIO_SWITCH_HAL_EINT_PIN         (GPIO2 | 0x80000000)

/************************************** function list**************************************/
static void elan_reset(int state)
{
    if(!g_elan_ic_init_work_finish) 
    {
printk("--***colin**elan_reset*!g_elan_ic_init_work_finish \n");
        state = 1; //LCD_B
    }

    if(state == 1)
    {
        printk("elan_reset >>>>> GPIO_CTP_B_RST_PIN  \n");
        mt_set_gpio_mode(GPIO_CTP_B_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_B_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_B_RST_PIN, 1);
        msleep(10);
        mt_set_gpio_out(GPIO_CTP_B_RST_PIN, 0);
        msleep(10);
        mt_set_gpio_out(GPIO_CTP_B_RST_PIN, 1);
        msleep(10);

    } else {

        printk("elan_reset >>>>> GPIO_CTP_RST_PIN \n");
    	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
    	msleep(10);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
    	msleep(10);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
    	msleep(10);  
    }
}

static void elan_switch_irq(int on)
{
	printk("[elan] %s enter, irq switch on = %d\n", __func__, on);
	if(on){
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_B_NUM);	
	}
	else
    {
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_B_NUM);
	}
}

static int elan_i2c_send_data(struct i2c_client *client, uint8_t *buf, uint8_t len)
{
	int rc = 0;
	int i = 0;

if(g_Touch_Send_finish==1)//by colin
{
    g_Touch_Send_finish=0;

    if(lcd_b==1)
    {
        i2c_addr = 0x10;
    }
    else
    {
        i2c_addr = 0x15;
    }  

    client->addr = i2c_addr; //i2c_addr;

    if(!g_elan_ic_init_work_finish) 
    {
        client->addr = 0x10;  //LCD_B
    }

//printk("--***colin***send***--client->addr=0x%x\n",client->addr);

#ifdef ELAN_I2C_DMA_MOD	
	if(buf == NULL || gpDMABuf_va == NULL){
		printk("[elan] BUFFER is NULL!!!!!\n");
		return -1;
	}
	
	if(len > 8){
		for(i = 0 ; i < len; i++){
			gpDMABuf_va[i] = buf[i];
			printk("%02x ", buf[i]);
		}
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		rc = i2c_master_send(client, gpDMABuf_pa, len);
		client->addr = client->addr & I2C_MASK_FLAG;		
	}
	else{
		rc = i2c_master_send(client, buf, len);
	}
#else
	#if 0
		for(i = 0 ; i < len/8; i++){
			rc += i2c_master_send(client, buf+i*8, 8);
		}
		if(len%8 != 0){
			rc += i2c_master_send(client, buf+i*8, len%8);
		}
	#else
		rc = i2c_master_send(client, buf, len);
	#endif
#endif

    g_Touch_Send_finish=1;
}

	return rc;
}


static int elan_i2c_recv_data(struct i2c_client *client, uint8_t *buf, uint8_t len)
{
    int rc = 0;
    int i = 0;

if(g_Touch_Recv_finish==1)//by colin
{
    g_Touch_Recv_finish=0;
 
    if(lcd_b == 1)
    {
        i2c_addr = 0x10;
    }
    else
    {
        i2c_addr = 0x15;
    }  

    client->addr = i2c_addr; //i2c_addr;

    if(!g_elan_ic_init_work_finish) 
    {
        client->addr = 0x10;  //LCD_B
    }
 
//printk("--***colin***recv***--client->addr=0x%x\n",client->addr);

#ifdef ELAN_I2C_DMA_MOD	
	if(buf == NULL || gpDMABuf_va == NULL){
		printk("[elan] BUFFER is NULL!!!!!\n");
		return -1;
	}
	
	memset(buf, 0, len);

	if(len > 8){
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		rc = i2c_master_recv(client, gpDMABuf_pa, len);
		client->addr = client->addr & I2C_MASK_FLAG;
		if(rc >= 0){
		    for(i = 0 ; i < len; i++){
				buf[i] = gpDMABuf_va[i];
				printk("%02x ", buf[i]);
			}
		}
	}	
	else{
		rc = i2c_master_recv(client, buf, len);
	}
#else
	#if 0
		for(i = 0 ; i < len/8; i++){
			rc += i2c_master_recv(client, buf+i*8, 8);
		}
		if(len%8 != 0){
			rc += i2c_master_recv(client, buf+i*8, len%8);
		}
	#else
		rc = i2c_master_recv(client, buf, len);
	#endif
#endif

    g_Touch_Recv_finish=1;
}
	
	return rc;
}

 static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
	strcpy(info->type, ELAN_TS_NAME);	
	return 0;
}

static int elan_ts_poll(void)
{
	int status = 0, retry = 20;

	do {
	        if (lcd_b == 1)
	        {
		    status = mt_get_gpio_in(GPIO_CTP_B_EINT_PIN);
	        } 
		else 
		{
		    status = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
	        }
                    
                if(!g_elan_ic_init_work_finish) 
                {
                   status = mt_get_gpio_in(GPIO_CTP_B_EINT_PIN); //LCD_B
printk("--***colin**elan_ts_poll*!g_elan_ic_init_work_finish \n");
                }

		printk("[elan]: %s: status = %d\n", __func__, status);
		retry--;
		msleep(40);
	} while (status == 1 && retry > 0);

	printk( "[elan]%s: poll interrupt status %s\n", __func__, status == 1 ? "high" : "low");
	
	return status == 0 ? 0 : -ETIMEDOUT;
}

static int elan_ts_send_cmd(struct i2c_client *client, uint8_t *cmd, size_t size)
{
	printk("[elan] dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);
	if (elan_i2c_send_data(client, cmd, size) != size) {
		printk("[elan error]%s: elan_ts_send_cmd failed\n", __func__);
		return -EINVAL;
	}
	else{
		elan_info("[elan] elan_ts_send_cmd ok");
	}
	return size;
}

static int elan_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t size)
{
	int rc;

	if (buf == NULL){
		return -EINVAL;
    }
	if (elan_ts_send_cmd(client, cmd, size) != size){
		return -EINVAL;
    }
	msleep(2);

//printk("[elan] %s:----**colin**---elan_ts_poll--\n", __func__);
	
	rc = elan_ts_poll();
	if (rc < 0){
		return -EINVAL;
	}
	else {
		if (elan_i2c_recv_data(client, buf, size) != size ||buf[0] != CMD_S_PKT){
			printk("[elan error]%s: elan_i2c_recv_data failed\n", __func__);
			return -EINVAL;
		}
		printk("[elan] %s: respone packet %2x:%2X:%2x:%2x\n", __func__, buf[0], buf[1], buf[2], buf[3]);
	}
	
	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };
	uint8_t cmd[] = {0x53, 0x00, 0x00, 0x01};
	
	rc = elan_ts_poll();
	if(rc != 0){
		printk("[elan] %s: Int poll 55 55 55 55 failed!\n", __func__);
	}
			
	rc = elan_i2c_recv_data(client, buf_recv, sizeof(buf_recv));
	if(rc != sizeof(buf_recv)){
		printk("[elan error] __hello_packet_handler recv error\n");
	}
	printk("[elan] %s: hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	
	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80){
		printk("[elan] %s: boot code packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
		rc = 0x80;
	}
	else if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x55 && buf_recv[3]==0x55){
		printk("[elan] __hello_packet_handler recv ok\n");
		rc = 0x0;
	}
	else{
		if(rc != sizeof(buf_recv)){
			rc = elan_i2c_send_data(client, cmd, sizeof(cmd));
			if(rc != sizeof(cmd)){
				msleep(5);
				rc = elan_i2c_recv_data(client, buf_recv, sizeof(buf_recv));
			}
		}
	}
	
	return rc;
}

static int __fw_packet_handler_init(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[]			= {0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	
	uint8_t cmd_id[] 		= {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[]		= {0x53, 0x10, 0x00, 0x01};/* Get BootCode Version*/

	uint8_t cmd_x[] 		= {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] 		= {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/

	uint8_t buf_recv[4] 	= {0};
// Firmware version
	rc = elan_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
        {
		//return rc;
        }
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;

if(ts->fw_ver==0x5515)
{
// Firmware version
	ts->fw_ver = 0x5515; //major << 8 | minor;
	
// Firmware ID
	ts->fw_id = 0x2a14; //major << 8 | minor;

//X Y Resolution
	// X Resolution
	ts->x_resolution = 720; //minor; //by Colin

	// Y Resolution	
	ts->y_resolution = 1280; //minor; //by Colin
	
// Firmware BC
	ts->fw_bcd = 0x2104; //major << 8 | minor;	
}
else
{
// Firmware version
	ts->fw_ver = 0x5519; //major << 8 | minor;
	
// Firmware ID
	ts->fw_id = 0x2a96; //major << 8 | minor;

//X Y Resolution
	// X Resolution
	ts->x_resolution = 2160; //minor; //by Colin

	// Y Resolution	
	ts->y_resolution = 3840; //minor; //by Colin
	
// Firmware BC
	ts->fw_bcd = 0x2104; //major << 8 | minor;	
}
	
	printk( "[elan] %s:init firmware version: 0x%4.4x\n",__func__, ts->fw_ver);
	printk( "[elan] %s:init firmware ID: 0x%4.4x\n",__func__, ts->fw_id);
	printk( "[elan] %s:init firmware BC: 0x%4.4x\n",__func__, ts->fw_bcd);
	printk( "[elan] %s:init x resolution: %d, y resolution: %d\n",__func__, ts->x_resolution, ts->y_resolution);
	
	return 1;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[]			= {0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	
	uint8_t cmd_id[] 		= {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[]		= {0x53, 0x10, 0x00, 0x01};/* Get BootCode Version*/

	uint8_t cmd_x[] 		= {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] 		= {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/

	uint8_t buf_recv[4] 	= {0};
// Firmware version
	rc = elan_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	
// Firmware ID
	rc = elan_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;

//X Y Resolution
	// X Resolution
	rc = elan_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution = minor; 

	// Y Resolution	
	rc = elan_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution = minor; 
	
// Firmware BC
	rc = elan_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_bcd = major << 8 | minor;	
	
	printk( "[elan] %s: firmware version: 0x%4.4x\n",__func__, ts->fw_ver);
	printk( "[elan] %s: firmware ID: 0x%4.4x\n",__func__, ts->fw_id);
	printk( "[elan] %s: firmware BC: 0x%4.4x\n",__func__, ts->fw_bcd);
	printk( "[elan] %s: x resolution: %d, y resolution: %d\n",__func__, ts->x_resolution, ts->y_resolution);
	
	return 0;
}

#if defined IAP_PORTION || defined ELAN_RAM_XX
int WritePage(struct i2c_client *client, uint8_t * szPage, int byte, int which)
{
	int len = 0;
	
	len = elan_i2c_send_data(client, szPage,  byte);
	if (len != byte) {
		printk("[elan] ERROR: write the %d th page error, write error. len=%d\n", which, len);
		return -1;
	}
	
	return 0;
}

/*every page write to recv 2 bytes ack */
int GetAckData(struct i2c_client *client, uint8_t *ack_buf)
{
	int len = 0;
	
	len=elan_i2c_recv_data(client, ack_buf, 2);
	if (len != 2) {
		printk("[elan] ERROR: GetAckData. len=%d\r\n", len);
		return -1;
	}
	
	if (ack_buf[0] == 0xaa && ack_buf[1] == 0xaa) {
		return ACK_OK;
	}
	else if (ack_buf[0] == 0x55 && ack_buf[1] == 0x55){
		return ACK_REWRITE;
	}
	else{
		return ACK_Fail;
	}
	return 0;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(struct i2c_client *client)
{
	char buff[4] = {0};
	int rc = 0;
	
	rc = elan_i2c_recv_data(client, buff, 4);
	if (rc != 4) {
		printk("[elan] ERROR: CheckIapMode. len=%d\r\n", rc);
		return -1;
	}
	else
		printk("[elan] Mode= 0x%x 0x%x 0x%x 0x%x\r\n", buff[0], buff[1], buff[2], buff[3]);
		
	return 0;	
}

void update_fw_one_a(struct i2c_client *client)
{
	uint8_t ack_buf[2] = {0};
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	
	int res = 0;
	int iPage = 0;
	
	uint8_t data;

	const int PageSize = 132;

	const int PageNum = sizeof(file_fw_data)/PageSize;//by Colin
	
	const int PAGERETRY = 10;
	const int IAPRESTART = 3;
	
	int restartCnt = 0; // For IAP_RESTART
	int rewriteCnt = 0;// For IAP_REWRITE
	
	int iap_mod;
	
	uint8_t *szBuff = NULL;
	int curIndex = 0;

#ifdef ELAN_2K_XX	
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};	 //54 00 12 34
	iap_mod = 2;
#endif

	elan_switch_irq(0);
	ts->power_lock = 1;
	
	data=ELAN_7BITS_ADDR;
	printk( "[elan] %s: address data=0x%x iap_mod=%d PageNum = %d\r\n", __func__, data, iap_mod, PageNum);
	
IAP_RESTART:
	//reset tp
	if(iap_mod == 3){
		elan_reset(0);
	}
	
	if((iap_mod != 2) || (ts->recover != 0x80)){
		printk("[elan] Firmware update normal mode !\n");
		//Step 1 enter isp mod
		res = elan_ts_send_cmd(client, isp_cmd, sizeof(isp_cmd));
		//Step 2 Chech IC's status is 55 aa 33 cc
		if(iap_mod == 2){
			res = CheckIapMode(client);
		}
	} else{
		printk("[elan] Firmware update recovery mode !\n");	
	}
	
	//Step 3 Send Dummy Byte
	res = elan_i2c_send_data(client, &data,  sizeof(data));
	if(res!=sizeof(data)){
		printk("[elan] dummy error code = %d\n",res);
		return;
	}
	else{
		printk("[elan] send Dummy byte sucess data:%x", data);
	}	
	
	msleep(10);
	

	//Step 4 Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) {
		szBuff = file_fw_data + curIndex;//by Colin
		curIndex =  curIndex + PageSize;

PAGE_REWRITE:
		res = WritePage(client, szBuff, PageSize, iPage);
		
		if(iPage==PageNum || iPage==1){
			msleep(300); 			 
		}
		else{
			msleep(50); 			 
		}
		
		res = GetAckData(client, ack_buf);
		if (ACK_OK != res) {
			
			msleep(50); 
			printk("[elan]: %d page ack error: ack0:%x ack1:%x\n",  iPage, ack_buf[0], ack_buf[1]);
			
			if ( res == ACK_REWRITE ){
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt != PAGERETRY){
					printk("[elan] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					return;
				}
				else{
					printk("[elan] ---%d--- page ReWrite %d times! failed\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			}
			else{
				restartCnt = restartCnt + 1;
				if (restartCnt != IAPRESTART){
					printk("[elan] try to ReStart %d times !\n", restartCnt);
					return;
				}
				else{
					printk("[elan] ReStart %d times fails!\n", restartCnt);
					curIndex = 0;
					goto IAP_RESTART;
				}
			}
		}
		else{
			printk("[elan]---%d--- page flash ok", iPage);
			rewriteCnt=0;
		}
	}

	elan_reset(0);
	elan_switch_irq(1);
	ts->power_lock = 0;
	
	printk("[elan] Update ALL Firmware successfully!\n");
	
	return;
}

void update_fw_one_b(struct i2c_client *client)
{
	uint8_t ack_buf[2] = {0};
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	
	int res = 0;
	int iPage = 0;
	
	uint8_t data;

	const int PageSize = 132;
	const int PageNum = sizeof(file_fw_data_b)/PageSize; //by Colin
	
	const int PAGERETRY = 10;
	const int IAPRESTART = 3;
	
	int restartCnt = 0; // For IAP_RESTART
	int rewriteCnt = 0;// For IAP_REWRITE
	
	int iap_mod;
	
	uint8_t *szBuff = NULL;
	int curIndex = 0;

#ifdef ELAN_2K_XX	
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};	 //54 00 12 34
	iap_mod = 2;
#endif

	elan_switch_irq(0);
	ts->power_lock = 1;
	
	data=ELAN_7BITS_ADDR;
	printk( "[elan] %s: address data=0x%x iap_mod=%d PageNum = %d\r\n", __func__, data, iap_mod, PageNum);
	
IAP_RESTART:
	//reset tp
	if(iap_mod == 3){
		elan_reset(1);
	}
	
	if((iap_mod != 2) || (ts->recover != 0x80)){
		printk("[elan] Firmware update normal mode !\n");
		//Step 1 enter isp mod
		res = elan_ts_send_cmd(client, isp_cmd, sizeof(isp_cmd));
		//Step 2 Chech IC's status is 55 aa 33 cc
		if(iap_mod == 2){
			res = CheckIapMode(client);
		}
	} else{
		printk("[elan] Firmware update recovery mode !\n");	
	}
	
	//Step 3 Send Dummy Byte
	res = elan_i2c_send_data(client, &data,  sizeof(data));
	if(res!=sizeof(data)){
		printk("[elan] dummy error code = %d\n",res);
		return;
	}
	else{
		printk("[elan] send Dummy byte sucess data:%x", data);
	}	
	
	msleep(10);
	
	//Step 4 Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) {
		szBuff = file_fw_data_b + curIndex;
		curIndex =  curIndex + PageSize;

PAGE_REWRITE:
		res = WritePage(client, szBuff, PageSize, iPage);
		
		if(iPage==PageNum || iPage==1){
			msleep(300); 			 
		}
		else{
			msleep(50); 			 
		}
		
		res = GetAckData(client, ack_buf);
		if (ACK_OK != res) {
			
			msleep(50); 
			printk("[elan]: %d page ack error: ack0:%x ack1:%x\n",  iPage, ack_buf[0], ack_buf[1]);
			
			if ( res == ACK_REWRITE ){
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt != PAGERETRY){
					printk("[elan] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					return;
				}
				else{
					printk("[elan] ---%d--- page ReWrite %d times! failed\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			}
			else{
				restartCnt = restartCnt + 1;
				if (restartCnt != IAPRESTART){
					printk("[elan] try to ReStart %d times !\n", restartCnt);
					return;
				}
				else{
					printk("[elan] ReStart %d times fails!\n", restartCnt);
					curIndex = 0;
					goto IAP_RESTART;
				}
			}
		}
		else{
			printk("[elan]---%d--- page flash ok", iPage);
			rewriteCnt=0;
		}
	}

	elan_reset(1);
	elan_switch_irq(1);
	ts->power_lock = 0;
	
	printk("[elan] Update ALL Firmware successfully!\n");
	
	return;
}
#endif
static inline int elan_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ts_setup(struct i2c_client *client)
{
	int rc = 0;
	elan_reset(1);//reset LCD_B
	msleep(50);

	rc = __hello_packet_handler(client);
	if (rc < 0){
		printk("[elan error] %s, hello_packet_handler fail, rc = %d\n", __func__, rc);
	}
	return rc;
}

static int elan_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	int size = sizeof(cmd);
	
	cmd[1] |= (state << 3);
	if (elan_ts_send_cmd(client, cmd, size) != size){
		return -EINVAL;
	}	

	return 0;
}

static void elan_ts_touch_down(struct elan_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(ts->input_dev);

	elan_info("Touch ID:%d, X:%d, Y:%d, W:%d down", id, x, y, w); 
}

static void elan_ts_touch_up(struct elan_ts_data* ts,s32 id,s32 x,s32 y)
{
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts->input_dev);
	
	elan_info("Touch all release!");
}

static void elan_ts_report_key(struct elan_ts_data *ts, uint8_t button_data)
{
	static unsigned int x = 0,y = 0;
    static unsigned int isKey = 0;

	switch (button_data) {
		case ELAN_KEY_MENU:
			x = 690;
			y = TPD_BUTTON_MAX;
            isKey = 1;
			if(boot_normal_flag == 1){
				elan_ts_touch_down(ts, 0, x, y, 8);
			}
			else{
				tpd_button(x, y, 1);
			}
			break;
		case ELAN_KEY_HOME:
			x = 360;
			y = TPD_BUTTON_MAX;
            isKey = 1;
			if(boot_normal_flag == 1){
				elan_ts_touch_down(ts, 0, x, y, 8);
			}
			else{
				tpd_button(x, y, 1);
			}
			break;
		case ELAN_KEY_BACK:		
			x = 30;
			y = TPD_BUTTON_MAX;
            isKey = 1;
			if(boot_normal_flag == 1){
				elan_ts_touch_down(ts, 0, x, y, 8);
			}
			else{
				tpd_button(x, y, 1);
			}
			break;
		default:
			if(boot_normal_flag == 1 || isKey == 0){
				elan_ts_touch_up(ts, 0, x, y);
			}
			else{
				tpd_button(x, y, 0);
                isKey = 0;
			}
			break;
	}
}

void send_fake_touch_up_event()
{
	struct elan_ts_data *ts = private_ts;
	if(boot_normal_flag == 1){
		elan_ts_touch_up(ts, 0, 400, 400);
	}else{
		tpd_button(400,400,0);
	}
	input_sync(ts->input_dev);
}

EXPORT_SYMBOL(send_fake_touch_up_event);

#if defined ELAN_RAM_XX
static void elan_ts_iap_ram_continue(struct i2c_client *client)
{
	uint8_t cmd[] = { 0x33, 0x33, 0x33, 0x33 };
	int size = sizeof(cmd);

	elan_ts_send_cmd(client, cmd, size);
}
#endif

static void elan_ts_handler_event(struct elan_ts_data *ts, uint8_t *buf)
{
	int rc = 0;
	
	if(buf[0] == 0x55){
		if(buf[2] == 0x55){
			ts->recover = 0;
		}
		else if(buf[2] == 0x80){
			ts->recover = 0x80;
		}
	}
#ifdef ELAN_ESD_CHECK	
	else if(buf[0] == 0x52 || buf[0] == 0x78){
		atomic_set(&elan_cmd_response, 1);
	}
#endif	
}

#ifdef ELAN_IAP_DEV

int elan_iap_open(struct inode *inode, struct file *filp)
{ 
	elan_info("%s enter", __func__);
	pre_addr = i2c_addr;
	i2c_addr = 0x10;

	if (private_ts == NULL){
		printk("private_ts is NULL~~~");
	}	
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{    
	elan_info("%s enter", __func__);
	i2c_addr = pre_addr;

	printk("%s enter, %x\n", __func__,i2c_addr);
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{  
	int ret;
	char *tmp;
	struct i2c_client *client = private_ts->client;
	
	elan_info("%s enter", __func__);
	if (count > 8192){
		count = 8192;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
		return -ENOMEM;
	}
	if (copy_from_user(tmp, buff, count)) {
		return -EFAULT;
	}
	
	ret = elan_i2c_send_data(client, tmp, count);
	if (ret != count){ 
		printk("elan elan_i2c_send_data fail, ret=%d \n", ret);
	}
	kfree(tmp);
	
	return ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{    
	char *tmp;
	int ret;  
	long rc;
	
	struct i2c_client *client = private_ts->client;
	
	elan_info("%s enter", __func__);
	
	if (count > 8192){
		count = 8192;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
		return -ENOMEM;
	}
	ret = elan_i2c_recv_data(client, tmp, count);
	if (ret != count){ 
		printk("elan elan_i2c_recv_data fail, ret=%d \n", ret);
	}
	if (ret == count){
		rc = copy_to_user(buff, tmp, count);
	}
	kfree(tmp);
	return ret;
}

static long elan_iap_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
	int __user *ip = (int __user *)arg;
	char buf[4] = {0};
	
	elan_info("%s enter, cmd value %x\n",__func__, cmd);

	switch (cmd) {        
		case IOCTL_I2C_SLAVE:
			printk("[elan debug] pre addr is %X\n",  private_ts->client->addr); 
			private_ts->client->addr = (int __user)arg;
			printk("[elan debug] new addr is %X\n",  private_ts->client->addr); 
			break;
		case IOCTL_RESET:
			elan_reset(1);
			break;
		case IOCTL_IAP_MODE_LOCK:
			if(private_ts->power_lock==0){
				private_ts->power_lock=1;
				elan_switch_irq(0);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(private_ts->power_lock==1){			
				private_ts->power_lock=0;
				elan_switch_irq(1);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return private_ts->recover;;
			break;
		case IOCTL_I2C_INT:
			put_user(mt_get_gpio_in(GPIO_CTP_B_EINT_PIN), ip);
			break;
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
	.open = elan_iap_open,    
	.write = elan_iap_write,    
	.read = elan_iap_read,    
	.release =	elan_iap_release,    
	.unlocked_ioctl = elan_iap_ioctl, 
 };

#endif


#ifdef ELAN_IAP_DEV
int elan_iap_open_15(struct inode *inode, struct file *filp)
{ 
	elan_info("%s enter", __func__);
	pre_addr = i2c_addr;
	i2c_addr = 0x15;

	if (private_ts == NULL){
		printk("private_ts is NULL~~~");
	}	
	return 0;
}

int elan_iap_release_15(struct inode *inode, struct file *filp)
{    
	elan_info("%s enter", __func__);
//	private_ts->client->addr = i2c_addr;
	i2c_addr = pre_addr;

	printk("%s enter, %x\n", __func__,i2c_addr);
	return 0;
}

static ssize_t elan_iap_write_15(struct file *filp, const char *buff, size_t count, loff_t *offp)
{  
	int ret;
	char *tmp;
	struct i2c_client *client = private_ts->client;
	
	elan_info("%s enter", __func__);
	if (count > 8192){
		count = 8192;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
		return -ENOMEM;
	}
	if (copy_from_user(tmp, buff, count)) {
		return -EFAULT;
	}
	
	ret = elan_i2c_send_data(client, tmp, count);
	if (ret != count){ 
		printk("elan elan_i2c_send_data fail, ret=%d \n", ret);
	}
	kfree(tmp);
	
	return ret;
}

ssize_t elan_iap_read_15(struct file *filp, char *buff, size_t count, loff_t *offp)
{    
	char *tmp;
	int ret;  
	long rc;
	
	struct i2c_client *client = private_ts->client;
	
	elan_info("%s enter", __func__);
	
	if (count > 8192){
		count = 8192;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
		return -ENOMEM;
	}
	ret = elan_i2c_recv_data(client, tmp, count);
	if (ret != count){ 
		printk("elan elan_i2c_recv_data fail, ret=%d \n", ret);
	}
	if (ret == count){
		rc = copy_to_user(buff, tmp, count);
	}
	kfree(tmp);
	return ret;
}

static long elan_iap_ioctl_15( struct file *filp, unsigned int cmd, unsigned long arg)
{
	int __user *ip = (int __user *)arg;
	char buf[4] = {0};
	
	elan_info("%s enter, cmd value %x\n",__func__, cmd);

	switch (cmd) {        
		case IOCTL_I2C_SLAVE:
			printk("[elan debug] pre addr is %X\n",  private_ts->client->addr); 
			private_ts->client->addr = (int __user)arg;
			printk("[elan debug] new addr is %X\n",  private_ts->client->addr); 
			break;
		case IOCTL_RESET:
			elan_reset(0);
			break;
		case IOCTL_IAP_MODE_LOCK:
			if(private_ts->power_lock==0){
				private_ts->power_lock=1;
				elan_switch_irq(0);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(private_ts->power_lock==1){			
				private_ts->power_lock=0;
				elan_switch_irq(1);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return private_ts->recover;;
			break;
		case IOCTL_I2C_INT:
			put_user(mt_get_gpio_in(GPIO_CTP_EINT_PIN), ip);
			break;
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops_15 = {    
	.open = elan_iap_open_15,    
	.write = elan_iap_write_15,    
	.read = elan_iap_read_15,    
	.release =	elan_iap_release_15,    
	.unlocked_ioctl = elan_iap_ioctl_15, 
 };  

#endif
#ifdef ELAN_ESD_CHECK
static void elan_touch_esd_func(struct work_struct *work)
{	
	int res;	
	uint8_t cmd[] = {0x53, 0x00, 0x00, 0x01};	
	struct i2c_client *client = private_ts->client;	

	elan_info("esd %s: enter.......", __FUNCTION__);
	
	if(private_ts->power_lock == 1){
		goto elan_esd_check_out;
	}
	
	if(atomic_read(&have_interrupts) == 1){
		elan_info("esd %s: had interrup not need check", __func__);
		goto elan_esd_check_out;
	}
	else{

	#ifdef ELAN_ESD_IAM_ALIVE
		if(atomic_read(&elan_cmd_response) == 0){
			elan_info("esd %s: have none response", __func__);
		}
		else{
			elan_info("esd %s: 78 78 78 78 response ok", __func__);
			atomic_set(&elan_cmd_response, 0);
			goto elan_esd_check_out;
		}
	#else
		atomic_set(&elan_cmd_response, 0);
		res = elan_ts_send_cmd(client, cmd, sizeof(cmd));
		if (res != sizeof(cmd)){
			elan_info("[elan esd] %s: elan_ts_send_cmd failed reset now", __func__);
		}
		else{
			msleep(10);
			
			if(atomic_read(&elan_cmd_response) == 0){
				elan_info("esd %s: elan_ts_send_cmd successful, response failed", __func__);
			}
			else{
				elan_info("esd %s: elan_ts_send_cmd successful, response ok", __func__);
				goto elan_esd_check_out;
			}
		}
	#endif	
	}
	elan_reset(lcd_b);
elan_esd_check_out:	
	atomic_set(&have_interrupts, 0);
	queue_delayed_work(esd_wq, &esd_work, delay);
	elan_info("[elan esd] %s: out.......", __FUNCTION__);	
	return;
}
#endif	


#ifdef SYS_ATTR_FILE
static ssize_t elan_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
#ifdef PRINT_INT_INFO 	
	debug_flage = !debug_flage;
	if(debug_flage)
		printk("elan debug switch open\n");
	else
		printk("elan debug switch close\n");
#endif	
	return ret;
}
static DEVICE_ATTR(debug, S_IRUGO, elan_debug_show, NULL);


static ssize_t elan_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ts_data *ts = private_ts;
	elan_switch_irq(0);
	__fw_packet_handler(ts->client);
	elan_switch_irq(1);
	sprintf(buf, "elan fw ver:%X,id:%X,x:%d,y:%d\n", ts->fw_ver, ts->fw_id, ts->x_resolution, ts->y_resolution);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(info, S_IRUGO, elan_info_show, NULL);

static ssize_t set_cmd_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	char cmd[4] = {0};
	if (size > 4)
		return -EINVAL;
	
	if (sscanf(buf, "%02x:%02x:%02x:%02x\n", (int *)&cmd[0], (int *)&cmd[1], (int *)&cmd[2], (int *)&cmd[3]) != 4){
		printk("elan cmd format error\n");
		return -EINVAL;
	}
	elan_ts_send_cmd(private_ts->client, cmd, 4);
	return size;
}
static DEVICE_ATTR(set_cmd, S_IRUGO, NULL, set_cmd_store);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_debug.attr,
	&dev_attr_info.attr,
	&dev_attr_set_cmd.attr,
	NULL
};
static struct attribute_group elan_attribute_group[] = {
	{.attrs = sysfs_attrs_ctrl },
};
#endif

static void elan_touch_node_init(void)
{
	int ret ;
	struct elan_ts_data *ts = private_ts;
#ifdef SYS_ATTR_FILE		
	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[elan]%s: kobject_create_and_add failed\n", __func__);
		return;
	}
	
	ret = sysfs_create_group(android_touch_kobj, elan_attribute_group);
	if (ret < 0) {
		printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
	}
#endif
	
#ifdef ELAN_IAP_DEV	
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap_10";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO; 
	if (misc_register(&ts->firmware) < 0)
		printk("[elan debug] misc_register failed!!\n");
	else
		printk("[elan debug] misc_register ok!!\n");  

	ts->firmware_15.minor = MISC_DYNAMIC_MINOR;
	ts->firmware_15.name = "elan-iap_15";
	ts->firmware_15.fops = &elan_touch_fops_15;
	ts->firmware_15.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&ts->firmware_15) < 0)
		printk("[elan debug] misc_register failed!!\n");
	else
		printk("[elan debug] misc_register ok!!\n");   
#endif
	return;
}

static void elan_touch_node_deinit(void)
{
#ifdef SYS_ATTR_FILE
	sysfs_remove_group(android_touch_kobj, elan_attribute_group);
	kobject_del(android_touch_kobj);
#endif	
}

static int elan_ts_recv_data(struct elan_ts_data *ts, uint8_t *buf)
{
	int rc;
	int i = 0;
	
	rc = elan_i2c_recv_data(ts->client, buf, PACKET_SIZE);
	if(PACKET_SIZE != rc){
		printk("[elan error] elan_ts_recv_data\n");
		return -1;
	}
#ifdef PRINT_INT_INFO
	for(i = 0; i < (PACKET_SIZE+7)/8; i++){
		elan_info("%02x %02x %02x %02x %02x %02x %02x %02x", buf[i*8+0],buf[i*8+1],buf[i*8+2],buf[i*8+3],buf[i*8+4],buf[i*8+5],buf[i*8+6],buf[i*8+7]);
	}
#endif
	
	if(FINGERS_PKT != buf[0]){
#ifndef PRINT_INT_INFO		
		printk("[elan] other event packet:%02x %02x %02x %02x %02x %02x %02x %02x\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
#endif
		elan_ts_handler_event(ts, buf);
		return -1;
	}
	
#ifdef ELAN_ESD_CHECK
	atomic_set(&have_interrupts, 1);
#endif
	return 0;
}

static void elan_ts_report_data(struct elan_ts_data *ts, uint8_t *buf)
{
	uint16_t fbits=0;
	int reported = 0;
	uint8_t idx;
	int finger_num;
	int num = 0;
	uint16_t x = 0;
	uint16_t y = 0;
	int position = 0;
	uint8_t button_byte = 0;

	finger_num = FINGERS_NUM;	
#ifdef TWO_FINGERS
	num = buf[7] & 0x03; 
	fbits = buf[7] & 0x03;
	idx=1;
	button_byte = buf[PACKET_SIZE-1];
#endif
	
#ifdef FIVE_FINGERS
	num = buf[1] & 0x07; 
	fbits = buf[1] >>3;
	idx=2;
	button_byte = buf[PACKET_SIZE-1];
#endif
	
#ifdef TEN_FINGERS
	fbits = buf[2] & 0x30;	
	fbits = (fbits << 4) | buf[1];  
	num = buf[2] &0x0f;
	idx=3;
	button_byte = buf[PACKET_SIZE-1];
#endif

	if (num == 0){
		elan_ts_report_key(ts, button_byte);
	} 
	else{
		elan_info( "[elan] %d fingers", num);
				
		for(position=0; (position<finger_num) && (reported < num);position++){
			if((fbits & 0x01)){
				elan_ts_parse_xy(&buf[idx],&x, &y);
				//x = ts->x_resolution-x;
				//y = ts->y_resolution-y;
				x = x*LCM_X_MAX/ts->x_resolution;
				y = y*LCM_Y_MAX/ts->y_resolution;
				//if(lcd_b)
				//{
				//	x = LCM_X_MAX-x;///
				//	y = LCM_Y_MAX-y;///
				//}
				elan_ts_touch_down(ts, position, x, y, 8);
				reported++;
			}
			fbits = fbits >> 1;
			idx += 3;
		}
	}
	
	input_sync(ts->input_dev);
	return;
}

int g_Touch_Int_finish=1; //by colin
static void tpd_eint_interrupt_handler(void)
{
    if(g_Touch_Int_finish==1)//by colin
    {
        g_Touch_Int_finish = 0;
        elan_switch_irq(0);
        tpd_flag = 1;
        wake_up_interruptible(&waiter);
        g_Touch_Int_finish = 1;
    }
}

#if defined IAP_PORTION

static void check_update_flage_a(struct elan_ts_data *ts)
{
	int NEW_FW_VERSION = 0;
	int New_FW_ID = 0;
	int rc = 0;
	
	if(ts->recover == 0x80)
        {
	    printk("[elan] ***fw is miss, force update!!!!***\n");
	    goto update_elan_fw;    
        }
	
	New_FW_ID  = file_fw_data[0x7D67]<<8  | file_fw_data[0x7D66];
	NEW_FW_VERSION = file_fw_data[0x7D65]<<8  | file_fw_data[0x7D64];
	
	printk("[elan] FW_ID=0x%x, New_FW_ID=0x%x \n",ts->fw_id, New_FW_ID);
	printk("[elan] FW_VERSION=0x%x,New_FW_VER=0x%x \n",ts->fw_ver,NEW_FW_VERSION);

	if((ts->fw_id&0xff) != (New_FW_ID&0xff)){
		printk("[elan] ***fw id is different, can not update !***\n");
		goto no_update_elan_fw;
	}
	else{
		printk("[elan] fw id is same !\n");
	} 
	
	if((ts->fw_ver&0xff) >= (NEW_FW_VERSION&0xff)){
		printk("[elan] fw version is newest!!\n");
		goto no_update_elan_fw;
	}
	
update_elan_fw:
   // update_fw_one_a(ts->client);
    msleep(500);
    elan_switch_irq(0);
	rc = __fw_packet_handler(ts->client);
	if (rc < 0){
		printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
	}
    elan_switch_irq(1);
	
no_update_elan_fw:
	printk("[elan] %s, fw check end..............\n", __func__);
            	
	return;
}

static void check_update_flage_b(struct elan_ts_data *ts)
{
	int NEW_FW_VERSION = 0;
	int New_FW_ID = 0;
	int rc = 0;
	
	if(ts->recover == 0x80)
        {
	    printk("[elan] ***fw is miss, force update!!!!***\n");
	    goto update_elan_fw;    
        }
	
	New_FW_ID  = file_fw_data_b[0x7D67]<<8  | file_fw_data_b[0x7D66];
	NEW_FW_VERSION = file_fw_data_b[0x7D65]<<8  | file_fw_data_b[0x7D64];		
	
	printk("[elan] FW_ID=0x%x, New_FW_ID=0x%x \n",ts->fw_id, New_FW_ID);
	printk("[elan] FW_VERSION=0x%x,New_FW_VER=0x%x \n",ts->fw_ver,NEW_FW_VERSION);

	if((ts->fw_id&0xff) != (New_FW_ID&0xff)){
		printk("[elan] ***fw id is different, can not update !***\n");
		goto no_update_elan_fw;
	}
	else{
		printk("[elan] fw id is same !\n");
	} 
	
	if((ts->fw_ver&0xff) >= (NEW_FW_VERSION&0xff)){
		printk("[elan] fw version is newest!!\n");
		goto no_update_elan_fw;
	}
	
update_elan_fw:
   // update_fw_one_b(ts->client);
    msleep(500);
    elan_switch_irq(0);
	rc = __fw_packet_handler(ts->client);
	if (rc < 0){
		printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
	}
    elan_switch_irq(1);
	
no_update_elan_fw:
	printk("[elan] %s, fw check end..............\n", __func__);
            	
	return;
}
#endif

static int touch_event_handler(void *unused)
{
	uint8_t buf[64] = {0};	
	int rc = 0;
	struct elan_ts_data *ts = private_ts;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD};
	sched_setscheduler(current, SCHED_RR, &param);
	
	do{
//		ts->client->addr = i2c_addr;  //Shibo 
		set_current_state(TASK_INTERRUPTIBLE);
		elan_switch_irq(1);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		rc = elan_ts_recv_data(ts, buf);
		if(rc < 0)
		{
			continue;
		}
		elan_ts_report_data(ts, buf);

	}while(!kthread_should_stop());

    return 0;
}

void tpd_switch_a(void)
{
    if(!g_elan_ic_init_work_finish) {
printk("--***colin**tpd_switch_a*!g_elan_ic_init_work_finish \n");
        return;
    }

    lcd_b = 1;

    struct elan_ts_data *ts = private_ts;
    elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);

    lcd_b = 0;
    elan_reset(0); //LCD_A
}

void tpd_switch_b(void)
{
    if(!g_elan_ic_init_work_finish) {
printk("--***colin**tpd_switch_b*!g_elan_ic_init_work_finish \n");
        return;
    }

    lcd_b = 0;

    struct elan_ts_data *ts = private_ts;
    elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);

    lcd_b = 1;
    elan_reset(1);  //LCD_B
}

static int get_hall_stage() {
    int nLevel;
    mt_set_gpio_mode(GPIO_SWITCH_HAL_EINT_PIN, GPIO_SWITCH_HAL_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_SWITCH_HAL_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_SWITCH_HAL_EINT_PIN, GPIO_PULL_DISABLE);

    nLevel = mt_get_gpio_in(GPIO_SWITCH_HAL_EINT_PIN);
    
    return (1 - nLevel);
}

static void elan_ic_init_work(struct work_struct *work)
{
    int rc = 0;
    
		if(private_ts->recover == 0){
			elan_switch_irq(0);
			rc = __fw_packet_handler_init(private_ts->client);
			if (rc < 0){
			printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
			}
			elan_switch_irq(1);
		}
#if defined IAP_PORTION	
		//check_update_flage_b(private_ts);
		//mdelay(20); 
#endif 



#ifdef ELAN_ESD_CHECK
	INIT_DELAYED_WORK(&esd_work, elan_touch_esd_func);
	esd_wq = create_singlethread_workqueue("esd_wq");	
	if (!esd_wq) {
		return -ENOMEM;
	}
	queue_delayed_work(esd_wq, &esd_work, delay);
#endif  

g_elan_ic_init_work_finish = 1;

lcd_b = get_hall_stage();
if(lcd_b==1)
{
    tpd_switch_b();
}
else
{
    tpd_switch_a();
}

printk("[elan] **colin**elan_ic_init_work end----\n");
    		
} 


static int elan_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = 0;
	struct elan_ts_data *ts;
	
	printk("[elan] %s enter i2c addr %x\n", __func__, client->addr);

g_elan_ic_init_work_finish = 0;

//power on, need confirm with SA
	hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_2800, "TP");
	hwPowerOn(MT6328_POWER_LDO_VIO18, VOL_1800, "TP");
    msleep(10);
 
#ifdef ELAN_I2C_DMA_MOD	
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev,4096, &gpDMABuf_pa, GFP_KERNEL);//4096
    if(NULL == gpDMABuf_va) {
		printk("[elan] Allocate DMA I2C Buffer failed\n");
		return -ENOMEM;
    }
#endif

	/*elan IC init here*/
	retval = elan_ts_setup(client);
	if (retval < 0) {
		printk("[elan error]: %s No Elan chip inside, return now\n", __func__);
		return -ENODEV;
	}
	
	/*elan struct elan_ts_data init*/
	ts = kzalloc(sizeof(struct elan_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk("[elan error] %s: allocate elan_ts_data failed\n", __func__);
		return -ENOMEM;
	}

	ts->recover = retval;
	client->timing = 100;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	private_ts = ts;
	ts->input_dev = tpd->dev;
	
	elan_touch_node_init();
	
	INIT_DELAYED_WORK(&init_work, elan_ic_init_work);
	init_elan_ic_wq = create_singlethread_workqueue("init_elan_ic_wq");	
	if (!init_elan_ic_wq) {
		return -ENOMEM;
	}
	queue_delayed_work(init_elan_ic_wq, &init_work, delay);

// Setup Interrupt Pin	
	/*elan interrupt handler registration*/
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1); 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    mdelay(10);
// End Setup Interrupt Pin   

// Setup Interrupt B Pin   
	mt_set_gpio_mode(GPIO_CTP_B_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_B_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_B_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_B_EINT_PIN, GPIO_PULL_UP);

	mt_eint_registration(CUST_EINT_TOUCH_PANEL_B_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1); 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_B_NUM);
    mdelay(10);
// End Setup Interrupt B Pin   
   
	ts->work_thread = kthread_run(touch_event_handler, 0, ELAN_TS_NAME);
	if(IS_ERR(ts->work_thread)) {
		retval = PTR_ERR(ts->work_thread);
		printk("[elan error] failed to create kernel thread\n");
		return -EINVAL;
	}
 
    tpd_load_status = 1;
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()){
		boot_normal_flag = 0;
	}
  
	printk("[elan]+++++++++end porbe+++++++++!\n");
		
	return 0;
}

static int elan_ts_remove(struct i2c_client *client)
{
	printk("[elan] elan_ts_remove\n");
#ifdef ELAN_I2C_DMA_MOD		
	if(gpDMABuf_va){
		dma_free_coherent(NULL, 4096, gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = NULL;
	}
#endif
	
	return 0;
}

static struct i2c_driver elan_ts_driver = {
	.probe = elan_ts_probe,
	.remove = elan_ts_remove,
	.id_table = elan_ts_id,
	.driver	= {
		.name = ELAN_TS_NAME,
	},
	.detect = tpd_detect,
#ifndef I2C_BORAD_REGISTER 	
	.address_data = &addr_data,
#endif	
};


int tpd_local_init(void) 
{
	if(i2c_add_driver(&elan_ts_driver)!=0) {
		TPD_DMESG("[elan error] unable to add i2c driver.\n");
		return -1;
	}

	if(tpd_load_status == 0){
		TPD_DMESG("[elan error] add error touch panel driver.\n");
		i2c_del_driver(&elan_ts_driver);
		return -1;
	}

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
	TPD_DMESG("[elan] end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;

	return 0;
}

 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	struct elan_ts_data *ts = private_ts;
	int rc = 0;

	printk( "[elan] %s: enter\n", __func__);
	if(ts->power_lock==0){
		elan_switch_irq(0);
		rc = elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);
		tpd_is_suspend = 1;
	}
#ifdef ELAN_ESD_CHECK	
	cancel_delayed_work_sync(&esd_work);
#endif	
	return rc;
}

static int tpd_resume(struct i2c_client *client)
{
	struct elan_ts_data *ts = private_ts;
	int rc = 0;

	printk("[elan] %s: enter\n", __func__);
	if(ts->power_lock==0){
		printk("[elan] reset gpio to resum tp\n");		
		elan_reset(lcd_b);
		elan_switch_irq(1);
		tpd_is_suspend = 0;
	}
#ifdef ELAN_ESD_CHECK
	queue_delayed_work(esd_wq, &esd_work, delay);	
#endif
	return rc;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "elan_ts",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init tpd_driver_init(void) 
{
	printk("[elan] MediaTek elan touch panel driver init\n");

#ifdef I2C_BORAD_REGISTER 	
	i2c_register_board_info(I2C_NUM, &i2c_tpd, 1);
#endif	
	if(tpd_driver_add(&tpd_device_driver) < 0){
		TPD_DMESG("[elan error] add generic driver failed\n");
	}

	return 0;
}

static void __exit tpd_driver_exit(void) 
{
	TPD_DMESG("[elan] MediaTek elan touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
