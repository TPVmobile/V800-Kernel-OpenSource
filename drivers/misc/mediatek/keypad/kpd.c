/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*kpd.h file path: ALPS/mediatek/kernel/include/linux */
#include <linux/kpd.h>
#ifdef CONFIG_MTK_TC1_FM_AT_SUSPEND
#include <linux/wakelock.h>
#endif
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif
#include <linux/i2c.h>
#include <linux/leds.h>
#include "cust_leds.h"

#define KPD_NAME	"mtk-kpd"
#define MTK_KP_WAKESOURCE	/* this is for auto set wake up source */

#ifdef CONFIG_OF
void __iomem *kp_base;
static unsigned int kp_irqnr;
#endif	
struct input_dev *kpd_input_dev;
static bool kpd_suspend = false;
static int kpd_show_hw_keycode = 1;
static int kpd_show_register = 1;
static volatile int call_status = 0;
#ifdef CONFIG_MTK_TC1_FM_AT_SUSPEND
struct wake_lock kpd_suspend_lock; /* For suspend usage */
#endif

/*for kpd_memory_setting() function*/
static u16 kpd_keymap[KPD_NUM_KEYS];
static u16 kpd_keymap_state[KPD_NUM_MEMS];
/***********************************/


#define KPD_HAS_EXTEND_KEYPAD	1
#if KPD_HAS_EXTEND_KEYPAD

#define AW9523_delay_1us(us)        udelay(us)

#define EXTEND_KEYPAD_I2C_BUSNUM    1
#define EXTEND_KEYPAD_I2C_ID_NAME   "aw9523"
#define EXTEND_KEYPAD_I2C_ADDR      0xB6 >> 1

#define Y_NUM  5
#define X_NUM  5

static struct i2c_board_info __initdata aw9523_board_info = {I2C_BOARD_INFO(EXTEND_KEYPAD_I2C_ID_NAME, EXTEND_KEYPAD_I2C_ADDR)};
static struct i2c_client *aw9523_i2c_client = NULL;
struct timer_list   		timer_kpd;  /*kpd blacklight timer */

#define KPD_BLACKLIGHT_TIMEOUT      7   
#define AW9523_RESET_PIN	        (GPIO54 | 0x80000000)
#define AW9523_EINT_GPIO            (GPIO4 | 0x80000000) 
#define AW9523_EINT_NO              4
#define AW9523_EINT_POLARITY        CUST_EINTF_TRIGGER_FALLING
#define AW9523_EINT_DEBOUNCE_CN     10


void Set_P0_X_AND_P1_Y(void);
void aw9523_init();
static void kpd_aw9523_eint_handler(void);


typedef enum {
    P0_0=0,
    P0_1,
    P0_2,
    P0_3,
    P0_4,
    P0_5,
    P0_6,
    P0_7
} P0_Enum;

typedef enum {
    P1_0=0,
    P1_1,
    P1_2,
    P1_3,
    P1_4, 
    P1_5,
    P1_6,
    P1_7
} P1_Enum;

typedef  enum {
    KEY_STATE_PRESSED=0,
    KEY_STATE_RELEASED,
    KEY_STATE_LONGPRESS,
    KEY_STATE_REPEATED, 
    KEY_STATE_NULL 
}TOUCHKEY_STATE;


const P0_Enum COL[X_NUM] = {P0_0, P0_1, P0_2, P0_3, P0_4};
const P1_Enum Line[Y_NUM] = {P1_0, P1_1, P1_2, P1_3, P1_4};
const u8  aw9523_key[Y_NUM][X_NUM] = {
    {KEY_0,         KEY_POUND,      KEY_8,       KEY_7,      KEY_STAR},
    {KEY_6,         KEY_9,          KEY_5,      KEY_4,      KEY_3},
    {KEY_DEL,       KEY_CALL,       KEY_2,      KEY_1,      KEY_ENDCALL},
    {KEY_SEARCH,    KEY_BACK,       KEY_RIGHT,  KEY_HOME,   KEY_MENU},
    {KEY_UP,        KEY_OK,         KEY_DOWN,   KEY_LEFT,   KEY_1},
};

u8 P0_kbd_used[8] = {1, 1, 1, 1, 1, 0, 0, 0};
u8 P1_kbd_used[8] = {1, 1, 1, 1, 1, 0, 0, 0}; 

u8  P0_INT_STATE = 0x0;
u8  P1_INT_STATE = 0x0;
u8  P0_IN_OUT_STATE = 0x0;
u8  P1_IN_OUT_STATE = 0x0;
u8  P0_kbd_used_temp = 0x0;
u8  pre_x = 0x00;
u8  pre_y = 0x00;
u8  P0_X[X_NUM];
u8  P1_Y[Y_NUM];
u8  P1_VALUE = 0;
u8 KeyBoard_Key = 0xFF;
u8 KeyBoard_Key_Previous = 0xFF;
//static u8 kpd_aw9523_state = 1;

TOUCHKEY_STATE KeyBoardKey_State = KEY_STATE_NULL;


static struct workqueue_struct * kpd_aw9523_eint_workqueue = NULL;
static struct work_struct kpd_aw9523_eint_work;
static void kpd_aw9523_handler(unsigned long data);
extern int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness level);
static struct work_struct led_disable_work;
static struct work_struct led_enable_work;
static struct workqueue_struct * led_workqueue = NULL;
static int probe_complete = 0;
static int led_state = 0;

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int aw9523_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int aw9523_remove(struct i2c_client *client);


static const struct i2c_device_id aw9523_id[] = {
	{EXTEND_KEYPAD_I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver aw9523_iic_driver = {
	.id_table	= aw9523_id,
	.probe		= aw9523_probe,
	.remove		= aw9523_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "aw9523",
	},
};

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
 static void close_blacklight_callback()
{
    led_state = 0;
    mt65xx_leds_brightness_set(MT65XX_LED_TYPE_KEYBOARD, LED_OFF);
}

static void open_blacklight_callback()
{
    led_state = 1;
    mt65xx_leds_brightness_set(MT65XX_LED_TYPE_KEYBOARD, LED_FULL);
}

static void close_blacklight()
{
    int ret = 0;
    if(probe_complete == 0)
        return;

    if(led_state == 0) 
        return;
    
    ret = queue_work(led_workqueue, &led_disable_work);	
    if(!ret)
    {
        printk("[Accdet]disable_micbias:accdet_work return:%d!\n", ret);  		
    }
}

static void open_blacklight()
{
    int ret = 0;
    if(probe_complete == 0)
        return ;

    if(led_state == 1) 
        return;
        
    ret = queue_work(led_workqueue, &led_enable_work);	
    if(!ret)
    {
        printk("[Accdet]disable_micbias:accdet_work return:%d!\n", ret);  		
    }
}


/* the state of cover
* 1: close
* 0: open
*/
void hall_switch_for_kpd(int state){

    if(state == 1){
        del_timer_sync(&timer_kpd);
        close_blacklight();
    }
}

static int aw9523_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "aw9523_probe\n");
	printk("NT: info==>name=%s addr=0x%x\n",client->name,client->addr);
	aw9523_i2c_client  = client;

    led_workqueue = create_singlethread_workqueue("led_workqueue");
    INIT_WORK(&led_disable_work, close_blacklight_callback);
    INIT_WORK(&led_enable_work, open_blacklight_callback);
    
    init_timer(&timer_kpd);
	timer_kpd.expires	= jiffies + KPD_BLACKLIGHT_TIMEOUT * HZ;
	timer_kpd.function	= close_blacklight;
    
    
    kpd_aw9523_eint_workqueue = create_singlethread_workqueue("kpd_aw9523");
    INIT_WORK(&kpd_aw9523_eint_work, kpd_aw9523_handler);

    mt_set_gpio_mode(AW9523_EINT_GPIO, GPIO_MODE_00);//GPIO_MODE_00 GPIO_MHALL_EINT_PIN_M_EINT
    mt_set_gpio_dir(AW9523_EINT_GPIO, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(AW9523_EINT_GPIO, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(AW9523_EINT_GPIO, GPIO_PULL_UP);
    Set_P0_X_AND_P1_Y();

    aw9523_init();
    mt_eint_set_hw_debounce(AW9523_EINT_NO , AW9523_EINT_DEBOUNCE_CN);
    mt_eint_registration(AW9523_EINT_NO , AW9523_EINT_POLARITY, kpd_aw9523_eint_handler, 0);//false
    mt_eint_unmask(AW9523_EINT_NO);

    probe_complete = 1;
	return 0;      
}

static int aw9523_remove(struct i2c_client *client)
{  	
	printk( "aw9523_remove\n");
	aw9523_i2c_client = NULL;
	i2c_unregister_device(client);
    
    del_timer_sync(&timer_kpd);
	return 0;
}

static int aw9523_write_byte(unsigned char addr, unsigned char value)
{	
    int ret = 0;
    struct i2c_client *client = aw9523_i2c_client;
    char write_data[2]={0};	

    if (client == NULL) {
        pr_warn("i2c_client = NULL, skip aw9523_write_bytes\n");
        return 0;
    }
    
    write_data[0] = addr;
    write_data[1] = value;
    
    ret=i2c_master_send(client, write_data, 2);
    if(ret<0)
        printk("tps65132 write data fail !!\n");
    
    return ret ;
}


static u8 aw9523_read_byte(unsigned char addr)
{	
    int ret = 0;
    u8 value;
    struct i2c_client *client = aw9523_i2c_client;
    if (client == NULL) {
        pr_warn("i2c_client = NULL, skip aw9523_write_bytes\n");
        return 0;
    }
    
    ret=i2c_master_send(client, &addr, 1);
    if(ret<0)
        printk("aw9523 write data fail !!\n");


    ret = i2c_master_recv(client, &value, 1);
    if(ret<0)
        printk("aw9523 receive data fail !!\n");
    
    return value ;
}

u8 aw9523_get_p0(void)  
{
    return aw9523_read_byte(0x00);
}

void aw9523_p0_int_restore(void)
{
    aw9523_write_byte(0x06, 0xe0);
}

void aw9523_set_p1(u8 data)
{
    aw9523_write_byte(0x03,data);
}
void aw9523_keylight_open(u8 enable)
{


}

void aw9523_p0_int_disable()
{
    aw9523_write_byte(0x06, 0xff);
}

void aw9523_p0_p1_in_out_setting(void)  
{
    P0_IN_OUT_STATE=0x1F;
    aw9523_write_byte(0x04,P0_IN_OUT_STATE);
    P1_IN_OUT_STATE=0x00;
    aw9523_write_byte(0x05,P1_IN_OUT_STATE);
}

void aw9523_p0_p1_interrupt_setting(void)  
{
    u8 i=0;
    P0_INT_STATE=0x0; //0x0
    for (i=0;i<X_NUM;i++) {
        P0_INT_STATE=P0_INT_STATE|(1<<COL[i]);
    }
    P0_INT_STATE=~P0_INT_STATE;
    aw9523_write_byte(0x06,0xe0);//0x07
    P1_INT_STATE=0xFF;//0xFF
    aw9523_write_byte(0x07,P1_INT_STATE);
}
void Set_P0_X_AND_P1_Y(void) 
{
    u8 i=0;
    u8 temp=0;
    for (i = 0;i < X_NUM;i++) {
        temp = temp|(1<<COL[i]);
   
    }
    for (i=0;i<X_NUM;i++) {
        P0_X[i]=temp&(~(1<<COL[i]));
    }
    temp=0;
    for (i=0;i<Y_NUM;i++) {
        temp=temp|(1<<Line[i]);
    }
    for (i=0;i<Y_NUM;i++) {
        P1_Y[i]=temp&(~(1<<Line[i]));
    }
   	for(i=0;i<8;i++)
	{
		if(P0_kbd_used[i]==1)
		{
			P0_kbd_used_temp|=1<<i;
		}
	}
}

static u8 keyboard_get_press_key(void)
{
    u8 x = 0xFF,y = 0XFF;
    u8 i = 0,j = 0,k = 0;
    u8 get_p0 = 0xff; 
    get_p0 = aw9523_get_p0();
    i = (get_p0)|(~(P0_kbd_used_temp));
    if (i == 0xff) 
    {
        return 0xFF;
    }
    else 
    {
        if(KeyBoardKey_State == KEY_STATE_PRESSED || KeyBoardKey_State == KEY_STATE_LONGPRESS)
        {
            aw9523_write_byte(0x05,P1_Y[pre_y]);
            get_p0=aw9523_get_p0(); 
            if ((get_p0&(1<<COL[pre_x])) == 0)
            {
                aw9523_write_byte(0x05,P1_VALUE); 
                return aw9523_key[pre_y][pre_x];
            }
            else 
            {
                aw9523_write_byte(0x05,P1_VALUE);
                return 0xFF;
            }
        } 
        else 
        {
           for (j=0;j<X_NUM;j++) 
            {
                if((i&(1<<COL[j]))==0)
                {
                    x=j;
                    break;
                }
            }
            if (x==0xFF)
            {
                return 0xFF;
            }
            for (j=0;j<Y_NUM;j++) 
            {
                aw9523_write_byte(0x05,P1_Y[j]);
                get_p0 = aw9523_get_p0(); 
                k=(get_p0)|(~(P0_kbd_used_temp));           
                if ((k&(1<<COL[x]))==0)
                {	
                    y=j;
                    break;
                }
            }
            aw9523_write_byte(0x05,P1_VALUE);
            if (x!=0xFF && y!=0xFF ) 
            {
                pre_x = x;
                pre_y = y;
                return aw9523_key[y][x];
            }
            else 
            {
                return 0xFF;
            }
        }
    }
} 

static void kpd_aw9523_eint_handler(void)
{
    disable_irq(kp_irqnr);
    mt_eint_mask(AW9523_EINT_NO);
    queue_work(kpd_aw9523_eint_workqueue, &kpd_aw9523_eint_work);
}

static void kpd_aw9523_handler(unsigned long data)
{

    if (KeyBoardKey_State == KEY_STATE_NULL||KeyBoardKey_State == KEY_STATE_RELEASED) 
    {
        KeyBoard_Key = keyboard_get_press_key();
        if (KeyBoard_Key != 0xFF) 
        {
            KeyBoardKey_State = KEY_STATE_PRESSED;
            KeyBoard_Key_Previous = KeyBoard_Key;
            input_report_key(kpd_input_dev, KeyBoard_Key, 1);
            input_sync(kpd_input_dev);

            open_blacklight();
            mod_timer(&timer_kpd, jiffies + KPD_BLACKLIGHT_TIMEOUT * HZ);        
        }
    } 
    else if (KeyBoardKey_State == KEY_STATE_PRESSED) 
    {
        KeyBoard_Key = keyboard_get_press_key();
        if (KeyBoard_Key != KeyBoard_Key_Previous) 
        {
            KeyBoardKey_State = KEY_STATE_RELEASED;
            input_report_key(kpd_input_dev, KeyBoard_Key_Previous, 0);
            input_sync(kpd_input_dev);
        }
    }

    aw9523_write_byte(0x05,0x00);//P1_VALUE 0x07
    aw9523_p0_int_restore();  
    aw9523_read_byte(0x00);
    aw9523_read_byte(0x01);
    if (0xf8 != aw9523_read_byte(0x00))
    {
        aw9523_keylight_open(1); 
    }
    else
    {
        aw9523_keylight_open(0); 
    }

    enable_irq(kp_irqnr);	
    mt_eint_unmask(AW9523_EINT_NO);  // mt65xx_eint_unmask(AW9523_EINT_NO);
}





void AW9523_Hw_reset(void)
{   
    printk("kpd AW9523_Hw_reset \n");
    mt_set_gpio_mode(AW9523_RESET_PIN, GPIO_MODE_00); //GPIO143
    mt_set_gpio_dir(AW9523_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(AW9523_RESET_PIN, GPIO_OUT_ZERO); 
    AW9523_delay_1us(200); 
    mt_set_gpio_mode(AW9523_RESET_PIN, GPIO_MODE_00);    
    mt_set_gpio_dir(AW9523_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(AW9523_RESET_PIN, GPIO_OUT_ONE); 
    mdelay(20);
}
void aw9523_init()
{
    printk("aw9523_init\r\n");   
    AW9523_Hw_reset();  
#if 1
    aw9523_write_byte(0x7F, 0x00);   // sw reset
    mdelay(10);
    aw9523_write_byte(0x12, 0x1F);       // P1_0, P1_1 working mode 0xF8
    aw9523_write_byte(0x13, 0X1F);       //FF haizhou add 0xF8
    aw9523_keylight_open(0);        // close P1_0, P1_1 mos fet
    aw9523_p0_int_disable(); 
    aw9523_p0_p1_in_out_setting();
    aw9523_p0_p1_interrupt_setting();
    aw9523_set_p1(P1_VALUE);
    aw9523_read_byte(0x00);
    aw9523_read_byte(0x01);
#endif
}


#endif



/* for slide QWERTY */
#if KPD_HAS_SLIDE_QWERTY
static void kpd_slide_handler(unsigned long data);
static DECLARE_TASKLET(kpd_slide_tasklet, kpd_slide_handler, 0);
static u8 kpd_slide_state = !KPD_SLIDE_POLARITY;
#endif

/* for Power key using EINT */
#if KPD_PWRKEY_USE_EINT
static void kpd_pwrkey_handler(unsigned long data);
static DECLARE_TASKLET(kpd_pwrkey_tasklet, kpd_pwrkey_handler, 0);
#endif

/* for keymap handling */
static void kpd_keymap_handler(unsigned long data);
static DECLARE_TASKLET(kpd_keymap_tasklet, kpd_keymap_handler, 0);

/*********************************************************************/
static void kpd_memory_setting(void);

/*********************************************************************/
static int kpd_pdrv_probe(struct platform_device *pdev);
static int kpd_pdrv_remove(struct platform_device *pdev);
#ifndef USE_EARLY_SUSPEND
static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int kpd_pdrv_resume(struct platform_device *pdev);
#endif

#ifdef CONFIG_OF
static const struct of_device_id kpd_of_match[] = {
	{ .compatible = "mediatek,KP", },
	{},
};
#endif

static struct platform_driver kpd_pdrv = {
	.probe = kpd_pdrv_probe,
	.remove = kpd_pdrv_remove,
#ifndef USE_EARLY_SUSPEND
	.suspend = kpd_pdrv_suspend,
	.resume = kpd_pdrv_resume,
#endif
	.driver = {
		.name = KPD_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = kpd_of_match,
#endif
	},
};

/********************************************************************/
static void kpd_memory_setting(void)
{
	kpd_init_keymap(kpd_keymap);
	kpd_init_keymap_state(kpd_keymap_state);
	return;
}


/*****************for kpd auto set wake up source*************************/

static ssize_t kpd_store_call_state(struct device_driver *ddri, const char *buf, size_t count)
{
	if (sscanf(buf, "%u", &call_status) != 1) {
		kpd_print("kpd call state: Invalid values\n");
		return -EINVAL;
	}

	switch (call_status) {
	case 1:
		kpd_print("kpd call state: Idle state!\n");
		break;
	case 2:
		kpd_print("kpd call state: ringing state!\n");
		break;
	case 3:
		kpd_print("kpd call state: active or hold state!\n");
		break;

	default:
		kpd_print("kpd call state: Invalid values\n");
		break;
	}
	return count;
}

static ssize_t kpd_show_call_state(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	res = snprintf(buf, PAGE_SIZE, "%d\n", call_status);
	return res;
}

static DRIVER_ATTR(kpd_call_state, S_IWUSR | S_IRUGO, kpd_show_call_state, kpd_store_call_state);

static struct driver_attribute *kpd_attr_list[] = {
	&driver_attr_kpd_call_state,
};

/*----------------------------------------------------------------------------*/
static int kpd_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(kpd_attr_list) / sizeof(kpd_attr_list[0]));
	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		if ((err = driver_create_file(driver, kpd_attr_list[idx]))) {
			kpd_print("driver_create_file (%s) = %d\n", kpd_attr_list[idx]->attr.name,
				  err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int kpd_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(kpd_attr_list) / sizeof(kpd_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, kpd_attr_list[idx]);
	}

	return err;
}

/*----------------------------------------------------------------------------*/
/********************************************************************************************/
/************************************************************************************************************************************************/
/* for autotest */
#if KPD_AUTOTEST
static const u16 kpd_auto_keymap[] = {
	KEY_MENU,
	KEY_HOME, KEY_BACK,
	KEY_CALL, KEY_ENDCALL,
	KEY_VOLUMEUP, KEY_VOLUMEDOWN,
	KEY_FOCUS, KEY_CAMERA,
};
#endif
/* for AEE manual dump */
#define AEE_VOLUMEUP_BIT	0
#define AEE_VOLUMEDOWN_BIT	1
#define AEE_DELAY_TIME		15
/* enable volup + voldown was pressed 5~15 s Trigger aee manual dump */
#define AEE_ENABLE_5_15		1
static struct hrtimer aee_timer;
static unsigned long aee_pressed_keys;
static bool aee_timer_started;

#if AEE_ENABLE_5_15
#define AEE_DELAY_TIME_5S	5
static struct hrtimer aee_timer_5s;
static bool aee_timer_5s_started;
static bool flags_5s;
#endif

static inline void kpd_update_aee_state(void)
{
	if (aee_pressed_keys == ((1 << AEE_VOLUMEUP_BIT) | (1 << AEE_VOLUMEDOWN_BIT))) {
		/* if volumeup and volumedown was pressed the same time then start the time of ten seconds */
		aee_timer_started = true;

#if AEE_ENABLE_5_15
		aee_timer_5s_started = true;
		hrtimer_start(&aee_timer_5s, ktime_set(AEE_DELAY_TIME_5S, 0), HRTIMER_MODE_REL);
#endif
		hrtimer_start(&aee_timer, ktime_set(AEE_DELAY_TIME, 0), HRTIMER_MODE_REL);
		kpd_print("aee_timer started\n");
	} else {
		if (aee_timer_started) {
/*
  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
  * Returns:
  *	0 when the timer was not active.
  *	1 when the timer was active.
 */
			if (hrtimer_cancel(&aee_timer)) {
				kpd_print("try to cancel hrtimer\n");
#if AEE_ENABLE_5_15
				if (flags_5s) {
					printk
					    ("Pressed Volup + Voldown5s~15s then trigger aee manual dump.\n");
					aee_kernel_reminding("manual dump",
							     "Trigger Vol Up +Vol Down 5s");
				}
#endif

			}
#if AEE_ENABLE_5_15
			flags_5s = false;
#endif
			aee_timer_started = false;
			kpd_print("aee_timer canceled\n");
		}
#if AEE_ENABLE_5_15
		if (aee_timer_5s_started) {
/*
  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
  * Returns:
  *	0 when the timer was not active.
  *	1 when the timer was active.
 */
			if (hrtimer_cancel(&aee_timer_5s)) {
				kpd_print("try to cancel hrtimer (5s)\n");
			}
			aee_timer_5s_started = false;
			kpd_print("aee_timer canceled (5s)\n");
		}
#endif
	}
}

static void kpd_aee_handler(u32 keycode, u16 pressed)
{
	if (pressed) {
		if (keycode == KEY_VOLUMEUP) {
			__set_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		} else if (keycode == KEY_VOLUMEDOWN) {
			__set_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		} else {
			return;
		}
		kpd_update_aee_state();
	} else {
		if (keycode == KEY_VOLUMEUP) {
			__clear_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		} else if (keycode == KEY_VOLUMEDOWN) {
			__clear_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		} else {
			return;
		}
		kpd_update_aee_state();
	}
}

static enum hrtimer_restart aee_timer_func(struct hrtimer *timer)
{
	/* printk("kpd: vol up+vol down AEE manual dump!\n"); */
	/* aee_kernel_reminding("manual dump ", "Triggered by press KEY_VOLUMEUP+KEY_VOLUMEDOWN"); */
	aee_trigger_kdb();
	return HRTIMER_NORESTART;
}

#if AEE_ENABLE_5_15
static enum hrtimer_restart aee_timer_5s_func(struct hrtimer *timer)
{

	/* printk("kpd: vol up+vol down AEE manual dump timer 5s !\n"); */
	flags_5s = true;
	return HRTIMER_NORESTART;
}
#endif

/************************************************************************************************************************************************/

#if KPD_HAS_SLIDE_QWERTY
static void kpd_slide_handler(unsigned long data)
{
	bool slid;
	u8 old_state = kpd_slide_state;

	kpd_slide_state = !kpd_slide_state;
	slid = (kpd_slide_state == !!KPD_SLIDE_POLARITY);
	/* for SW_LID, 1: lid open => slid, 0: lid shut => closed */
	input_report_switch(kpd_input_dev, SW_LID, slid);
	input_sync(kpd_input_dev);
	kpd_print("report QWERTY = %s\n", slid ? "slid" : "closed");

	if (old_state) {
		mt_set_gpio_pull_select(GPIO_QWERTYSLIDE_EINT_PIN, 0);
	} else {
		mt_set_gpio_pull_select(GPIO_QWERTYSLIDE_EINT_PIN, 1);
	}
	/* for detecting the return to old_state */
	mt65xx_eint_set_polarity(KPD_SLIDE_EINT, old_state);
	mt65xx_eint_unmask(KPD_SLIDE_EINT);
}

static void kpd_slide_eint_handler(void)
{
	tasklet_schedule(&kpd_slide_tasklet);
}
#endif

#if KPD_PWRKEY_USE_EINT
static void kpd_pwrkey_handler(unsigned long data)
{
	kpd_pwrkey_handler_hal(data);
}

static void kpd_pwrkey_eint_handler(void)
{
	tasklet_schedule(&kpd_pwrkey_tasklet);
}
#endif
/*********************************************************************/

/*********************************************************************/
#if KPD_PWRKEY_USE_PMIC
void kpd_pwrkey_pmic_handler(unsigned long pressed)
{
	printk(KPD_SAY "Power Key generate, pressed=%ld\n", pressed);
	if (!kpd_input_dev) {
		printk("KPD input device not ready\n");
		return;
	}
	kpd_pmic_pwrkey_hal(pressed);
}
#endif


void kpd_pmic_rstkey_handler(unsigned long pressed)
{
	printk(KPD_SAY "PMIC reset Key generate, pressed=%ld\n", pressed);
	if (!kpd_input_dev) {
		printk("KPD input device not ready\n");
		return;
	}
	kpd_pmic_rstkey_hal(pressed);
#ifdef KPD_PMIC_RSTKEY_MAP
	kpd_aee_handler(KPD_PMIC_RSTKEY_MAP, pressed);
#endif
}

/*********************************************************************/

/*********************************************************************/
static void kpd_keymap_handler(unsigned long data)
{
	int i, j;
	bool pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, linux_keycode;
	kpd_get_keymap_state(new_state);

#ifdef CONFIG_MTK_TC1_FM_AT_SUSPEND
	wake_lock_timeout(&kpd_suspend_lock, HZ / 2);
#endif

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ kpd_keymap_state[i];
		if (!change)
			continue;

		for (j = 0; j < 16; j++) {
			mask = 1U << j;
			if (!(change & mask))
				continue;

			hw_keycode = (i << 4) + j;
			/* bit is 1: not pressed, 0: pressed */
			pressed = !(new_state[i] & mask);
			if (kpd_show_hw_keycode) {
				printk(KPD_SAY "(%s) HW keycode = %u\n",
				       pressed ? "pressed" : "released", hw_keycode);
			}
			BUG_ON(hw_keycode >= KPD_NUM_KEYS);
			linux_keycode = kpd_keymap[hw_keycode];
			if (unlikely(linux_keycode == 0)) {
				kpd_print("Linux keycode = 0\n");
				continue;
			}
			kpd_aee_handler(linux_keycode, pressed);

			kpd_backlight_handler(pressed, linux_keycode);
			input_report_key(kpd_input_dev, linux_keycode, pressed);
			input_sync(kpd_input_dev);
			kpd_print("report Linux keycode = %u\n", linux_keycode);
		}
	}

	memcpy(kpd_keymap_state, new_state, sizeof(new_state));
	kpd_print("save new keymap state\n");
#ifdef CONFIG_OF
	enable_irq(kp_irqnr);
#else
	enable_irq(MT_KP_IRQ_ID);
#endif
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
#ifdef CONFIG_OF
	disable_irq_nosync(kp_irqnr);
#else
	disable_irq_nosync(MT_KP_IRQ_ID);
#endif
	tasklet_schedule(&kpd_keymap_tasklet);
	return IRQ_HANDLED;
}

/*********************************************************************/

/*****************************************************************************************/
long kpd_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* void __user *uarg = (void __user *)arg; */

	switch (cmd) {
#if KPD_AUTOTEST
	case PRESS_OK_KEY:	/* KPD_AUTOTEST disable auto test setting to resolve CR ALPS00464496 */
		if (test_bit(KEY_OK, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS OK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_OK, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support OK KEY!!\n");
		}
		break;
	case RELEASE_OK_KEY:
		if (test_bit(KEY_OK, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE OK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_OK, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support OK KEY!!\n");
		}
		break;
	case PRESS_MENU_KEY:
		if (test_bit(KEY_MENU, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS MENU KEY!!\n");
			input_report_key(kpd_input_dev, KEY_MENU, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support MENU KEY!!\n");
		}
		break;
	case RELEASE_MENU_KEY:
		if (test_bit(KEY_MENU, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE MENU KEY!!\n");
			input_report_key(kpd_input_dev, KEY_MENU, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support MENU KEY!!\n");
		}

		break;
	case PRESS_UP_KEY:
		if (test_bit(KEY_UP, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS UP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_UP, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support UP KEY!!\n");
		}
		break;
	case RELEASE_UP_KEY:
		if (test_bit(KEY_UP, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE UP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_UP, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support UP KEY!!\n");
		}
		break;
	case PRESS_DOWN_KEY:
		if (test_bit(KEY_DOWN, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS DOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_DOWN, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support DOWN KEY!!\n");
		}
		break;
	case RELEASE_DOWN_KEY:
		if (test_bit(KEY_DOWN, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE DOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_DOWN, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support DOWN KEY!!\n");
		}
		break;
	case PRESS_LEFT_KEY:
		if (test_bit(KEY_LEFT, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS LEFT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_LEFT, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support LEFT KEY!!\n");
		}
		break;
	case RELEASE_LEFT_KEY:
		if (test_bit(KEY_LEFT, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE LEFT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_LEFT, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support LEFT KEY!!\n");
		}
		break;

	case PRESS_RIGHT_KEY:
		if (test_bit(KEY_RIGHT, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS RIGHT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_RIGHT, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support RIGHT KEY!!\n");
		}
		break;
	case RELEASE_RIGHT_KEY:
		if (test_bit(KEY_RIGHT, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE RIGHT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_RIGHT, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support RIGHT KEY!!\n");
		}
		break;
	case PRESS_HOME_KEY:
		if (test_bit(KEY_HOME, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS HOME KEY!!\n");
			input_report_key(kpd_input_dev, KEY_HOME, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support HOME KEY!!\n");
		}
		break;
	case RELEASE_HOME_KEY:
		if (test_bit(KEY_HOME, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE HOME KEY!!\n");
			input_report_key(kpd_input_dev, KEY_HOME, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support HOME KEY!!\n");
		}
		break;
	case PRESS_BACK_KEY:
		if (test_bit(KEY_BACK, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS BACK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_BACK, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support BACK KEY!!\n");
		}
		break;
	case RELEASE_BACK_KEY:
		if (test_bit(KEY_BACK, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE BACK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_BACK, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support BACK KEY!!\n");
		}
		break;
	case PRESS_CALL_KEY:
		if (test_bit(KEY_CALL, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS CALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CALL, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support CALL KEY!!\n");
		}
		break;
	case RELEASE_CALL_KEY:
		if (test_bit(KEY_CALL, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE CALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CALL, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support CALL KEY!!\n");
		}
		break;

	case PRESS_ENDCALL_KEY:
		if (test_bit(KEY_ENDCALL, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS ENDCALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_ENDCALL, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support ENDCALL KEY!!\n");
		}
		break;
	case RELEASE_ENDCALL_KEY:
		if (test_bit(KEY_ENDCALL, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE ENDCALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_ENDCALL, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support ENDCALL KEY!!\n");
		}
		break;
	case PRESS_VLUP_KEY:
		if (test_bit(KEY_VOLUMEUP, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS VOLUMEUP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEUP, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support VOLUMEUP KEY!!\n");
		}
		break;
	case RELEASE_VLUP_KEY:
		if (test_bit(KEY_VOLUMEUP, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE VOLUMEUP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEUP, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support VOLUMEUP KEY!!\n");
		}
		break;
	case PRESS_VLDOWN_KEY:
		if (test_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS VOLUMEDOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support VOLUMEDOWN KEY!!\n");
		}
		break;
	case RELEASE_VLDOWN_KEY:
		if (test_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE VOLUMEDOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support VOLUMEDOWN KEY!!\n");
		}
		break;
	case PRESS_FOCUS_KEY:
		if (test_bit(KEY_FOCUS, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS FOCUS KEY!!\n");
			input_report_key(kpd_input_dev, KEY_FOCUS, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support FOCUS KEY!!\n");
		}
		break;
	case RELEASE_FOCUS_KEY:
		if (test_bit(KEY_FOCUS, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE FOCUS KEY!!\n");
			input_report_key(kpd_input_dev, KEY_FOCUS, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support RELEASE KEY!!\n");
		}
		break;
	case PRESS_CAMERA_KEY:
		if (test_bit(KEY_CAMERA, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS CAMERA KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CAMERA, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support CAMERA KEY!!\n");
		}
		break;
	case RELEASE_CAMERA_KEY:
		if (test_bit(KEY_CAMERA, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE CAMERA KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CAMERA, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support CAMERA KEY!!\n");
		}
		break;
	case PRESS_POWER_KEY:
		if (test_bit(KEY_POWER, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] PRESS POWER KEY!!\n");
			input_report_key(kpd_input_dev, KEY_POWER, 1);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support POWER KEY!!\n");
		}
		break;
	case RELEASE_POWER_KEY:
		if (test_bit(KEY_POWER, kpd_input_dev->keybit)) {
			printk("[AUTOTEST] RELEASE POWER KEY!!\n");
			input_report_key(kpd_input_dev, KEY_POWER, 0);
			input_sync(kpd_input_dev);
		} else {
			printk("[AUTOTEST] Not Support POWER KEY!!\n");
		}
		break;
#endif

	case SET_KPD_KCOL:
		kpd_auto_test_for_factorymode();	/* API 3 for kpd factory mode auto-test */
		printk("[kpd_auto_test_for_factorymode] test performed!!\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


int kpd_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations kpd_dev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = kpd_dev_ioctl,
	.open = kpd_dev_open,
};

/*********************************************************************/
static struct miscdevice kpd_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = KPD_NAME,
	.fops = &kpd_dev_fops,
};

static int kpd_open(struct input_dev *dev)
{
	kpd_slide_qwerty_init();	/* API 1 for kpd slide qwerty init settings */
	return 0;
}


static int kpd_pdrv_probe(struct platform_device *pdev)
{

	int i,j,r;
	int err = 0;

#ifdef CONFIG_OF
	kp_base = of_iomap(pdev->dev.of_node, 0);
	if (!kp_base) {
		pr_warn(KPD_SAY "KP iomap failed\n");
		return -ENODEV;
	};

	kp_irqnr = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!kp_irqnr) {
		pr_warn(KPD_SAY "KP get irqnr failed\n");
		return -ENODEV;
	}
	pr_warn(KPD_SAY "kp base: 0x%p, addr:0x%p,  kp irq: %d\n", kp_base,&kp_base, kp_irqnr);
#endif

	kpd_ldvt_test_init();	/* API 2 for kpd LFVT test enviroment settings */

	/* initialize and register input device (/dev/input/eventX) */
	kpd_input_dev = input_allocate_device();
	if (!kpd_input_dev)
		return -ENOMEM;

	kpd_input_dev->name = KPD_NAME;
	kpd_input_dev->id.bustype = BUS_HOST;
	kpd_input_dev->id.vendor = 0x2454;
	kpd_input_dev->id.product = 0x6500;
	kpd_input_dev->id.version = 0x0010;
	kpd_input_dev->open = kpd_open;

	/* fulfill custom settings */
	kpd_memory_setting();

	__set_bit(EV_KEY, kpd_input_dev->evbit);

#if (KPD_PWRKEY_USE_EINT || KPD_PWRKEY_USE_PMIC)
	__set_bit(KPD_PWRKEY_MAP, kpd_input_dev->keybit);
	kpd_keymap[8] = 0;
#endif

#if !KPD_USE_EXTEND_TYPE
	for (i = 17; i < KPD_NUM_KEYS; i += 9)	/* only [8] works for Power key */
		kpd_keymap[i] = 0;
#endif

	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (kpd_keymap[i] != 0)
			__set_bit(kpd_keymap[i], kpd_input_dev->keybit);
	}

#if KPD_AUTOTEST
	for (i = 0; i < ARRAY_SIZE(kpd_auto_keymap); i++)
		__set_bit(kpd_auto_keymap[i], kpd_input_dev->keybit);
#endif

#if KPD_HAS_SLIDE_QWERTY
	__set_bit(EV_SW, kpd_input_dev->evbit);
	__set_bit(SW_LID, kpd_input_dev->swbit);
#endif

#ifdef KPD_PMIC_RSTKEY_MAP
	__set_bit(KPD_PMIC_RSTKEY_MAP, kpd_input_dev->keybit);
#endif

#ifdef KPD_KEY_MAP
		__set_bit(KPD_KEY_MAP, kpd_input_dev->keybit);
#endif


#if KPD_HAS_EXTEND_KEYPAD
for (i = 0; i < 5 ; i++)
{
    for(j = 0; j < 5 ; j++)
		__set_bit(aw9523_key[i][j], kpd_input_dev->keybit);
}
#endif



	kpd_input_dev->dev.parent = &pdev->dev;
	r = input_register_device(kpd_input_dev);
	if (r) {
		printk(KPD_SAY "register input device failed (%d)\n", r);
		input_free_device(kpd_input_dev);
		return r;
	}

	/* register device (/dev/mt6575-kpd) */
	kpd_dev.parent = &pdev->dev;
	r = misc_register(&kpd_dev);
	if (r) {
		printk(KPD_SAY "register device failed (%d)\n", r);
		input_unregister_device(kpd_input_dev);
		return r;
	}

#ifdef CONFIG_MTK_TC1_FM_AT_SUSPEND
	wake_lock_init(&kpd_suspend_lock, WAKE_LOCK_SUSPEND, "kpd wakelock");
#endif

	/* register IRQ and EINT */
	kpd_set_debounce(KPD_KEY_DEBOUNCE);
#ifdef CONFIG_OF
	r = request_irq(kp_irqnr, kpd_irq_handler, IRQF_TRIGGER_NONE, KPD_NAME, NULL);
#else
	r = request_irq(MT_KP_IRQ_ID, kpd_irq_handler, IRQF_TRIGGER_FALLING, KPD_NAME, NULL);
#endif
	if (r) {
		printk(KPD_SAY "register IRQ failed (%d)\n", r);
		misc_deregister(&kpd_dev);
		input_unregister_device(kpd_input_dev);
		return r;
	}
	mt_eint_register();

#ifndef KPD_EARLY_PORTING	/*add for avoid early porting build err the macro is defined in custom file */
	long_press_reboot_function_setting();	/* /API 4 for kpd long press reboot function setting */
#endif
	hrtimer_init(&aee_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer.function = aee_timer_func;

#if AEE_ENABLE_5_15
	hrtimer_init(&aee_timer_5s, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer_5s.function = aee_timer_5s_func;
#endif

	if ((err = kpd_create_attr(&kpd_pdrv.driver))) {
		kpd_print("create attr file fail\n");
		kpd_delete_attr(&kpd_pdrv.driver);
		return err;
	}
    pr_warn(KPD_SAY "%s Done\n", __FUNCTION__);
	return 0;
}

/* should never be called */
static int kpd_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	kpd_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n", kpd_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		kpd_print("kpd_early_suspend wake up source disable!! (%d)\n", kpd_suspend);
	}
#endif
	kpd_disable_backlight();
	kpd_print("suspend!! (%d)\n", kpd_suspend);
	return 0;
}

static int kpd_pdrv_resume(struct platform_device *pdev)
{
	kpd_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n", kpd_suspend);
	} else {
		kpd_print("kpd_early_suspend wake up source resume!! (%d)\n", kpd_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	kpd_print("resume!! (%d)\n", kpd_suspend);
	return 0;
}
#else
#define kpd_pdrv_suspend	NULL
#define kpd_pdrv_resume		NULL
#endif


#ifdef USE_EARLY_SUSPEND
static void kpd_early_suspend(struct early_suspend *h)
{
	kpd_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n", kpd_suspend);
	} else {
		/* kpd_wakeup_src_setting(0); */
		kpd_print("kpd_early_suspend wake up source disable!! (%d)\n", kpd_suspend);
	}
#endif
	kpd_disable_backlight();
	kpd_print("early suspend!! (%d)\n", kpd_suspend);
}

static void kpd_early_resume(struct early_suspend *h)
{
	kpd_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_early_resume wake up source resume!! (%d)\n", kpd_suspend);
	} else {
		kpd_print("kpd_early_resume wake up source enable!! (%d)\n", kpd_suspend);
		/* kpd_wakeup_src_setting(1); */
	}
#endif
	kpd_print("early resume!! (%d)\n", kpd_suspend);
}

static struct early_suspend kpd_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = kpd_early_suspend,
	.resume = kpd_early_resume,
};
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
static struct sb_handler kpd_sb_handler_desc = {
	.level = SB_LEVEL_DISABLE_KEYPAD,
	.plug_in = sb_kpd_enable,
	.plug_out = sb_kpd_disable,
};
#endif
#endif

static int __init kpd_mod_init(void)
{
	int r;

	r = platform_driver_register(&kpd_pdrv);
	if (r) {
		printk(KPD_SAY "register driver failed (%d)\n", r);
		return r;
	}
#ifdef USE_EARLY_SUSPEND
	register_early_suspend(&kpd_early_suspend_desc);
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&kpd_sb_handler_desc);
#endif
#endif

#if KPD_HAS_EXTEND_KEYPAD

   i2c_register_board_info(EXTEND_KEYPAD_I2C_BUSNUM, &aw9523_board_info, 1);
   printk( "tps65132_iic_init2\n");
   i2c_add_driver(&aw9523_iic_driver);
   printk( "tps65132_iic_init success\n");	

#endif

	return 0;
}

/* should never be called */
static void __exit kpd_mod_exit(void)
{

#if KPD_HAS_EXTEND_KEYPAD
    printk( "kpd_mod_exit tps65132_iic_exit\n");
    i2c_del_driver(&aw9523_iic_driver);  
#endif
}
module_init(kpd_mod_init);
module_exit(kpd_mod_exit);

module_param(kpd_show_hw_keycode, int, 0644);
module_param(kpd_show_register, int, 0644);

MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver v0.4");
MODULE_LICENSE("GPL");
