#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>

#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       
//#ifndef TRUE
//    #define TRUE 1
//#endif

//#ifndef FALSE
//    #define FALSE 0
//#endif

//unsigned int lcm_esd_test = FALSE;		///only for ESD test


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void init_lcm_registers(void)
{
	unsigned int data_array[16];


data_array[0]=0x00043902;
data_array[1]=0x9983FFB9;
dsi_set_cmdq(data_array, 2, 1);


//data_array[0]=0x00023902;
//data_array[1]=0x000011d2;
//dsi_set_cmdq(data_array, 2, 1);


data_array[0]=0x000d3902;
data_array[1]=0x2C7400B1;
data_array[2]=0x2209882C;
data_array[3]=0xA5F17122;
data_array[4]=0x0000006F;
dsi_set_cmdq(data_array, 5, 1);


data_array[0]=0x000B3902;
data_array[1]=0x008000B2;
data_array[2]=0x2307057F;
data_array[3]=0x0002024D;
dsi_set_cmdq(data_array, 4, 1);


data_array[0]=0x00293902;
data_array[1]=0x004200b4;
data_array[2]=0x003A0040;
data_array[3]=0x00000200;
data_array[4]=0x01100001;
data_array[5]=0x00310402;
data_array[6]=0x40004001;
data_array[7]=0x00003A00;
data_array[8]=0x01000002;
data_array[9]=0x02011000;
data_array[10]=0x01000104;
data_array[11]=0x00000040;
dsi_set_cmdq(data_array, 12, 1);


data_array[0]=0x00203902;
data_array[1]=0x000400d3;
data_array[2]=0x00060000;
data_array[3]=0x00051032;
data_array[4]=0x00000005;
data_array[5]=0x00000000;
data_array[6]=0x01000000;
data_array[7]=0x00030505;	
data_array[8]=0x08050000;
dsi_set_cmdq(data_array, 9, 1);

	

data_array[0]=0x00213902;
data_array[1]=0x001818d5;
data_array[2]=0x00000000;
data_array[3]=0x18191900;
data_array[4]=0x00000018;
data_array[5]=0x00000000;
data_array[6]=0x03000100;
data_array[7]=0x30212002;	
data_array[8]=0x32313130;
data_array[9]=0x00000032;
dsi_set_cmdq(data_array, 10, 1);


data_array[0]=0x00213902;
data_array[1]=0x401818d6;
data_array[2]=0x40404040;
data_array[3]=0x19181840;
data_array[4]=0x40404019;
data_array[5]=0x40404040;
data_array[6]=0x00030240;
data_array[7]=0x30202101;	
data_array[8]=0x32313130;
data_array[9]=0x00000032;
dsi_set_cmdq(data_array, 10, 1);

	data_array[0]=0x00313902;
	data_array[1]=0x000000D8;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	data_array[5]=0x00000000;
	data_array[6]=0x00000000;
	data_array[7]=0x00000000;	
	data_array[8]=0x00000000;
	data_array[9]=0x00000000;
	data_array[10]=0x00000000;	
	data_array[11]=0x0A00C000;
	data_array[12]=0x0A00C0BF;
	data_array[13]=0x000000BF;
	dsi_set_cmdq(data_array, 14, 1);


	data_array[0]=0x002b3902;
	data_array[1]=0x131008E0;
	data_array[2]=0x1C3f2E28;
	data_array[3]=0x0F0C0737;
	data_array[4]=0x14131613;
	data_array[5]=0x170A1711;
	data_array[6]=0x10081207;
	data_array[7]=0x3F2E2813;	
	data_array[8]=0x0C07371C;
	data_array[9]=0x1316130F;
	data_array[10]=0x0A171114;
	data_array[11]=0x00120717;
	dsi_set_cmdq(data_array, 12, 1);

	
data_array[0]=0x00033902;
data_array[1]=0x002424b6;
dsi_set_cmdq(data_array, 2, 1);
	

data_array[0]=0x00023902;
data_array[1]=0x000008cc;
dsi_set_cmdq(data_array, 2, 1);



data_array[0]= 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(200);

data_array[0]= 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(10);


}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 86;
	params->dsi.horizontal_backporch				= 55;
	params->dsi.horizontal_frontporch				= 55;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

//#ifndef CONFIG_FPGA_EARLY_PORTING
//#if (LCM_DSI_CMD_MODE)
//	params->dsi.PLL_CLOCK = 350; //this value must be in MTK suggested table
//#else
	params->dsi.PLL_CLOCK = 460; //this value must be in MTK suggested table
//#endif
//#else
//	params->dsi.pll_div1 = 0;
//	params->dsi.pll_div2 = 0;
//	params->dsi.fbk_div = 0x1;
//#endif

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd			= 0x0a;
	params->dsi.lcm_esd_check_table[0].count		= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
//	params->dsi.vertical_vfp_lp = 100;

}


static void lcm_init(void)
{
//    SET_RESET_PIN(1);
//    SET_RESET_PIN(0);
//    MDELAY(10);
//    SET_RESET_PIN(1);
//    MDELAY(20);

//	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);	
	init_lcm_registers();

//    SET_RESET_PIN(1);
//	MDELAY(10);
//    SET_RESET_PIN(0);
//    MDELAY(20);
//    SET_RESET_PIN(1);
//    MDELAY(120);	
//	init_lcm_registers();	
}


static void lcm_suspend(void)
{
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(120);


}


static void lcm_resume(void)
{
	lcm_init();
}



static unsigned int lcm_compare_id(void)
{
	char  buffer,lcd_id;
	unsigned int data_array[2];

//	return 1;

    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(120);
    SET_RESET_PIN(1);
    MDELAY(50);	



	data_array[0]= 0x00043902;
	data_array[1]= (0x99<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00043902;
	data_array[1]= (0x99<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xf4, &buffer, 1);

#ifdef BUILD_LK
		printf("%s, LK debug: hx8399d guanjie id = 0x%08x\n", __func__, buffer);
#else
		printk("%s, kernel debug: hx8399d guanjie id = 0x%08x\n", __func__, buffer);
#endif

	mt_set_gpio_mode(GPIO_LCD_ID_PIN, GPIO_LCD_ID_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_LCD_ID_PIN, GPIO_DIR_IN);	  
	mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN, GPIO_PULL_DISABLE);
	
	MDELAY(50);

	lcd_id = mt_get_gpio_in(GPIO_LCD_ID_PIN);

#ifdef BUILD_LK
	printf("%s, LK debug: aeon_factory id = %d\n", __func__, lcd_id);
#else
	printk("%s, kernel debug: aeon_factory id = %d\n", __func__, lcd_id);
#endif


	if(buffer == 0x99)
	{
		if(lcd_id == 1) 	
		{	
			return 1;
		}		
		else 
		{	
			return 0;
		}	
	}
	else
	{
		return 0;
	}

//	return (buffer == 0x94 ? 1 : 0);

}




LCM_DRIVER hx8399_fhd_dsi_vdo_e550_guanjie_lcm_drv = 
{
    .name			= "hx8399_fhd_dsi_vdo_e550_guanjie",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,

};
