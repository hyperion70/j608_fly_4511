#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//RGK add
// ---------------------------------------------------------------------------
#include <cust_adc.h>    	// zhoulidong  add for lcm detect
#define MIN_VOLTAGE (210)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (400)     // zhoulidong  add for lcm detect

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

#define LCM_ID_HX8389B 0x89


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

 unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static LCM_setting_table_V3 lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0xB9,3,{0xFF,0x83,0x89}},
	
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},
	//{0x39,0xBC,2,{0x03,0x00}},
//------------ HX5186 set power-------------------------------//

	{0xBA,2,{0x41,0x93}},
		
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},
//------------------------------------------------------
 	//{0x39,0xBC,2,{0x14,0x00}},

 	{0xB1,19,{0x00,0x00,0x06,0xe8,0x59,0x10,0x11,0xd3,0xEF,
		       0x39,0x41,0x3F,0x3F,0x43,0x01,0x5A,0xF2,0x00,0xE6}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},

 	//{0x39,0xBC,2,{0x08,0x00}},		
 		
 	{0xB2,7,{0x00,0x00,0x78,0x0C,0x07,0x3F,0x80}},		
 	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},	     
 	//{0x39,0xBC,2,{0x18,0x00}}, 	              	     
 	{0xB4,23,{0x80,0x08,0x00,0x32,0x10,0x04,0x32,0x10,0x00,
                       0x32,0x10,0x00,0x37,0x0A,0x40,0x08,0x37,0x02,0x40,
                       0x14,0x50,0x58,0x0A}},

 	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},            
 	//{0x39,0xBC,2,{0x39,0x00}}, 	             
 	{0xD5,56,{0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x60,
                       0x00,0x99,0x88,0xAA,0xBB,0x88,0x23,0x88,0x01,0x88,
                       0x67,0x88,0x45,0x01,0x23,0x88,0x88,0x88,0x88,0x88,
                       0x88,0x99,0xBB,0xAA,0x88,0x54,0x88,0x76,0x88,0x10,
                       0x88,0x32,0x32,0x10,0x88,0x88,0x88,0x88,0x88,0x00,
                       0x04,0x00,0x00,0x00,0x00,0x00,0x00}},
 	//{0x39,0xBC,2,{0x04,0x00}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},
		
 
 	{0xB7,3,{0x00,0x00,0x50}},
		
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},
	   
 	//{0x39,0xBC,2,{0x05,0x00}},
	
	   
 	{0xB6,4,{0x00,0x97,0x00,0x97}},

{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},
	//{0x39,0xBC,2,{0x23,0x00}},
	{0xE0,34,{0x00,0x0F,0x1E,0x2E,0x34,0x3F,0x28,0x3F,0x07,
                       0x0D,0x0F,0x13,0x14,0x12,0x13,0x10,0x17,0x00,0x0F,
                       0x1E,0x2E,0x34,0x3F,0x28,0x3F,0x07,0x0D,0x0F,0x13,
                       0x14,0x12,0x13,0x10,0x17}},
	//{0x39,0xBC,2,{0x02,0x00}},
	
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},
	{0xCC,1,{0x02}},		
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},

	{0x11, 1 ,{0x00}}, 
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 150, {}},

	{0x29, 1 ,{0x00}}, 
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 150, {}},




};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 0x02;// 3    2
		params->dsi.vertical_backporch					= 0x0e;// 20   1
		params->dsi.vertical_frontporch					= 0x09; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 0x60;// 50  2
		params->dsi.horizontal_backporch				= 0x60;
		params->dsi.horizontal_frontporch				= 0x60;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=200;
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
		params->dsi.fbk_div =9;
#else
		params->dsi.fbk_div =9;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
}

static void lcm_init(void)
{
unsigned int data_array[16];
		SET_RESET_PIN(1);
		MDELAY(20); 
		SET_RESET_PIN(0);
		MDELAY(50); 
		SET_RESET_PIN(1);
		MDELAY(20); 
#if 1
data_array[0] = 0x00043902;
	data_array[1] = 0x8983FFB9;//0Xf0 0x55, 0xAA, 0x52
	dsi_set_cmdq(data_array, 2, 1);
/*
	data_array[0] = 0x12B01500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12B11500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00B21500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07B31500;dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x14B61500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14B71500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x24B81500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x34B91500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14BA1500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01BF1500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04C31500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00C21500;dsi_set_cmdq(data_array, 1, 1);
	
*/
	data_array[0] = 0x00033902;
	data_array[1] = 0x009341BA;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00143902;
	data_array[1] = 0x060000B1;
		data_array[2] = 0x111059E8;
			data_array[3] = 0x4139EFD3;
				data_array[4] = 0x01433F3F;
				data_array[5] = 0xE600F25A;

	dsi_set_cmdq(data_array, 6, 1);
	
		data_array[0] = 0x00083902;
	data_array[1] = 0x780000B2;
		data_array[2] = 0x803F070C;
	dsi_set_cmdq(data_array, 3, 1);
	
	
	data_array[0] = 0x00183902;
	data_array[1] = 0x000880B4;
		data_array[2] = 0x32041032;
			data_array[3] = 0x10320010;
	data_array[4] = 0x400A3700;	
		data_array[5] = 0x40023708;	
	data_array[6] = 0x0A585014;
	
	dsi_set_cmdq(data_array, 7, 1);
	
	data_array[0] = 0x00393902;
	data_array[1] = 0x000000D5;
	data_array[2] = 0x00000100;
	data_array[3] = 0x99006000;		
	data_array[4] = 0x88BBAA88;
		data_array[5] = 0x88018823;		
		data_array[6] = 0x01458867;
		data_array[7] = 0x88888823;
		data_array[8] = 0x99888888;
		data_array[9] = 0x5488AABB;
		data_array[10] = 0x10887688;
		data_array[11] = 0x10323288;
		data_array[12] = 0x88888888;
		data_array[13] = 0x00040088;
		data_array[14] = 0x00000000 ;
		data_array[15] = 0x00000000;
	dsi_set_cmdq(data_array, 16, 1);
	
	data_array[0] = 0x00043902;
	data_array[1] = 0x500000B7;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00053902;
	data_array[1] = 0x009700B6;
	data_array[2] = 0x00000097;
	dsi_set_cmdq(data_array, 3, 1);
	
		data_array[0] = 0x00233902;
	data_array[1] = 0x1E0F00E0;
	data_array[2] = 0x283F342E;
	data_array[3] = 0x0F0D073F;		
	data_array[4] = 0x13121413;
		data_array[5] = 0x0F001710;		
		data_array[6] = 0x3F342E1E;
		data_array[7] = 0x0D073F28;
		data_array[8] = 0x1214130F;
		data_array[9] = 0x00171013;

	dsi_set_cmdq(data_array, 10, 1);
	
	
	data_array[0] = 0x00023902;
	data_array[1] = 0x000002CC;
	dsi_set_cmdq(data_array, 2, 1);
	

	
			data_array[0] = 0x773A1500;
	dsi_set_cmdq(data_array, 1, 1);
	
	    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(150);
    
    	

	

	

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(20);

#else
		dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
		
#endif
    
}


static  LCM_setting_table_V3 lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x05, 0x28, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

    // Sleep Mode On
	{0x05, 0x10, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};
static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120); 

	
	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(20); // 1ms
	
	SET_RESET_PIN(1);
	MDELAY(120);      
}


static void lcm_resume(void)
{
	lcm_init();

}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{

	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 

	//{0xB9,3,{0xFF,0x83,0x89}},

	array[0] = 0x00043902;
	array[1] = 0x8983FFB9;
	dsi_set_cmdq(array, 2, 1);	

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID

#ifdef BUILD_LK
	printf("%s, id = 0x%08x, 0x%08x\n", __func__, buffer[0],buffer[1]);
#else
	printk("%s, id = 0x%08x\n", __func__, id);
#endif

	return (0x89 == id)?1:0;


}

// zhoulidong  add for lcm detect (start)
static unsigned int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    { 
	#ifdef BUILD_LK
	printf("[adc_uboot]: get data error\n");
	#endif
	return 0;
		   
    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

    #ifdef BUILD_LK
    printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
    #endif
	
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE &&lcm_compare_id())
    {
	return 1;
    }

    return 0;

}

static unsigned int lcm_esd_check(void)
{
	unsigned int ret=FALSE;
  #ifndef BUILD_LK
	char  buffer[6];
	int   array[4];

#if 1
	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}
#endif
	array[0] = 0x00083700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 2);
	//printk(" esd buffer0 =%x,buffer1 =%x  \n",buffer[0],buffer[1]);
	//read_reg_v2(0x09,buffer,5);
	//printk(" esd buffer0=%x, buffer1 =%x buffer2=%x,buffer3=%x,buffer4=%x \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
#if 1
	if(buffer[0]==0x1C)
	{
		ret=FALSE;
	}
	else
	{			 
		ret=TRUE;
	}
#endif
 #endif
 return ret;

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	
	#ifndef BUILD_LK
	printk("lcm_esd_recover  hx8389b_video_tianma \n");
	#endif
	return TRUE;
}




LCM_DRIVER hx8389b_dsi_vdo_txd_qhd_lcm_drv = 
{
    .name			= "hx8389b_dsi_vdo_txd_qhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = rgk_lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
