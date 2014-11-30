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
#include <cust_adc.h>	//zhoulidong  add for lcm detect

#define MIN_VOLTAGE (1300)
#define MAX_VOLTAGE (1600)


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  			(480)
#define FRAME_HEIGHT 			(800)

#define LCM_ID_HX8379			0x79  //D3


#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

 unsigned static int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)		lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE					0

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);


static LCM_setting_table_V3 lcm_initialization_setting[] = {
{0x39,0xB9,3,{0xFF,0x83,0x79}},
{0x39,0xB1,20,{0x44,0x18,0x18,0x31,0x51,0x90,0xD0,0xEE,0x94,0x80,0x38,0xF8,0x22,0x22,0x22,0x00,0x80,0x30,0x00}},
{0x39,0xB2,9,{0x80,0x3C,0x0B,0x04,0x00,0x50,0x11,0x42,0x1D}},
{0x39,0xB4,10,{0x50,0x51,0x50,0x51,0x50,0x51,0x12,0xA0,0x13,0xA0}},
{0x39,0xD3,29,{0x00,0x07,0x00,0x00,0x00,0x06,0x06,0x32,0x10,0x05,0x00,0x05,0x03,0x6F,0x03,0x6F,0x00,0x07,0x00,0x07,0x21,0x22,0x05,0x05,0x23,0x05,0x05,0x23,0x09}},
{0x39,0xD5,32,{0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
{0x39,0xD6,32,{0x18,0x18,0x18,0x18,0x02,0x03,0x00,0x01,0x20,0x21,0x19,0x19,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
{0x39,0xE0,42,{0x00,0x0F,0x14,0x36,0x38,0x3C,0x22,0x3F,0x06,0x0A,0x0D,0x17,0x0F,0x13,0x14,0x13,0x13,0x07,0x13,0x11,0x14,0x00,0x0F,0x14,0x36,0x38,0x3C,0x22,0x3F,0x06,0x0A,0x0D,0x17,0x0F,0x13,0x14,0x13,0x13,0x07,0x13,0x11,0x14}},
{0x39,0xB6,2,{0x54,0x51}},
{0x15,0xCC,1,{0x0E}},

{0x05,0x11,0,{}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 150, {}},
{0x05,0x29,0,{}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

};

static void init_lcm_registers(void)
{

    unsigned int data_array[16];

   //HX8379C_BOE3.97IPS
    data_array[0]=0x00043902;//Enable external Command
    data_array[1]=0x7983FFB9; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);//3000
      
    data_array[0]=0x00153902;
    data_array[1]=0x181844B1;
    data_array[2]=0xD0905131;
    data_array[3]=0x388094EE;  
    data_array[4]=0x2222F838; 
    data_array[5]=0x30800022;  
    data_array[6]=0x00000000;
    dsi_set_cmdq(&data_array, 7, 1);


    data_array[0]=0x000A3902;
    data_array[1]=0x0B3C80b2; //  
    data_array[2]=0x11500004;
    data_array[3]=0x00001D42;   
    dsi_set_cmdq(&data_array, 4, 1); 
    MDELAY(1); 


    data_array[0]=0x000B3902;
    data_array[1]=0x505150b4;   
    data_array[2]=0x12515051; 
    data_array[3]=0x00A013A0;    
    dsi_set_cmdq(&data_array, 4, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000002CC;
    dsi_set_cmdq(&data_array, 2, 1);

    data_array[0]=0x00033902;
    data_array[1]=0x005454B6;  
    dsi_set_cmdq(&data_array, 2, 1);
  
    data_array[0]=0x002B3902;
    data_array[1]=0x140F00E0; 
    data_array[2]=0x223C3836; 
    data_array[3]=0x0D0A063F; 
    data_array[4]=0x14130F17; 
    data_array[5]=0x13071313; 
    data_array[6]=0x0F001411; 
    data_array[7]=0x3C383614; 
    data_array[8]=0x0A063F22; 
    data_array[9]=0x130F170D; 
    data_array[10]=0x07131314;
    data_array[11]=0x00141113;  
    dsi_set_cmdq(&data_array, 12, 1);
    MDELAY(5);

    data_array[0]=0x001E3902;
    data_array[1]=0x000700d3; 
    data_array[2]=0x06060000; 
    data_array[3]=0x00051032; 
    data_array[4]=0x036F0305; 
    data_array[5]=0x0007006F; 
    data_array[6]=0x05222107; 
    data_array[7]=0x05052305; 
    data_array[8]=0x00002309;  
    dsi_set_cmdq(&data_array, 9, 1);

    data_array[0]=0x00213902;//Enable external Command//3
    data_array[1]=0x191818d5; 
    data_array[2]=0x03000119; 
    data_array[3]=0x18202102; 
    data_array[4]=0x18181818; 
    data_array[5]=0x18181818; 
    data_array[6]=0x18181818; 
    data_array[7]=0x18181818; 
    data_array[8]=0x18181818; 
    data_array[9]=0x00000018; 
    dsi_set_cmdq(&data_array, 10, 1);

    data_array[0]=0x00213902;
    data_array[1]=0x181818d6; 
    data_array[2]=0x00030218; 
    data_array[3]=0x19212001; 
    data_array[4]=0x18181819; 
    data_array[5]=0x18181818; 
    data_array[6]=0x18181818; 
    data_array[7]=0x18181818; 
    data_array[8]=0x18181818; 
    data_array[9]=0x00000018; 
    dsi_set_cmdq(&data_array, 10, 1);

    //data_array[0]=0x00033902;
    //data_array[1]=0x00010CE4;
    //dsi_set_cmdq(&data_array, 2, 1);


    data_array[0]=0x00023902;
    data_array[1]=0x0000773A;
    dsi_set_cmdq(&data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000036;
    dsi_set_cmdq(&data_array, 2, 1);
    
    data_array[0] = 0x00110500; 
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(150);
    
    data_array[0] = 0x00290500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(30);    

}

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

	params->physical_width  = 52;
       params->physical_height = 87;
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	params->dsi.vertical_sync_active				= 6;
	params->dsi.vertical_backporch					= 5;
	params->dsi.vertical_frontporch					= 6;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;


	params->dsi.horizontal_sync_active				= 36;
	params->dsi.horizontal_backporch				= 36;
	params->dsi.horizontal_frontporch				= 36;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		
	//params->dsi.LPX=8; 

	// Bit rate calculation
	//1 Every lane speed
	//params->dsi.pll_select=1;
	//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
	params->dsi.PLL_CLOCK=220;
	/*params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
	params->dsi.fbk_div =7;
#else
	params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
	//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
*/
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	init_lcm_registers();

}


static LCM_setting_table_V3  lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	//{0x05, 0x28, 0, {}},
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

	//Sleep Mode On
	{0x05, 0x10, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};

static void lcm_suspend(void)
{	

	dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
	//SET_RESET_PIN(1);	
	//SET_RESET_PIN(0);
	//MDELAY(20); // 1ms
	
	//SET_RESET_PIN(1);
	//MDELAY(120);      
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
	int array[4];
	char buffer[4]={0,0,0,0};
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0]=0x00043902;
	array[1]=0x7983FFB9;
	dsi_set_cmdq(array, 2, 1);

	MDELAY(10);
	array[0]=0x00033902;
	array[1]=0x009351ba;
	dsi_set_cmdq(array, 2, 1);

	MDELAY(10);
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	MDELAY(10);
	read_reg_v2(0xF4, buffer, 4);//	NC 0x00  0x98 0x16

	id = buffer[0];
	//id_low = buffer[1];
	//id = (id_high<<8) | id_low;
	
	#ifdef BUILD_LK

		printf("hx8379a uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);

	#else
		printk("hx8379a kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);

	#endif


	return (LCM_ID_HX8379 == id)?1:0;

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
    printf("@@@@@@@[adc_uboot]: lcm_vol= %d , file : %s, line : %d\n",lcm_vol, __FILE__, __LINE__);
    #endif
	
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE &&lcm_compare_id())
    {
	return 1;
    }

    return 0;
	
}
// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
	#ifdef BUILD_LK
		//printf("lcm_esd_check()\n");
	#else
		//printk("lcm_esd_check()\n");
	#endif 
 #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	if(buffer[0]==0x9c)
	{
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif
		return FALSE;
	}
	else
	{	
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif		 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	
	#ifdef BUILD_LK
		printf("lcm_esd_recover()\n");
	#else
		printk("lcm_esd_recover()\n");
	#endif	
	
	lcm_init();	

	return TRUE;
}
// zhoulidong add for eds(end)
LCM_DRIVER hx8379c_dsi_vdo_azet_wvga_ips_lcm_drv = 
{
    	.name			= "hx8379c_dsi_vdo_azet_wvga_ips",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = rgk_lcm_compare_id,
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)

    .update         = lcm_update,
#endif
};

