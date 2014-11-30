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
#define MIN_VOLTAGE (200)	//++++rgk bug-id:null modify by yangjuwei 20140310
#define MAX_VOLTAGE (400)	//++++rgk bug-id:null modify by yangjuwei 20140310

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  						(480)
#define FRAME_HEIGHT 						(854)

#define LCM_ID_ILI9806						0x9806


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

#define SET_RESET_PIN(v)    					(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        	lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE	0
// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static unsigned int rgk_lcm_compare_id(void);

static  LCM_setting_table_V3 lcm_initialization_setting[] = {
	


{0x39,0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1		
{0x15,0x08,1,{0x10}},                 // output SDA		
{0x15,0x21,1,{0x01}},                 // DE = 1 Active		
{0x15,0x30,1,{0x01}},                 // 480 X 854		
{0x15,0x31,1,{0x00}},                 // 2 dot		
{0x15,0x40,1,{0x16}},                // BT  		
{0x15,0x41,1,{0x33}},                 // DVDDH DVDDL clamp 		
{0x15,0x42,1,{0x03}},                 // VGH/VGL 		
{0x15,0x43,1,{0x09}},                 // VGH_CLAMP 0FF ;	     	
{0x15,0x44,1,{0x07}},                 // VGL_CLAMP OFF ; 		
{0x15,0x45,1,{0x16}},                 // VGL_REG  -11V 		
{0x15,0x50,1,{0x78}},                   //		
{0x15,0x51,1,{0x78}},                   		
{0x15,0x52,1,{0x00}},                   //Flicker		
{0x15,0x53,1,{0x4A}},                   //Flicker		
{0x15,0x57,1,{0x50}},                   //Low voltage detect		
{0x15,0x60,1,{0x07}},                 // SDTI		
{0x15,0x61,1,{0x00}},                // CRTI		
{0x15,0x62,1,{0x08}},                 // EQTI		
{0x15,0x63,1,{0x00}},                // PCTI		  
{0x15,0xA0,1,{0x00}},  // Gamma 255 	       	
{0x15,0xA1,1,{0x09}},  //				 Gamm				a 25									1 		
{0x15,0xA2,1,{0x0F}}, 			  //	Gamma				 24									7		
{0x15,0xA3,1,{0x0B}},  //								 Gamma 23									9		
{0x15,0xA4,1,{0x06}},  //								 Gamma 23									1		
{0x15,0xA5,1,{0x09}},  //								 Gamma 20									3		
{0x15,0xA6,1,{0x07}},  //								 Gamma 17									5		
{0x15,0xA7,1,{0x05}},  // Gamma 				147										
{0x15,0xA8,1,{0x08}},  // Gamma 				108											
{0x15,0xA9,1,{0x0C}},  // Gamma 				80	 										
{0x15,0xAA,1,{0x12}},  // Gamma 				52											
{0x15,0xAB,1,{0x08}},  // Gamma 				24		
{0x15,0xAC,1,{0x0D}},  // Gamma 				16		
{0x15,0xAD,1,{0x17}},  // Gamma 				8		
{0x15,0xAE,1,{0x0E}},  // Gamma 				4		
{0x15,0xAF,1,{0x00}},  // Gamma 				0	
{0x15,0xC0,1,{0x00}},  // Gamma 				255 		
{0x15,0xC1,1,{0x08}},  // Gamma 				251		
{0x15,0xC2,1,{0x0E}},  // Gamma 				247		
{0x15,0xC3,1,{0x0B}},  // Gamma 				239		
{0x15,0xC4,1,{0x05}},  // Gamma 				231		
{0x15,0xC5,1,{0x09}},  // Gamma 				203		
{0x15,0xC6,1,{0x07}},  // Gamma 				175		
{0x15,0xC7,1,{0x04}},  // Gamma 				147		
{0x15,0xC8,1,{0x08}},  // Gamma 				108		
{0x15,0xC9,1,{0x0C}},  // Gamma 				80		
{0x15,0xCA,1,{0x11}},  // Gamma 				52		
{0x15,0xCB,1,{0x07}},  // Gamma 				24		
{0x15,0xCC,1,{0x0D}},  // Gamma 				16		
{0x15,0xCD,1,{0x17}},  // Gamma 				8		
{0x15,0xCE,1,{0x0E}},  // Gamma 				4		
{0x15,0xCF,1,{0x00}},  // Gamma 				0	
{0x39,0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6		
{0x15,0x00,1,{0x21}},		
{0x15,0x01,1,{0x0A}},		
{0x15,0x02,1,{0x00}},  	
{0x15,0x03,1,{0x00}},	
{0x15,0x04,1,{0x01}},	
{0x15,0x05,1,{0x01}},		
{0x15,0x06,1,{0x80}},   	
{0x15,0x07,1,{0x06}},	
{0x15,0x08,1,{0x01}},	
{0x15,0x09,1,{0x80}},  	
{0x15,0x0A,1,{0x00}},  	
{0x15,0x0B,1,{0x00}},  	
{0x15,0x0C,1,{0x01}},	
{0x15,0x0D,1,{0x01}},		
{0x15,0x0E,1,{0x00}},		
{0x15,0x0F,1,{0x00}},		
{0x15,0x10,1,{0xF0}},		
{0x15,0x11,1,{0xF4}},		
{0x15,0x12,1,{0x04}},		
{0x15,0x13,1,{0x00}},		
{0x15,0x14,1,{0x00}},		
{0x15,0x15,1,{0xC0}},		
{0x15,0x16,1,{0x08}},		
{0x15,0x17,1,{0x00}},		
{0x15,0x18,1,{0x00}},		
{0x15,0x19,1,{0x00}},		
{0x15,0x1A,1,{0x00}},		
{0x15,0x1B,1,{0x00}},		
{0x15,0x1C,1,{0x00}},		
{0x15,0x1D,1,{0x00}},	
{0x15,0x20,1,{0x01}},		
{0x15,0x21,1,{0x23}},		
{0x15,0x22,1,{0x45}},		
{0x15,0x23,1,{0x67}},		
{0x15,0x24,1,{0x01}},		
{0x15,0x25,1,{0x23}},		
{0x15,0x26,1,{0x45}},		
{0x15,0x27,1,{0x67}},	
{0x15,0x30,1,{0x01}},		
{0x15,0x31,1,{0x11}},	//GOUT1	
{0x15,0x32,1,{0x00}},	//GOUT2	
{0x15,0x33,1,{0xEE}},	//GOUT3	
{0x15,0x34,1,{0xFF}},	//GOUT4	
{0x15,0x35,1,{0xBB}},	//GOUT5	
{0x15,0x36,1,{0xCA}},	//GOUT6	
{0x15,0x37,1,{0xDD}},	//GOUT7	
{0x15,0x38,1,{0xAC}},	//GOUT8	
{0x15,0x39,1,{0x76}},	//GOUT9	
{0x15,0x3A,1,{0x67}},	//GOUT10	
{0x15,0x3B,1,{0x22}},	//GOUT11	
{0x15,0x3C,1,{0x22}},	//GOUT12	
{0x15,0x3D,1,{0x22}},	//GOUT13	
{0x15,0x3E,1,{0x22}},	//GOUT14	
{0x15,0x3F,1,{0x22}},		
{0x15,0x40,1,{0x22}},	
{0x15,0x52,1,{0x10}},		
{0x15,0x53,1,{0x10}},        //12:VGLO tie VGL_REG; 10: VGLO tie VGL	    
{0x39,0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},    // Change to Page 7	
{0x15,0x17,1,{0x22}}, // VGL_REG ON	     	
{0x15,0x02,1,{0x77}}, // 		
{0x15,0x06,1,{0x13}}, //		
{0x15,0xE1,1,{0x79}}, // 		
{0x39,0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},    // Change to Page 0		
{0x15,0x11,1,{0x00}},                 // Sleep-Out	
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},    
{0x15,0x29,1,{0x00}},                 // Display on		
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},
{0x15,0x35,1,{0x00}},                 // TE ON	

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
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 6;// 3    2
	params->dsi.vertical_backporch					= 14;// 20   1
	params->dsi.vertical_frontporch					= 20; // 1  12
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;// 50  2
	params->dsi.horizontal_backporch				= 90;
	params->dsi.horizontal_frontporch				= 90;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	//params->dsi.LPX=8; 
	// Bit rate calculation

	//params->dsi.noncont_clock =1;
	params->dsi.PLL_CLOCK=208;
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
	#if (LCM_DSI_CMD_MODE)
	params->dsi.fbk_div =7;
	#else
	params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
}


static void lcm_init(void)
{
	
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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
	dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(20); // 1ms

	SET_RESET_PIN(1);
	MDELAY(120);
}


static void lcm_resume(void)
{
	lcm_init();
//	rgk_lcm_compare_id();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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





// zhoulidong  add for lcm detect (start)

static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00063902;
	array[1] = 0x0698FFFF;
	array[2] = 0x00000104;
	dsi_set_cmdq(array, 3, 1);

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x00, &buffer[0], 1);  //0x98

	array[0] = 0x00013700;		
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x01, &buffer[1], 1);  //0x06

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x02, &buffer[2], 1);  //0x04

	id_high = buffer[0];
	id_low = buffer[1];
	id = (id_high<<8) | id_low;

	
       #ifdef BUILD_LK
		printf("zbuffer %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("zbuffer %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	
       #endif
	 
  	return (LCM_ID_ILI9806 == id)?1:0;

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
    #else
printk("J503  lcm_vol= 0x%x\n",lcm_vol);
    #endif
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE && lcm_compare_id())
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
	lcm_init();
	
	#ifndef BUILD_LK
	printk("lcm_esd_recover  ili9806e_dsi_vdo_hlt_fwvga_ips_j502 \n");
	#endif
	return TRUE;
}




LCM_DRIVER ili9806e_dsi_vdo_hlt_fwvga_ips_d5020_lcm_drv = 
{
	.name		= "ili9806e_dsi_vdo_hlt_fwvga_ips_d5020",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id	= rgk_lcm_compare_id,	
	//.esd_check     = lcm_esd_check,
	//.esd_recover   = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
	//.update         = lcm_update,
#endif
};

