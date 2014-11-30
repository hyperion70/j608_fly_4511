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
#define MIN_VOLTAGE (1700)	// ++++rgk bug-id:no modify by yangjuwei 20140221
#define MAX_VOLTAGE (1900)	// ++++rgk bug-id:no modify by yangjuwei 20140221
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif
static int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0


// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static unsigned int lcm_compare_id(void);

//update initial param for IC boe_nt35521 0.01
static  LCM_setting_table_V3 lcm_initialization_setting[] = {
{0x39,0xFF,4,{0xAA,0x55,0xA5,0x80}},
{0x39,0x6F,2,{0x11,0x00}},
{0x39,0xF7,2,{0x20,0x00}},
{0x15,0x6F,1,{0x06}},
{0x15,0xF7,1,{0xA0}},
{0x15,0x6F,1,{0x19}},
{0x15,0xF7,1,{0x12}},
{0x15,0x6F,1,{0x02}},
{0x15,0xF7,1,{0x47}},
{0x15,0x6F,1,{0x17}},
{0x15,0xF4,1,{0x70}},
{0x15,0x6F,1,{0x01}},
{0x15,0xF9,1,{0x46}},

//page 0
{0x39,0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
{0x39,0xBD,5,{0x01,0xA0,0x10,0x10,0x01}}, 
{0x39,0xB8,4,{0x01,0x02,0x0C,0x02}},
{0x39,0xBB,2,{0x11,0x11}},
{0x39,0xBC,2,{0x00,0x00}},
{0x15,0xB6,1,{0x04}},
{0x15,0xC8,1,{0x80}},

//page 1
{0x39,0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
{0x39,0xB0,2,{0x09,0x09}},
{0x39,0xB1,2,{0x09,0x09}},

{0x39,0xBC,2,{0x98,0x00}},
{0x39,0xBD,2,{0x98,0x00}},

{0x15,0xCA,1,{0x00}},
{0x15,0xC0,1,{0x0C}},
{0x39,0xB5,2,{0x03,0x03}},
{0x15,0xBE,1,{0x3A}}, //VCOM
{0x39,0xB3,2,{0x19,0x19}},
{0x39,0xB4,2,{0x19,0x19}},
{0x39,0xB9,2,{0x26,0x26}},
{0x39,0xBA,2,{0x24,0x24}},

{0x39,0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
{0x15,0xEE,1,{0x01}},

//update gamma code
{0x39,0xB0,16,{0x00,0x00,0x00,0x0E,0x00,0x28,0x00,0x3E,0x00,0x52,0x00,0x6B,0x00,0x88,0x00,0xBD}},	
{0x39,0xB1,16,{0x00,0xE3,0x01,0x1F,0x01,0x4F,0x01,0x9C,0x01,0xDD,0x01,0xDE,0x02,0x19,0x02,0x5C}},	
{0x39,0xB2,16,{0x02,0x87,0x02,0xC1,0x02,0xEC,0x03,0x23,0x03,0x47,0x03,0x78,0x03,0x95,0x03,0xBB}},	
{0x39,0xB3,4,{0x03,0xF1,0x03,0xFF}},	

//page 6
{0x39,0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},
{0x39,0xB0,2,{0x10,0x12}},
{0x39,0xB1,2,{0x14,0x16}},
{0x39,0xB2,2,{0x00,0x02}},
{0x39,0xB3,2,{0x31,0x31}},
{0x39,0xB4,2,{0x31,0x34}},
{0x39,0xB5,2,{0x34,0x34}},
{0x39,0xB6,2,{0x34,0x31}},
{0x39,0xB7,2,{0x31,0x31}},
{0x39,0xB8,2,{0x31,0x31}},
{0x39,0xB9,2,{0x2D,0x2E}},
{0x39,0xBA,2,{0x2E,0x2D}},
{0x39,0xBB,2,{0x31,0x31}},
{0x39,0xBC,2,{0x31,0x31}},
{0x39,0xBD,2,{0x31,0x34}},
{0x39,0xBE,2,{0x34,0x34}},
{0x39,0xBF,2,{0x34,0x31}},
{0x39,0xC0,2,{0x31,0x31}},
{0x39,0xC1,2,{0x03,0x01}},
{0x39,0xC2,2,{0x17,0x15}},
{0x39,0xC3,2,{0x13,0x11}},
{0x39,0xE5,2,{0x31,0x31}},
{0x39,0xC4,2,{0x17,0x15}},
{0x39,0xC5,2,{0x13,0x11}},
{0x39,0xC6,2,{0x03,0x01}},
{0x39,0xC7,2,{0x31,0x31}},
{0x39,0xC8,2,{0x31,0x34}},
{0x39,0xC9,2,{0x34,0x34}},
{0x39,0xCA,2,{0x34,0x31}},
{0x39,0xCB,2,{0x31,0x31}},
{0x39,0xCC,2,{0x31,0x31}},
{0x39,0xCD,2,{0x2E,0x2D}},
{0x39,0xCE,2,{0x2D,0x2E}},
{0x39,0xCF,2,{0x31,0x31}},
{0x39,0xD0,2,{0x31,0x31}},
{0x39,0xD1,2,{0x31,0x34}},
{0x39,0xD2,2,{0x34,0x34}},
{0x39,0xD3,2,{0x34,0x31}},
{0x39,0xD4,2,{0x31,0x31}},
{0x39,0xD5,2,{0x00,0x02}},
{0x39,0xD6,2,{0x10,0x12}},
{0x39,0xD7,2,{0x14,0x16}},
{0x39,0xE6,2,{0x32,0x32}},
{0x39,0xD8,5,{0x00,0x00,0x00,0x00,0x00}},
{0x39,0xD9,5,{0x00,0x00,0x00,0x00,0x00}},
{0x15,0xE7,1,{0x00}},

//page 5
{0x39,0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
{0x15,0xED,1,{0x30}},
{0x39,0xB0,2,{0x17,0x06}},
{0x15,0xB8,1,{0x00}},
{0x15,0xC0,1,{0x0D}},
{0x15,0xC1,1,{0x0B}},
{0x15,0xC2,1,{0x23}},
{0x15,0xC3,1,{0x40}},
{0x15,0xC4,1,{0x84}},
{0x15,0xC5,1,{0x82}},
{0x15,0xC6,1,{0x82}},
{0x15,0xC7,1,{0x80}},
{0x39,0xC8,2,{0x0B,0x30}},
{0x39,0xC9,2,{0x05,0x10}},
{0x39,0xCA,2,{0x01,0x10}},
{0x39,0xCB,2,{0x01,0x10}},
{0x39,0xD1,5,{0x03,0x05,0x05,0x07,0x00}},
{0x39,0xD2,5,{0x03,0x05,0x09,0x03,0x00}},
{0x39,0xD3,5,{0x00,0x00,0x6A,0x07,0x10}},
{0x39,0xD4,5,{0x30,0x00,0x6A,0x07,0x10}},

//page 3
{0x39,0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
{0x39,0xB0,2,{0x00,0x00}},
{0x39,0xB1,2,{0x00,0x00}},
{0x39,0xB2,5,{0x05,0x00,0x0A,0x00,0x00}},
{0x39,0xB3,5,{0x05,0x00,0x0A,0x00,0x00}},
{0x39,0xB4,5,{0x05,0x00,0x0A,0x00,0x00}},
{0x39,0xB5,5,{0x05,0x00,0x0A,0x00,0x00}},
{0x39,0xB6,5,{0x02,0x00,0x0A,0x00,0x00}},
{0x39,0xB7,5,{0x02,0x00,0x0A,0x00,0x00}},
{0x39,0xB8,5,{0x02,0x00,0x0A,0x00,0x00}},
{0x39,0xB9,5,{0x02,0x00,0x0A,0x00,0x00}},
{0x39,0xBA,5,{0x53,0x00,0x0A,0x00,0x00}},
{0x39,0xBB,5,{0x53,0x00,0x0A,0x00,0x00}},
{0x39,0xBC,5,{0x53,0x00,0x0A,0x00,0x00}},
{0x39,0xBD,5,{0x53,0x00,0x0A,0x00,0x00}},
{0x15,0xC4,1,{0x60}},
{0x15,0xC5,1,{0x40}},
{0x15,0xC6,1,{0x64}},
{0x15,0xC7,1,{0x44}},

{0x15,0x6F,1,{0x11}},
{0x15,0xF3,1,{0x01}},
//CCMOFF
//CCMRUN
{0x05,0x11,0,{}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},    
{0x05,0x29,0,{}},
};

/*static struct LCM_setting_table lcm_sleep_out_setting[] = {
    //Sleep Out
    {0x15,0x11, 1, {0x00}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},    

    // Display ON
    {0x15,0x29, 1, {0x00}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 20, {}},    
    //{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static  LCM_setting_table_V3 lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x15,0x28, 1, {0x00}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 20, {}},   

    // Sleep Mode On
    {0x15,0x10, 1, {0x00}},
   {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},   
   // {REGFLAG_END_OF_TABLE, 0x00, {}}
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
    	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_THREE_LANE;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	//video mode timing
    	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 6;// 3    2
	params->dsi.vertical_backporch					= 14;// 20   1
	params->dsi.vertical_frontporch					= 20; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
/*

*/

	params->dsi.horizontal_sync_active				= 10;// 50  2
	params->dsi.horizontal_backporch				= 50;
	params->dsi.horizontal_frontporch				= 50;
	// params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    /*
    params->dsi.horizontal_sync_active				= 6;
    params->dsi.horizontal_backporch				= 37;
    params->dsi.horizontal_frontporch				= 37;
    params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    */
    // Bit rate calculation

	params->dsi.PLL_CLOCK= 285;
//	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
//	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
//	#if (LCM_DSI_CMD_MODE)
//	params->dsi.fbk_div =7;
//	#else
//	params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
//#endif
}

/*to prevent electric leakage*/
/*static void lcm_id_pin_handle(void)
{
   mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_UP);
    mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DOWN);
}*/

static void lcm_init(void)
{
	 //enable VSP & VSN
    lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
    lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    MDELAY(50);
	//reset high to low to high
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    MDELAY(5);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    MDELAY(20);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    MDELAY(10);

    //lcm_id_pin_handle();
 
    dsi_set_cmdq_V3(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(lcm_initialization_setting[0]), 1);  
	
	// when phone initial , config output high, enable backlight drv chip 
  //  lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);
	
   // printk("uboot:boe_nt35521_lcm_init\n");
}

static void lcm_suspend(void)
{
    //Back to MP.P7 baseline , solve LCD display abnormal On the right
    // when phone sleep , config output low, disable backlight drv chip  
  //  lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ZERO);  	

    dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
    //reset low
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
    //disable VSP & VSN
	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
    MDELAY(5);	
  /*#ifdef BUILD_LK
	;
	 #else
printk("sym lcm_suspend\n");
	 #endif
  lcm_compare_id();*/
  //  printk("kernel:boe_nt35521_lcm_suspend\n");
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

// zhoulidong  add for lcm detect (start)
static unsigned int lcm_compare_id(void)
{
		unsigned int id=0;
	unsigned char buffer[3];
	unsigned int array[16];  
	unsigned int data_array[16];
	int lcm_adc = 0, data[4] = {0,0,0,0};

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(100);
	
	SET_RESET_PIN(1);
	MDELAY(120); 
	
	data_array[0] = 0x00063902;
	data_array[1] = 0x52AA55F0;  
	data_array[2] = 0x00000108;                
	dsi_set_cmdq(&data_array, 3, 1); 

	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xC5, buffer, 3);
	id = buffer[1]; //we only need ID
		
	IMM_GetOneChannelValue(0,&data,&lcm_adc);  //2.8V  MTK6592 LCM_ID ADC Channel 
  	 #ifdef BUILD_LK
         printf("%s, LK al900_nt35521_hd720_dsi_vdo_DJI_152225146201 debug: nt35521 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
	 printf("LK al900_nt35521_hd720_dsi_vdo_DJI_152225146201 lcm_adc=%d\n",lcm_adc); 
        #else
         printk("%s, LK al900_nt35521_hd720_dsi_vdo_DJI_152225146201 debug: nt35521 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
	 printk("LK al900_nt35521_hd720_dsi_vdo_DJI_152225146201 lcm_adc=%d\n",lcm_adc);   
        #endif

        if((buffer[0]==0x55) && (buffer[1]==0x21)) ////3738
           return 1;
	else
	    return 0; 	
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


LCM_DRIVER nt35521_hd720_dsi_vdo_djn_boe_ips_oncell_lcm_drv =
{
    .name           	= "nt35521_hd720_dsi_vdo_djn_boe_ips_oncell",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
	.compare_id    = rgk_lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
	//.update         = lcm_update,
#endif
};
/* END PN: , Added by h84013687, 2013.08.13*/
