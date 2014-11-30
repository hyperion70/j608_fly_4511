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
#define MIN_VOLTAGE (1400)	// ++++rgk bug-id:no modify by yangjuwei 20140221
#define MAX_VOLTAGE (1600)	// ++++rgk bug-id:no modify by yangjuwei 20140221
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  						(480)
#define FRAME_HEIGHT 						(854)

#define LCM_ID_OTM8018B	0x8009


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

#define UDELAY(n) 						(lcm_util.udelay(n))
#define MDELAY(n) 						(lcm_util.mdelay(n))


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
static unsigned int rgk_lcm_compare_id(void);


static  LCM_setting_table_V3 lcm_initialization_setting[] = {
	
	//Gamma2.2                                                                                     
                                                                                                 
{0x15,0x00,1,{0x00}},                                                                              
{0x39,0xFF,3,{0x80,0x09,0x01}},                                                                                                                 
{0x15,0x00,1,{0x80}},                                                                              
{0x39,0xFF,2,{0x80,0x09}},                                                                                             
{0x39,0x00,3,{0xD2,0xD2,0xD2}},                                                                    
{0x15,0xB0,1,{0x01}},                                                                                                                                                                                                                                                               
{0x15,0x00,1,{0x80}},                                                                              
{0x15,0xC4,1,{0x30}}, //放电代码                                                                   
                                                                                                          
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},  //{0x15,REGFLAG_DELAY, 10, {}}},                
{0x15,0x00,1,{0x8A}},                                                                              
{0x15,0xC4,1,{0x40}}, //放电代码                                                                                                                                                                                   
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}}, //{0x15,REGFLAG_DELAY, 10, {}}},                 
{0x15,0x00,1,{0xB2}},                                                                                                                                                                                   
{0x39,0xF5,4,{0x15,0x00,0x15,0x00}},                                                               
{0x15,0x00,1,{0x8B}},                                                                                                                                                                                    
{0x15,0xB0,1,{0x40}},                                                                              
{0x15,0x00,1,{0xC6}},                                                                                                                                                                                  
{0x15,0xB0,1,{0x03}},                                                                              
{0x15,0x00,1,{0x90}},                                                                                                                                                                                    
{0x15,0xB3,1,{0x02}},                                                                              
{0x15,0x00,1,{0x92}},                                                                                                                                                                                    
{0x15,0xB3,1,{0x40}},                                                                              
{0x15,0x00,1,{0xA6}},                                                                                                                                                                                    
{0x39,0xB3,2,{0x20,0x01}},                                                                         
{0x15,0x00,1,{0x90}},                                                                                                                                                                                    
{0x39,0xC0,6,{0x00,0x44,0x00,0x00,0x00,0x03}},                                                     
{0x15,0x00,1,{0xB4}},                                                                                                                                                                                   
{0x15,0xC0,1,{0x10}},                                                                              
{0x15,0x00,1,{0x81}},                                                                                                                                                                                   
{0x15,0xC1,1,{0x66}},                                                                              
{0x15,0x00,1,{0xA0}},                                                                                                                                                                                    
{0x15,0xC1,1,{0xEA}},                                                                              
{0x15,0x00,1,{0xA6}},                                                                                                                                                                                    
{0x39,0xC1,3,{0x01,0x00,0x00}},                                                                    
{0x15,0x00,1,{0x81}},                                                                                                                                                                                    
{0x15,0xC4,1,{0x81}},                                                                              
{0x15,0x00,1,{0x82}},                                                                                                                                                                                   
{0x15,0xC5,1,{0x83}},                                                                              
{0x15,0x00,1,{0x90}},                                                                                                                                                                                   
{0x39,0xC5,7,{0x96,0x34,0x06,0x91,0x33,0x34,0x23}},                                                
{0x15,0x00,1,{0xB1}},                                                                                                                                                                                   
{0x15,0xC5,1,{0xA9}},                                                                              
{0x15,0x00,1,{0xC0}},                                                                                                                                                                                   
{0x15,0xC5,1,{0x00}},                                                                              
{0x15,0x00,1,{0x80}},                                                                              
{0x39,0xCE,6,{0x86,0x01,0x00,0x85,0x01,0x00}},                                                     
{0x15,0x00,1,{0xA0}},                                                                              
{0x39,0xCE,14,{0x18,0x05,0x03,0x5b,0x00,0x00,0x00,0x18,0x04,0x03,0x5c,0x00,0x00,0x00}},            
{0x15,0x00,1,{0xB0}},                                                                                                                                                                                  
{0x39,0xCE,14,{0x18,0x03,0x03,0x5d,0x00,0x00,0x00,0x18,0x02,0x03,0x5e,0x00,0x00,0x00}},            
{0x15,0x00,1,{0xC0}},                                                                                                                                                                                  
{0x39,0xCF,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x02,0x00,0x00}},                                
{0x15,0x00,1,{0xC0}},                                                                              
{0x39,0xCB,15,{0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},       
{0x15,0x00,1,{0xD0}},                                                                              
{0x39,0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00}},       
{0x15,0x00,1,{0xE0}},                                                                              
{0x39,0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},                                
{0x15,0x00,1,{0xF0}},                                                                                                                                                                                  
{0x39,0xCB,10,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},                                
{0x15,0x00,1,{0x80}},                                                                              
{0x39,0xCC,10,{0x00,0x26,0x09,0x0B,0x01,0x25,0x00,0x00,0x00,0x00}},                                
{0x15,0x00,1,{0x90}},                                                                              
{0x39,0xCC,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x0A,0x0C,0x02}},       
{0x15,0x00,1,{0xA0}},                                                                              
{0x39,0xCC,15,{0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},       
{0x15,0x00,1,{0xB0}},                                                                              
{0x39,0xCC,10,{0x00,0x25,0x0C,0x0A,0x02,0x26,0x00,0x00,0x00,0x00}},                                
{0x15,0x00,1,{0xC0}},                                                                              
{0x39,0xCC,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x0B,0x09,0x01}},       
{0x15,0x00,1,{0xD0}},                                                                                                                                                                                   
{0x39,0xCC,15,{0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},       
{0x15,0x00,1,{0x00}},                                                                                                                                                                                    
{0x39,0xD8,2,{0x77,0x77}},                                                                         
{0x15,0x00,1,{0x00}},                                                                                                                                                                                  
{0x15,0xD9,1,{0x48}},                                                                              
{0x15,0x00,1,{0x00}},                                                                              
{0x39,0xE1,16,{0x01,0x08,0x0D,0x0D,0x05,0x0F,0x0C,0x0B,0x02,0x06,0x0B,0x08,0x0F,0x12,0x0D,0x03}},  
{0x15,0x00,1,{0x00}},                                                                                                                                                                                                                                          
{0x39,0xE2,16,{0x01,0x08,0x0D,0x0D,0x05,0x0F,0x0C,0x0B,0x02,0x06,0x0B,0x08,0x0F,0x12,0x0D,0x03}},  
{0x15,0x00,1,{0xA0}},                                                                                                                                                                                    
{0x15,0xC1,1,{0xEA}},                                                                              
{0x15,0x00,1,{0xA6}},                                                                                                                                                                                   
{0x39,0xC1,3,{0x01,0x00,0x00}},                                                                    
{0x15,0x00,1,{0xC6}},                                                                                                                                                                                   
{0x15,0xB0,1,{0x03}},                                                                              
{0x15,0x00,1,{0x81}},                                                                                                                                                                                  
{0x15,0xC5,1,{0x66}}, //Auto pump 2x                                                               
{0x15,0x00,1,{0xB6}},                                                                                                                                                                                   
{0x15,0xF5,1,{0x06}}, //VGL_REG timing setting.                                                    
{0x15,0x00,1,{0x8B}},                                                                                                                                                                                   
{0x15,0xB0,1,{0x40}}, //ULPM                                                                       
{0x15,0x00,1,{0xB1}},                                                                                                                                                                                  
{0x15,0xB0,1,{0x80}}, //MIPI RC delay setting                                                      
{0x15,0x00,1,{0x00}},                                                                                                                                                                                                                                                                                           
{0x39,0xFF,3,{0xFF,0xFF,0xFF}},                                                                    
{0x15,0x11,1,{0x00}},                                                                                                                                                                                 
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},  //{0x15,REGFLAG_DELAY, 120, {}}},              
{0x15,0x29,1,{0x00}},                                                                                             
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}}, //{0x15,REGFLAG_DELAY, 10, {}}},                 

};

/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/


/*
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

/*static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}*/


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

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
	params->dsi.horizontal_sync_active				= 6;
	params->dsi.horizontal_backporch				= 37;
	params->dsi.horizontal_frontporch				= 37;
	// params->dsi.horizontal_blanking_pixel				= 60;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    
	// Bit rate calculation

	params->dsi.PLL_CLOCK= 190; //208;
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
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);	

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;
	
   
	#ifdef BUILD_LK
		printf("OTM8018B uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM8018B kernel %s \n", __func__);
		printk("J503 %s id = 0x%08x \n", __func__, id);
	#endif
	   
	return (LCM_ID_OTM8018B == id)?1:0;	//++++rgk bug-id:no modify by yangjuwei 20140221
}


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
printk("J503  lcm_vol= 0x%x,id=%x\n",lcm_vol);
   #endif
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE && lcm_compare_id())//++++rgk bug-id:no modify by yangjuwei 20140221
    {
	return 1;
    }

    return 0;
	
}


// zhoulidong  add for lcm detect (end)

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
	printk("lcm_esd_recover  otm8018b_dsi_vdo_txd_fwvga_tn \n");
	#endif
	return TRUE;
}
// zhoulidong add for eds(end)
LCM_DRIVER otm8018b_dsi_vdo_dijing_fwvga_boe_ips_lcm_drv = 
{
	.name		= "otm8018b_dsi_vdo_dijing_fwvga_boe_ips",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id	= rgk_lcm_compare_id,
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};

