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

#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
	#include <platform/upmu_common.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#include <cust_adc.h>			// zhoulidong  add for lcm detect
#define MIN_VOLTAGE		(200)	// zhoulidong  add for lcm detect
#define MAX_VOLTAGE		(400)	// zhoulidong  add for lcm detect

#define FRAME_WIDTH		(1200)
#define FRAME_HEIGHT		(1920)

#define LCM_ID_R63315 (0x01223315)

#define REGFLAG_DELAY    		0XFE
#define REGFLAG_END_OF_TABLE		0xFF   // END OF REGISTERS MARKER

#ifndef TRUE
	#define TRUE 1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

#define dsi_lcm_set_gpio_out(pin, out)					lcm_util.set_gpio_out(pin, out)
#define dsi_lcm_set_gpio_mode(pin, mode)				lcm_util.set_gpio_mode(pin, mode)
#define dsi_lcm_set_gpio_dir(pin, dir)					lcm_util.set_gpio_dir(pin, dir)
#define dsi_lcm_set_gpio_pull_enable(pin, en)				lcm_util.set_gpio_pull_enable(pin, en)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE					1

static struct LCM_setting_table lcm_initialization_setting[] = {
	


};


static void init_lcm_registers(void)
{
	unsigned int data_array[16];

#if 0
	//{0x01, 0,   {}},
	data_array[0] = 0x00010500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);

	data_array[0] = 0x04B02300;  //MCAP						 
    	dsi_set_cmdq(data_array, 1, 1); 			 
	

	data_array[0] = 0x000B2902;  //interface setting	
	data_array[1] = 0x000001E1;  //5 paras
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;   	
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x00062902;  //interface setting	
	data_array[1] = 0x000804B3;  //5 paras
	data_array[2] = 0x00000002;  	
	dsi_set_cmdq(data_array, 3, 1);
	

	data_array[0] = 0x00022902;  //interface ID setting
	data_array[1] = 0x00000CB4;  
	dsi_set_cmdq(data_array, 2, 1); 

	data_array[0] = 0x00032902;  //DSI control
	data_array[1] = 0x00D33AB6;   
	dsi_set_cmdq(data_array, 2, 1); 

	data_array[0] = 0xE6511500; 
	dsi_set_cmdq(data_array, 1, 1);
 
	data_array[0] = 0x2C531500; 
	dsi_set_cmdq(data_array, 1, 1); 

	data_array[0] = 0x00022902;  //DSI control
	data_array[1] = 0x000000C0;   
	dsi_set_cmdq(data_array, 2, 1); 
	
	data_array[0] = 0x00252902;  //DSI control
	data_array[1] = 0x006104C1;
	data_array[2] = 0x40A10020;
	data_array[3] = 0xF71C0001;  
	data_array[4] = 0xB1AD62E0;
	data_array[5] = 0x003B9F05; 
	data_array[6] = 0x00940280;  
	data_array[7] = 0x11110001;
	data_array[8] = 0x02801111;
	data_array[9] = 0x00320000;  
	data_array[10] = 0x00000001;   
	dsi_set_cmdq(data_array, 11, 1); 

	data_array[0] = 0x000A2902;
	data_array[1] = 0x80F731C2;
	data_array[2] = 0x08000800;
	data_array[3] = 0x00000000; 
	dsi_set_cmdq(data_array, 4, 1);	
	
	data_array[0] = 0x000D2902;
	data_array[1] = 0x000070C4;
	data_array[2] = 0x00000000;
	data_array[3] = 0x02070000; 
	data_array[4] = 0x00000000; 
	dsi_set_cmdq(data_array, 5, 1);


	data_array[0] = 0x00152902;
	data_array[1] = 0x450172C6;
	data_array[2] = 0x0A676705;
	data_array[3] = 0x01010101;
	data_array[4] = 0x01010101;
	data_array[5] = 0x190A0101;  
	data_array[6] = 0x00000005; 
	dsi_set_cmdq(data_array, 7, 1);
	
	
	//Gamma setting Common
	//{0xC7, 30, {0x0B,0x1E,0x27,0x31,0x3F,0x4C,0x55,0x64,0x47,0x4D,
	//0x56,0x5E,0x62,0x66,0x76,0x0B,0x1E,0x27,0x31,0x3F,
	//0x4C,0x55,0x64,0x47,0x4D,0x56,0x5E,0x62,0x66,0x76}},

	data_array[0] = 0x001F2902;
	data_array[1] = 0x271E0BC7;
	data_array[2] = 0x554C3F31;
	data_array[3] = 0x564D4764;
	data_array[4] = 0x7666625E;
	data_array[5] = 0x31271E0B;  
	data_array[6] = 0x64554C3F;
	data_array[7] = 0x5E564D47;
	data_array[8] = 0x00766662; 
	dsi_set_cmdq(data_array, 9, 1);

	//Digital gamma setting
	
	data_array[0] = 0x00142902;
	data_array[1] = 0x000000C8;
	data_array[2] = 0xF0FCFEFE;
	data_array[3] = 0x00000000;
	data_array[4] = 0x000000FC;
	data_array[5] = 0x00FC0000;  
	dsi_set_cmdq(data_array, 6, 1);

	//MFD
	data_array[0] = 0x00052902;
	data_array[1] = 0x010301C9;
	data_array[2] = 0x00000053;
	  
	dsi_set_cmdq(data_array, 3, 1);


	//Panel pin control
	data_array[0] = 0x000D2902;
	data_array[1] = 0xFFFC7ACB;
	data_array[2] = 0x00040178;
	data_array[3] = 0x3C008000;
	data_array[4] = 0x000000CF;
	dsi_set_cmdq(data_array, 5, 1);


	//Panel interface control	
	//{0xCC, 1, {0x07}},
	data_array[0] = 0x07CC2300;  //MCAP						 
	dsi_set_cmdq(data_array, 1, 1); 

	//Power control function
	data_array[0] = 0x000B2902;
	data_array[1] = 0xBB8155D0;
	data_array[2] = 0x194CCE55;
	data_array[3] = 0x00000C19;
	dsi_set_cmdq(data_array, 4, 1);


	data_array[0] = 0x001B2902;
	data_array[1] = 0xBF330BD3;
	data_array[2] = 0x3333B3BB;
	data_array[3] = 0x00010037;
	data_array[4] = 0x04A0D8A0;
	data_array[5] = 0x3B334D4D;
	data_array[6] = 0x3D077037;
	data_array[7] = 0x001155BF;
	dsi_set_cmdq(data_array, 8, 1);	
	
	//Vcom setting function		
	data_array[0] = 0x00082902;
	data_array[1] = 0x000006D5;
	data_array[2] = 0x23012301;
	dsi_set_cmdq(data_array, 3, 1);

	//test register
	data_array[0] = 0x00172902;
	data_array[1] = 0x7FFC84D7;
	data_array[2] = 0xFC38CEA8;
	data_array[3] = 0x8FE71EC1;
	data_array[4] = 0xFA103C1F;
	data_array[5] = 0x41040FC3;
	data_array[6] = 0x00000020;
	dsi_set_cmdq(data_array, 7, 1);			
			
	//{0xD6, 1, {0x01}},
	data_array[0] = 0x01D62300;  //MCAP						 
	dsi_set_cmdq(data_array, 1, 1); 

	//{0xE3, 1, {0x3D}},
	data_array[0] = 0x3DE32300;  //MCAP						 
	dsi_set_cmdq(data_array, 1, 1); 

	//{0x35, 1, {0x01}},
	data_array[0] = 0x01351500; 
	dsi_set_cmdq(data_array, 1, 1); 
	//{0xE0, 1, {0x00}},
	data_array[0] = 0x00052902;
	data_array[1] = 0x000011E0;
	data_array[2] = 0x00000000;
	dsi_set_cmdq(data_array, 3, 1);


	data_array[0] = 0x00052902;
	data_array[1] = 0x000011E0;
	data_array[2] = 0x00000040;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(2000);

	data_array[0] = 0x00061402;
	data_array[1] = 0x000000E0;
	data_array[2] = 0x00000300;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00052902;
	data_array[1] = 0x000000E0;
	data_array[2] = 0x00000000;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);	
#else

	data_array[0] = 0x00010500; // RESET
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
	
	data_array[0] = 0x00B02300; //MCAP
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00062902;
	data_array[1] = 0x000804B3;
	data_array[2] = 0x00000022;
	dsi_set_cmdq(data_array, 3, 1);
	
	
	
	data_array[0] = 0x00022902;
	data_array[1] = 0x00000CB4;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00032902;
	data_array[1] = 0x00D33AB6;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0xE6511500; 
	dsi_set_cmdq(data_array, 1, 1);
 
	data_array[0] = 0x2C531500; 
	dsi_set_cmdq(data_array, 1, 1); 
	
	data_array[0] = 0x773A1500; 
	dsi_set_cmdq(data_array, 1, 1); 
	
	data_array[0] = 0x00053902;
	data_array[1] = 0x0400002A;
	data_array[2] = 0x000000AF;
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0] = 0x00053902;
	data_array[1] = 0x0700002B;
	data_array[2] = 0x0000007F;
	dsi_set_cmdq(data_array, 3, 1);
		
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
		
	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

#endif
	
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;
		switch (cmd)
		{
		case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE :
                	break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_get_params(LCM_PARAMS *params)
{

	memset(params, 0, sizeof(LCM_PARAMS));
#ifdef BUILD_LK
	printf("[LK]---lcm_get_params_start----\n");
#else
	printk("[KERNEL]---lcm_get_params_start----\n");
#endif
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	//params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//params->dsi.word_count=720*3;	

		
	params->dsi.vertical_sync_active				= 4;//2
	params->dsi.vertical_backporch					= 4;//8
	params->dsi.vertical_frontporch					= 8;//25
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 16;// 4
	params->dsi.horizontal_backporch				= 45;// 118
	params->dsi.horizontal_frontporch				= 144;// 118
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	params->dsi.pll_select=0;//closed
	// Bit rate calculation
	//1 Every lane speed
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
	params->dsi.fbk_div =23;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
	//params->dsi.PLL_CLOCK = 40;

}


static void rgk_lcm_power_on(void)
{

#ifdef BUILD_LK
	upmu_set_rg_vgp1_vosel(7);//lcd_3.3v
	upmu_set_rg_vgp1_en(1);
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP1, VOL_3300, "LCM");
#endif 
	MDELAY(20);
	dsi_lcm_set_gpio_mode(GPIO_LCM_PWR2_EN, GPIO_MODE_GPIO);//GPIO80 lcd_power_en
	dsi_lcm_set_gpio_dir(GPIO_LCM_PWR2_EN, GPIO_DIR_OUT);
	dsi_lcm_set_gpio_out(GPIO_LCM_PWR2_EN, GPIO_OUT_ONE);
	MDELAY(20);
	dsi_lcm_set_gpio_mode(GPIO_LCM_PWR , GPIO_MODE_GPIO);//GPIO168 lcd_dcdc_en
	dsi_lcm_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
	dsi_lcm_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ONE);
	MDELAY(20);
}

static void rgk_lcm_supend_power(void)
{

#ifdef BUILD_LK
	upmu_set_rg_vgp1_vosel(0);//lcd_3.3v
	upmu_set_rg_vgp1_en(0);
#else
	hwPowerDown(MT65XX_POWER_LDO_VGP1,"LCM");
#endif 
	dsi_lcm_set_gpio_out(GPIO_LCM_PWR2_EN, GPIO_OUT_ZERO);//GPIO80 lcd_power_en
	dsi_lcm_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);//GPIO168 lcd_dcdc_en
	MDELAY(20);
}
static void lcm_init(void)
{


	rgk_lcm_power_on();

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);	
	SET_RESET_PIN(1);
	MDELAY(100); 
	
	init_lcm_registers();

	#ifdef BUILD_LK
		printf("[LK]---lcm ok----\n");
  	#else
		printk("[KERNEL]---lcm ok----\n");
    	#endif  

}



static void lcm_suspend(void)
{
	unsigned int data_array[16];
	SET_RESET_PIN(1);
	MDELAY(20);	

	SET_RESET_PIN(0);
	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(20);	
	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50); 

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	rgk_lcm_supend_power(); 

}


static void lcm_resume(void)
{
	unsigned int data_array[16];
#ifdef BUILD_LK
	printf("[LK]---lcm resume----\n");
#else
	printk("[KERNEL]---lcm resume----\n");
#endif  
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

//	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
//	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[5];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 

	array[0] = 0x00043700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xBF, buffer, 5);
	id = buffer[0]<<24 | buffer[1]<<16 |buffer[2]<<8 | buffer[3];

	if(id == LCM_ID_R63315)
    		return 1;
	else
        	return 0;
}


static int rgk_lcm_compare_id(void)
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
	
	if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE)
	{
		return 1;
	}

	return 0;
}

int lcm_setbacklight(unsigned int level)
{
	unsigned int data_array[16];
	unsigned int tmp_level;
	char buffer[5];
	int flags = 0;
    static int bl_en = 0;
    
	if(level > 255)
	{
		level = 255;
	}
	if(level>0)
	{ 	
        if(!bl_en)
        {
            dsi_lcm_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_GPIO);//GPIO90
            dsi_lcm_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
            dsi_lcm_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE); 
            bl_en = 1;
        }
//		#ifndef BUILD_LK
//		local_irq_save(flags); 
//		#endif
		tmp_level = level<<24;	
		data_array[0] = tmp_level | 0x00511500;		
		dsi_set_cmdq(data_array, 1, 1);
//		#ifndef BUILD_LK
//		local_irq_restore(flags); 
//		#endif

	}
	else
	{
        if(bl_en)
        {	
		    data_array[0] = 0x00511500;		
		    dsi_set_cmdq(data_array, 1, 1);
            dsi_lcm_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_GPIO);//GPIO90
            dsi_lcm_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
            dsi_lcm_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
            bl_en = 0;
        }
	}
	return 1;	
}



LCM_DRIVER r69429_wuxga_dsi_cmd_songxia_lcm_drv = 
{
    .name			= "r69429_wuxga_dsi_cmd_songxia",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.set_backlight	= lcm_setbacklight,
//	.resume_power	= rgk_lcm_power_on,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
	
#endif
    };
