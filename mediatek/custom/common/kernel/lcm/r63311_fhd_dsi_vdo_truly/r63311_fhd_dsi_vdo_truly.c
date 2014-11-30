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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_R63311 (0x01223311)

#define REGFLAG_DELAY    		0XFE
#define REGFLAG_END_OF_TABLE   	0xFF   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0

static void init_lcm_registers(void)
{
    
	#ifdef BUILD_LK
		printf("WKP nt35521 init_lcm_registers\n");
	#else
		printk("WKP nt35521 init_lcm_registers\n");
	#endif

    unsigned int data_array[16];

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1,1);
    MDELAY(120);

    
    data_array[0] = 0x00022902;
    data_array[1] = 0x000004B0;
    dsi_set_cmdq(data_array, 2, 1);


    data_array[0] = 0x00072902;
    data_array[1] = 0x000014B3;
    data_array[2] = 0x00000000;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00032902;
    data_array[1] = 0x00B33AB6;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00032902;
    data_array[1] = 0x000000C0;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00232902;
    data_array[1] = 0x106084C1;
    data_array[2] = 0xCE6FFFEB;
    data_array[3] = 0x1293FFFF;
    data_array[4] = 0x31AE7358;
    data_array[5] = 0xFFFFC430;
    data_array[6] = 0x5FFFF31F;
    data_array[7] = 0x10101010;
    data_array[8] = 0x22036202;
    data_array[9] = 0x00010022;
    dsi_set_cmdq(data_array, 10, 1);

    data_array[0] = 0x00072902;
    data_array[1] = 0x80F731C2;
    data_array[2] = 0x00800806;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00172902;
    data_array[1] = 0x000070C4;
    data_array[2] = 0x00000000;
    data_array[3] = 0x060C0000;
    data_array[4] = 0x00000000;
    data_array[5] = 0x00000000;
    data_array[6] = 0x00060C00;
    dsi_set_cmdq(data_array, 7, 1);


    data_array[0] = 0x00292902;
    data_array[1] = 0x006900C6;
    data_array[2] = 0x00690069;
    data_array[3] = 0x00000000;
    data_array[4] = 0x00690069;
    data_array[5] = 0x07191069;
    data_array[6] = 0x69000100;
    data_array[7] = 0x69006900;
    data_array[8] = 0x00000000;
    data_array[9] = 0x69006900;
    data_array[10] = 0x19106900;
    data_array[11] = 0x00000007;
    dsi_set_cmdq(data_array, 12, 1);

    data_array[0] = 0x000A2902;
    data_array[1] = 0x3FFC31CB;
    data_array[2] = 0x0000008C;
    data_array[3] = 0x000000C0;
    dsi_set_cmdq(data_array, 4, 1);

    data_array[0] = 0x000A2902;
    data_array[1] = 0x00000BCC;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x000B2902;
    data_array[1] = 0xBB8111D0;
    data_array[2] = 0x194C1B1B;
    data_array[3] = 0x00000C19;
    dsi_set_cmdq(data_array, 4, 1);

    data_array[0] = 0x001A2902;
    data_array[1] = 0xBB331BD3;
    data_array[2] = 0x3333B3BB;
    data_array[3] = 0x00010133;
    data_array[4] = 0x0DA0D8A0;
    data_array[5] = 0x3B335F5F;
    data_array[6] = 0x3D077222;
    data_array[7] = 0x000077BF;
    dsi_set_cmdq(data_array, 8, 1);

    data_array[0] = 0x00082902;
    data_array[1] = 0x000006D5;
    data_array[2] = 0x32015101;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x001F2902;
    data_array[1] = 0x0B0800C7;
    data_array[2] = 0x41332415;
    data_array[3] = 0x52453C54;
    data_array[4] = 0x786F6862;
    data_array[5] = 0x150B0800;
    data_array[6] = 0x54413324;
    data_array[7] = 0x6252453C;
    data_array[8] = 0x00786F68;   
    dsi_set_cmdq(data_array, 9, 1);

    data_array[0] = 0x00142902;
    data_array[1] = 0x000001C8;
    data_array[2] = 0x00FC0000;
    data_array[3] = 0x00000000;
    data_array[4] = 0x000000FC;
    data_array[5] = 0x00FC0000; 
    dsi_set_cmdq(data_array, 6, 1);

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(200);
}



static struct LCM_setting_table lcm_initialization_setting[] = {
	
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


//20131122		 
	{0xB0, 1, {0x04}},

	{0x00, 0,   {0x00}},

	{0x00, 0,   {0x00}},	

	{0xB3, 6, {0x14,0x00,0x00,0x00,0x00,0x00}},

	{0xB4, 3, {0x0C,0x00,0x00}},
	
	{0xB6, 3, {0x3A,0xB3,0x00}},	
	
	{0xC1, 34, {0x84,0x60,0x40,0xEB,0xFF,
		    0x6F,0xCE,0xFF,0xFF,0x17,
		    0x02,0x58,0x73,0xAE,0xB1,
		    0x20,0xC6,0xFF,0xFF,0x1F,
		    0xF3,0xFF,0x5F,0x10,0x10,
		    0x10,0x10,0x00,0x00,0x01,
		    0x02,0x02,0x00,0x01}},
	
	{0xC2, 7, {0x31,0xF7,0x80,0x0A,0x08,
		   0x00,0x00}},

	{0xC3, 7, {0x01,0x00,0x00}},

	{0xC4, 22, {0x70,0x00,0x00,0x00,0x00,
		    0x04,0x00,0x03,0x00,0x0C,
		    0x06,0x00,0x00,0x00,0x00,
		    0x00,0x04,0x00,0x03,0x00,
		    0x0C,0x06}},
	
	{0xC6, 40, {0x00,0x79,0x00,0x79,0x00,
		    0x79,0x00,0x00,0x00,0x00,
		    0x00,0x79,0x00,0x79,0x00,
		    0x79,0x10,0x19,0x07,0x00,
		    0x01,0x00,0x79,0x00,0x79,
		    0x00,0x79,0x00,0x00,0x00,
		    0x00,0x00,0x79,0x00,0x79,
		    0x00,0x79,0x10,0x19,0x07}},
	
	{0xC7, 24, {0x04,0x09,0x13,0x1B,0x26,
		    0x40,0x39,0x49,0x57,0x66,
		    0x6A,0x70,0x04,0x09,0x13,
		    0x1B,0x26,0x40,0x36,0x49,
                    0x5F,0x66,0x6A,0x70}},

	{0xC8, 24, {0x04,0x09,0x13,0x1B,0x26,
		    0x40,0x39,0x49,0x57,0x66,
		    0x6A,0x70,0x04,0x09,0x13,
		    0x1B,0x26,0x40,0x36,0x49,
                    0x5F,0x66,0x6A,0x70}},

	{0xC9, 24, {0x04,0x09,0x13,0x1B,0x26,
		    0x40,0x39,0x49,0x57,0x66,
		    0x6A,0x70,0x04,0x09,0x13,
		    0x1B,0x26,0x40,0x36,0x49,
                    0x5F,0x66,0x6A,0x70}},


	{0xCA, 34, {0x00,0xA0,0x80,0x80,0x80,
                    0x80,0x80,0x80,0x0C,0x20,
		    0x00,0xFF,0x0A,0x4A,0x37,
                    0xA0,0x55,0xF8,0x0C,0x0C,
                    0x20,0x10,0x20,0x20,0x00,
                    0x00,0x10,0x10,0x3F,0x3F,
                    0x3F,0x3F,0x00,0x00}},	

	{0xCB, 9, {0x31,0xFC,0x3F,0x8C,0x00,
		   0x00,0x00,0x00,0xC0}},

	{0xCC, 1, {0x0A}},


	{0xCD, 3, {0x00,0x00,0xFF}},


	{0xCE, 7, {0x00,0x06,0x08,0xC1,0x00,
		   0x1E,0x04}},

	{0xCF, 5, {0x00,0x00,0xC1,0x05,0x3F}},


	{0xD0, 14, {0x00,0x00,0x19,0x18,0x99,
		    0x99,0x19,0x01,0x89,0x00,
                    0x55,0x19,0x99,0x01}},

	{0xD1, 29, {0x28,0x00,0x00,0x08,0x0C,
		    0x10,0x18,0x00,0x00,0x00,
		    0x00,0x00,0x3C,0x04,0x28,
	            0x00,0x00,0x08,0x0C,0x10,
	            0x18,0x00,0x00,0x3C,0x05,
                    0x40,0x00,0x32,0x31}},

	{0xD2, 3, {0x5C,0x00,0x00}},
	
	{0xD3, 26, {0x1B,0x33,0xBB,0xCC,0xC4,
		    0x33,0x33,0x33,0x00,0x01,
		    0x00,0xA0,0xD8,0xA0,0x0D,
	            0x41,0x33,0x44,0x22,0x70,
	            0x02,0x41,0x03,0x3D,0xBF,
                    0x00}},
	
	{0xD5, 7, {0x06,0x00,0x00,0x01,0x4F,
		   0x01,0x4F}},
	//send 2 times
	{0xD5, 7, {0x06,0x00,0x00,0x01,0x42,
		   0x01,0x42}},	



	{0xD7, 20, {0x84,0xE0,0x7F,0xA8,0xCE,
		    0x38,0xFC,0xC1,0x83,0xE7,
		    0x8F,0x1F,0x3C,0x10,0xFA,
	            0xC3,0x0F,0x04,0x41,0x00}},
	
	{0xD8, 6,  {0x00,0x80,0x80,0x40,0x42,
		    0x14}},

	{0xD9, 2, {0x00,0x00}},


	{0xDD, 2,  {0x10,0xB3}},

	{0xDE, 6,  {0x00,0xFF,0x07,0x10,0x00,
                    0x73}},
   	
	 	

	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag	{REGFLAG_END_OF_TABLE, 0x00, {}}
	{REGFLAG_END_OF_TABLE, 0x00, {}}			 
};


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
	
}

static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
	//	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

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
	//	params->dsi.word_count=720*3;	
		
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 25;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active			= 20;
		params->dsi.horizontal_backporch				= 70;
		params->dsi.horizontal_frontporch				= 100;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.pll_select=0;	//0: MIPI_PLL; 1: LVDS_PLL
		// Bit rate calculation
		//1 Every lane speed
		
		params->dsi.PLL_CLOCK = 460;

}

static void lcm_init(void)
{


	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	
	SET_RESET_PIN(1);
	MDELAY(50); 


#if 1	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	
	unsigned int data_array[16];
	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

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



//deep standby
    data_array[0] = 0x00022902;
    data_array[1] = 0x000004B0;
    dsi_set_cmdq(data_array, 2,1);
    MDELAY(10); 
	
    data_array[0] = 0x00022902;
    data_array[1] = 0x000001B1;
    dsi_set_cmdq(data_array, 2,1);	
    MDELAY(90); 

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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
  #ifdef BUILD_LK
				printf("[LK]---WKP r63311 lcm_compare_id----\n");
    #else
				printk("[KERNEL]---WKP r63311 lcm_compare_id----\n");
    #endif

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
	
	read_reg_v2(0xBF, buffer, 4);
	id = buffer[0]<<24 | buffer[1]<<16 |buffer[2]<<8 | buffer[3];

    if(id == LCM_ID_R63311)
    	return 1;
    else
        return 0;
}

LCM_DRIVER r63311_fhd_dsi_vdo_truly_lcm_drv = 
{
    .name			= "r63311_fhd_dsi_vdo_lcm_drv:5.0FHD",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
