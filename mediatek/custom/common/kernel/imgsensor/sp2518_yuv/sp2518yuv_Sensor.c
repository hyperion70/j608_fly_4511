/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sp2518yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.0.0
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2011/11/03 Firsty Released By Mormo;
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "sp2518yuv_Sensor.h"
#include "sp2518yuv_Camera_Sensor_para.h"
#include "sp2518yuv_CameraCustomized.h"

/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

//#define DEBUG_SENSOR_SP2518


//#define SP2518_24M_72M


kal_uint16 SP2518_write_cmos_sensor(kal_uint8 addr, kal_uint8 para);
kal_uint16 SP2518_read_cmos_sensor(kal_uint8 addr);
BOOL SP2518_set_param_wb(UINT16 para);

typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
  //0    :No-Fix FrameRate 
  //other:Fixed FrameRate
} Sp2518Status;
Sp2518Status Sp2518CurrentStatus;

static DEFINE_SPINLOCK(sp2518yuv_drv_lock);
#ifdef DEBUG_SENSOR_SP2518
	#define SP2518_OP_CODE_INI		0x00		/* Initial value. */
	#define SP2518_OP_CODE_REG		0x01		/* Register */
	#define SP2518_OP_CODE_DLY		0x02		/* Delay */
	#define SP2518_OP_CODE_END		0x03		/* End of initial setting. */
	kal_uint16 fromsd1;

		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} SP2518_initial_set_struct;

	SP2518_initial_set_struct SP2518_Init_Reg[1000];
	
 u32 strtol1(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

 u8 SP2518_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */

	
	struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static u8 data_buff[10*1024] ;
 
    fp = filp_open("/mnt/sdcard/sp2518_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
        printk("create file error\n"); 
	//	fp = filp_open("/storage/sdcard0/sp2518_sd.dat", O_RDONLY , 0); 
    	if (IS_ERR(fp)) { 
	        printk("create file error 1\n"); 
	        return 2;//-1; 

      //  return -1; 
                     } 
    } 
    else
		printk("SP2518_Initialize_from_T_Flash Open File Success\n");

    fs = get_fs(); 
    set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
    vfs_read(fp, data_buff, file_size, &pos); 
    //printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
    set_fs(fs);
	
	



	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */
			
			continue ;
		}
		
		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);
	
						
		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			SP2518_Init_Reg[i].op_code = SP2518_OP_CODE_REG;
			
			SP2518_Init_Reg[i].init_reg = strtol1((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			SP2518_Init_Reg[i].init_val = strtol1((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP2518_Init_Reg[i].op_code = SP2518_OP_CODE_DLY;
			
			SP2518_Init_Reg[i].init_reg = 0xFF;
			SP2518_Init_Reg[i].init_val = strtol1((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;
		

		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	SP2518_Init_Reg[i].op_code = SP2518_OP_CODE_END;
	SP2518_Init_Reg[i].init_reg = 0xFF;
	SP2518_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP2518_Init_Reg[j].init_reg, SP2518_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (SP2518_Init_Reg[j].op_code == SP2518_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP2518_Init_Reg[j].op_code == SP2518_OP_CODE_DLY)
		{
			msleep(SP2518_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP2518_Init_Reg[j].op_code == SP2518_OP_CODE_REG)
		{
		
			SP2518_write_cmos_sensor(SP2518_Init_Reg[j].init_reg, SP2518_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif


	return 1;	
}

#endif

//auto lum
#define SP2518_NORMAL_Y0ffset  0x10//0x20
#define SP2518_LOWLIGHT_Y0ffset  0x28//0x20

#define SP2518_MAX_GAIN_NOR   0xc0//a8
#define SP2518_MAX_GAIN_LOW  0xc8

//AE target
#define  SP2518_P0_0xf7  0x80
#define  SP2518_P0_0xf8  0x74
#define  SP2518_P0_0xf9  0x80
#define  SP2518_P0_0xfa  0x74

kal_bool   SP2518_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 SP2518_dummy_pixels = 0, SP2518_dummy_lines = 0;
kal_bool   SP2518_MODE_CAPTURE = KAL_FALSE;
kal_bool   SP2518_CAM_BANDING_50HZ = KAL_FALSE;
kal_bool   SP2518_CAM_Nightmode = 0;
kal_bool	setshutter = KAL_FALSE;
kal_uint32 SP2518_isp_master_clock;
static kal_uint32 SP2518_g_fPV_PCLK = 24;

kal_uint8 SP2518_sensor_write_I2C_address = SP2518_WRITE_ID;
kal_uint8 SP2518_sensor_read_I2C_address = SP2518_READ_ID;

UINT8 SP2518PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT SP2518SensorConfigData;

#define SP2518_SET_PAGE0 	SP2518_write_cmos_sensor(0xfd, 0x00)
#define SP2518_SET_PAGE1 	SP2518_write_cmos_sensor(0xfd, 0x01)

#define PFX "[sp2518]:"
#define SP2518YUV_DEBUG
#ifdef SP2518YUV_DEBUG
//#define SENSORDB(fmt, arg...)  printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define SENSORDB(fmt, arg...)  printk(PFX "%s: " fmt, __FUNCTION__ ,##arg)

#else
#define SENSORDB(x,...)
#endif

#if 1/*Superpix Ronlus*/
	#define WINDOW_SIZE_UXGA	0
	#define WINDOW_SIZE_720P 	1
	#define WINDOW_SIZE_SVGA 	2
	#define WINDOW_SIZE_VGA 	3
	#define SP2518_72M_8_13_FPS_50HZ	1
	#define SP2518_72M_8_13_FPS_60HZ    1
#endif
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
kal_uint16 SP2518_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2, SP2518_WRITE_ID);

}
kal_uint16 SP2518_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, SP2518_WRITE_ID);
	
    return get_byte;
}

/*************************************************************************
 * FUNCTION
 *	SP2518_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of SP2518 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP2518_Set_Shutter(kal_uint16 iShutter)
{
	SENSORDB("Ronlus SP2518_Set_Shutter\r\n");
} /* Set_SP2518_Shutter */


/*************************************************************************
 * FUNCTION
 *	SP2518_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of SP2518 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 SP2518_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	//temp_reg1 = SP2518_read_cmos_sensor(0x04);
	//temp_reg2 = SP2518_read_cmos_sensor(0x03);

	//shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	SENSORDB("Ronlus SP2518_Read_Shutter\r\n");
	return shutter;
} /* SP2518_read_shutter */


/*************************************************************************
 * FUNCTION
 *	SP2518_write_reg
 *
 * DESCRIPTION
 *	This function set the register of SP2518.
 *
 * PARAMETERS
 *	addr : the register index of SP2518
 *  para : setting parameter of the specified register of SP2518
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP2518_write_reg(kal_uint32 addr, kal_uint32 para)
{
	SP2518_write_cmos_sensor(addr, para);
} /* SP2518_write_reg() */


/*************************************************************************
 * FUNCTION
 *	SP2518_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from SP2518.
 *
 * PARAMETERS
 *	addr : the register index of SP2518
 *
 * RETURNS
 *	the data that read from SP2518
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 SP2518_read_reg(kal_uint32 addr)
{
	return SP2518_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	SP2518_ae_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void SP2518_ae_enable(kal_bool enable)
{	 
	kal_uint16 temp_AWB_reg = 0;
	SENSORDB("Ronlus SP2518_ae_enable\r\n");
	temp_AWB_reg = SP2518_read_cmos_sensor(0x32);//Ronlus 20120511
	if (enable == 1)
	{
		SP2518_write_cmos_sensor(0x32, (temp_AWB_reg |0x05));
	}
	else if(enable == 0)
	{
		SP2518_write_cmos_sensor(0x32, (temp_AWB_reg & (~0x05)));
	}
}

/*************************************************************************
* FUNCTION
*	SP2518_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void SP2518_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;
	SENSORDB("Ronlus SP2518_awb_enable\r\n");
}


static void Sp2518InitialPara(void)
{
  /*Initial status setting 
  Can be better by sync with MT9V114InitialSetting*/
  spin_lock(&sp2518yuv_drv_lock);
  Sp2518CurrentStatus.iNightMode = 0xFFFF;
  Sp2518CurrentStatus.iWB = AWB_MODE_AUTO;
  Sp2518CurrentStatus.iEffect = MEFFECT_OFF;
  Sp2518CurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
  Sp2518CurrentStatus.iEV = AE_EV_COMP_n03;
  Sp2518CurrentStatus.iMirror = IMAGE_NORMAL;
  Sp2518CurrentStatus.iFrameRate = 0;//No Fix FrameRate
  spin_unlock(&sp2518yuv_drv_lock);
}


void Sp2518GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = Sp2518CurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}
/*************************************************************************
* FUNCTION
*	SP2518_set_hb_shutter
*
* DESCRIPTION
*	This function set the dummy pixels(Horizontal Blanking) when capturing, it can be
*	used to adjust the frame rate  for back-end process.
*	
*	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
*
* PARAMETERS
*	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
*	2. kal_uint32 : shutter (Vertical Blanking)
*
* RETURNS
*	None
*
*************************************************************************/
static void SP2518_set_hb_shutter(kal_uint32 hb_add,  kal_uint32 shutter)
{
	kal_uint32 hb_ori, hb_total;
	kal_uint32 temp_reg, banding_step;
	SENSORDB("Ronlus SP2518_set_hb_shutter\r\n");
}    /* SP2518_set_dummy */

/*************************************************************************
 * FUNCTION
 *	SP2518_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of SP2518 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from SP2518
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
//void SP2518_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)//Ronlus
void SP2518_config_window(kal_uint8 index)
{
	SENSORDB("Ronlus SP2518_config_window,index = %d\r\n",index);
	switch(index)
	{
		case WINDOW_SIZE_UXGA:
			/*1600x1200 this is for resize para*/
			SP2518_write_cmos_sensor(0xfd , 0x00);	
			SP2518_write_cmos_sensor(0x47 , 0x00);
			SP2518_write_cmos_sensor(0x48 , 0x00);
			SP2518_write_cmos_sensor(0x49 , 0x04);
			SP2518_write_cmos_sensor(0x4a , 0xb0);
			SP2518_write_cmos_sensor(0x4b , 0x00);
			SP2518_write_cmos_sensor(0x4c , 0x00);
			SP2518_write_cmos_sensor(0x4d , 0x06);
			SP2518_write_cmos_sensor(0x4e , 0x40);

			SP2518_write_cmos_sensor(0xfd , 0x01);
			SP2518_write_cmos_sensor(0x06 , 0x00);
			SP2518_write_cmos_sensor(0x07 , 0x40);
			SP2518_write_cmos_sensor(0x08 , 0x00);
			SP2518_write_cmos_sensor(0x09 , 0x40);
			SP2518_write_cmos_sensor(0x0a , 0x02);
			SP2518_write_cmos_sensor(0x0b , 0x58);
			SP2518_write_cmos_sensor(0x0c , 0x03);
			SP2518_write_cmos_sensor(0x0d , 0x20);
			SP2518_write_cmos_sensor(0x0e , 0x00);//resize_en
			SP2518_write_cmos_sensor(0x0f , 0x00);
			SP2518_write_cmos_sensor(0xfd , 0x00);
			SP2518_write_cmos_sensor(0x2f , 0x00);
			break;
		case WINDOW_SIZE_720P:
			/**1280*720  this is for crop para*/
			SP2518_write_cmos_sensor(0xfd , 0x01);
			SP2518_write_cmos_sensor(0x47 , 0x00);
			SP2518_write_cmos_sensor(0x48 , 0xf0);
			SP2518_write_cmos_sensor(0x49 , 0x02);
			SP2518_write_cmos_sensor(0x4a , 0xd0);
			SP2518_write_cmos_sensor(0x4b , 0x00);
			SP2518_write_cmos_sensor(0x4c , 0xa0);
			SP2518_write_cmos_sensor(0x4d , 0x05);
			SP2518_write_cmos_sensor(0x4e , 0x00);
			break;
		case WINDOW_SIZE_SVGA:
			/*SVGA this is for resize para*/							 			
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0x4b,0x00);
			SP2518_write_cmos_sensor(0x4c,0x00);
			SP2518_write_cmos_sensor(0x47,0x00);
			SP2518_write_cmos_sensor(0x48,0x00);
			SP2518_write_cmos_sensor(0xfd,0x01);
			SP2518_write_cmos_sensor(0x06,0x00);
			SP2518_write_cmos_sensor(0x07,0x40);
			SP2518_write_cmos_sensor(0x08,0x00);
			SP2518_write_cmos_sensor(0x09,0x40);
			SP2518_write_cmos_sensor(0x0a,0x02);
			SP2518_write_cmos_sensor(0x0b,0x58);
			SP2518_write_cmos_sensor(0x0c,0x03);
			SP2518_write_cmos_sensor(0x0d,0x20);
			SP2518_write_cmos_sensor(0x0e,0x01);
			SP2518_write_cmos_sensor(0xfd,0x00);
			break;
		case WINDOW_SIZE_VGA:
			/*VGA this is for resize para*/							 
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0x35,0x00);
			SP2518_write_cmos_sensor(0x4b,0x00);
			SP2518_write_cmos_sensor(0x4c,0x00);
			SP2518_write_cmos_sensor(0x47,0x00);
			SP2518_write_cmos_sensor(0x48,0x00);
			SP2518_write_cmos_sensor(0xfd,0x01);
			SP2518_write_cmos_sensor(0x06,0x00);
			SP2518_write_cmos_sensor(0x07,0x50);
			SP2518_write_cmos_sensor(0x08,0x00);
			SP2518_write_cmos_sensor(0x09,0x50);
			SP2518_write_cmos_sensor(0x0a,0x01);
			SP2518_write_cmos_sensor(0x0b,0xe0);
			SP2518_write_cmos_sensor(0x0c,0x02);
			SP2518_write_cmos_sensor(0x0d,0x80);
			SP2518_write_cmos_sensor(0x0e,0x01);
			SP2518_write_cmos_sensor(0xfd,0x00);
			break;
		default:
			break;
	}
} /* SP2518_config_window */



/*************************************************************************
 * FUNCTION
 *	SP2518_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 SP2518_SetGain(kal_uint16 iGain)
{
	SENSORDB("Ronlus SP2518_SetGain\r\n");
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	SP2518_NightMode
 *
 * DESCRIPTION
 *	This function night mode of SP2518.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP2518_night_mode(kal_bool bEnable)
{
	//kal_uint16 night = SP2518_read_cmos_sensor(0x20);
	SENSORDB("Ronlus SP2518_night_mode\r\n");//ÒÚÐÂ½Ý³Â¿µsp2518      a8-10 ´´ºÓÌï810 3p  
	//sensorlist.cpp kd_imagesensor.h add related files  
	#if 1//
	if (bEnable)/*night mode settings*/
	{
		SENSORDB("Ronlus night mode\r\n");
		SP2518_CAM_Nightmode = 1;
		SP2518_write_cmos_sensor(0xfd,0x00);
		SP2518_write_cmos_sensor(0xb2,SP2518_LOWLIGHT_Y0ffset);
		if(SP2518_MPEG4_encode_mode == KAL_TRUE) /*video night mode*/
		{
			SENSORDB("Ronlus video night mode\r\n");

            if(SP2518_CAM_BANDING_50HZ == KAL_TRUE)/*video night mode 50hz*/
			{	
				//capture preview night 48M 50hz fix 6FPS maxgain 
				SP2518_write_cmos_sensor(0xfd,0x00);
				SP2518_write_cmos_sensor(0x03,0x01);
				SP2518_write_cmos_sensor(0x04,0xd4);
				SP2518_write_cmos_sensor(0x09,0x02);
				SP2518_write_cmos_sensor(0x0a,0x64); 
				SP2518_write_cmos_sensor(0xf0,0x4e);
				SP2518_write_cmos_sensor(0xf1,0x00);
				SP2518_write_cmos_sensor(0xfd,0x01);	
				SP2518_write_cmos_sensor(0x90,0x10);	
				SP2518_write_cmos_sensor(0x92,0x01);
				SP2518_write_cmos_sensor(0x98,0x4e);
				SP2518_write_cmos_sensor(0x99,0x00);
				SP2518_write_cmos_sensor(0x9a,0x01);
				SP2518_write_cmos_sensor(0x9b,0x00);
								  
				// status				
				SP2518_write_cmos_sensor(0xfd,0x01);
				SP2518_write_cmos_sensor(0xce,0xe0);
				SP2518_write_cmos_sensor(0xcf,0x04);
				SP2518_write_cmos_sensor(0xd0,0xe0);
				SP2518_write_cmos_sensor(0xd1,0x04);
				SP2518_write_cmos_sensor(0xd7,0x50);//exp_nr_outd_8lsb
				SP2518_write_cmos_sensor(0xd8,0x00);
				SP2518_write_cmos_sensor(0xd9,0x54);//exp_outd_nr_8lsb
				SP2518_write_cmos_sensor(0xda,0x00);
				SP2518_write_cmos_sensor(0xfd,0x00);	
				
				SENSORDB("Ronlus video night mode 50hz\r\n");
			}
			else/*video night mode 60hz*/
			{ 
				SENSORDB("Ronlus video night mode 60hz\r\n");     
			//capture preview night 48M 60hz fix 6FPS maxgain
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0x03,0x01);
			SP2518_write_cmos_sensor(0x04,0x86);
			SP2518_write_cmos_sensor(0x09,0x02);
			SP2518_write_cmos_sensor(0x0a,0x64); 
			SP2518_write_cmos_sensor(0xf0,0x41);
			SP2518_write_cmos_sensor(0xf1,0x00);
			SP2518_write_cmos_sensor(0xfd,0x01);	
			SP2518_write_cmos_sensor(0x90,0x14);	
			SP2518_write_cmos_sensor(0x92,0x01);
			SP2518_write_cmos_sensor(0x98,0x41);
			SP2518_write_cmos_sensor(0x99,0x00);
			SP2518_write_cmos_sensor(0x9a,0x01);
			SP2518_write_cmos_sensor(0x9b,0x00);
							  
			// status				
			SP2518_write_cmos_sensor(0xfd,0x01);
			SP2518_write_cmos_sensor(0xce,0x14);
			SP2518_write_cmos_sensor(0xcf,0x05);
			SP2518_write_cmos_sensor(0xd0,0x14);
			SP2518_write_cmos_sensor(0xd1,0x05);
			SP2518_write_cmos_sensor(0xd7,0x43);//exp_nr_outd_8lsb
			SP2518_write_cmos_sensor(0xd8,0x00);
			SP2518_write_cmos_sensor(0xd9,0x47);//exp_outd_nr_8lsb
			SP2518_write_cmos_sensor(0xda,0x00);
			SP2518_write_cmos_sensor(0xfd,0x00);

			}
       	 }/*capture night mode*/
		else 
		{   
			SENSORDB("Ronlus capture night mode\r\n");
            if(SP2518_CAM_BANDING_50HZ == KAL_TRUE)/*capture night mode 50hz*/
			{	
				SENSORDB("Ronlus capture night mode 50hz\r\n");//3
		//capture preview night 48M 50hz fix 6FPS maxgain 
		SP2518_write_cmos_sensor(0xfd,0x00);
		SP2518_write_cmos_sensor(0x03,0x01);
		SP2518_write_cmos_sensor(0x04,0xd4);
		SP2518_write_cmos_sensor(0x09,0x02);
		SP2518_write_cmos_sensor(0x0a,0x64); 
		SP2518_write_cmos_sensor(0xf0,0x4e);
		SP2518_write_cmos_sensor(0xf1,0x00);
		SP2518_write_cmos_sensor(0xfd,0x01);	
		SP2518_write_cmos_sensor(0x90,0x10);	
		SP2518_write_cmos_sensor(0x92,0x01);
		SP2518_write_cmos_sensor(0x98,0x4e);
		SP2518_write_cmos_sensor(0x99,0x00);
		SP2518_write_cmos_sensor(0x9a,0x01);
		SP2518_write_cmos_sensor(0x9b,0x00);
						  
		// status				
		SP2518_write_cmos_sensor(0xfd,0x01);
		SP2518_write_cmos_sensor(0xce,0xe0);
		SP2518_write_cmos_sensor(0xcf,0x04);
		SP2518_write_cmos_sensor(0xd0,0xe0);
		SP2518_write_cmos_sensor(0xd1,0x04);
		SP2518_write_cmos_sensor(0xd7,0x50);//exp_nr_outd_8lsb
		SP2518_write_cmos_sensor(0xd8,0x00);
		SP2518_write_cmos_sensor(0xd9,0x54);//exp_outd_nr_8lsb
		SP2518_write_cmos_sensor(0xda,0x00);
		SP2518_write_cmos_sensor(0xfd,0x00);	

			}
			else/*capture night mode 60hz*/
			{ 
				SENSORDB("Ronlus capture night mode 60hz\r\n");  
			//capture preview night 48M 60hz fix 6FPS maxgain
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0x03,0x01);
			SP2518_write_cmos_sensor(0x04,0x86);
			SP2518_write_cmos_sensor(0x09,0x02);
			SP2518_write_cmos_sensor(0x0a,0x64); 
			SP2518_write_cmos_sensor(0xf0,0x41);
			SP2518_write_cmos_sensor(0xf1,0x00);
			SP2518_write_cmos_sensor(0xfd,0x01);	
			SP2518_write_cmos_sensor(0x90,0x14);	
			SP2518_write_cmos_sensor(0x92,0x01);
			SP2518_write_cmos_sensor(0x98,0x41);
			SP2518_write_cmos_sensor(0x99,0x00);
			SP2518_write_cmos_sensor(0x9a,0x01);
			SP2518_write_cmos_sensor(0x9b,0x00);
							  
			// status				
			SP2518_write_cmos_sensor(0xfd,0x01);
			SP2518_write_cmos_sensor(0xce,0x14);
			SP2518_write_cmos_sensor(0xcf,0x05);
			SP2518_write_cmos_sensor(0xd0,0x14);
			SP2518_write_cmos_sensor(0xd1,0x05);
			SP2518_write_cmos_sensor(0xd7,0x43);//exp_nr_outd_8lsb
			SP2518_write_cmos_sensor(0xd8,0x00);
			SP2518_write_cmos_sensor(0xd9,0x47);//exp_outd_nr_8lsb
			SP2518_write_cmos_sensor(0xda,0x00);
			SP2518_write_cmos_sensor(0xfd,0x00);

			}
		}
	}
	else /*normal mode settings*/
	{
		SENSORDB("Ronlus normal mode\r\n");
		SP2518_CAM_Nightmode = 0;
		SP2518_write_cmos_sensor(0xfd,0x00);
		SP2518_write_cmos_sensor(0xb2,SP2518_NORMAL_Y0ffset);
        if (SP2518_MPEG4_encode_mode == KAL_TRUE) 
		{
			SENSORDB("Ronlus video normal mode\r\n");
			if(SP2518_CAM_BANDING_50HZ == KAL_TRUE)/*video normal mode 50hz*/
			{
				SENSORDB("Ronlus video normal mode 50hz\r\n");
				//capture preview daylight 48M 50hz fix 9FPS maxgain   
				SP2518_write_cmos_sensor(0xfd,0x00);  
				SP2518_write_cmos_sensor(0x03,0x02);  
				SP2518_write_cmos_sensor(0x04,0xbe);  
				SP2518_write_cmos_sensor(0x09,0x00);  
				SP2518_write_cmos_sensor(0x0a,0x64);  
				SP2518_write_cmos_sensor(0xf0,0x75);  
				SP2518_write_cmos_sensor(0xf1,0x00);  
				SP2518_write_cmos_sensor(0xfd,0x01);  
				SP2518_write_cmos_sensor(0x90,0x0b);  
				SP2518_write_cmos_sensor(0x92,0x01);  
				SP2518_write_cmos_sensor(0x98,0x75);  
				SP2518_write_cmos_sensor(0x99,0x00);  
				SP2518_write_cmos_sensor(0x9a,0x01);  
				SP2518_write_cmos_sensor(0x9b,0x00);  
				SP2518_write_cmos_sensor(0xfd,0x01);  
				SP2518_write_cmos_sensor(0xce,0x07);  
				SP2518_write_cmos_sensor(0xcf,0x05);  
				SP2518_write_cmos_sensor(0xd0,0x07);  
				SP2518_write_cmos_sensor(0xd1,0x05);  
				SP2518_write_cmos_sensor(0xd7,0x71);  
				SP2518_write_cmos_sensor(0xd8,0x00);  
				SP2518_write_cmos_sensor(0xd9,0x75);  
				SP2518_write_cmos_sensor(0xda,0x00);  
				SP2518_write_cmos_sensor(0xfd,0x00);  
			}
			else/*video normal mode 60hz*/
			{
				SENSORDB("Ronlus video normal mode 60hz\r\n");  
	//capture preview daylight 48M 60Hz fix 9FPS maxgain   
				SP2518_write_cmos_sensor(0xfd,0x00);   
				SP2518_write_cmos_sensor(0x03,0x02);   
				SP2518_write_cmos_sensor(0x04,0x4c);   
				SP2518_write_cmos_sensor(0x09,0x00);   
				SP2518_write_cmos_sensor(0x0a,0x5e);   
				SP2518_write_cmos_sensor(0xf0,0x62);   
				SP2518_write_cmos_sensor(0xf1,0x00);   
				SP2518_write_cmos_sensor(0xfd,0x01);   
				SP2518_write_cmos_sensor(0x90,0x0d);   
				SP2518_write_cmos_sensor(0x92,0x01);   
				SP2518_write_cmos_sensor(0x98,0x62);   
				SP2518_write_cmos_sensor(0x99,0x00);   
				SP2518_write_cmos_sensor(0x9a,0x01);   
				SP2518_write_cmos_sensor(0x9b,0x00);   
				SP2518_write_cmos_sensor(0xfd,0x01);   
				SP2518_write_cmos_sensor(0xce,0xfa);   
				SP2518_write_cmos_sensor(0xcf,0x04);   
				SP2518_write_cmos_sensor(0xd0,0xfa);   
				SP2518_write_cmos_sensor(0xd1,0x04);   
				SP2518_write_cmos_sensor(0xd7,0x5e);   
				SP2518_write_cmos_sensor(0xd8,0x00);   
				SP2518_write_cmos_sensor(0xd9,0x62);   
				SP2518_write_cmos_sensor(0xda,0x00);   
				SP2518_write_cmos_sensor(0xfd,0x00);  
	
			}
		}
		else/*capture normal mode*/
		{
			SENSORDB("Ronlus capture normal mode\r\n");
			if(SP2518_CAM_BANDING_50HZ == KAL_TRUE)/*capture normal mode 50hz*/
			{
				SENSORDB("Ronlus capture normal mode 50hz\r\n");/*72M 9fps 50hz*/

		SP2518_write_cmos_sensor(0xfd,0x00);  
		SP2518_write_cmos_sensor(0x03,0x02);  
		SP2518_write_cmos_sensor(0x04,0xbe);  
		SP2518_write_cmos_sensor(0x09,0x00);  
		SP2518_write_cmos_sensor(0x0a,0x64);  
		SP2518_write_cmos_sensor(0xf0,0x75);  
		SP2518_write_cmos_sensor(0xf1,0x00);  
		SP2518_write_cmos_sensor(0xfd,0x01);  
		SP2518_write_cmos_sensor(0x90,0x0b);  
		SP2518_write_cmos_sensor(0x92,0x01);  
		SP2518_write_cmos_sensor(0x98,0x75);  
		SP2518_write_cmos_sensor(0x99,0x00);  
		SP2518_write_cmos_sensor(0x9a,0x01);  
		SP2518_write_cmos_sensor(0x9b,0x00);  
		SP2518_write_cmos_sensor(0xfd,0x01);  
		SP2518_write_cmos_sensor(0xce,0x07);  
		SP2518_write_cmos_sensor(0xcf,0x05);  
		SP2518_write_cmos_sensor(0xd0,0x07);  
		SP2518_write_cmos_sensor(0xd1,0x05);  
		SP2518_write_cmos_sensor(0xd7,0x71);  
		SP2518_write_cmos_sensor(0xd8,0x00);  
		SP2518_write_cmos_sensor(0xd9,0x75);  
		SP2518_write_cmos_sensor(0xda,0x00);  
		SP2518_write_cmos_sensor(0xfd,0x00);  

			}
			else/*video normal mode 60hz*/
			{
				SENSORDB("Ronlus capture normal mode 60hz\r\n"); /*72M 9fps 60hz*/
		SP2518_write_cmos_sensor(0xfd,0x00);   
		SP2518_write_cmos_sensor(0x03,0x02);   
		SP2518_write_cmos_sensor(0x04,0x4c);   
		SP2518_write_cmos_sensor(0x09,0x00);   
		SP2518_write_cmos_sensor(0x0a,0x5e);   
		SP2518_write_cmos_sensor(0xf0,0x62);   
		SP2518_write_cmos_sensor(0xf1,0x00);   
		SP2518_write_cmos_sensor(0xfd,0x01);   
		SP2518_write_cmos_sensor(0x90,0x0d);   
		SP2518_write_cmos_sensor(0x92,0x01);   
		SP2518_write_cmos_sensor(0x98,0x62);   
		SP2518_write_cmos_sensor(0x99,0x00);   
		SP2518_write_cmos_sensor(0x9a,0x01);   
		SP2518_write_cmos_sensor(0x9b,0x00);   
		SP2518_write_cmos_sensor(0xfd,0x01);   
		SP2518_write_cmos_sensor(0xce,0xfa);   
		SP2518_write_cmos_sensor(0xcf,0x04);   
		SP2518_write_cmos_sensor(0xd0,0xfa);   
		SP2518_write_cmos_sensor(0xd1,0x04);   
		SP2518_write_cmos_sensor(0xd7,0x5e);   
		SP2518_write_cmos_sensor(0xd8,0x00);   
		SP2518_write_cmos_sensor(0xd9,0x62);   
		SP2518_write_cmos_sensor(0xda,0x00);   
		SP2518_write_cmos_sensor(0xfd,0x00);   

			}
		}
	}
	#endif
} /* SP2518_NightMode */


/*************************************************************************
* FUNCTION
*	SP2518_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/

void SP2518_Sensor_Init(void)
{

#ifdef DEBUG_SENSOR_SP2518
			if(fromsd1== 1)//ÊÇ·ñ´ÓSD¶ÁÈ¡//gepeiwei   120903
				{
				
			printk("________________from t!\n");
			SP2518_Initialize_from_T_Flash();//´ÓSD¿¨¶ÁÈ¡µÄÖ÷Òªº¯Êý
				}
			else
#endif

{
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x1b,0x1a);
SP2518_write_cmos_sensor(0x0e,0x01);
SP2518_write_cmos_sensor(0x0f,0x2f);
SP2518_write_cmos_sensor(0x10,0x2e);
SP2518_write_cmos_sensor(0x11,0x00);
SP2518_write_cmos_sensor(0x12,0x4f);
SP2518_write_cmos_sensor(0x14,0x20);
SP2518_write_cmos_sensor(0x16,0x02);
SP2518_write_cmos_sensor(0x17,0x10);
SP2518_write_cmos_sensor(0x1a,0x1f);
SP2518_write_cmos_sensor(0x1e,0x81);
SP2518_write_cmos_sensor(0x21,0x00);
SP2518_write_cmos_sensor(0x22,0x1b);
SP2518_write_cmos_sensor(0x25,0x10);
SP2518_write_cmos_sensor(0x26,0x25);
SP2518_write_cmos_sensor(0x27,0x6d);
SP2518_write_cmos_sensor(0x2c,0x23);
SP2518_write_cmos_sensor(0x2d,0x75);
SP2518_write_cmos_sensor(0x2e,0x38);
SP2518_write_cmos_sensor(0x31,0x10);
SP2518_write_cmos_sensor(0x44,0x03);
SP2518_write_cmos_sensor(0x6f,0x00);
SP2518_write_cmos_sensor(0xa0,0x04);
SP2518_write_cmos_sensor(0x5f,0x01);
SP2518_write_cmos_sensor(0x32,0x00);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x2c,0x00);
SP2518_write_cmos_sensor(0x2d,0x00);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xfb,0x83);
SP2518_write_cmos_sensor(0xf4,0x09);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0xc6,0x90);
SP2518_write_cmos_sensor(0xc7,0x90);
SP2518_write_cmos_sensor(0xc8,0x90);
SP2518_write_cmos_sensor(0xc9,0x90);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x65,0x08);
SP2518_write_cmos_sensor(0x66,0x08);
SP2518_write_cmos_sensor(0x67,0x08);
SP2518_write_cmos_sensor(0x68,0x08);
SP2518_write_cmos_sensor(0x46,0xff);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xe0,0x6c);
SP2518_write_cmos_sensor(0xe1,0x54);
SP2518_write_cmos_sensor(0xe2,0x48);
SP2518_write_cmos_sensor(0xe3,0x40);
SP2518_write_cmos_sensor(0xe4,0x40);
SP2518_write_cmos_sensor(0xe5,0x3e);
SP2518_write_cmos_sensor(0xe6,0x3e);
SP2518_write_cmos_sensor(0xe8,0x3a);
SP2518_write_cmos_sensor(0xe9,0x3a);
SP2518_write_cmos_sensor(0xea,0x3a);
SP2518_write_cmos_sensor(0xeb,0x38);
SP2518_write_cmos_sensor(0xf5,0x38);
SP2518_write_cmos_sensor(0xf6,0x38);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x94,0xc0);
SP2518_write_cmos_sensor(0x95,0x38);
SP2518_write_cmos_sensor(0x9c,0x6c);
SP2518_write_cmos_sensor(0x9d,0x38);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x30,0x04);
SP2518_write_cmos_sensor(0x03,0x02);
SP2518_write_cmos_sensor(0x04,0xbe);
SP2518_write_cmos_sensor(0x09,0x00);
SP2518_write_cmos_sensor(0x0a,0x64);
SP2518_write_cmos_sensor(0xf0,0x75);
SP2518_write_cmos_sensor(0xf1,0x00);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x90,0x0b);
SP2518_write_cmos_sensor(0x92,0x01);
SP2518_write_cmos_sensor(0x98,0x75);
SP2518_write_cmos_sensor(0x99,0x00);
SP2518_write_cmos_sensor(0x9a,0x01);
SP2518_write_cmos_sensor(0x9b,0x00);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0xce,0x07);
SP2518_write_cmos_sensor(0xcf,0x05);
SP2518_write_cmos_sensor(0xd0,0x07);
SP2518_write_cmos_sensor(0xd1,0x05);
SP2518_write_cmos_sensor(0xd7,0x71);
SP2518_write_cmos_sensor(0xd8,0x00);
SP2518_write_cmos_sensor(0xd9,0x75);
SP2518_write_cmos_sensor(0xda,0x00);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0xca,0x30);
SP2518_write_cmos_sensor(0xcb,0x50);
SP2518_write_cmos_sensor(0xcc,0xc0);
SP2518_write_cmos_sensor(0xcd,0xc0);
SP2518_write_cmos_sensor(0xd5,0x80);
SP2518_write_cmos_sensor(0xd6,0x90);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xa1,0x20);
SP2518_write_cmos_sensor(0xa2,0x20);
SP2518_write_cmos_sensor(0xa3,0x20);
SP2518_write_cmos_sensor(0xa4,0xff);
SP2518_write_cmos_sensor(0xa5,0x80);
SP2518_write_cmos_sensor(0xa6,0x80);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x64,0x26);
SP2518_write_cmos_sensor(0x65,0x21);
SP2518_write_cmos_sensor(0x66,0x23);
SP2518_write_cmos_sensor(0x67,0x23);
SP2518_write_cmos_sensor(0x68,0x26);
SP2518_write_cmos_sensor(0x69,0x20);
SP2518_write_cmos_sensor(0x6a,0x23);
SP2518_write_cmos_sensor(0x6b,0x21);
SP2518_write_cmos_sensor(0x6c,0x1b);
SP2518_write_cmos_sensor(0x6d,0x16);
SP2518_write_cmos_sensor(0x6e,0x18);
SP2518_write_cmos_sensor(0x6f,0x18);
SP2518_write_cmos_sensor(0xb8,0x0a);
SP2518_write_cmos_sensor(0xb9,0x0c);
SP2518_write_cmos_sensor(0xba,0x0c);
SP2518_write_cmos_sensor(0xbb,0x08);
SP2518_write_cmos_sensor(0xbc,0x04);
SP2518_write_cmos_sensor(0xbd,0x05);
SP2518_write_cmos_sensor(0xbe,0x05);
SP2518_write_cmos_sensor(0xbf,0x07);
SP2518_write_cmos_sensor(0xc0,0x06);
SP2518_write_cmos_sensor(0xc1,0x08);
SP2518_write_cmos_sensor(0xc2,0x09);
SP2518_write_cmos_sensor(0xc3,0x0d);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0xde,0x0f);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x57,0x04);
SP2518_write_cmos_sensor(0x58,0x04);
SP2518_write_cmos_sensor(0x56,0x04);
SP2518_write_cmos_sensor(0x59,0x08);
SP2518_write_cmos_sensor(0x5a,0xf0);
SP2518_write_cmos_sensor(0xc4,0xf0);
SP2518_write_cmos_sensor(0x43,0xf0);
SP2518_write_cmos_sensor(0xad,0xc0);
SP2518_write_cmos_sensor(0x4f,0xf0);
SP2518_write_cmos_sensor(0xc3,0xf0);
SP2518_write_cmos_sensor(0x3f,0xf0);
SP2518_write_cmos_sensor(0x42,0xc0);
SP2518_write_cmos_sensor(0xc2,0x15);
SP2518_write_cmos_sensor(0xb6,0xf0);
SP2518_write_cmos_sensor(0xb7,0xf0);
SP2518_write_cmos_sensor(0xb8,0xf0);
SP2518_write_cmos_sensor(0xb9,0xc0);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x50,0x06);
SP2518_write_cmos_sensor(0x51,0x06);
SP2518_write_cmos_sensor(0x52,0x08);
SP2518_write_cmos_sensor(0x53,0x08);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x11,0x10);
SP2518_write_cmos_sensor(0x12,0x1f);
SP2518_write_cmos_sensor(0x16,0x1c);
SP2518_write_cmos_sensor(0x18,0x00);
SP2518_write_cmos_sensor(0x19,0x00);
SP2518_write_cmos_sensor(0x1b,0x96);
SP2518_write_cmos_sensor(0x1a,0x9a);
SP2518_write_cmos_sensor(0x1e,0x2f);
SP2518_write_cmos_sensor(0x1f,0x29);
SP2518_write_cmos_sensor(0x28,0xce);
SP2518_write_cmos_sensor(0x29,0x8a);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xe7,0x03);
SP2518_write_cmos_sensor(0xe7,0x00);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x2a,0xf0);
SP2518_write_cmos_sensor(0x2b,0x10);
SP2518_write_cmos_sensor(0x2e,0x04);
SP2518_write_cmos_sensor(0x2f,0x18);
SP2518_write_cmos_sensor(0x21,0x60);
SP2518_write_cmos_sensor(0x23,0x60);
SP2518_write_cmos_sensor(0x8b,0xab);
SP2518_write_cmos_sensor(0x8f,0x12);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x1a,0x80);
SP2518_write_cmos_sensor(0x1b,0x80);
SP2518_write_cmos_sensor(0x43,0x80);
SP2518_write_cmos_sensor(0x00,0xd4);
SP2518_write_cmos_sensor(0x01,0xb0);
SP2518_write_cmos_sensor(0x02,0x90);
SP2518_write_cmos_sensor(0x03,0x78);
SP2518_write_cmos_sensor(0x35,0xd0);
SP2518_write_cmos_sensor(0x36,0xf4);
SP2518_write_cmos_sensor(0x37,0x70);
SP2518_write_cmos_sensor(0x38,0x94);
SP2518_write_cmos_sensor(0x39,0xc7);
SP2518_write_cmos_sensor(0x3a,0xf4);
SP2518_write_cmos_sensor(0x3b,0x89);
SP2518_write_cmos_sensor(0x3c,0xb2);
SP2518_write_cmos_sensor(0x31,0x84);
SP2518_write_cmos_sensor(0x32,0xa8);
SP2518_write_cmos_sensor(0x33,0xb8);
SP2518_write_cmos_sensor(0x34,0xe0);
SP2518_write_cmos_sensor(0x3d,0xb4);
SP2518_write_cmos_sensor(0x3e,0xd8);
SP2518_write_cmos_sensor(0x3f,0xaa);
SP2518_write_cmos_sensor(0x40,0xd0);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x1c,0xb8);
SP2518_write_cmos_sensor(0x1d,0xa0);
SP2518_write_cmos_sensor(0xa0,0x73);
SP2518_write_cmos_sensor(0xa1,0xcd);
SP2518_write_cmos_sensor(0xa2,0x40);
SP2518_write_cmos_sensor(0xa3,0xf4);
SP2518_write_cmos_sensor(0xa4,0xa6);
SP2518_write_cmos_sensor(0xa5,0xe7);
SP2518_write_cmos_sensor(0xa6,0xed);
SP2518_write_cmos_sensor(0xa7,0xd4);
SP2518_write_cmos_sensor(0xa8,0xc7);
SP2518_write_cmos_sensor(0xa9,0x0c);
SP2518_write_cmos_sensor(0xaa,0x33);
SP2518_write_cmos_sensor(0xab,0x0f);
SP2518_write_cmos_sensor(0xac,0x97);
SP2518_write_cmos_sensor(0xad,0xf5);
SP2518_write_cmos_sensor(0xae,0xf5);
SP2518_write_cmos_sensor(0xaf,0xee);
SP2518_write_cmos_sensor(0xb0,0xa5);
SP2518_write_cmos_sensor(0xb1,0xee);
SP2518_write_cmos_sensor(0xb2,0xf7);
SP2518_write_cmos_sensor(0xb3,0xdb);
SP2518_write_cmos_sensor(0xb4,0xaf);
SP2518_write_cmos_sensor(0xb5,0x3c);
SP2518_write_cmos_sensor(0xb6,0x33);
SP2518_write_cmos_sensor(0xb7,0x0f);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xbf,0x01);
SP2518_write_cmos_sensor(0xbe,0xbb);
SP2518_write_cmos_sensor(0xc0,0xb0);
SP2518_write_cmos_sensor(0xc1,0xf0);
SP2518_write_cmos_sensor(0xd3,0x68);
SP2518_write_cmos_sensor(0xd4,0x68);
SP2518_write_cmos_sensor(0xd6,0x68);
SP2518_write_cmos_sensor(0xd7,0x68);
SP2518_write_cmos_sensor(0xd8,0x68);
SP2518_write_cmos_sensor(0xd9,0x68);
SP2518_write_cmos_sensor(0xda,0x68);
SP2518_write_cmos_sensor(0xdb,0x68);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xf3,0x03);
SP2518_write_cmos_sensor(0xb0,0x00);
SP2518_write_cmos_sensor(0xb1,0x23);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x8b,0x00);
SP2518_write_cmos_sensor(0x8c,0x0A);
SP2518_write_cmos_sensor(0x8d,0x13);
SP2518_write_cmos_sensor(0x8e,0x25);
SP2518_write_cmos_sensor(0x8f,0x43);
SP2518_write_cmos_sensor(0x90,0x5D);
SP2518_write_cmos_sensor(0x91,0x74);
SP2518_write_cmos_sensor(0x92,0x88);
SP2518_write_cmos_sensor(0x93,0x9A);
SP2518_write_cmos_sensor(0x94,0xA9);
SP2518_write_cmos_sensor(0x95,0xB5);
SP2518_write_cmos_sensor(0x96,0xC0);
SP2518_write_cmos_sensor(0x97,0xCA);
SP2518_write_cmos_sensor(0x98,0xD4);
SP2518_write_cmos_sensor(0x99,0xDD);
SP2518_write_cmos_sensor(0x9a,0xE6);
SP2518_write_cmos_sensor(0x9b,0xEF);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x8d,0xF7);
SP2518_write_cmos_sensor(0x8e,0xFF);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x78,0x0 );
SP2518_write_cmos_sensor(0x79,0xA );
SP2518_write_cmos_sensor(0x7a,0x13);
SP2518_write_cmos_sensor(0x7b,0x25);
SP2518_write_cmos_sensor(0x7c,0x43);
SP2518_write_cmos_sensor(0x7d,0x5D);
SP2518_write_cmos_sensor(0x7e,0x74);
SP2518_write_cmos_sensor(0x7f,0x88);
SP2518_write_cmos_sensor(0x80,0x9A);
SP2518_write_cmos_sensor(0x81,0xA9);
SP2518_write_cmos_sensor(0x82,0xB5);
SP2518_write_cmos_sensor(0x83,0xC0);
SP2518_write_cmos_sensor(0x84,0xCA);
SP2518_write_cmos_sensor(0x85,0xD4);
SP2518_write_cmos_sensor(0x86,0xDD);
SP2518_write_cmos_sensor(0x87,0xE6);
SP2518_write_cmos_sensor(0x88,0xEF);
SP2518_write_cmos_sensor(0x89,0xF7);
SP2518_write_cmos_sensor(0x8a,0xFF);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x96,0x46);
SP2518_write_cmos_sensor(0x97,0x14);
SP2518_write_cmos_sensor(0x9f,0x06);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xdd,0x74);
SP2518_write_cmos_sensor(0xde,0x94);
SP2518_write_cmos_sensor(0xdf,0x80);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xec,0x6c);
SP2518_write_cmos_sensor(0xed,0x7c);
SP2518_write_cmos_sensor(0xee,0x6c);
SP2518_write_cmos_sensor(0xef,0x7c);
SP2518_write_cmos_sensor(0xf7,0x80);
SP2518_write_cmos_sensor(0xf8,0x74);
SP2518_write_cmos_sensor(0xf9,0x80);
SP2518_write_cmos_sensor(0xfa,0x74);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0xdf,0x0f);
SP2518_write_cmos_sensor(0xe5,0x10);
SP2518_write_cmos_sensor(0xe7,0x10);
SP2518_write_cmos_sensor(0xe8,0x40);
SP2518_write_cmos_sensor(0xec,0x40);
SP2518_write_cmos_sensor(0xe9,0x40);
SP2518_write_cmos_sensor(0xed,0x40);
SP2518_write_cmos_sensor(0xea,0x28);
SP2518_write_cmos_sensor(0xef,0x20);
SP2518_write_cmos_sensor(0xeb,0x20);
SP2518_write_cmos_sensor(0xf0,0x18);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x70,0x76);
SP2518_write_cmos_sensor(0x7b,0x40);
SP2518_write_cmos_sensor(0x81,0x30);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0xb2,0x10);
SP2518_write_cmos_sensor(0xb3,0x1f);
SP2518_write_cmos_sensor(0xb4,0x30);
SP2518_write_cmos_sensor(0xb5,0x50);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x5b,0x20);
SP2518_write_cmos_sensor(0x61,0x80);
SP2518_write_cmos_sensor(0x77,0x80);
SP2518_write_cmos_sensor(0xca,0x80);
SP2518_write_cmos_sensor(0xab,0x00);
SP2518_write_cmos_sensor(0xac,0x02);
SP2518_write_cmos_sensor(0xae,0x08);
SP2518_write_cmos_sensor(0xaf,0x20);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x35,0x40);
SP2518_write_cmos_sensor(0x32,0x0d);
SP2518_write_cmos_sensor(0x33,0xcf);
SP2518_write_cmos_sensor(0x34,0x7f);
SP2518_write_cmos_sensor(0xe7,0x03);
SP2518_write_cmos_sensor(0xe7,0x00);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x47,0x00);
SP2518_write_cmos_sensor(0x48,0x00);
SP2518_write_cmos_sensor(0x49,0x04);
SP2518_write_cmos_sensor(0x4a,0xb0);
SP2518_write_cmos_sensor(0x4b,0x00);
SP2518_write_cmos_sensor(0x4c,0x00);
SP2518_write_cmos_sensor(0x4d,0x06);
SP2518_write_cmos_sensor(0x4e,0x40);
SP2518_write_cmos_sensor(0xfd,0x01);
SP2518_write_cmos_sensor(0x06,0x00);
SP2518_write_cmos_sensor(0x07,0x40);
SP2518_write_cmos_sensor(0x08,0x00);
SP2518_write_cmos_sensor(0x09,0x40);
SP2518_write_cmos_sensor(0x0a,0x02);
SP2518_write_cmos_sensor(0x0b,0x58);
SP2518_write_cmos_sensor(0x0c,0x03);
SP2518_write_cmos_sensor(0x0d,0x20);
SP2518_write_cmos_sensor(0x0e,0x00);
SP2518_write_cmos_sensor(0x0f,0x00);
SP2518_write_cmos_sensor(0xfd,0x00);
SP2518_write_cmos_sensor(0x2f,0x00);

}
}
	
/*************************************************************************
* FUNCTION
*	SP2518_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_SP2518() directly.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void SP2518_Write_More_Registers(void)
{
	//Ronlus this function is used for FAE debug
}



/*************************************************************************
* FUNCTION
*	SP2518_PV_setting
*
* DESCRIPTION
*	This function apply the preview mode setting, normal the preview size is 1/4 of full size.
*	Ex. 2M (1600 x 1200)
*	Preview: 800 x 600 (Use sub-sample or binning to acheive it)
*	Full Size: 1600 x 1200 (Output every effective pixels.)
*
* PARAMETERS
*	1. image_sensor_exposure_window_struct : Set the grab start x,y and width,height.
*	2. image_sensor_config_struct : Current operation mode.
*
* RETURNS
*	None
*
*************************************************************************/
static void SP2518_PV_setting(void)
{
	SENSORDB("Ronlus SP2518_PV_setting\r\n");
} /* SP2518_PV_setting */


/*************************************************************************
* FUNCTION
*	SP2518_CAP_setting
*
* DESCRIPTION
*	This function apply the full size mode setting.
*	Ex. 2M (1600 x 1200)
*	Preview: 800 x 600 (Use sub-sample or binning to acheive it)
*	Full Size: 1600 x 1200 (Output every effective pixels.)
*
* PARAMETERS
*	1. image_sensor_exposure_window_struct : Set the grab start x,y and width,height.
*	2. image_sensor_config_struct : Current operation mode.
*
* RETURNS
*	None
*
*************************************************************************/
static void SP2518_CAP_setting(void)
{
	kal_uint16 corr_r_offset,corr_g_offset,corr_b_offset,temp = 0;
	SENSORDB("Ronlus SP2518_CAP_setting\r\n");	
} /* SP2518_CAP_setting */
/*************************************************************************
 * FUNCTION
 *	SP2518Open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 SP2518Open(void)
{
    kal_uint16 sensor_id=0;
    int i;
	SENSORDB("Ronlus SP2518Open\r\n");
	// check if sensor ID correct
	for(i = 0; i < 3; i++)
	{
		SP2518_write_cmos_sensor(0xfd,0x00);
		sensor_id = SP2518_read_cmos_sensor(0x02);
		SENSORDB("Ronlus SP2518 Sensor id = %x\n", sensor_id);
		if (sensor_id == SP2518_SENSOR_ID)
		{
			break;
		}
	}
	mdelay(50);
    if(sensor_id != SP2518_SENSOR_ID)
    {
        SENSORDB("SP2518 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

#ifdef DEBUG_SENSOR_SP2518  //gepeiwei   120903
						//ÅÐ¶ÏÊÖ»ú¶ÔÓ¦Ä¿Â¼ÏÂÊÇ·ñÓÐÃûÎªsp2528_sd µÄÎÄ¼þ,Ã»ÓÐÄ¬ÈÏ²ÎÊý
	
						//½éÓÚ¸÷ÖÖÔ­Òò£¬±¾°æ±¾³õÊ¼»¯²ÎÊýÔÚ_s_fmtÖÐ¡£
	 struct file *fp; 
		mm_segment_t fs; 
		loff_t pos = 0; 
		static char buf[10*1024] ;
	 
		fp = filp_open("/mnt/sdcard/sp2518_sd", O_RDONLY , 0); 
		if (IS_ERR(fp)) { 
			fromsd1 = 0;   
			printk("open file error\n");
			
		} 
		else 
			{
			fromsd1 = 1;
		printk("open file ok\n");
			
		//SP2518_Initialize_from_T_Flash();
	
	
		filp_close(fp, NULL); 
		set_fs(fs);
		}
#endif  
Sp2518InitialPara();
    //RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
    // initail sequence write in
    SP2518_Sensor_Init();
    SP2518_Write_More_Registers();//added for FAE to debut
    return ERROR_NONE;
} /* SP2518Open */


/*************************************************************************
 * FUNCTION
 *	SP2518Close
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 SP2518Close(void)
{
	SENSORDB("Ronlus SP2518Close\r\n");
    return ERROR_NONE;
} /* SP2518Close */


/*************************************************************************
 * FUNCTION
 * SP2518Preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 SP2518Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;
	SENSORDB("Ronlus SP2518Preview fun start\r\n");
	setshutter = KAL_FALSE;
    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        SENSORDB("Ronlus video preview\r\n");
        SP2518_MPEG4_encode_mode = KAL_TRUE;
    }
    else
    {
        SENSORDB("Ronlus capture preview\r\n");
        SP2518_MPEG4_encode_mode = KAL_FALSE;
    }
	//SP2518_config_window(WINDOW_SIZE_SVGA);//add zch test(use this for SVGA)
    image_window->GrabStartX= IMAGE_SENSOR_SVGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_SVGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_FULL_WIDTH; 
    image_window->ExposureWindowHeight =IMAGE_SENSOR_FULL_HEIGHT;
    // copy sensor_config_data
    memcpy(&SP2518SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));//rotation
    SENSORDB("Ronlus SP2518Preview fun end\r\n");
    return ERROR_NONE;
} /* SP2518Preview */


/*************************************************************************
 * FUNCTION
 *	SP2518Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 SP2518Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
	 SENSORDB("Ronlus SP2518Capture fun start\r\n");
    SP2518_MODE_CAPTURE=KAL_TRUE;

	#if 1
     if ((image_window->ImageTargetWidth<=IMAGE_SENSOR_SVGA_WIDTH)&&
        (image_window->ImageTargetHeight<=IMAGE_SENSOR_SVGA_HEIGHT))
		{    /* Less than PV Mode */
			image_window->GrabStartX = IMAGE_SENSOR_SVGA_GRAB_PIXELS;
			image_window->GrabStartY = IMAGE_SENSOR_SVGA_GRAB_LINES;
			image_window->ExposureWindowWidth= IMAGE_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = IMAGE_SENSOR_PV_HEIGHT;
		}
     else
	 #endif
	 
	{
		kal_uint32 shutter, cap_dummy_pixels = 0; 
		if(!setshutter)
		{
			//SP2518_ae_enable(KAL_FALSE);
			//SP2518_awb_enable(KAL_FALSE);
			//shutter = SP2518_Read_Shutter();
			//SP2518_CAP_setting();
			//SP2518_set_hb_shutter(cap_dummy_pixels, shutter);
		}	
		//SP2518_config_window(WINDOW_SIZE_UXGA);//add zch test(user this for SVGA)
		image_window->GrabStartX = IMAGE_SENSOR_UXGA_GRAB_PIXELS;
		image_window->GrabStartY = IMAGE_SENSOR_UXGA_GRAB_LINES;
		image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
		image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;
	}
    // copy sensor_config_data
    memcpy(&SP2518SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    SENSORDB("Ronlus SP2518Capture fun end\r\n");
    return ERROR_NONE;
} /* SP2518_Capture() */



UINT32 SP2518GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SENSORDB("Ronlus SP2518GetResolution\r\n");
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_UXGA_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_UXGA_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_FULL_WIDTH;//IMAGE_SENSOR_SVGA_WIDTH;(user this for SVGA)
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_FULL_HEIGHT;//IMAGE_SIMAGE_SENSOR_SVGA_HEIGHT;(use this for SVGA)
        pSensorResolution->SensorVideoWidth =IMAGE_SENSOR_FULL_WIDTH;//IMAGE_SENSOR_SVGA_WIDTH;(user this for SVGA)
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_FULL_HEIGHT;//IMAGE_SIMAGE_SENSOR_SVGA_HEIGHT;(use this for SVGA)
    return ERROR_NONE;
} /* SP2518GetResolution() */


UINT32 SP2518GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("Ronlus SP2518GetInfo\r\n");
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_UXGA_WIDTH;//IMAGE_SENSOR_PV_WIDTH(use this for SVGA);
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_UXGA_HEIGHT;//IMAGE_SENSOR_PV_HEIGHT(user this for SVGA);
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

    pSensorInfo->CaptureDelayFrame = 1;
	pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 4;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_SVGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_SVGA_GRAB_LINES;

        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
  //  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_UXGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_UXGA_GRAB_LINES;
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_SVGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_SVGA_GRAB_LINES;
        break;
    }
    SP2518PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &SP2518SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP2518GetInfo() */


UINT32 SP2518Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("Ronlus SP2518Control\r\n");
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        SP2518Preview(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        SP2518Capture(pImageWindow, pSensorConfigData);
        break;
    }


    return TRUE;
}	/* SP2518Control() */

BOOL SP2518_set_param_wb(UINT16 para)
{
	SENSORDB("Ronlus SP2518_set_param_wb\r\n");
	switch (para)
	 {
	   case AWB_MODE_OFF:
			 //SP2518_write_cmos_sensor(0xfd,0x00);
			// SP2518_write_cmos_sensor(0x32,0x05);
		 break; 					
	   case AWB_MODE_AUTO:
	   	        SP2518_write_cmos_sensor(0xfd,0x00);
	   	        SP2518_write_cmos_sensor(0xe7,0x03);
			 SP2518_write_cmos_sensor(0xfd,0x01);
			 SP2518_write_cmos_sensor(0x28,0xce);
			 SP2518_write_cmos_sensor(0x29,0x8a);	   
			 SP2518_write_cmos_sensor(0xfd,0x00);
			 SP2518_write_cmos_sensor(0xe7,0x00);
			 SP2518_write_cmos_sensor(0x32,0x0d);
			 
		 break;
	   case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			 SP2518_write_cmos_sensor(0xfd,0x00);
			 SP2518_write_cmos_sensor(0x32,0x05);
			 SP2518_write_cmos_sensor(0xfd,0x01);
			 SP2518_write_cmos_sensor(0x28,0xe2);
			 SP2518_write_cmos_sensor(0x29,0x82);
			 SP2518_write_cmos_sensor(0xfd,0x00);
		 break;
	   case AWB_MODE_DAYLIGHT: //sunny
			 SP2518_write_cmos_sensor(0xfd,0x00);
			 SP2518_write_cmos_sensor(0x32,0x05);
			 SP2518_write_cmos_sensor(0xfd,0x01);
			 SP2518_write_cmos_sensor(0x28,0xc1);
			 SP2518_write_cmos_sensor(0x29,0x88);
			 SP2518_write_cmos_sensor(0xfd,0x00);
		 break;
	   case AWB_MODE_INCANDESCENT: //office
			 SP2518_write_cmos_sensor(0xfd,0x00);
			 SP2518_write_cmos_sensor(0x32,0x05);
			 SP2518_write_cmos_sensor(0xfd,0x01);
			 SP2518_write_cmos_sensor(0x28,0x7b);
			 SP2518_write_cmos_sensor(0x29,0xd3);
			 SP2518_write_cmos_sensor(0xfd,0x00);
		 break;
	   case AWB_MODE_TUNGSTEN: //home
			 SP2518_write_cmos_sensor(0xfd,0x00);
			 SP2518_write_cmos_sensor(0x32,0x05);
			 SP2518_write_cmos_sensor(0xfd,0x01);
			 SP2518_write_cmos_sensor(0x28,0xae);
			 SP2518_write_cmos_sensor(0x29,0xcc);
			 SP2518_write_cmos_sensor(0xfd,0x00);
		 break;
	   case AWB_MODE_FLUORESCENT:
			 SP2518_write_cmos_sensor(0xfd,0x00);
			 SP2518_write_cmos_sensor(0x32,0x05);
			 SP2518_write_cmos_sensor(0xfd,0x01);
			 SP2518_write_cmos_sensor(0x28,0x98);
			 SP2518_write_cmos_sensor(0x29,0xc4);
			SP2518_write_cmos_sensor(0xfd,0x00);
		default:
		 return FALSE;
	 }  
	spin_lock(&sp2518yuv_drv_lock);
    Sp2518CurrentStatus.iWB = para;
	spin_unlock(&sp2518yuv_drv_lock);
	
	return TRUE;
} /* SP2518_set_param_wb */


BOOL SP2518_set_param_effect(UINT16 para)
{
	SENSORDB("Ronlus SP2518_set_param_effect\r\n");
	 switch (para)
	  {
	    case MEFFECT_OFF:
			SP2518_write_cmos_sensor(0xfd, 0x00);
			SP2518_write_cmos_sensor(0x62, 0x00);
			SP2518_write_cmos_sensor(0x63, 0x80);
			SP2518_write_cmos_sensor(0x64, 0x80);
	      break;
	    case MEFFECT_SEPIA:
			SP2518_write_cmos_sensor(0xfd, 0x00);
			SP2518_write_cmos_sensor(0x62, 0x10);
			SP2518_write_cmos_sensor(0x63, 0xb0);
			SP2518_write_cmos_sensor(0x64, 0x40);
	      break;
	    case MEFFECT_NEGATIVE://----datasheet
			SP2518_write_cmos_sensor(0xfd, 0x00);
			SP2518_write_cmos_sensor(0x62, 0x04);
			SP2518_write_cmos_sensor(0x63, 0x80);
			SP2518_write_cmos_sensor(0x64, 0x80);
	      break;
	    case MEFFECT_SEPIAGREEN://----datasheet aqua
			SP2518_write_cmos_sensor(0xfd, 0x00);
			SP2518_write_cmos_sensor(0x62, 0x10);
			SP2518_write_cmos_sensor(0x63, 0x50);
			SP2518_write_cmos_sensor(0x64, 0x50);
	      break;
	    case MEFFECT_SEPIABLUE:
			SP2518_write_cmos_sensor(0xfd, 0x00);
			SP2518_write_cmos_sensor(0x62, 0x10);
			SP2518_write_cmos_sensor(0x63, 0x80);
			SP2518_write_cmos_sensor(0x64, 0xb0);
	      break;
	    case MEFFECT_MONO: //----datasheet black & white
			SP2518_write_cmos_sensor(0xfd, 0x00);
			SP2518_write_cmos_sensor(0x62, 0x20);
			SP2518_write_cmos_sensor(0x63, 0x80);
			SP2518_write_cmos_sensor(0x64, 0x80);
	      break;
	    default:
	      return FALSE;
	  }
  return TRUE;
} /* SP2518_set_param_effect */

UINT8 index = 1;
BOOL SP2518_set_param_banding(UINT16 para)
{
	//UINT16 buffer = 0;
	SENSORDB("Ronlus SP2518_set_param_banding para = %d ---- index = %d\r\n",para,index); 
	SENSORDB("Ronlus SP2518_set_param_banding ---- SP2518_MPEG4_encode_mode = %d\r\n",SP2518_MPEG4_encode_mode);
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			SP2518_CAM_BANDING_50HZ = KAL_TRUE;
			break;
		case AE_FLICKER_MODE_60HZ:
			SP2518_CAM_BANDING_50HZ = KAL_FALSE;
			break;
		default:
			//SP2518_write_cmos_sensor(0xfd,0x00);//ronlus test
			//buffer = SP2518_read_cmos_sensor(0x35);
			return FALSE;
	}
	#if 0/*Superpix Ronlus some vertical line when switching*/
		SP2518_config_window(WINDOW_SIZE_SVGA);
		SP2518_write_cmos_sensor(0xfd,0x00);
		SP2518_write_cmos_sensor(0x35,0x00);
	#endif
	return TRUE;
} /* SP2518_set_param_banding */


BOOL SP2518_set_param_exposure(UINT16 para)
{
	SENSORDB("Ronlus SP2518_set_param_exposure\r\n");
	switch (para)
	  {
	    case AE_EV_COMP_n13:              /* EV -2 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0xc0);
	      break;
	    case AE_EV_COMP_n10:              /* EV -1.5 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0xd0);
	      break;
	    case AE_EV_COMP_n07:              /* EV -1 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0xe0);
	      break;
	    case AE_EV_COMP_n03:              /* EV -0.5 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0xf0);
	      break;
	    case AE_EV_COMP_00:                /* EV 0 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0x00);
	      break;
	    case AE_EV_COMP_03:              /* EV +0.5 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0x10);
	      break;
	    case AE_EV_COMP_07:              /* EV +1 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0x20);
	      break;
	    case AE_EV_COMP_10:              /* EV +1.5 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0x30);
	      break;
	    case AE_EV_COMP_13:              /* EV +2 */
			SP2518_write_cmos_sensor(0xfd,0x00);
			SP2518_write_cmos_sensor(0xdc,0x40);
	      break;
	    default:
	      return FALSE;
	  }
  return TRUE;
} /* SP2518_set_param_exposure */


UINT32 SP2518YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    #ifdef DEBUG_SENSOR_SP2518
    return TRUE;
    #endif
	SENSORDB("Ronlus SP2518YUVSensorSetting\r\n");
    switch (iCmd) 
	{
		case FID_SCENE_MODE:	    
		    if (iPara == SCENE_MODE_OFF)
		    {
				SP2518_night_mode(0); 
		    }
		    else if (iPara == SCENE_MODE_NIGHTSCENE)
		    {
				SP2518_night_mode(1); 
		    }	    
		    break; 	    
	    case FID_AWB_MODE:
	        SP2518_set_param_wb(iPara);
	        break;
	    case FID_COLOR_EFFECT:
	        SP2518_set_param_effect(iPara);
	        break;
	    case FID_AE_EV:
	        SP2518_set_param_exposure(iPara);
	        break;
	    case FID_AE_FLICKER:
	        SP2518_set_param_banding(iPara);
			SP2518_night_mode(SP2518_CAM_Nightmode); 
	        break;
	    default:
        	break;
    }
    return TRUE;
} /* SP2518YUVSensorSetting */

UINT32 sp2518_get_sensor_id(UINT32 *sensorID) 
{
		   volatile signed char i;
		kal_uint16 sensor_id=0;
		  SENSORDB("xieyang SP2518GetSensorID ");
	
		  for(i=0;i<3;i++)
		{
			   SP2518_write_cmos_sensor(0xfd, 0x00); 
			sensor_id = SP2518_read_cmos_sensor(0x02);
			SENSORDB("%s sensor_id=%d\n", __func__, sensor_id);
			if(sensor_id != SP2518_SENSOR_ID)
			{
			        *sensorID = 0xFFFFFFFF;
				return ERROR_SENSOR_CONNECT_FAIL;
			}
		}
	 *sensorID=sensor_id;
		return ERROR_NONE;
}



UINT32 SP2518FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SP2518SensorRegNumber;
    UINT32 i;
	SENSORDB("Ronlus SP2518FeatureControl.---FeatureId = %d\r\n",FeatureId);
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    RETAILMSG(1, (_T("gaiyang SP2518FeatureControl FeatureId=%d\r\n"), FeatureId));

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=UXGA_PERIOD_PIXEL_NUMS;
        *pFeatureReturnPara16=UXGA_PERIOD_LINE_NUMS;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(SVGA_PERIOD_PIXEL_NUMS)+SP2518_dummy_pixels;
        *pFeatureReturnPara16=(SVGA_PERIOD_LINE_NUMS)+SP2518_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = SP2518_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        #ifdef DEBUG_SENSOR_SP2518
        SP2518_night_mode((BOOL) *pFeatureData16);
    #endif
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        SP2518_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        SP2518_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = SP2518_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &SP2518SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        sp2518_get_sensor_id(pFeatureReturnPara32); 
        break; 
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        SP2518YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32); 
			Sp2518GetExifInfo(*pFeatureData32);
			break;
    default:
        break;
	}
return ERROR_NONE;
}	/* SP2518FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP2518YUV=
{
	SP2518Open,
	SP2518GetInfo,
	SP2518GetResolution,
	SP2518FeatureControl,
	SP2518Control,
	SP2518Close
};


UINT32 SP2518_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	SENSORDB("Ronlus SP2518_YUV_SensorInit\r\n");
	if (pfFunc!=NULL)
	{
		 SENSORDB("Ronlus SP2518_YUV_SensorInit fun_config success\r\n");
		*pfFunc=&SensorFuncSP2518YUV;
	}
	return ERROR_NONE;
} /* SensorInit() */
