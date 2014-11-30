/*******************************************************************************************/
     

/*******************************************************************************************/

#include <linux/videodev2.h>    
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov8858mipiraw_Sensor.h"
#include "ov8858mipiraw_Camera_Sensor_para.h"
#include "ov8858mipiraw_CameraCustomized.h"
//#include "TRULY_OTP_OV8858.c"
static DEFINE_SPINLOCK(ov8858mipiraw_drv_lock);

#define OV8858_DEBUG
#ifdef OV8858_DEBUG
	#define OV8858DB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV8858Raw] ",  fmt, ##arg)
#else
	#define OV8858DB(fmt, arg...)
#endif


kal_uint32 OV8858_FeatureControl_PERIOD_PixelNum=OV8858_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV8858_FeatureControl_PERIOD_LineNum=OV8858_PV_PERIOD_LINE_NUMS;

UINT16 OV8858_VIDEO_MODE_TARGET_FPS = 30;

MSDK_SCENARIO_ID_ENUM OV8858CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT OV8858SensorConfigData;
static OV8858_PARA_STRUCT ov8858;
kal_uint32 OV8858_FAC_SENSOR_REG;


SENSOR_REG_STRUCT OV8858SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV8858SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;


//TODO~
#define OV8858_TEST_PATTERN_CHECKSUM 0x71ac8b4d
kal_bool OV8858_During_testpattern = KAL_FALSE;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV8858_CHECK_OTP
#ifdef OV8858_CHECK_OTP
extern bool otp_update_wb(unsigned short golden_rg, unsigned short golden_bg);
extern bool otp_update_lenc(void);
#endif
//#define OV8858_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV8858MIPI_WRITE_ID)

kal_uint16 OV8858_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,OV8858MIPI_WRITE_ID);
    return get_byte;
}

kal_uint16 OV8858_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	kal_uint16 err = 0;
	err = iWriteReg((u16) addr , (u32) para , 1, OV8858MIPI_WRITE_ID);
	return err;
}

void OV8858_Init_Para(void)
{

	spin_lock(&ov8858mipiraw_drv_lock);
	ov8858.sensorMode = SENSOR_MODE_INIT;
	ov8858.OV8858AutoFlickerMode = KAL_FALSE;
	ov8858.OV8858VideoMode = KAL_FALSE;
	ov8858.DummyLines= 0;
	ov8858.DummyPixels= 0;
	ov8858.pvPclk =  (7200); 
	ov8858.videoPclk = (7200);
	ov8858.capPclk = (7200);

	ov8858.shutter = 0x4C00;
	ov8858.ispBaseGain = BASEGAIN;		//64
	ov8858.sensorGlobalGain = 0x0200;  //512
	spin_unlock(&ov8858mipiraw_drv_lock);
}

kal_uint32 GetOv8858LineLength(void)
{
	kal_uint32 OV8858_line_length = 0;
	if ( SENSOR_MODE_PREVIEW == ov8858.sensorMode )  
	{
		OV8858_line_length = OV8858_PV_PERIOD_PIXEL_NUMS + ov8858.DummyPixels;
	}
	else if( SENSOR_MODE_VIDEO == ov8858.sensorMode ) 
	{
		OV8858_line_length = OV8858_VIDEO_PERIOD_PIXEL_NUMS + ov8858.DummyPixels;
	}
	else
	{
		OV8858_line_length = OV8858_FULL_PERIOD_PIXEL_NUMS + ov8858.DummyPixels;
	}
	
#ifdef OV8858_DEBUG
	OV8858DB("[GetOv8858LineLength]: ov8858.sensorMode = %d, OV8858_line_length =%d, ov8858.DummyPixels = %d\n", ov8858.sensorMode,OV8858_line_length,ov8858.DummyPixels);
#endif


    return OV8858_line_length;

}


kal_uint32 GetOv8858FrameLength(void)
{
	kal_uint32 OV8858_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == ov8858.sensorMode )  
	{
		OV8858_frame_length = OV8858_PV_PERIOD_LINE_NUMS + ov8858.DummyLines ;
	}
	else if( SENSOR_MODE_VIDEO == ov8858.sensorMode ) 
	{
		OV8858_frame_length = OV8858_VIDEO_PERIOD_LINE_NUMS + ov8858.DummyLines ;
	}
	else
	{
		OV8858_frame_length = OV8858_FULL_PERIOD_LINE_NUMS + ov8858.DummyLines ;
	}

#ifdef OV8858_DEBUG
		OV8858DB("[GetOv8858FrameLength]: ov8858.sensorMode = %d, OV8858_frame_length =%d, ov8858.DummyLines = %d\n", ov8858.sensorMode,OV8858_frame_length,ov8858.DummyLines);
#endif


	return OV8858_frame_length;
}


kal_uint32 OV8858_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	
	if (shutter_value <4 ){
		shutter_value = 4;
	}

	
	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

#ifdef OV8858_DEBUG
			OV8858DB("[OV8858_CalcExtra_For_ShutterMargin]: shutter_value = %d, shutterLimitation =%d, extra_lines = %d\n", shutter_value,shutterLimitation,extra_lines);
#endif

    return extra_lines;

}


//TODO~
kal_uint32 OV8858_CalcFrameLength_For_AutoFlicker(void)
{

    kal_uint32 AutoFlicker_min_framelength = 0;

	switch(OV8858CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			AutoFlicker_min_framelength = (ov8858.capPclk*10000) /(OV8858_FULL_PERIOD_LINE_NUMS + ov8858.DummyPixels)/OV8858_AUTOFLICKER_OFFSET_30*10 ;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(OV8858_VIDEO_MODE_TARGET_FPS==30)
			{
				AutoFlicker_min_framelength = (ov8858.videoPclk*10000) /(OV8858_VIDEO_PERIOD_PIXEL_NUMS + ov8858.DummyPixels)/OV8858_AUTOFLICKER_OFFSET_30*10 ;
			}
			else if(OV8858_VIDEO_MODE_TARGET_FPS==15)
			{
				AutoFlicker_min_framelength = (ov8858.videoPclk*10000) /(OV8858_VIDEO_PERIOD_PIXEL_NUMS + ov8858.DummyPixels)/OV8858_AUTOFLICKER_OFFSET_15*10 ;
			}
			else
			{
				AutoFlicker_min_framelength = OV8858_VIDEO_PERIOD_LINE_NUMS + ov8858.DummyLines;
			}
			break;
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			AutoFlicker_min_framelength = (ov8858.pvPclk*10000) /(OV8858_PV_PERIOD_PIXEL_NUMS + ov8858.DummyPixels)/OV8858_AUTOFLICKER_OFFSET_30*10 ;
			break;
	}

	#ifdef OV8858_DEBUG 
	OV8858DB("AutoFlicker_min_framelength =%d,OV8858CurrentScenarioId =%d\n", AutoFlicker_min_framelength,OV8858CurrentScenarioId);
	#endif

	return AutoFlicker_min_framelength;

}


void OV8858_write_shutter(kal_uint32 shutter)
{
	//kal_uint32 min_framelength = OV8858_PV_PERIOD_PIXEL_NUMS;
	//the init code write as up line;
	//modify it as follow
	kal_uint32 min_framelength = OV8858_PV_PERIOD_LINE_NUMS;
	kal_uint32 max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	//TODO~
	kal_uint32 read_shutter_1 = 0;
	kal_uint32 read_shutter_2 = 0;
	kal_uint32 read_shutter_3 = 0;

	//TODO~
    if(shutter > 0x90f7)//500ms for capture SaturationGain
    {
    	#ifdef OV8858_DEBUG
		OV8858DB("[OV8858_write_shutter] shutter > 0x90f7 [warn.] shutter=%x, \n", shutter);
		#endif
		shutter = 0x90f7;
    }
	
    line_length  = GetOv8858LineLength();
	frame_length = GetOv8858FrameLength();
	
	max_shutter  = frame_length-OV8858_SHUTTER_MARGIN;

    frame_length = frame_length + OV8858_CalcExtra_For_ShutterMargin(shutter,max_shutter);
	


	if(ov8858.OV8858AutoFlickerMode == KAL_TRUE)
	{
        min_framelength = OV8858_CalcFrameLength_For_AutoFlicker();

        if(frame_length < min_framelength)
			frame_length = min_framelength;
	}
	

	spin_lock_irqsave(&ov8858mipiraw_drv_lock,flags);
	OV8858_FeatureControl_PERIOD_PixelNum = line_length;
	OV8858_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&ov8858mipiraw_drv_lock,flags);

	//Set total frame length
	OV8858_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV8858_write_cmos_sensor(0x380f, frame_length & 0xFF);
	
	//Set shutter 
	OV8858_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	OV8858_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	OV8858_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	

	#ifdef OV8858_DEBUG
	OV8858DB("[OV8858_write_shutter]ov8858 write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);
	#endif

}


void OV8858_SetShutter(kal_uint32 iShutter)
{

   spin_lock(&ov8858mipiraw_drv_lock);
   ov8858.shutter= iShutter;
   spin_unlock(&ov8858mipiraw_drv_lock);

   OV8858_write_shutter(iShutter);
   return;
}


UINT32 OV8858_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV8858_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV8858_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV8858_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

static kal_uint16 OV8858Reg2Gain(const kal_uint16 iReg)
{
    kal_uint16 iGain =0; 

	iGain = iReg*BASEGAIN/OV8858_GAIN_BASE;
	return iGain;
}

static kal_uint16 OV8858Gain2Reg(const kal_uint32 iGain)
{
    kal_uint32 iReg = 0x0000;

	iReg = iGain * 2; //(iGain/BASEGAIN)*OV8858_GAIN_BASE;

    return iReg;
}

void write_OV8858_gain(kal_uint16 gain)
{
	//kal_uint16 read_gain=0;

	OV8858_write_cmos_sensor(0x3508,(gain>>8));
	OV8858_write_cmos_sensor(0x3509,(gain&0xff));

	//read_gain=(((OV8858_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8865_read_cmos_sensor(0x3509));
	//OV8858DB("[OV8858_SetGain]0x3508|0x3509=0x%x \n",read_gain);

	return;
}

void OV8858_SetGain(UINT16 iGain)
{
	unsigned long flags;

	
	OV8858DB("OV8858_SetGain iGain = %d :\n ",iGain);

	spin_lock_irqsave(&ov8858mipiraw_drv_lock,flags);
	ov8858.realGain = iGain;
	ov8858.sensorGlobalGain = OV8858Gain2Reg(iGain);
	spin_unlock_irqrestore(&ov8858mipiraw_drv_lock,flags);
	write_OV8858_gain(ov8858.sensorGlobalGain);
	#ifdef OV8858_DEBUG
	OV8858DB(" [OV8858_SetGain]ov8858.sensorGlobalGain=0x%x,ov8858.realGain =0x%x",ov8858.sensorGlobalGain,ov8858.realGain); 
	#endif
	//temperature test
	//OV8858_write_cmos_sensor(0x4d12,0x01);
	//OV8858DB("Temperature read_reg  0x4d13  =%x \n",OV8865_read_cmos_sensor(0x4d13));
}   

kal_uint16 read_OV8858_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=(((OV8858_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8858_read_cmos_sensor(0x3509));

	spin_lock(&ov8858mipiraw_drv_lock);
	ov8858.sensorGlobalGain = read_gain;
	ov8858.realGain = OV8858Reg2Gain(ov8858.sensorGlobalGain);
	spin_unlock(&ov8858mipiraw_drv_lock);

	OV8858DB("ov8858.sensorGlobalGain=0x%x,ov8858.realGain=%d\n",ov8858.sensorGlobalGain,ov8858.realGain);

	return ov8858.sensorGlobalGain;
}  



static void OV8858_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == ov8858.sensorMode )
	{
		line_length = OV8858_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== ov8858.sensorMode )
	{
		line_length = OV8858_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = OV8858_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858_FULL_PERIOD_LINE_NUMS + iLines;
	}

	spin_lock(&ov8858mipiraw_drv_lock);
	OV8858_FeatureControl_PERIOD_PixelNum = line_length;
	OV8858_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&ov8858mipiraw_drv_lock);

	//Set total frame length
	OV8858_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV8858_write_cmos_sensor(0x380f, frame_length & 0xFF);
	//Set total line length
	OV8858_write_cmos_sensor(0x380c, (line_length >> 8) & 0xFF);
	OV8858_write_cmos_sensor(0x380d, line_length & 0xFF);

	#ifdef OV8858_DEBUG
	OV8858DB(" [OV8858_SetDummy]ov8858.sensorMode = %d, line_length = %d,iPixels = %d, frame_length =%d, iLines = %d\n",ov8858.sensorMode, line_length,iPixels, frame_length, iLines); 
	#endif

}   


#if 1
void OV8858_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV8858SensorReg[i].Addr; i++)
    {
        OV8858_write_cmos_sensor(OV8858SensorReg[i].Addr, OV8858SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8858SensorReg[i].Addr; i++)
    {
        OV8858_write_cmos_sensor(OV8858SensorReg[i].Addr, OV8858SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV8858_write_cmos_sensor(OV8858SensorCCT[i].Addr, OV8858SensorCCT[i].Para);
    }
}

void OV8858_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=OV8858SensorReg[i].Addr; i++)
    {
         temp_data = OV8858_read_cmos_sensor(OV8858SensorReg[i].Addr);
		 spin_lock(&ov8858mipiraw_drv_lock);
		 OV8858SensorReg[i].Para =temp_data;
		 spin_unlock(&ov8858mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8858SensorReg[i].Addr; i++)
    {
        temp_data = OV8858_read_cmos_sensor(OV8858SensorReg[i].Addr);
		spin_lock(&ov8858mipiraw_drv_lock);
		OV8858SensorReg[i].Para = temp_data;
		spin_unlock(&ov8858mipiraw_drv_lock);
    }
}

kal_int32  OV8858_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV8858_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void OV8858_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= OV8858SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/ov8865.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= OV8858_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= OV8858_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool OV8858_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * ov8865.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&ov8858mipiraw_drv_lock);
          OV8858SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&ov8858mipiraw_drv_lock);
          OV8858_write_cmos_sensor(OV8858SensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&ov8858mipiraw_drv_lock);
                    OV8858_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&ov8858mipiraw_drv_lock);
                    break;
                case 1:
                    OV8858_write_cmos_sensor(OV8858_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}
#endif

void OV8858_1224pSetting(void)
{

	/*   Preview of ov8858 setting                                 */
	/*   @@5.1.2.2 Raw 10bit 1632x1224 30fps 2lane 720M bps/lane   */
	/*   ;Pclk 72MHz                                               */
	/*   ;pixels per line=1928(0x788)                              */
	/*   ;lines per frame=1244(0x4dc)                              */
	OV8858_write_cmos_sensor(0x0100, 0x00); //
	OV8858_write_cmos_sensor(0x030e, 0x00); // ; pll2_rdiv
	OV8858_write_cmos_sensor(0x030f, 0x09); // ; pll2_divsp
	OV8858_write_cmos_sensor(0x0312, 0x01); // ; pll2_pre_div0, pll2_r_divdac
	OV8858_write_cmos_sensor(0x3015, 0x01); //
	OV8858_write_cmos_sensor(0x3501, 0x4d); // ; exposure M
	OV8858_write_cmos_sensor(0x3502, 0x40); // ; exposure L
	OV8858_write_cmos_sensor(0x3508, 0x04); // ; gain H
	OV8858_write_cmos_sensor(0x3706, 0x35); //
	OV8858_write_cmos_sensor(0x370a, 0x00); //
	OV8858_write_cmos_sensor(0x370b, 0xb5); //
	OV8858_write_cmos_sensor(0x3778, 0x1b); //
	OV8858_write_cmos_sensor(0x3808, 0x06); // ; x output size H 1632 
	OV8858_write_cmos_sensor(0x3809, 0x60); // ; x output size L
	OV8858_write_cmos_sensor(0x380a, 0x04); // ; y output size H 1224
	OV8858_write_cmos_sensor(0x380b, 0xc8); // ; y output size L
	OV8858_write_cmos_sensor(0x380c, 0x07); // ; HTS H
	OV8858_write_cmos_sensor(0x380d, 0x88); // ; HTS L
	OV8858_write_cmos_sensor(0x380e, 0x04); // ; VTS H
	OV8858_write_cmos_sensor(0x380f, 0xdc); // ; VTS L
	OV8858_write_cmos_sensor(0x3814, 0x03); // ; x odd inc
	OV8858_write_cmos_sensor(0x3821, 0x67); // ; mirror on, bin on
	OV8858_write_cmos_sensor(0x382a, 0x03); // ; y odd inc
	OV8858_write_cmos_sensor(0x3830, 0x08); //
	OV8858_write_cmos_sensor(0x3836, 0x02); //
	OV8858_write_cmos_sensor(0x3f0a, 0x80); //
	OV8858_write_cmos_sensor(0x4001, 0x10); // ; total 128 black column
	OV8858_write_cmos_sensor(0x4022, 0x04); // ; Anchor left end H
	OV8858_write_cmos_sensor(0x4023, 0xb9); // ; Anchor left end L
	OV8858_write_cmos_sensor(0x4024, 0x05); // ; Anchor right start H
	OV8858_write_cmos_sensor(0x4025, 0x2a); // ; Anchor right start L
	OV8858_write_cmos_sensor(0x4026, 0x05); // ; Anchor right end H
	OV8858_write_cmos_sensor(0x4027, 0x2b); // ; Anchor right end L
	OV8858_write_cmos_sensor(0x402b, 0x04); // ; top black line number
	OV8858_write_cmos_sensor(0x402e, 0x08); // ; bottom black line start
	OV8858_write_cmos_sensor(0x4500, 0x38); //
	OV8858_write_cmos_sensor(0x4600, 0x00); //
	OV8858_write_cmos_sensor(0x4601, 0xcb); //
	OV8858_write_cmos_sensor(0x382d, 0x7f); //
	OV8858_write_cmos_sensor(0x0100, 0x01); //
}

void OV8858PreviewSetting(void)
{
	//		 @@5.1.2.2 Raw 10bit 1632x1224 30fps 2lane 672M bps/lane
	//		 ;Pclk 72MHz
	//		 ;pixels per line=1928(0x788) 
	//		 ;lines per frame=1244(0x4dc)
		OV8858_write_cmos_sensor(0x0100,  0x00);	//
		OV8858_write_cmos_sensor(0x030e,  0x00);	// ; pll2_rdiv
		OV8858_write_cmos_sensor(0x030f,  0x09);	// ; pll2_divsp
		OV8858_write_cmos_sensor(0x0312,  0x01);	// ; pll2_pre_div0, pll2_r_divdac
		OV8858_write_cmos_sensor(0x3015,  0x01);	//
		OV8858_write_cmos_sensor(0x3501,  0x4d);	// ; exposure M
		OV8858_write_cmos_sensor(0x3502,  0x40);	// ; exposure L
		OV8858_write_cmos_sensor(0x3508,  0x04);	// ; gain H
		OV8858_write_cmos_sensor(0x3706,  0x35);	//
		OV8858_write_cmos_sensor(0x370a,  0x00);	//
		OV8858_write_cmos_sensor(0x370b,  0xb5);	//
		OV8858_write_cmos_sensor(0x3778,  0x1b);	//
		OV8858_write_cmos_sensor(0x3813,  0x02); 	// ; ISP y win L
		OV8858_write_cmos_sensor(0x3808,  0x06);	// ; x output size H
		OV8858_write_cmos_sensor(0x3809,  0x60);	// ; x output size L
		OV8858_write_cmos_sensor(0x380a,  0x04);	// ; y output size H
		OV8858_write_cmos_sensor(0x380b,  0xc8);	// ; y output size L
		OV8858_write_cmos_sensor(0x380c,  0x07);	// ; HTS H
		OV8858_write_cmos_sensor(0x380d,  0x88);	// ; HTS L
		OV8858_write_cmos_sensor(0x380e,  0x04);	// ; VTS H
		OV8858_write_cmos_sensor(0x380f,  0xdc);	// ; VTS L
		OV8858_write_cmos_sensor(0x3814,  0x03);	// ; x odd inc
		OV8858_write_cmos_sensor(0x3821,  0x67);	// ; mirror on, bin on
		OV8858_write_cmos_sensor(0x382a,  0x03);	// ; y odd inc
		OV8858_write_cmos_sensor(0x3830,  0x08);	//
		OV8858_write_cmos_sensor(0x3836,  0x02);	//
		OV8858_write_cmos_sensor(0x3845,  0x02);	//	
		OV8858_write_cmos_sensor(0x3f0a,  0x80);	//
		OV8858_write_cmos_sensor(0x4001,  0x10);	// ; total 128 black column
		OV8858_write_cmos_sensor(0x4022,  0x04);	// ; Anchor left end H
		OV8858_write_cmos_sensor(0x4023,  0xb9);	// ; Anchor left end L
		OV8858_write_cmos_sensor(0x4024,  0x05);	// ; Anchor right start H
		OV8858_write_cmos_sensor(0x4025,  0x2a);	// ; Anchor right start L
		OV8858_write_cmos_sensor(0x4026,  0x05);	// ; Anchor right end H
		OV8858_write_cmos_sensor(0x4027,  0x2b);	// ; Anchor right end L
		OV8858_write_cmos_sensor(0x402b,  0x04);	// ; top black line number
		OV8858_write_cmos_sensor(0x402e,  0x08);	// ; bottom black line start
		OV8858_write_cmos_sensor(0x4500,  0x38);	//
		OV8858_write_cmos_sensor(0x4600,  0x00);	//
		OV8858_write_cmos_sensor(0x4601,  0xcb);	//
		OV8858_write_cmos_sensor(0x382d,  0x7f);	//
		OV8858_write_cmos_sensor(0x0100,  0x01);	//



}

void OV8858CaptureSetting(void)
{
	//	 @@5.1.2.3 Raw 10bit 3264x2448 15fps 2lane 672M bps/lane
	//	 ;; MIPI=720Mbps, SysClk=72Mhz,Dac Clock=360Mhz.
	//	 ;Pclk 72MHz
	//	 ;pixels per line=2566(0xa06) 
	//	 ;lines per frame=1872(0x750)
		OV8858_write_cmos_sensor(0x0100,  0x00);	   //
		OV8858_write_cmos_sensor(0x030e,  0x02);	   // ; pll2_rdiv
		OV8858_write_cmos_sensor(0x030f,  0x04);	   // ; pll2_divsp
		OV8858_write_cmos_sensor(0x0312,  0x03);	   // ; pll2_pre_div0, pll2_r_divdac
		OV8858_write_cmos_sensor(0x3015,  0x00);	   //
		OV8858_write_cmos_sensor(0x3501,  0x9a);	   //
		OV8858_write_cmos_sensor(0x3502,  0x20);	   //
		OV8858_write_cmos_sensor(0x3508,  0x02);	   //
		OV8858_write_cmos_sensor(0x3706,  0x6a);	   //
		OV8858_write_cmos_sensor(0x370a,  0x01);	   //
		OV8858_write_cmos_sensor(0x370b,  0x6a);	   //
		OV8858_write_cmos_sensor(0x3778,  0x32);	   //
		OV8858_write_cmos_sensor(0x3813,  0x02); 	// ; ISP y win L
		OV8858_write_cmos_sensor(0x3808,  0x0c);	   // ; x output size H
		OV8858_write_cmos_sensor(0x3809,  0xc0);	   // ; x output size L
		OV8858_write_cmos_sensor(0x380a,  0x09);	   // ; y output size H
		OV8858_write_cmos_sensor(0x380b,  0x90);	   // ; y output size L
		OV8858_write_cmos_sensor(0x380c,  0x07);	   // ; HTS H
		OV8858_write_cmos_sensor(0x380d,  0x94);	   // ; HTS L
		OV8858_write_cmos_sensor(0x380e,  0x09);	   // ; VTS H
		OV8858_write_cmos_sensor(0x380f,  0xaa);	   // ; VTS L
		OV8858_write_cmos_sensor(0x3814,  0x01);	   // ; x odd inc
		OV8858_write_cmos_sensor(0x3821,  0x46);	   // ; mirror on, bin off
		OV8858_write_cmos_sensor(0x382a,  0x01);	   // ; y odd inc
		OV8858_write_cmos_sensor(0x3830,  0x06);	   //
		OV8858_write_cmos_sensor(0x3836,  0x01);	   //
		OV8858_write_cmos_sensor(0x3845,  0x00);	//	
		OV8858_write_cmos_sensor(0x3f0a,  0x00);	   //
		OV8858_write_cmos_sensor(0x4001,  0x00);	   // ; total 256 black column
		OV8858_write_cmos_sensor(0x4022,  0x0b);	   // ; Anchor left end H
		OV8858_write_cmos_sensor(0x4023,  0xc3);	   // ; Anchor left end L
		OV8858_write_cmos_sensor(0x4024,  0x0c);	   // ; Anchor right start H
		OV8858_write_cmos_sensor(0x4025,  0x36);	   // ; Anchor right start L
		OV8858_write_cmos_sensor(0x4026,  0x0c);	   // ; Anchor right end H
		OV8858_write_cmos_sensor(0x4027,  0x37);	   // ; Anchor right end L
		OV8858_write_cmos_sensor(0x402b,  0x08);	   // ; top black line number
		OV8858_write_cmos_sensor(0x402e,  0x0c);	   // ; bottom black line start
		OV8858_write_cmos_sensor(0x4500,  0x58);	   //
		OV8858_write_cmos_sensor(0x4600,  0x01);	   //
		OV8858_write_cmos_sensor(0x4601,  0x97);	   //
		OV8858_write_cmos_sensor(0x382d,  0xff);	   //
		OV8858_write_cmos_sensor(0x0100,  0x01);	   //

}

void OV8858VideoSetting(void)
{
	//		 @@5.1.2.2 Raw 10bit 1632x1224 30fps 2lane 672M bps/lane
	//		 ;Pclk 72MHz
	//		 ;pixels per line=1928(0x788) 
	//		 ;lines per frame=1244(0x4dc)
		OV8858_write_cmos_sensor(0x0100,  0x00);	//
		OV8858_write_cmos_sensor(0x030e,  0x00);	// ; pll2_rdiv
		OV8858_write_cmos_sensor(0x030f,  0x09);	// ; pll2_divsp
		OV8858_write_cmos_sensor(0x0312,  0x01);	// ; pll2_pre_div0, pll2_r_divdac
		OV8858_write_cmos_sensor(0x3015,  0x01);	//
		OV8858_write_cmos_sensor(0x3501,  0x4d);	// ; exposure M
		OV8858_write_cmos_sensor(0x3502,  0x40);	// ; exposure L
		OV8858_write_cmos_sensor(0x3508,  0x04);	// ; gain H
		OV8858_write_cmos_sensor(0x3706,  0x35);	//
		OV8858_write_cmos_sensor(0x370a,  0x00);	//
		OV8858_write_cmos_sensor(0x370b,  0xb5);	//
		OV8858_write_cmos_sensor(0x3778,  0x1b);	//
		OV8858_write_cmos_sensor(0x3808,  0x06);	// ; x output size H
		OV8858_write_cmos_sensor(0x3809,  0x60);	// ; x output size L
		OV8858_write_cmos_sensor(0x380a,  0x04);	// ; y output size H
		OV8858_write_cmos_sensor(0x380b,  0xc8);	// ; y output size L
		OV8858_write_cmos_sensor(0x380c,  0x07);	// ; HTS H
		OV8858_write_cmos_sensor(0x380d,  0x88);	// ; HTS L
		OV8858_write_cmos_sensor(0x380e,  0x04);	// ; VTS H
		OV8858_write_cmos_sensor(0x380f,  0xdc);	// ; VTS L
		OV8858_write_cmos_sensor(0x3813,  0x02); 	// ; ISP y win L
		OV8858_write_cmos_sensor(0x3814,  0x03);	// ; x odd inc
		OV8858_write_cmos_sensor(0x3821,  0x67);	// ; mirror on, bin on
		OV8858_write_cmos_sensor(0x382a,  0x03);	// ; y odd inc
		OV8858_write_cmos_sensor(0x3830,  0x08);	//
		OV8858_write_cmos_sensor(0x3836,  0x02);	//
		OV8858_write_cmos_sensor(0x3f0a,  0x80);	//
		OV8858_write_cmos_sensor(0x4001,  0x10);	// ; total 128 black column
		OV8858_write_cmos_sensor(0x4022,  0x04);	// ; Anchor left end H
		OV8858_write_cmos_sensor(0x4023,  0xb9);	// ; Anchor left end L
		OV8858_write_cmos_sensor(0x4024,  0x05);	// ; Anchor right start H
		OV8858_write_cmos_sensor(0x4025,  0x2a);	// ; Anchor right start L
		OV8858_write_cmos_sensor(0x4026,  0x05);	// ; Anchor right end H
		OV8858_write_cmos_sensor(0x4027,  0x2b);	// ; Anchor right end L
		OV8858_write_cmos_sensor(0x402b,  0x04);	// ; top black line number
		OV8858_write_cmos_sensor(0x402e,  0x08);	// ; bottom black line start
		OV8858_write_cmos_sensor(0x4500,  0x38);	//
		OV8858_write_cmos_sensor(0x4600,  0x00);	//
		OV8858_write_cmos_sensor(0x4601,  0xcb);	//
		OV8858_write_cmos_sensor(0x382d,  0x7f);	//
		OV8858_write_cmos_sensor(0x0100,  0x01);	//



}

static void OV8858_Sensor_Init(void)
{
	//	  ;; v00_01_00 (05/29/2013) : initial setting
	//	  ;;
	//	OV8858_write_cmos_sensor(0x0103, 0x01); 	// ; software reset
		OV8858_write_cmos_sensor(0x0100, 0x00); 	// ; software standby
		OV8858_write_cmos_sensor(0x0100, 0x00); 	// ;
		OV8858_write_cmos_sensor(0x0100, 0x00); 	// ;
		OV8858_write_cmos_sensor(0x0100, 0x00); 	// ;
		OV8858_write_cmos_sensor(0x0302, 0x1c); 	//;1e ; pll1_multi
		OV8858_write_cmos_sensor(0x0303, 0x00); 	// ; pll1_divm
		OV8858_write_cmos_sensor(0x0304, 0x03); 	// ; pll1_div_mipi
		OV8858_write_cmos_sensor(0x030e, 0x00); 	// ; pll2_rdiv
		OV8858_write_cmos_sensor(0x030f, 0x09); 	// ; pll2_divsp
		OV8858_write_cmos_sensor(0x0312, 0x01); 	// ; pll2_pre_div0, pll2_r_divdac
		OV8858_write_cmos_sensor(0x031e, 0x0c); 	// ; pll1_no_lat
		OV8858_write_cmos_sensor(0x3600, 0x00); 	//
		OV8858_write_cmos_sensor(0x3601, 0x00); 	//
		OV8858_write_cmos_sensor(0x3602, 0x00); 	//
		OV8858_write_cmos_sensor(0x3603, 0x00); 	//
		OV8858_write_cmos_sensor(0x3604, 0x22); 	//
		OV8858_write_cmos_sensor(0x3605, 0x30); 	//
		OV8858_write_cmos_sensor(0x3606, 0x00); 	//
		OV8858_write_cmos_sensor(0x3607, 0x20); 	//
		OV8858_write_cmos_sensor(0x3608, 0x11); 	//
		OV8858_write_cmos_sensor(0x3609, 0x28); 	//
		OV8858_write_cmos_sensor(0x360a, 0x00); 	//
		OV8858_write_cmos_sensor(0x360b, 0x06); 	//
		OV8858_write_cmos_sensor(0x360c, 0xdc); 	//
		OV8858_write_cmos_sensor(0x360d, 0x40); 	//
		OV8858_write_cmos_sensor(0x360e, 0x0c); 	//
		OV8858_write_cmos_sensor(0x360f, 0x20); 	//
		OV8858_write_cmos_sensor(0x3610, 0x07); 	//
		OV8858_write_cmos_sensor(0x3611, 0x20); 	//
		OV8858_write_cmos_sensor(0x3612, 0x88); 	//
		OV8858_write_cmos_sensor(0x3613, 0x80); 	//
		OV8858_write_cmos_sensor(0x3614, 0x58); 	//
		OV8858_write_cmos_sensor(0x3615, 0x00); 	//
		OV8858_write_cmos_sensor(0x3616, 0x4a); 	//
		OV8858_write_cmos_sensor(0x3617, 0x90); 	//
		OV8858_write_cmos_sensor(0x3618, 0x56); 	//
		OV8858_write_cmos_sensor(0x3619, 0x70); 	//
		OV8858_write_cmos_sensor(0x361a, 0x99); 	//
		OV8858_write_cmos_sensor(0x361b, 0x00); 	//
		OV8858_write_cmos_sensor(0x361c, 0x07); 	//
		OV8858_write_cmos_sensor(0x361d, 0x00); 	//
		OV8858_write_cmos_sensor(0x361e, 0x00); 	//
		OV8858_write_cmos_sensor(0x361f, 0x00); 	//
		OV8858_write_cmos_sensor(0x3638, 0xff); 	//
		OV8858_write_cmos_sensor(0x3633, 0x0c); 	//
		OV8858_write_cmos_sensor(0x3634, 0x0c); 	//
		OV8858_write_cmos_sensor(0x3635, 0x0c); 	//
		OV8858_write_cmos_sensor(0x3636, 0x0c); 	//
		OV8858_write_cmos_sensor(0x3645, 0x13); 	//
		OV8858_write_cmos_sensor(0x3646, 0x83); 	//
		OV8858_write_cmos_sensor(0x364a, 0x07); 	//
		OV8858_write_cmos_sensor(0x3015, 0x01); 	// ;
		OV8858_write_cmos_sensor(0x3018, 0x32); 	// ; MIPI 2 lane
		OV8858_write_cmos_sensor(0x3020, 0x93); 	// ; Clock switch output normal, pclk_div =/1
		OV8858_write_cmos_sensor(0x3022, 0x01); 	// ; pd_mipi enable when rst_sync
		OV8858_write_cmos_sensor(0x3031, 0x0a); 	// ; MIPI 10-bit mode
		OV8858_write_cmos_sensor(0x3034, 0x00); 	//
		OV8858_write_cmos_sensor(0x3106, 0x01); 	// ; sclk_div, sclk_pre_div
		OV8858_write_cmos_sensor(0x3305, 0xf1); 	//
		OV8858_write_cmos_sensor(0x3308, 0x00); 	//
		OV8858_write_cmos_sensor(0x3309, 0x28); 	//
		OV8858_write_cmos_sensor(0x330a, 0x00); 	//
		OV8858_write_cmos_sensor(0x330b, 0x20); 	//
		OV8858_write_cmos_sensor(0x330c, 0x00); 	//
		OV8858_write_cmos_sensor(0x330d, 0x00); 	//
		OV8858_write_cmos_sensor(0x330e, 0x00); 	//
		OV8858_write_cmos_sensor(0x330f, 0x40); 	//
		OV8858_write_cmos_sensor(0x3307, 0x04); 	//
		OV8858_write_cmos_sensor(0x3500, 0x00); 	// ; exposure H
		OV8858_write_cmos_sensor(0x3501, 0x4d); 	// ; exposure M
		OV8858_write_cmos_sensor(0x3502, 0x40); 	// ; exposure L
		OV8858_write_cmos_sensor(0x3503, 0x00); 	// ; gain delay 1 frame, exposure delay 1 frame, real gain
		OV8858_write_cmos_sensor(0x3505, 0x80); 	// ; gain option
		OV8858_write_cmos_sensor(0x3508, 0x04); 	// ; gain H
		OV8858_write_cmos_sensor(0x3509, 0x00); 	// ; gain L
		OV8858_write_cmos_sensor(0x350c, 0x00); 	// ; short gain H
		OV8858_write_cmos_sensor(0x350d, 0x80); 	// ; short gain L
		OV8858_write_cmos_sensor(0x3510, 0x00); 	// ; short exposure H
		OV8858_write_cmos_sensor(0x3511, 0x02); 	// ; short exposure M
		OV8858_write_cmos_sensor(0x3512, 0x00); 	// ; short exposure L
		OV8858_write_cmos_sensor(0x3700, 0x18); 	//
		OV8858_write_cmos_sensor(0x3701, 0x0c); 	//
		OV8858_write_cmos_sensor(0x3702, 0x28); 	//
		OV8858_write_cmos_sensor(0x3703, 0x19); 	//
		OV8858_write_cmos_sensor(0x3704, 0x14); 	//
		OV8858_write_cmos_sensor(0x3705, 0x00); 	//
		OV8858_write_cmos_sensor(0x3706, 0x35); 	//
		OV8858_write_cmos_sensor(0x3707, 0x04); 	//
		OV8858_write_cmos_sensor(0x3708, 0x24); 	//
		OV8858_write_cmos_sensor(0x3709, 0x33); 	//
		OV8858_write_cmos_sensor(0x370a, 0x00); 	//
		OV8858_write_cmos_sensor(0x370b, 0xb5); 	//
		OV8858_write_cmos_sensor(0x370c, 0x04); 	//
		OV8858_write_cmos_sensor(0x3718, 0x12); 	//
		OV8858_write_cmos_sensor(0x3719, 0x31); 	//
		OV8858_write_cmos_sensor(0x3712, 0x42); 	//
		OV8858_write_cmos_sensor(0x3714, 0x24); 	//
		OV8858_write_cmos_sensor(0x371e, 0x19); 	//
		OV8858_write_cmos_sensor(0x371f, 0x40); 	//
		OV8858_write_cmos_sensor(0x3720, 0x05); 	//
		OV8858_write_cmos_sensor(0x3721, 0x05); 	//
		OV8858_write_cmos_sensor(0x3724, 0x06); 	//
		OV8858_write_cmos_sensor(0x3725, 0x01); 	//
		OV8858_write_cmos_sensor(0x3726, 0x06); 	//
		OV8858_write_cmos_sensor(0x3728, 0x05); 	//
		OV8858_write_cmos_sensor(0x3729, 0x02); 	//
		OV8858_write_cmos_sensor(0x372a, 0x03); 	//
		OV8858_write_cmos_sensor(0x372b, 0x53); 	//
		OV8858_write_cmos_sensor(0x372c, 0xa3); 	//
		OV8858_write_cmos_sensor(0x372d, 0x53); 	//
		OV8858_write_cmos_sensor(0x372e, 0x06); 	//
		OV8858_write_cmos_sensor(0x372f, 0x10); 	//
		OV8858_write_cmos_sensor(0x3730, 0x01); 	//
		OV8858_write_cmos_sensor(0x3731, 0x06); 	//
		OV8858_write_cmos_sensor(0x3732, 0x14); 	//
		OV8858_write_cmos_sensor(0x3733, 0x10); 	//
		OV8858_write_cmos_sensor(0x3734, 0x40); 	//
		OV8858_write_cmos_sensor(0x3736, 0x20); 	//
		OV8858_write_cmos_sensor(0x373a, 0x05); 	//
		OV8858_write_cmos_sensor(0x373b, 0x06); 	//
		OV8858_write_cmos_sensor(0x373c, 0x0a); 	//
		OV8858_write_cmos_sensor(0x373e, 0x03); 	//
		OV8858_write_cmos_sensor(0x3755, 0x10); 	//
		OV8858_write_cmos_sensor(0x3758, 0x00); 	//
		OV8858_write_cmos_sensor(0x3759, 0x4c); 	//
		OV8858_write_cmos_sensor(0x375a, 0x06); 	//
		OV8858_write_cmos_sensor(0x375b, 0x13); 	//
		OV8858_write_cmos_sensor(0x375c, 0x20); 	//
		OV8858_write_cmos_sensor(0x375d, 0x02); 	//
		OV8858_write_cmos_sensor(0x375e, 0x00); 	//
		OV8858_write_cmos_sensor(0x375f, 0x14); 	//
		OV8858_write_cmos_sensor(0x3768, 0x22); 	//
		OV8858_write_cmos_sensor(0x3769, 0x44); 	//
		OV8858_write_cmos_sensor(0x376a, 0x44); 	//
		OV8858_write_cmos_sensor(0x3761, 0x00); 	//
		OV8858_write_cmos_sensor(0x3762, 0x00); 	//
		OV8858_write_cmos_sensor(0x3763, 0x00); 	//
		OV8858_write_cmos_sensor(0x3766, 0xff); 	//
		OV8858_write_cmos_sensor(0x376b, 0x00); 	//
		OV8858_write_cmos_sensor(0x3772, 0x23); 	//
		OV8858_write_cmos_sensor(0x3773, 0x02); 	//
		OV8858_write_cmos_sensor(0x3774, 0x16); 	//
		OV8858_write_cmos_sensor(0x3775, 0x12); 	//
		OV8858_write_cmos_sensor(0x3776, 0x04); 	//
		OV8858_write_cmos_sensor(0x3777, 0x00); 	//
		OV8858_write_cmos_sensor(0x3778, 0x1b); 	//
		OV8858_write_cmos_sensor(0x37a0, 0x44); 	//
		OV8858_write_cmos_sensor(0x37a1, 0x3d); 	//
		OV8858_write_cmos_sensor(0x37a2, 0x3d); 	//
		OV8858_write_cmos_sensor(0x37a3, 0x00); 	//
		OV8858_write_cmos_sensor(0x37a4, 0x00); 	//
		OV8858_write_cmos_sensor(0x37a5, 0x00); 	//
		OV8858_write_cmos_sensor(0x37a6, 0x00); 	//
		OV8858_write_cmos_sensor(0x37a7, 0x44); 	//
		OV8858_write_cmos_sensor(0x37a8, 0x4c); 	//
		OV8858_write_cmos_sensor(0x37a9, 0x4c); 	//
		OV8858_write_cmos_sensor(0x3760, 0x00); 	//
		OV8858_write_cmos_sensor(0x376f, 0x01); 	//
		OV8858_write_cmos_sensor(0x37aa, 0x44); 	//
		OV8858_write_cmos_sensor(0x37ab, 0x2e); 	//
		OV8858_write_cmos_sensor(0x37ac, 0x2e); 	//
		OV8858_write_cmos_sensor(0x37ad, 0x33); 	//
		OV8858_write_cmos_sensor(0x37ae, 0x0d); 	//
		OV8858_write_cmos_sensor(0x37af, 0x0d); 	//
		OV8858_write_cmos_sensor(0x37b0, 0x00); 	//
		OV8858_write_cmos_sensor(0x37b1, 0x00); 	//
		OV8858_write_cmos_sensor(0x37b2, 0x00); 	//
		OV8858_write_cmos_sensor(0x37b3, 0x42); 	//
		OV8858_write_cmos_sensor(0x37b4, 0x42); 	//
		OV8858_write_cmos_sensor(0x37b5, 0x33); 	//
		OV8858_write_cmos_sensor(0x37b6, 0x00); 	//
		OV8858_write_cmos_sensor(0x37b7, 0x00); 	//
		OV8858_write_cmos_sensor(0x37b8, 0x00); 	//
		OV8858_write_cmos_sensor(0x37b9, 0xff); 	//
		OV8858_write_cmos_sensor(0x3800, 0x00); 	// ; x start H
		OV8858_write_cmos_sensor(0x3801, 0x0c); 	// ; x start L
		OV8858_write_cmos_sensor(0x3802, 0x00); 	// ; y start H
		OV8858_write_cmos_sensor(0x3803, 0x0c); 	// ; y start L
		OV8858_write_cmos_sensor(0x3804, 0x0c); 	// ; x end H
		OV8858_write_cmos_sensor(0x3805, 0xd3); 	// ; x end L
		OV8858_write_cmos_sensor(0x3806, 0x09); 	// ; y end H
		OV8858_write_cmos_sensor(0x3807, 0xa3); 	// ; y end L
		OV8858_write_cmos_sensor(0x3808, 0x06); 	// ; x output size H
		OV8858_write_cmos_sensor(0x3809, 0x60); 	// ; x output size L
		OV8858_write_cmos_sensor(0x380a, 0x04); 	// ; y output size H
		OV8858_write_cmos_sensor(0x380b, 0xc8); 	// ; y output size L
		OV8858_write_cmos_sensor(0x380c, 0x07); 	// ; 03 ; HTS H
		OV8858_write_cmos_sensor(0x380d, 0x88); 	// ; c4 ; HTS L
		OV8858_write_cmos_sensor(0x380e, 0x04); 	// ; VTS H
		OV8858_write_cmos_sensor(0x380f, 0xdc); 	// ; VTS L
		OV8858_write_cmos_sensor(0x3810, 0x00); 	// ; ISP x win H
		OV8858_write_cmos_sensor(0x3811, 0x04); 	// ; ISP x win L
		OV8858_write_cmos_sensor(0x3813, 0x02); 	// ; ISP y win L
		OV8858_write_cmos_sensor(0x3814, 0x03); 	// ; x odd inc
		OV8858_write_cmos_sensor(0x3815, 0x01); 	// ; x even inc
		OV8858_write_cmos_sensor(0x3820, 0x00); 	// ; vflip off
		OV8858_write_cmos_sensor(0x3821, 0x67); 	// ; mirror on, bin on
		OV8858_write_cmos_sensor(0x382a, 0x03); 	// ; y odd inc
		OV8858_write_cmos_sensor(0x382b, 0x01); 	// ; y even inc
		OV8858_write_cmos_sensor(0x3830, 0x08); 	//
		OV8858_write_cmos_sensor(0x3836, 0x02); 	//
		OV8858_write_cmos_sensor(0x3837, 0x18); 	//
		OV8858_write_cmos_sensor(0x3841, 0xff); 	// ; window auto size enable
		OV8858_write_cmos_sensor(0x3846, 0x48); 	//
		OV8858_write_cmos_sensor(0x3d85, 0x14); 	// ; OTP power up load data enable, OTP powerr up load setting disable
		OV8858_write_cmos_sensor(0x3f08, 0x08); 	//
		OV8858_write_cmos_sensor(0x3f0a, 0x80); 	//
		OV8858_write_cmos_sensor(0x4000, 0xf1); 	// ; out_range_trig, format_chg_trig, gain_trig, exp_chg_trig, median filter enable
		OV8858_write_cmos_sensor(0x4001, 0x10); 	// ; total 128 black column
		OV8858_write_cmos_sensor(0x4005, 0x10); 	// ; BLC target L
		OV8858_write_cmos_sensor(0x4002, 0x27); 	// ; value used to limit BLC offset
		OV8858_write_cmos_sensor(0x4006, 0x04); 	// ;;add for mode change blc frame stable
		OV8858_write_cmos_sensor(0x4007, 0x04); 	// ;;add for mode change blc frame stable
		OV8858_write_cmos_sensor(0x4009, 0x81); 	// ; final BLC offset limitation enable
		OV8858_write_cmos_sensor(0x400b, 0x0c); 	// ; DCBLC on, DCBLC manual mode on
		OV8858_write_cmos_sensor(0x401b, 0x00); 	// ; zero line R coefficient
		OV8858_write_cmos_sensor(0x401d, 0x00); 	// ; zoro line T coefficient
		OV8858_write_cmos_sensor(0x4020, 0x00); 	// ; Anchor left start H
		OV8858_write_cmos_sensor(0x4021, 0x04); 	// ; Anchor left start L
		OV8858_write_cmos_sensor(0x4022, 0x04); 	// ; Anchor left end H
		OV8858_write_cmos_sensor(0x4023, 0xb9); 	// ; Anchor left end L
		OV8858_write_cmos_sensor(0x4024, 0x05); 	// ; Anchor right start H
		OV8858_write_cmos_sensor(0x4025, 0x2a); 	// ; Anchor right start L
		OV8858_write_cmos_sensor(0x4026, 0x05); 	// ; Anchor right end H
		OV8858_write_cmos_sensor(0x4027, 0x2b); 	// ; Anchor right end L
		OV8858_write_cmos_sensor(0x4028, 0x00); 	// ; top zero line start
		OV8858_write_cmos_sensor(0x4029, 0x02); 	// ; top zero line number
		OV8858_write_cmos_sensor(0x402a, 0x04); 	// ; top black line start
		OV8858_write_cmos_sensor(0x402b, 0x04); 	// ; top black line number
		OV8858_write_cmos_sensor(0x402c, 0x02); 	// ; bottom zero line start
		OV8858_write_cmos_sensor(0x402d, 0x02); 	// ; bottom zoro line number
		OV8858_write_cmos_sensor(0x402e, 0x08); 	// ; bottom black line start
		OV8858_write_cmos_sensor(0x402f, 0x02); 	// ; bottom black line number
		OV8858_write_cmos_sensor(0x401f, 0x00); 	// ; interpolation x disable, interpolation y disable, Anchor one disable
		OV8858_write_cmos_sensor(0x4034, 0x3f); 	//
		OV8858_write_cmos_sensor(0x403d, 0x04); 	// ; md_precison_en
		OV8858_write_cmos_sensor(0x4300, 0xff); 	// ; clip max H
		OV8858_write_cmos_sensor(0x4301, 0x00); 	// ; clip min H
		OV8858_write_cmos_sensor(0x4302, 0x0f); 	// ; clip min L, clip max L
		OV8858_write_cmos_sensor(0x4316, 0x00); 	//
		OV8858_write_cmos_sensor(0x4500, 0x38); 	//
		OV8858_write_cmos_sensor(0x4503, 0x18); 	//
		OV8858_write_cmos_sensor(0x4600, 0x00); 	//
		OV8858_write_cmos_sensor(0x4601, 0xcb); 	//
		OV8858_write_cmos_sensor(0x481f, 0x32); 	// ; clk prepare min
		OV8858_write_cmos_sensor(0x4837, 0x17); 	//;16 ; global timing
		OV8858_write_cmos_sensor(0x4850, 0x10); 	// ; lane 1 = 1, lane 0 = 0
		OV8858_write_cmos_sensor(0x4851, 0x32); 	// ; lane 3 = 3, lane 2 = 2
		OV8858_write_cmos_sensor(0x4b00, 0x2a); 	//
		OV8858_write_cmos_sensor(0x4b0d, 0x00); 	//
		OV8858_write_cmos_sensor(0x4d00, 0x04); 	// ; temperature sensor
		OV8858_write_cmos_sensor(0x4d01, 0x18); 	// ;
		OV8858_write_cmos_sensor(0x4d02, 0xc3); 	// ;
		OV8858_write_cmos_sensor(0x4d03, 0xff); 	// ;
		OV8858_write_cmos_sensor(0x4d04, 0xff); 	// ;
		OV8858_write_cmos_sensor(0x4d05, 0xff); 	// ; temperature sensor
		OV8858_write_cmos_sensor(0x5000, 0x7e); 	// ; slave AWB gain enable, slave AWB statistics enable, master AWB gain enable,master AWB statistics enable, BPC on, WPC on
		OV8858_write_cmos_sensor(0x5001, 0x01); 	// ; BLC on
		OV8858_write_cmos_sensor(0x5002, 0x08); 	// ; H scale off, WBMATCH select slave sensor's gain, WBMATCH off, OTP_DPCoff,
		OV8858_write_cmos_sensor(0x5003, 0x20); 	// ; DPC_DBC buffer control enable, WB
		OV8858_write_cmos_sensor(0x5046, 0x12); 	//
		OV8858_write_cmos_sensor(0x5780, 0xfc); 	// ; tail en, saturate cross cluster en, remove corss cluster en,; remove connected dp in same channel,; en remove connected dp in different	OV8858_write_cmos_sensor(0xnnel;,0x s); 	//mooth en
		OV8858_write_cmos_sensor(0x5784, 0x0c); 	// ; black threshold
		OV8858_write_cmos_sensor(0x5787, 0x40); 	// ; thre3
		OV8858_write_cmos_sensor(0x5788, 0x08); 	// ; thre4
		OV8858_write_cmos_sensor(0x578a, 0x02); 	// ; wthre_list1
		OV8858_write_cmos_sensor(0x578b, 0x01); 	// ; wthre_list2
		OV8858_write_cmos_sensor(0x578c, 0x01); 	// ; wthre_list3
		OV8858_write_cmos_sensor(0x578e, 0x02); 	// ; bthre_list1
		OV8858_write_cmos_sensor(0x578f, 0x01); 	// ; bthre_list2
		OV8858_write_cmos_sensor(0x5790, 0x01); 	// ; bthre_list3
		OV8858_write_cmos_sensor(0x5901, 0x00); 	// ; H skip off, V skip off
		OV8858_write_cmos_sensor(0x5b00, 0x02); 	// ; OTP DPC start address
		OV8858_write_cmos_sensor(0x5b01, 0x10); 	// ; OTP DPC start address
		OV8858_write_cmos_sensor(0x5b02, 0x03); 	// ; OTP DPC end address
		OV8858_write_cmos_sensor(0x5b03, 0xcf); 	// ; OTP DPC end address
		OV8858_write_cmos_sensor(0x5b05, 0x6c); 	// ; recover method = 2b11, use fixed pattern to recover cluster,; use 0x3ff to recover cluster
		OV8858_write_cmos_sensor(0x5e00, 0x00); 	// ; test pattern off
		OV8858_write_cmos_sensor(0x5e01, 0x41); 	// ; window cut enable
		OV8858_write_cmos_sensor(0x382d, 0x7f); 	//
		OV8858_write_cmos_sensor(0x4825, 0x3a); 	// ; lpx_p_min
		OV8858_write_cmos_sensor(0x4826, 0x40); 	// ; hs_prepare_min
		OV8858_write_cmos_sensor(0x4808, 0x25); 	// ; wake up delay in 1/1024 s
	 #ifdef OV8858_CHECK_OTP
	OV8858_write_cmos_sensor(0x0100, 0x01);//
	mDELAY(50);
   #endif



}


UINT32 OV8858Open(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	OV8858DB("OV8858 Open enter :\n ");
	OV8858_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mdelay(10);

	for(i=0;i<2;i++)
	{
		sensor_id = (OV8858_read_cmos_sensor(0x300B)<<8)|OV8858_read_cmos_sensor(0x300C);
		OV8858DB("OV8858 READ ID :%x",sensor_id);
		if(sensor_id != OV8858_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	
	OV8858_Sensor_Init();
	#ifdef OV8858_CHECK_OTP
	   otp_update_wb(0x13c,0x12b);
     otp_update_lenc();
	OV8858DB("truly ov8858 otp:%x\n",OV8858_read_cmos_sensor(0x5000));
	#endif
	mdelay(40);
    OV8858_Init_Para();
	
	#ifdef OV8858_DEBUG
		OV8858DB("[OV8858Open] enter and exit."); 
	#endif

    return ERROR_NONE;
}


UINT32 OV8858GetSensorID(UINT32 *sensorID)
{
    int  retry = 2;

	OV8858DB("OV8858GetSensorID enter :\n ");
    mdelay(5);

    do {
        *sensorID = (OV8858_read_cmos_sensor(0x300B)<<8)|OV8858_read_cmos_sensor(0x300C);
        if (*sensorID == OV8858_SENSOR_ID)
        	{
        		OV8858DB("Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        OV8858DB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != OV8858_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

UINT32 OV8858Close(void)
{
	#ifdef OV8858_DEBUG
		OV8858DB("[OV8858Close]enter and exit.\n");
	#endif

    return ERROR_NONE;
}

#if 1
void OV8858SetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
	mirror= OV8858_read_cmos_sensor(0x3820);
	flip  = OV8858_read_cmos_sensor(0x3821);

    switch (imgMirror)
    {
        case IMAGE_H_MIRROR://IMAGE_NORMAL:
            OV8858_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set normal
            OV8858_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set normal
            break;
        case IMAGE_NORMAL://IMAGE_V_MIRROR:
            OV8858_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set flip
            OV8858_write_cmos_sensor(0x3821, (flip | (0x06)));	//Set flip
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            OV8858_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror
            OV8858_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set mirror
            break;
        case IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
        printk("sym 0x3820=%x,0x3821=%x\n",(mirror |(0x06)),(flip |(0x06)));
            OV8858_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror & flip
            OV8858_write_cmos_sensor(0x3821, (flip |(0x06)));	//Set mirror & flip
            break;
    }
}
#endif

kal_uint32 OV8858_SET_FrameLength_ByVideoMode(UINT16 Video_TargetFps)
{

    UINT32 frameRate = 0;
	kal_uint32 MIN_FrameLength=0;
	
	if(ov8858.OV8858AutoFlickerMode == KAL_TRUE)
	{
		if (Video_TargetFps==30)
			frameRate= OV8858_AUTOFLICKER_OFFSET_30;
		else if(Video_TargetFps==15)
			frameRate= OV8858_AUTOFLICKER_OFFSET_15;
		else
			frameRate=Video_TargetFps*10;
	
		MIN_FrameLength = (ov8858.videoPclk*10000)/(OV8858_VIDEO_PERIOD_PIXEL_NUMS + ov8858.DummyPixels)/frameRate*10;
	}
	else
		MIN_FrameLength = (ov8858.videoPclk*10000) /(OV8858_VIDEO_PERIOD_PIXEL_NUMS + ov8858.DummyPixels)/Video_TargetFps;

     return MIN_FrameLength;


}



UINT32 OV8858SetVideoMode(UINT16 u2FrameRate)
{
    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV8858DB("[OV8858SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov8858mipiraw_drv_lock);
	OV8858_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov8858mipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV8858DB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV8858DB("abmornal frame rate seting,pay attention~\n");

    if(ov8858.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {

        MIN_Frame_length = OV8858_SET_FrameLength_ByVideoMode(u2FrameRate);

		if((MIN_Frame_length <=OV8858_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV8858_VIDEO_PERIOD_LINE_NUMS;
			OV8858DB("[OV8858SetVideoMode]current fps = %d\n", (ov8858.videoPclk*10000)  /(OV8858_VIDEO_PERIOD_PIXEL_NUMS)/OV8858_VIDEO_PERIOD_LINE_NUMS);
		}
		OV8858DB("[OV8858SetVideoMode]current fps (10 base)= %d\n", (ov8858.videoPclk*10000)*10/(OV8858_VIDEO_PERIOD_PIXEL_NUMS + ov8858.DummyPixels)/MIN_Frame_length);
		extralines = MIN_Frame_length - OV8858_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&ov8858mipiraw_drv_lock);
		ov8858.DummyPixels = 0;//define dummy pixels and lines
		ov8858.DummyLines = extralines ;
		spin_unlock(&ov8858mipiraw_drv_lock);
		
		OV8858_SetDummy(0, extralines);
    }
	
	OV8858DB("[OV8858SetVideoMode]MIN_Frame_length=%d,ov8858.DummyLines=%d\n",MIN_Frame_length,ov8858.DummyLines);

    return KAL_TRUE;
}


UINT32 OV8858SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	if(bEnable) {   
		spin_lock(&ov8858mipiraw_drv_lock);
		ov8858.OV8858AutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov8858mipiraw_drv_lock);
        OV8858DB("OV8858 Enable Auto flicker\n");
    } else {
    	spin_lock(&ov8858mipiraw_drv_lock);
        ov8858.OV8858AutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov8858mipiraw_drv_lock);
        OV8858DB("OV8858 Disable Auto flicker\n");
    }

    return ERROR_NONE;
}


UINT32 OV8858SetTestPatternMode(kal_bool bEnable)
{
    OV8858DB("[OV8858SetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable == KAL_TRUE)
    {
        OV8858_During_testpattern = KAL_TRUE;
		OV8858_write_cmos_sensor(0x5E00,0x80);
    }
	else
	{
        OV8858_During_testpattern = KAL_FALSE;
		OV8858_write_cmos_sensor(0x5E00,0x00);
	}

    return ERROR_NONE;
}


/*************************************************************************
*
* DESCRIPTION:
* INTERFACE FUNCTION, FOR USER TO SET MAX  FRAMERATE;
* 
*************************************************************************/
UINT32 OV8858MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV8858DB("OV8858MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV8858_PREVIEW_PCLK;
			lineLength = OV8858_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8858mipiraw_drv_lock);
			ov8858.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&ov8858mipiraw_drv_lock);
			OV8858_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV8858_VIDEO_PCLK;
			lineLength = OV8858_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8858mipiraw_drv_lock);
			ov8858.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&ov8858mipiraw_drv_lock);
			OV8858_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV8858_CAPTURE_PCLK;
			lineLength = OV8858_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8858mipiraw_drv_lock);
			ov8858.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&ov8858mipiraw_drv_lock);
			OV8858_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;

}


UINT32 OV8858MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = OV8858_MAX_FPS_PREVIEW;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = OV8858_MAX_FPS_CAPTURE;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = OV8858_MAX_FPS_CAPTURE;
			break;		
		default:
			break;
	}

	return ERROR_NONE;

}


void OV8858_NightMode(kal_bool bEnable)
{
	
	#ifdef OV8858_DEBUG
	OV8858DB("[OV8858_NightMode]enter and exit.\n");
	printk("[OV8858_NightMode]enter and exit.\n");
	#endif
}


#if 0
#endif
UINT32 OV8858Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV8858DB("OV8858Preview enter:\n");
	printk("OV8858Preview enter:\n");

	OV8858PreviewSetting();

	spin_lock(&ov8858mipiraw_drv_lock);
	ov8858.sensorMode = SENSOR_MODE_PREVIEW; 
	ov8858.DummyPixels = 0;
	ov8858.DummyLines = 0 ;
	OV8858_FeatureControl_PERIOD_PixelNum=OV8858_PV_PERIOD_PIXEL_NUMS+ ov8858.DummyPixels;
	OV8858_FeatureControl_PERIOD_LineNum=OV8858_PV_PERIOD_LINE_NUMS+ov8858.DummyLines;
	//TODO~
	//ov8858.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov8858mipiraw_drv_lock);
	
	OV8858SetFlipMirror(IMAGE_HV_MIRROR/*sensor_config_data->SensorImageMirror*/);
	//TODO~
    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV8858DB("OV8858Preview exit:\n");

	  
    return ERROR_NONE;
}


UINT32 OV8858Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV8858DB("OV8858Video enter:\n");
	printk("OV8858Video enter:\n");
	OV8858VideoSetting();

	spin_lock(&ov8858mipiraw_drv_lock);
	ov8858.sensorMode = SENSOR_MODE_VIDEO;
	OV8858_FeatureControl_PERIOD_PixelNum=OV8858_VIDEO_PERIOD_PIXEL_NUMS+ ov8858.DummyPixels;
	OV8858_FeatureControl_PERIOD_LineNum=OV8858_VIDEO_PERIOD_LINE_NUMS+ov8858.DummyLines;
	ov8858.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov8858mipiraw_drv_lock);
	
	OV8858SetFlipMirror(IMAGE_HV_MIRROR/*sensor_config_data->SensorImageMirror*/);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV8858DB("OV8865Video exit:\n");
    return ERROR_NONE;
}


UINT32 OV8858Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = ov8858.shutter;
	kal_uint32 temp_data;


	OV8858DB("OV8858Capture enter:\n");

	OV8858CaptureSetting();

	spin_lock(&ov8858mipiraw_drv_lock);
	ov8858.sensorMode = SENSOR_MODE_CAPTURE;
	//TODO~
	//ov8858.imgMirror = sensor_config_data->SensorImageMirror;
	ov8858.DummyPixels = 0;
	ov8858.DummyLines = 0 ;
	OV8858_FeatureControl_PERIOD_PixelNum = OV8858_FULL_PERIOD_PIXEL_NUMS + ov8858.DummyPixels;
	OV8858_FeatureControl_PERIOD_LineNum = OV8858_FULL_PERIOD_LINE_NUMS + ov8858.DummyLines;
	spin_unlock(&ov8858mipiraw_drv_lock);

	OV8858SetFlipMirror(IMAGE_HV_MIRROR/*sensor_config_data->SensorImageMirror*/);
    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY

	#if 0
	if(OV8858_During_testpattern == KAL_TRUE)
	{
		//TODO~
		//Test pattern
		OV8858_write_cmos_sensor(0x5E00,0x80);
	}
	#endif
	OV8858DB("OV8865Capture exit:\n");
    return ERROR_NONE;

}	

#if 0
#endif

UINT32 OV8858GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV8858DB("OV8858GetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= OV8858_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV8858_IMAGE_SENSOR_PV_HEIGHT;
	
    pSensorResolution->SensorFullWidth		= OV8858_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV8858_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorVideoWidth		= OV8858_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV8858_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   

UINT32 OV8858GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    OV8858DB("OV8858GetInfo enter!!\n");
	spin_lock(&ov8858mipiraw_drv_lock);
	ov8858.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov8858mipiraw_drv_lock);

    pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 3;
    pSensorInfo->VideoDelayFrame = 3;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;	    
    pSensorInfo->AESensorGainDelayFrame = 0;
    pSensorInfo->AEISPGainDelayFrame = 2;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
	pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;


    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8858_PV_Y_START;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV8858_VIDEO_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;//0,4,14,32,40
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858_FULL_X_START;	
            pSensorInfo->SensorGrabStartY = OV8858_FULL_Y_START;	

            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8858_PV_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &OV8858SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    OV8858DB("OV8858GetInfo exit!!\n");

    return ERROR_NONE;
}   /* OV8858GetInfo() */



UINT32 OV8858Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov8858mipiraw_drv_lock);
		OV8858CurrentScenarioId = ScenarioId;
		spin_unlock(&ov8858mipiraw_drv_lock);
		
		OV8858DB("[OV8858Control]OV8858CurrentScenarioId=%d\n",OV8858CurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV8858Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV8858Video(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV8858Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* OV8858Control() */


UINT32 OV8858FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV8858_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV8858_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV8858_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV8858_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV8858CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV8858_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV8858_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV8858_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV8858_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV8858_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV8858_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:  
           	OV8858_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV8858_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV8858_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV8858_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov8858mipiraw_drv_lock);
                OV8858SensorCCT[i].Addr=*pFeatureData32++;
                OV8858SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov8858mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8858SensorCCT[i].Addr;
                *pFeatureData32++=OV8858SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov8858mipiraw_drv_lock);
                OV8858SensorReg[i].Addr=*pFeatureData32++;
                OV8858SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov8858mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8858SensorReg[i].Addr;
                *pFeatureData32++=OV8858SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV8858_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV8858SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV8858SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV8858SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV8858_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV8858_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV8858_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV8858_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV8858_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV8858_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
			//TODO~
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
			OV8858SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV8858GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			//TODO~
			OV8858SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV8858MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV8858MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			//TODO~
			OV8858SetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
			*pFeatureReturnPara32=OV8858_TEST_PATTERN_CHECKSUM; 		  
			*pFeatureParaLen=4; 							
		     break;
        default:
            break;
    }
    return ERROR_NONE;
}	


SENSOR_FUNCTION_STRUCT	SensorFuncOV8858=
{
    OV8858Open,
    OV8858GetInfo,
    OV8858GetResolution,
    OV8858FeatureControl,
    OV8858Control,
    OV8858Close
};

UINT32 OV8858_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV8858;

    return ERROR_NONE;
}  

