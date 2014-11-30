/*
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function
bool otp_update_wb(unsigned char golden_rg, unsigned char golden_bg)
and
bool otp_update_lenc(void)
and
then the calibration of AWB & LSC & BLC will be applied. 
After finishing the OTP written, we will provide you the typical value of golden sample.
*/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
	
#include "ov8858mipiraw_Sensor.h"
#include "ov8858mipiraw_Camera_Sensor_para.h"
#include "ov8858mipiraw_CameraCustomized.h"

extern kal_uint16 OV8858_write_cmos_sensor(kal_uint32 addr, kal_uint32 para);

extern kal_uint16 OV8858_read_cmos_sensor(kal_uint32 addr);


#define OV8858_DEBUG
#ifdef OV8858_DEBUG
#define OV8858OTPDB(fmt, arg...)  printk("[sym]" fmt, ##arg)
#else
#define 	OV8858OTPDB(fmt, arg...) 
#endif

//#define SUPPORT_FLOATING

#define OTP_LOAD_ADDR         0x3D81
#define OTP_BANK_ADDR         0x3D84

#define LENC_START_ADDR       0x5800
#define LENC_REG_SIZE         110
			
#define OTP_LENC_GROUP_FLAG   0x703A
#define OTP_LENC_GROUP_ADDR   0x703B

#define OTP_WB_GROUP_FLAG         0x7010
#define OTP_WB_GROUP_ADDR         0x7011
#define OTP_WB_GROUP_FLAG1        0x7020
#define OTP_WB_GROUP_ADDR1        0x7021

#define OTP_H_START_ADDR     0x3D88
#define OTP_L_START_ADDR     0x3D89
#define OTP_H_END_ADDR       0x3D8A
#define OTP_L_END_ADDR       0x3D8B
#define OTP_GROUP_SIZE     5

#define GAIN_RH_ADDR          0x5032
#define GAIN_RL_ADDR          0x5033
#define GAIN_GH_ADDR          0x5034
#define GAIN_GL_ADDR          0x5035
#define GAIN_BH_ADDR          0x5036
#define GAIN_BL_ADDR          0x5037

#define GAIN_DEFAULT_VALUE    0x0400 // 1x gain

#define OTP_MID               0x02


// R/G and B/G of current camera module
unsigned short rg_ratio = 0;
unsigned short bg_ratio = 0;
unsigned char otp_lenc_data[110];
USHORT otp_addr = 0;
USHORT otp_addr1 = 0;


// Enable OTP read function
void otp_read_enable(void)
{
	OV8858_write_cmos_sensor(OTP_LOAD_ADDR, 0x01);
	mdelay(5); // 
}

// Disable OTP read function
void otp_read_disable(void)
{
	OV8858_write_cmos_sensor(OTP_LOAD_ADDR, 0x00);
	mdelay(5); //
}

void otp_read(unsigned short otp_addr, unsigned char* otp_data)
{
	otp_read_enable();
	*otp_data = OV8858_read_cmos_sensor(otp_addr);
	otp_read_disable();
}

/*******************************************************************************
* Function    :  otp_clear
* Description :  Clear OTP buffer 
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
void otp_clear(unsigned short star, unsigned short end)
{
	unsigned short i;
	// After read/write operation, the OTP buffer should be cleared to avoid accident write
		for ( i=star; i<end; i++) 
	{
		OV8858_write_cmos_sensor(i, 0x00);
	}
	
}

/*******************************************************************************
* Function    :  otp_check_wb_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
                 1, group index has invalid data
                 2, group index has valid data
                -1, group index error
*******************************************************************************/	
signed char otp_check_wb_group(unsigned char index)
{   
	unsigned char  flag;
	unsigned char  flag1;

    if (index > 2)
	{
		OV8858OTPDB("OTP input wb group index %d error\n", index);
		return -1;
	}
		
	// select base information flag
  otp_addr = OTP_WB_GROUP_FLAG;
  otp_addr1 = OTP_WB_GROUP_FLAG1;
	OV8858_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
  OV8858_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8858_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
	msleep(5);
  otp_read(otp_addr, &flag);
	OV8858_write_cmos_sensor(otp_addr, 0x00);
	////////select AWB flag
	OV8858_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
  OV8858_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr1>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_START_ADDR, otp_addr1 & 0xff);
	OV8858_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr1>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_END_ADDR, otp_addr1 & 0xff);
	msleep(5);
  otp_read(otp_addr1, &flag1);
	OV8858_write_cmos_sensor(otp_addr1, 0x00);

	// Check all bytes of a group. If all bytes are '0', then the group is empty. 
	// Check from group 1 to group 2, then group 3.
	
if (index==0)
	{
      flag=(flag>>6) & 0x03;
      flag1=(flag1>>6) & 0x03;

	  if (!flag && !flag1)
	  {
		  OV8858OTPDB("wb group %d is empty", index);
		  return 0;
	  }
	  else if (flag == 0x01 && flag1 == 0x01 )
	  {
		  OV8858OTPDB("wb group %d has valid data", index);;
		  return 2;
	  }
	  else //if (flag == 0x11)
	  {
		  OV8858OTPDB("wb group %d has invalid data", index);
		  return 1;
	  }
	}

	else if (index == 1)
	{

		flag=(flag>>4) & 0x03;
		flag1=(flag1>>4) & 0x03;
		
		if (!flag && !flag1)
		{
			OV8858OTPDB("wb group %d is empty", index);
			return 0;
		}
		else if (flag == 0x01 && flag1 == 0x01)
		{
			OV8858OTPDB("wb group %d has valid data", index);
			return 2;
		}
		else //if (flag == 0x11)
		{
			OV8858OTPDB("wb group %d has invalid data", index);
			return 1;
		}


	}

	else
	{

		flag=(flag>>2) & 0x03;
		flag1=(flag1>>2) & 0x03;
		
		if (!flag && !flag1)
		{
			OV8858OTPDB("wb group %d is empty", index);
			return 0;
		}
		else if (flag == 0x01 && flag1 == 0x01)
		{
			OV8858OTPDB("wb group %d has valid data", index);
			return 2;
		}
		else //if (flag == 0x11)
		{
			OV8858OTPDB("wb group %d has invalid data", index);
			return 1;
		}
	
	
	}
}

/*******************************************************************************
* Function    :  otp_read_wb_group
* Description :  Read group value and store it in OTP Struct 
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
                 -1, error
*******************************************************************************/	
signed char otp_read_wb_group(signed char index)
{
	unsigned char  mid, AWB_light_LSB, rg_ratio_MSB, bg_ratio_MSB;

	if (index == -1)
	{
		// Check first OTP with valid data
		for (index=0; index<3; index++)
		{
			if (otp_check_wb_group(index) == 2)
			{
				OV8858OTPDB("read wb from group %d\n", index);
				break;
			}
		}

		if (index > 2)
		{
			OV8858OTPDB("no group has valid data\n");
			return -1;
		}
	}
	else
	{
		if (otp_check_wb_group(index) != 2)
		{
			OV8858OTPDB("read wb from group %d failed\n", index);
			return -1;
		}
	}

	// select adress
	USHORT otp_addr = OTP_WB_GROUP_ADDR + index * OTP_GROUP_SIZE;
	USHORT otp_addr1 = OTP_WB_GROUP_ADDR1 + index * OTP_GROUP_SIZE;

	otp_read(otp_addr, &mid);
	OV8858_write_cmos_sensor(otp_addr, 0x00);
	if (mid != OTP_MID)
	{
		return -1;
	}

	otp_read(otp_addr1,  &rg_ratio_MSB);
	otp_read(otp_addr1+1,  &bg_ratio_MSB);
	otp_read(otp_addr1+4, &AWB_light_LSB);	
	otp_clear(otp_addr1,otp_addr1+OTP_GROUP_SIZE);
	rg_ratio = (rg_ratio_MSB<<2) | ((AWB_light_LSB & 0xC0)>>6);
	bg_ratio = (bg_ratio_MSB<<2) | ((AWB_light_LSB & 0x30)>>4);
	OV8858OTPDB("read wb finished\n");
	return index;
}

#ifdef SUPPORT_FLOATING //Use this if support floating point values
/*******************************************************************************
* Function    :  otp_apply_wb
* Description :  Calcualte and apply R, G, B gain to module
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_apply_wb(unsigned short golden_rg, unsigned short golden_bg)
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	double ratio_r, ratio_g, ratio_b;
	double cmp_rg, cmp_bg;

	if (!golden_rg || !golden_bg)
	{
		OV8858OTPDB("golden_rg / golden_bg can not be zero\n");
		return 0;
	}

	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
        // and R/G, B/G of current module
	cmp_rg = 1.0 * rg_ratio / golden_rg;
	cmp_bg = 1.0 * bg_ratio / golden_bg;

	if ((cmp_rg<1) && (cmp_bg<1))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1;
		ratio_r = 1 / cmp_rg;
		ratio_b = 1 / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1;
		ratio_g = cmp_rg;
		ratio_b = cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1;
		ratio_g = cmp_bg;
		ratio_r = cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1)
	{
		gain_r = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_r);
		OV8858_write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 8);
		OV8858_write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x00ff);
	}

	if (ratio_g != 1)
	{
		gain_g = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_g);
		OV8858_write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 8);
		OV8858_write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x00ff);
	}

	if (ratio_b != 1)
	{
		gain_b = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_b);
		OV8858_write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 8);
		OV8858_write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x00ff);
	}

	OV8858OTPDB("cmp_rg=%f, cmp_bg=%f\n", cmp_rg, cmp_bg);
	OV8858OTPDB("ratio_r=%f, ratio_g=%f, ratio_b=%f\n", ratio_r, ratio_g, ratio_b);
	OV8858OTPDB("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}

#else //Use this if not support floating point values

#define OTP_MULTIPLE_FAC	10000
bool otp_apply_wb(unsigned short golden_rg, unsigned short golden_bg)
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	unsigned short ratio_r, ratio_g, ratio_b;
	unsigned short cmp_rg, cmp_bg;

	if (!golden_rg || !golden_bg)
	{
		OV8858OTPDB("golden_rg / golden_bg can not be zero\n");
		return 0;
	}

	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
    // and R/G, B/G of current module
	cmp_rg = OTP_MULTIPLE_FAC * rg_ratio / golden_rg;
	cmp_bg = OTP_MULTIPLE_FAC * bg_ratio / golden_bg;

	if ((cmp_rg < 1 * OTP_MULTIPLE_FAC) && (cmp_bg < 1 * OTP_MULTIPLE_FAC))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1 * OTP_MULTIPLE_FAC;
		ratio_r = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_rg;
		ratio_b = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_rg;
		ratio_b = OTP_MULTIPLE_FAC * cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_bg;
		ratio_r = OTP_MULTIPLE_FAC * cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1 * OTP_MULTIPLE_FAC)
	{
		gain_r = GAIN_DEFAULT_VALUE * ratio_r / OTP_MULTIPLE_FAC;
		OV8858_write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 8);
		OV8858_write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x00ff);
	}

	if (ratio_g != 1 * OTP_MULTIPLE_FAC)
	{
		gain_g = GAIN_DEFAULT_VALUE * ratio_g / OTP_MULTIPLE_FAC;
		OV8858_write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 8);
		OV8858_write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x00ff);
	}

	if (ratio_b != 1 * OTP_MULTIPLE_FAC)
	{
		gain_b = GAIN_DEFAULT_VALUE * ratio_b / OTP_MULTIPLE_FAC;
		OV8858_write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 8);
		OV8858_write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x00ff);
	}

	OV8858OTPDB("cmp_rg=%d, cmp_bg=%d\n", cmp_rg, cmp_bg);
	OV8858OTPDB("ratio_r=%d, ratio_g=%d, ratio_b=%d\n", ratio_r, ratio_g, ratio_b);
	OV8858OTPDB("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}
#endif /* SUPPORT_FLOATING */

/*******************************************************************************
* Function    :  otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_update_wb(unsigned short golden_rg, unsigned short golden_bg) 
{
	OV8858OTPDB("start wb update\n");
	printk("start wb update\n");

	if (otp_read_wb_group(-1) != -1)
	{
		if (otp_apply_wb(golden_rg, golden_bg) == 1)
		{
			OV8858OTPDB("wb update finished\n");
			return 1;
		}
	}

	OV8858OTPDB("wb update failed\n");
	return 0;
}

/*******************************************************************************
* Function    :  otp_check_lenc_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] BYTE index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
                 1, group index has invalid data
                 2, group index has valid data
                -1, group index error
*******************************************************************************/	
signed char otp_check_lenc_group(BYTE index)
{   
	unsigned char  flag;

    if (index > 2)
	{
		OV8858OTPDB("OTP input lenc group index %d error\n", index);
		return -1;
	}
		
	// select lenc flag
  otp_addr = OTP_LENC_GROUP_FLAG;
	OV8858_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
  OV8858_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8858_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
	msleep(5);
  otp_read(otp_addr, &flag);
	OV8858_write_cmos_sensor(otp_addr, 0x00);

	// Check all bytes of a group. If all bytes are '0', then the group is empty. 
	// Check from group 1 to group 2, then group 3.
	if (index==0)
	{
      flag=(flag>>6) & 0x03;

	  if (!flag)
	  {
		  OV8858OTPDB("lenc group %d is empty", index);
		  return 0;
	  }
	  else if (flag == 0x01)
	  {
		  OV8858OTPDB("lenc group %d has valid data", index);
		  return 2;
	  }
	  else //if (flag == 0x11)
	  {
		  OV8858OTPDB("lenc group %d has invalid data", index);
		  return 1;
	  }
	}

	else if (index == 1)
	{

		flag=(flag>>4) & 0x03;
		
		if (!flag)
		{
			OV8858OTPDB("lenc group %d is empty", index);
			return 0;
		}
		else if (flag == 0x01)
		{
			OV8858OTPDB("lenc group %d has valid data", index);
			return 2;
		}
		else //if (flag == 0x11)
		{
			OV8858OTPDB("lenc group %d has invalid data", index);
			return 1;
		}


	}

	else
	{

		flag=(flag>>2) & 0x03;
		
		if (!flag)
		{
			OV8858OTPDB("lenc group %d is empty", index);
			return 0;
		}
		else if (flag == 0x01)
		{
			OV8858OTPDB("lenc group %d has valid data", index);
			return 2;
		}
		else //if (flag == 0x11)
		{
			OV8858OTPDB("lenc group %d has invalid data", index);
			return 1;
		}
	
	
	}
}

/*******************************************************************************
* Function    :  otp_read_lenc_group
* Description :  Read group value and store it in OTP Struct 
* Parameters  :  [in] int index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
                 -1, error
*******************************************************************************/	
signed char otp_read_lenc_group(int index)
{
	unsigned short otp_addr;
	unsigned char  i;
	if (index == -1)
	{
		// Check first OTP with valid data
		for (index=0; index<3; index++)
		{
			if (otp_check_lenc_group(index) == 2)
			{
				OV8858OTPDB("read lenc from group %d\n", index);
				break;
			}
		}

		if (index > 2)
		{
			OV8858OTPDB("no group has valid data\n");
			return -1;
		}
	}
	else
	{
		if (otp_check_lenc_group(index) != 2) 
		{
			OV8858OTPDB("read lenc from group %d failed\n", index);
			return -1;
		}
	}

	// read lenc data
	otp_addr = OTP_LENC_GROUP_ADDR + index * LENC_REG_SIZE;
	OV8858_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
  OV8858_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8858_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8858_write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+LENC_REG_SIZE) & 0xff);
	otp_read_enable();
	for (i=0; i<LENC_REG_SIZE; i++) 
	{
		otp_lenc_data[i] = OV8858_read_cmos_sensor(otp_addr);
		otp_addr++;
	}
	otp_read_disable();
	otp_clear(otp_addr,otp_addr+LENC_REG_SIZE);
	
	OV8858OTPDB("read lenc finished\n");
	return index;
}

/*******************************************************************************
* Function    :  otp_apply_lenc
* Description :  Apply lens correction setting to module
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
void otp_apply_lenc(void)
{
	// write lens correction setting to registers
	OV8858OTPDB("apply lenc setting\n");

	unsigned char i;


	for (i=0; i<LENC_REG_SIZE; i++)
	{
		OV8858_write_cmos_sensor(LENC_START_ADDR+i, otp_lenc_data[i]);
		OV8858OTPDB("0x%x, 0x%x\n", LENC_START_ADDR+i, otp_lenc_data[i]);
	}
}

/*******************************************************************************
* Function    :  otp_update_lenc
* Description :  Get lens correction setting from otp, then apply to module
* Parameters  :  none
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_update_lenc(void) 
{
	OV8858OTPDB("start lenc update\n");

	if (otp_read_lenc_group(-1) != -1)
	{
		otp_apply_lenc();
		OV8858OTPDB("lenc update finished\n");
		return 1;
	}

	OV8858OTPDB("lenc update failed\n");
	return 0;
}

