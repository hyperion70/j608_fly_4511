/*===========================================================================

                        EDIT HISTORY FOR V11-wgs

when              comment tag        who                  what, where, why                           
----------    ------------     -----------      --------------------------      

===========================================================================*/
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
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

#include "imx179mipiraw_Sensor.h"
#include "imx179mipiraw_Camera_Sensor_para.h"
#include "imx179mipiraw_CameraCustomized.h"

#undef CDBG
#define CDBG(fmt, args...) printk(KERN_INFO "imx179_OTP.c: " fmt, ## args)


extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define IMX179MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX179MIPI_WRITE_ID)

extern kal_uint16 IMX179MIPI_read_cmos_sensor(kal_uint32 addr);


#define custom_TRULY  2

// R/G and B/G of typical camera module is defined here
//add for distinguish diffrent moudle Truly or Liteon
static uint32_t RG_Ratio_typical=0x00;
static uint32_t BG_Ratio_typical=0x00;
static uint32_t GG_Ratio_typical=0x00;

static uint32_t RG_Ratio_typical_Truly=0x00;
static uint32_t BG_Ratio_typical_Truly=0x00;
static uint32_t GG_Ratio_typical_Truly=0x00;
	
struct otp_struct {
    uint customer_id;
    uint year;
    uint month;
    uint day;
    uint lens_id;
    uint vcm_id;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
    uint32_t gg_ratio;
};

static uint32_t GAIN_DEFAULT=0x100;
static uint wb_flag=0;
kal_uint32 r_ratio;
kal_uint32 b_ratio;
uint32_t R_GAIN, B_GAIN, Gr_GAIN, Gb_GAIN, G_GAIN;

// index: index of otp group. (0, 1, 2)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int check_otp_awb(uint index)
{
    unsigned char  flag, page;

    // select page
    page = 1 + index;    
    // turn on otp read mode
	IMX179MIPI_write_cmos_sensor(0x0100, 0x00); // Set SW_Standby Mode

	IMX179MIPI_write_cmos_sensor(0x3382, 0x05); // Set OTP_Write CLK , 60us=data/Inclk , MCLK 修改，这个值也需要修改 目前用24MHz 
	IMX179MIPI_write_cmos_sensor(0x3383, 0xA0);
	IMX179MIPI_write_cmos_sensor(0x3368, 0x18); // Set MCLK data=mclk*256
	IMX179MIPI_write_cmos_sensor(0x3369, 0x00);

	IMX179MIPI_write_cmos_sensor(0x3380, 0x00); // Set ECC ON

	IMX179MIPI_write_cmos_sensor(0x3400, 0x01); // Read ON

	IMX179MIPI_write_cmos_sensor(0x3402, page); //Select Page

	mdelay(10);

   
    // read flag
    flag = IMX179MIPI_read_cmos_sensor(0x3404);
    printk("check_otp_flag:reg 0x3404flag=%d",flag);

	flag &= 0xc0;

    if (!flag)
     {
        return 0;
    	}
    else if(flag == 0x40) 
    	{
        return 2;
   		}
    else 
    	{
        return 1;
    	}
 
}

// index: index of otp group. (0, 1, 2)
// return: 0, group index is empty
//         1, group index has invalid data
//         2, group index has valid data

static int read_otp(uint page, uint32_t address, uint* array, uint32_t size)
{
    uint32_t i = 0, j = 0;
  	IMX179MIPI_write_cmos_sensor(0x0100, 0x00); // Set SW_Standby Mode

	IMX179MIPI_write_cmos_sensor(0x3382, 0x05); // Set OTP_Write CLK , 60us=data/Inclk , MCLK 修改，这个值也需要修改 目前用24MHz 
	IMX179MIPI_write_cmos_sensor(0x3383, 0xA0);
	IMX179MIPI_write_cmos_sensor(0x3368, 0x18); // Set MCLK data=mclk*256
	IMX179MIPI_write_cmos_sensor(0x3369, 0x00);

	IMX179MIPI_write_cmos_sensor(0x3380, 0x00); // Set ECC ON

	IMX179MIPI_write_cmos_sensor(0x3400, 0x01); // Read ON

	IMX179MIPI_write_cmos_sensor(0x3402, page); //Select Page

	mdelay(10);

	for(i=0;i<64;i++)
		{
			*(array+i)=IMX179MIPI_read_cmos_sensor(address+i);

			
            printk(" read_from_otp [%d] == 0x%x\n",i,array[i]);
       
		}
        
						printk(" page = 0x%x\n",page);
       
    return 1;
}


// index: index of otp group. (0, 1, 2)
// return:0,
static int read_otp_wb(uint index, struct otp_struct* otp)
{
    uint32_t address=0x3404;
    int page, temp;
    uint arr[64]={0};

    //1 select page
    page = 1 + index;    

    temp=read_otp(page, address, arr, 64);
    
	
   if (temp==1)
   	{
   	    uint32_t current_g,current_g1,current_r,current_b,golden_r,golden_b;
	    otp->customer_id = arr[1];
	    otp->year = arr[2];
	    otp->month = arr[3];
	    otp->day = arr[4];

	    otp->lens_id = arr[5];
	    otp->vcm_id = arr[6];

	    current_g= (((arr[14]<<8)|arr[15]) + ((arr[16]<<8)|arr[17]))/2;
	    printk(" wgs-current_g = 0x%x\n",current_g);
	    
	    current_r= ((arr[12]<<8) | arr[13]);
	    printk(" wgs-current_r = 0x%x\n",current_r);
	    
	    current_b= ((arr[18]<<8) | arr[19]);
	    printk(" wgs-current_b = 0x%x\n",current_b);
		 	
	    otp->rg_ratio =1024* current_r /current_g;
	    otp->bg_ratio = 1024* current_b /current_g;
	    printk(" wgs-RG_Ratio_current = 0x%x\n",otp->rg_ratio);
      printk(" wgs-BG_Ratio_current = 0x%x\n",otp->bg_ratio);

	     current_g1= (((arr[22]<<8)|arr[23]) + ((arr[24]<<8)|arr[25]))/2;
	     printk(" wgs-golden_g = 0x%x\n",current_g1);
	     
	     golden_r= ((arr[20]<<8) | arr[21]);
	   	 printk(" wgs-golden_r = 0x%x\n",golden_r);
	    
	    golden_b= ((arr[26]<<8) | arr[27]);
	    printk(" wgs-golden_b = 0x%x\n",golden_b);
		 	
	    RG_Ratio_typical_Truly =1024* golden_r /current_g1;
	    BG_Ratio_typical_Truly = 1024 * golden_b /current_g1;
	    printk(" wgs-RG_Ratio_typical = 0x%x\n",RG_Ratio_typical_Truly);
      printk(" wgs-BG_Ratio_typical = 0x%x\n",BG_Ratio_typical_Truly);
		
   }
   else{
   	
	   	printk(" read all otp fail\n");
			return 0;
   		}

    return 1;
}

// R_gain: red gain of sensor AWB, 0x400 = 1
// G_gain: green gain of sensor AWB, 0x400 = 1
// B_gain: blue gain of sensor AWB, 0x400 = 1
// return 0
int imx179_update_awb_gain()//uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
    
    if(1 == wb_flag)
    {
	printk("[Truly] update_awb_gain() call, wb_flag == 1.\n");
       
	IMX179MIPI_write_cmos_sensor(0x020E, G_GAIN>>8);
	IMX179MIPI_write_cmos_sensor(0x020F, G_GAIN& 0xFF);
	IMX179MIPI_write_cmos_sensor(0x0210, R_GAIN >>8);
	IMX179MIPI_write_cmos_sensor(0x0211, R_GAIN & 0xFF);
	IMX179MIPI_write_cmos_sensor(0x0212, B_GAIN >>8);
	IMX179MIPI_write_cmos_sensor(0x0213, B_GAIN & 0xFF);
	IMX179MIPI_write_cmos_sensor(0x0214, G_GAIN>>8);
	IMX179MIPI_write_cmos_sensor(0x0215, G_GAIN& 0xFF);
        return 0;
    }
    else
    	{
    		printk("[Truly] update_awb_gain()fail\n");
    	}
}


// call this function after ov8825 initialization
// return value: 0, update success
//               1, no OTP
int imx179_update_otp_wb(void)
{
    int i, temp, otp_index;
    struct otp_struct current_otp;
	
    // R/G and B/G of current camera module is read out from sensor OTP
    // check first wb OTP with valid data
    printk(" update_otp_wb++\n");
    for(i=0;i<3;i++) 
    {
        temp = check_otp_awb(i);
        if (temp == 2) 
        	{
            otp_index = i;
            break;
       		}
    }
    if(i==3)
     {
        // no valid wb OTP data
        printk(" no valid wb OTP data\n");
        return 1;
   	 }

    memset(&current_otp, 0, sizeof(current_otp));
    wb_flag = read_otp_wb(otp_index, &current_otp);
    if(1 == wb_flag)
    {
        //add for distinguish diffrent moudle Truly or Liteon
        printk(" #########current_otp->customer_id = 0x%x\n",current_otp.customer_id);


	 	 if(current_otp.customer_id == 0x02)		
        {
            RG_Ratio_typical = RG_Ratio_typical_Truly;
            BG_Ratio_typical = BG_Ratio_typical_Truly;
            GG_Ratio_typical = GG_Ratio_typical_Truly;
        }
		else
			{
				printk(" #########it is not Truly module = 0x%x\n",current_otp.customer_id);
			}
        //calculate gain
        //0x400 = 1x gain
        printk(" #########current_otp.rg_ratio = 0x%x\n",current_otp.rg_ratio);
        printk(" #########current_otp.bg_ratio = 0x%x\n",current_otp.bg_ratio);
        printk(" #########current_otp.gg_ratio = 0x%x\n",current_otp.gg_ratio);
        printk(" #########RG_Ratio_typical = 0x%x\n",RG_Ratio_typical);
        printk(" #########BG_Ratio_typical = 0x%x\n",BG_Ratio_typical);
        printk(" #########GG_Ratio_typical = 0x%x\n",GG_Ratio_typical);


	r_ratio = 512 * RG_Ratio_typical / current_otp.rg_ratio;
	b_ratio = 512 * BG_Ratio_typical / current_otp.bg_ratio;


	if(r_ratio >= 512 )
	{
		if(b_ratio>=512) 
		{
			R_GAIN = (USHORT)(GAIN_DEFAULT * r_ratio / 512);						
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (USHORT)(GAIN_DEFAULT * b_ratio / 512);
		}
		else
		{
			R_GAIN =  (USHORT)(GAIN_DEFAULT*r_ratio / b_ratio );
			G_GAIN = (USHORT)(GAIN_DEFAULT*512 / b_ratio );
			B_GAIN = GAIN_DEFAULT; 
		}
	}
	else                      
	{
		if(b_ratio >= 512)
		{
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = (USHORT)(GAIN_DEFAULT*512 /r_ratio);		
			B_GAIN =  (USHORT)(GAIN_DEFAULT*b_ratio / r_ratio );
		} 
		else 
		{
			Gr_GAIN = (USHORT)(GAIN_DEFAULT*512/ r_ratio );						
			Gb_GAIN = (USHORT)(GAIN_DEFAULT*512/b_ratio );						
			if(Gr_GAIN >= Gb_GAIN)						
			{						
				R_GAIN = GAIN_DEFAULT;						
				G_GAIN = (USHORT)(GAIN_DEFAULT *512/ r_ratio );						
				B_GAIN =  (USHORT)(GAIN_DEFAULT*b_ratio / r_ratio );						
			} 
			else
			{						
				R_GAIN =  (USHORT)(GAIN_DEFAULT*r_ratio  / b_ratio);						
				G_GAIN = (USHORT)(GAIN_DEFAULT*512 / b_ratio );						
				B_GAIN = GAIN_DEFAULT;
			}
		}        
	}	
        printk(" R_gain == 0x%x\n",R_GAIN);
        printk(" G_gain == 0x%x\n",G_GAIN);
        printk(" B_gain == 0x%x\n",B_GAIN);

  
		imx179_update_awb_gain();  
		
    }

    printk("update_otp_wb--\n");
    return 0;
}

#if 0
extern int IMM_GetOneChannelValue_Cali(int Channel, int*voltage);
int get_sensor_imx179_customer_id()
{
    int i, temp, otp_index;
    struct otp_struct current_otp;

    int camera_id_volt = 0;
    int ret = 0;

   ret = IMM_GetOneChannelValue_Cali(0, &camera_id_volt);
   if(ret != 0)
	    printk("[get_sensor_imx179_customer_id]camera_id_volt read fail\n");
    else
	    printk("[get_sensor_imx179_customer_id]camera_id_volt = %d\n", camera_id_volt);

   camera_id_volt/=1000;          //volt uV to mV
    if (( 600 < camera_id_volt )&&(camera_id_volt< 1200) )			//truly ID volt is 0.9V. shunyu is 0V
    {
	customer_id=2;
    }
    else if(camera_id_volt< 300)
    {
	customer_id=1;
    }
    else
    {
	customer_id=0;
    }

    IMX179MIPI_write_cmos_sensor(0x0100, 0x0);
		
    for(i=2;i>0;i--) {
        temp = check_otp_awb(i);
        if (temp == 2) {
            otp_index = i;
            break;
        }
    }
    printk("otp_index=%d,i=%d\n",otp_index,i);
    if(i == 0) {
        printk("no valid wb OTP data\n");
        return 0;
    }
    memset(&current_otp, 0, sizeof(current_otp));
    if(0 == read_otp_wb(otp_index, &current_otp))
    {
//	customer_id = current_otp.customer_id;
 	printk("current_otp.customer_id =%d\n",current_otp.customer_id);
    }
    printk("[get_sensor_imx179_customer_id =%d\n",customer_id);


     return customer_id;
}

#endif 
