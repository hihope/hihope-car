/* Includes ------------------------------------------------------------------*/
#include "IMU.h"
#include "common.h"
#include "i2c.h"
#include <math.h>
#include "log.h"
//正北为0 度.
//x<0 y>0 时+90度
//x<0 y<0 时+180度
//x>0,y<0 时+270度

char capration_flag = 0;
char capration_flag0 = 0;

float x_capration_value =0.0;
float x_max_value =0.0;
float x_min_value =0.0;

float y_capration_value =0.0;
float y_max_value =0.0;
float y_min_value =0.0;

void capration_imu_trigger(void)
{
  capration_flag = 1;
  capration_flag0 = 1;
  
  x_capration_value =0.0;
  x_max_value =0.0;
  x_min_value =0.0;

  y_capration_value =0.0;
  y_max_value =0.0;
  y_min_value =0.0;
}
void capration_imu_op(void){
  int a=0;
  if(capration_flag < MAX_CAPRATION_CIRCLE) {
    a = imu_read_azimuth();
    if(a <= 100) {
        capration_flag0 = 0;
    }
    if((a >= 1900)&&(a < 2500)) {
        if(capration_flag0 == 0) {
          capration_flag++;
		  capration_flag0 = 1;
		}
    }
  }else {
    x_capration_value =(x_max_value + x_min_value)/2;
    y_capration_value =(y_max_value + y_min_value)/2;
    capration_flag = 0;
  }
}
int imu_read_azimuth(void)
{
  uint8 BMX055_MBUF[6];
  char avg_count=0;
  int Mag_dat[3];
  float artan_v=0;
  float angle_t=0;

  float angle_t_b=0;
    
  float temp_value0=0;
  float temp_value1=0;
  float temp_value2=0;

  float temp_value0_b=0;
  float temp_value1_b=0;
  float temp_value2_b=0;
 
  float angle_add=0;
  int i =0;
  ReadData(MAG_XL, BMX055_MBUF, 6);
  for(i = 0; i < 2;i ++)//0 1   0 1 2 3
  {
          Mag_dat[i] = ((int)(BMX055_MBUF[2*i]>>3 | (BMX055_MBUF[2*i+1]<<5)));
  }
  Mag_dat[2] = ((int)(BMX055_MBUF[4]>>1 | (BMX055_MBUF[5]<<7)));

  if(Mag_dat[0]> 4095)
  {
          Mag_dat[0] -= 8192 ;
  }

  if(Mag_dat[1]> 4095)
  {
          Mag_dat[1] -= 8192 ;
  }

  if(Mag_dat[2] > 16383)
  {
          Mag_dat[2] -= 32768 ;
  }
  temp_value0 = Mag_dat[0];
  
  temp_value1 = Mag_dat[1];

  temp_value2 = Mag_dat[2];
  
  if(capration_flag)
  {
	if(temp_value0 > x_max_value )
		x_max_value = temp_value0;
    
	if(temp_value0 < x_min_value )
		x_min_value = temp_value0; 


	if(temp_value1 > y_max_value )
		y_max_value = temp_value1;

	if(temp_value1 < y_min_value )
		y_min_value = temp_value1; 
  
  }else {
	temp_value0 = temp_value0 - x_capration_value;
	temp_value1 = temp_value1 - y_capration_value;
  }

  if((temp_value0 < 0) && (temp_value1>0))
  {
          angle_add = 90;
  }else if((temp_value0 < 0) && (temp_value1 < 0))
  {
          angle_add = 180;
  }else if((temp_value0 > 0) && (temp_value1 < 0)){
          angle_add = 270;
  }

   if((temp_value0 == 0) && (temp_value1 > 0)){
		angle_t = 90;
               return (int)(angle_t * 10);
  }

  if((temp_value0 == 0) && (temp_value1 < 0)){
		angle_t = 270;
                return (int)(angle_t * 10);
  }

  if((temp_value0 > 0) && (temp_value1 == 0)){
		angle_t = 0;
                return (int)(angle_t * 10);
  }

  if((temp_value0 < 0) && (temp_value1 == 0)){
		angle_t = 180;
                return (int)(angle_t * 10);
  }
  
  
  if(temp_value0 < 0)
          temp_value0 = -temp_value0;

  if(temp_value1 < 0)
          temp_value1 = -temp_value1;
  
  artan_v = temp_value1 / temp_value0;		
  angle_t = (atan(artan_v) * 180) / PI;

  angle_t_b = angle_t;

  if((angle_add == 270) || (angle_add == 90))
          angle_t = 90 - angle_t;

  angle_t = angle_add + angle_t;


 

 return (int)(angle_t * 10);

}

int IMU_Init(void)
{
	char temp = 0;
	char reg_vlue = 1;

	HalI2CInit(Mag_addr, i2cClock_33KHZ);
    //enable mag hw
	WriteData(MAG_ret, 0x83);
	// Select Mag register(0x4C)
	// Normal Mode, ODR = 10 Hz(0x00)
	WriteData( 0x4C, 0x00);
	// Select Mag register(0x4E)
	// X, Y, Z-Axis enabled(0x84)
	WriteData( 0x4E, 0x84);
	// Select Mag register(0x51)
	// No. of Repetitions for X-Y Axis = 9(0x04)
	WriteData( 0x51, 0x04);
	// Select Mag register(0x52)
	// No. of Repetitions for Z-Axis = 15(0x0F)
	WriteData(0x52, 0x04);
//read hw id
	ReadData(MAG_ID, &temp, 1);
	while(temp != 0x32){
		HalHW_WaitMS(100);
		ReadData(MAG_ID, &temp, 1);
	}
//self test
	//entry sleep mode 
	WriteData(0x4C, 0x06);
	//set self-test bit
	WriteData(0x4C, 0x07);
	
	//wait self-test bit is 0
	ReadData(0x42, &reg_vlue, 1);
	while(!(reg_vlue & 0x1))
		ReadData(0x42, &reg_vlue, 1);
		 
    WriteData(0x4C, 0x00);
}
