#include "ctl_hw.h"
#include "IMU.h"
#include <ioCC2540.h>
#include "log.h"

//线顺序
//  蓝         紫         灰        白         黑         1to2         
//  SH_CP    ST_CP      DSS      /OE     舵机PWM      直流pwm     轮速里程计
//  p1.0     p1.3       p0.0     p0.1    p1.1        p1.0        p0.7
//直流pwm 控制脚换成p1.0
/*IO  口分配
	p1.0     SH_CP
	p1.1     舵机pwm
	p1.2     直流电机pwm
	p1.3     ST_CP
	p0.0     Data
	p0.1     /OE
*/
/////////////////////////////////////////////////////////
///////////方向控制/////////////////////////////////////
///////////////////////////////////////////////////////
static void car_stop();
static void car_start();
uint16 G_throttle = 0;
char g_dir = 0;
char g_start_flag = 0;
void send_serial_data(uint8 pinctl)
{
	int i = 0;
    ST_CP = 0;
    DATA_P = 0;
	for(i = 0; i < 8; i++)
	{
		if((pinctl<<i) & 0x80)
			DATA_P = 1;
		else
			DATA_P = 0;
		
	    SH_CP = 0;
		HalHW_WaitUs(1);
		SH_CP = 1;
		HalHW_WaitUs(1);
	}
	ST_CP = 0;
	HalHW_WaitUs(1);
	ST_CP = 1;
	HalHW_WaitUs(1);
}

int hw_set_direction(unsigned char dir)
{
	g_dir = dir;
	if(g_dir <= 0){
		send_serial_data(CODE_DIR_F);
	}else{
		send_serial_data(CODE_DIR_B);
	}
		N_OE = 0;
        return 0;
}


static void car_stop()
{
  if(g_start_flag){
		send_serial_data(0);
		g_start_flag = 0;
  }
}

static void car_start()
{
	if(g_start_flag == 0){
		hw_set_direction(g_dir);
		g_start_flag = 1;
	}
}


/////////////////////////////////////////////////
//////// 里程计/////////////////////////////////
///////////////////////////////////////////////
//轮周长22 cm
//轮周脉冲数量:20
unsigned int G_speed_count = 0;
unsigned int G_speed_timer = 0;
unsigned int G_real_speed = 0;

void space_counter_init(void)
{
//timer 1 的  channel 4 用来捕获论速计中断
	P0SEL &= 0x7f;
	P0DIR &= 0x7f;
	T1CCTL4 = 0x41; //捕获模式，中断使能
	T1CC4L = 0X00;
    T1CC4H = 0X0;

}
void space_counter()
{
  G_speed_count ++;
  walk_count();
  T1STAT = ~0x10;
}


void caculate_real_speed()
{
	G_speed_timer++;
	if(G_speed_timer == 10)  //600ms 计算一次速度
	{
		G_real_speed = (((G_speed_count * 100)) * 11) / 200; //(cm/s)
		G_speed_count = 0;
		G_speed_timer = 0;
		Gspeed_Real = G_real_speed;
		caculate_speed_pwm();
	}
}

//////////////////////////////////////////////////
/////////转向控制////////////////////////////////
////////////////////////////////////////////////
//4M
//1/32 * 8 = 2.5 / 1000
//1/32 * 8 * 16 = 2.5 * 4 * 4 / 1000
//1 /32 *8 * 16 = 10 *2   *2 / 10000 =10*2  / 0x1388 
void wheel_contrl_init()
{
  //����pwm�˿�Ϊ��

  P0DIR |= BV(0)|BV(1);
  P0SEL &= 0xfc;

  P1DIR |= BV(0)|BV(1)|BV(2)|BV(3)|BV(4);
  //����pwm�˿�Ϊ����˿ڣ���gpio
  P1SEL |= BV(0)|BV(1);//|BV(2);//|BV(3)|BV(4);
  //����uart�Ȼ�ռ�����ǵ�ǰʹ�õ�pwm�˿ڣ������Ҫ��uart����ӳ�䵽��Ķ˿�ȥ��
  PERCFG |= 0x40;             // Move USART1&2 to alternate2 location so that T1 is visible

  // Initialize Timer 1
  T1CTL = 0x0c;               // Div = 128(C) div=8 (4), CLR, MODE = Suspended          

  T1CCTL0 = 0x4C;   
  T1CC0L = 0x88;    //20ms 的周期
  T1CC0H = 0x13;     

  T1CCTL1 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare  
  T1CC1L = 0; 
  T1CC1H = 0;             

  T1CNTL = 0;                 // Reset timer to 0;  
  T1CTL |= 0x02;  //start t1
  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

void update_wheel_angle(void)
{  
  T1CC1L = (char)G_angle;
  T1CC1H = (char)(G_angle >> 8);
  T1CNTL = 0;
  T1CTL |= 0x02; 
}

void Throttle_init()
{
	T1CCTL2 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare  
	T1CC2L = 0; 
	T1CC2H = 0;          
}

void update_throttle(void)
{
        if(speed_pwm == 0)
          speed_pwm = 1;
        
	T1CC2L = (char)speed_pwm;
	T1CC2H = (char)(speed_pwm >> 8);

	if(speed_pwm == 0)
		car_stop();
	else
		car_start();

	if(capration_flag > 0)
		capration_imu_op();
        
    //    Log("speed_pwm is %d \n",speed_pwm);
}

//#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void T1_isr (void) {
    uint8 flags = T1STAT;
    // T1 ch 0
    if (flags & 0x01) {
      update_throttle();
      update_wheel_angle(); //更新舵机角度。
      caculate_real_speed(); //计算并反馈真实速度。
    }
	if (flags & 0x10){ 
      	 space_counter();//捕获里程计的一个脉冲。
    }
    T1STAT = ~ flags;
}

int ctl_hw_init(void)
{
	wheel_contrl_init();	
	Throttle_init();
	space_counter_init();
	IMU_Init();
	return 0;
}

