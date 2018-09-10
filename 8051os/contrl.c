#include "core.h"
#include "contrl.h"
#include "command.h"
#include "log.h"
/*
char cmd;
char need_ack;
cmd_call_back cmd_cb;
*/
//hw层获得:舵机角度的设置回调函数
//         speed的设置函数
//hal 层 管理req_speed real_speed req_angle IMU 等数据。
//req_speed ，req_angle 是由 command 发送下来.
//real_speed 是由底层feed_back 而来.
//IMU  数据是被动上报还是主动获取？
Contrl_hw *ctl_hw;
uint16 Gspeed_Req=0;
uint16 Gspeed_Real=0;
unsigned int G_azimuth=0;
uint16 speed_pwm=0;
uint16 G_angle=0;
char G_dir=0;
unsigned char G_walks_count=0;

struct _pid{
    int SetSpeed;            //定义设定值
    int ActualSpeed;        //定义实际值
    int err;                //定义偏差值
    int err_last;            //定义上一个偏差值
    int Kp,Ki,Kd;            //定义比例、积分、微分系数
    int voltage;          //定义电压值（控制执行器的变量）
    int integral;            //定义积分值
}pid;

void PID_init(){
  //  printf("PID_init begin \n");
    pid.SetSpeed=0.0;
    pid.ActualSpeed=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.voltage=0.0;
    pid.integral=0.0;
    pid.Kp=50;
    pid.Ki=3;
    pid.Kd=20;
 //   printf("PID_init end \n");
}

void caculate_speed_pwm(void){
  if(Gspeed_Req <= 0){
      speed_pwm =0;
      PID_init();
    }
    else{
    pid.SetSpeed=Gspeed_Req;
    pid.ActualSpeed = Gspeed_Real;
    pid.err=pid.SetSpeed-pid.ActualSpeed;
    pid.integral+=pid.err;
   // Log("pid.err = %d ,integral %d req %d real %d ",pid.err,pid.integral,Gspeed_Req,Gspeed_Real);
    pid.voltage=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
    pid.err_last=pid.err;
    pid.ActualSpeed=pid.voltage*1.0;
    if(pid.ActualSpeed < 0)
      pid.ActualSpeed = 0;
      speed_pwm = 1000 + (int)(pid.ActualSpeed);
    }
   // return pid.ActualSpeed;
}


unsigned int get_speed_pwm(void){
	return speed_pwm;
 }
#if 0
  void caculate_speed_pwm(void)
 {	 
	 int dec=0;
	 if(abs(Gspeed_Req - Gspeed_Real) > SPEED_RANGE_BIG)
		 dec = BIG_JUMP_VALUE;
	 else if(abs(Gspeed_Req - Gspeed_Real) > SPEED_RANGE_MID)
			 dec = MID_JUMP_VALUE;
	 else if(abs(Gspeed_Req - Gspeed_Real) > SPEED_RANGE_SMALL)
			 dec = SMALL_JUMP_VALUE;
	 else if(abs(Gspeed_Req - Gspeed_Real) > SPEED_RANGE_SSMALL)
			 dec = SSMALL_JUMP_VALUE;
 
	 if(Gspeed_Req > Gspeed_Real) {
		 if(speed_pwm  < (FULL_SPEED_PWM_VALUE - dec))
		 		speed_pwm = speed_pwm + dec;
		 else
		 	speed_pwm = FULL_SPEED_PWM_VALUE;
	 }
	 else { 
	 	if(speed_pwm > dec)
			speed_pwm = speed_pwm - dec;
		else
			speed_pwm = 0;
	 }
 }
#endif
  int set_direction(unsigned char dir)
 {
	hw_set_direction(dir);
        G_dir = dir;
	 return 0;
 }

 int set_speed(unsigned char speed_n)
{
	Gspeed_Req = speed_n;
	if(Gspeed_Req == 0x0)
		speed_pwm = 0;
	return 0;
}


 int read_azimuth(unsigned char n_need)
{
	 unsigned char G_out_buf[MAX_OUT_BUF_LEN];
	 out_buf G_cmd_out;

		if(n_need) {
	          G_azimuth = imu_read_azimuth();
		}
		G_out_buf[0] = CMD_AZIMUTH;
		G_out_buf[1] = G_azimuth & 0xff;
		G_out_buf[2] = (G_azimuth >> 8) & 0xff; 
		G_cmd_out.buf = G_out_buf;
		G_cmd_out.len = 3;
		send_data_cb(&G_cmd_out);
	return 0;
}


 int set_angle(unsigned char angle_n)
{
	uint32 angle_temp =((FULL_PWM_VALUE - PWM_ANGLE_BASE)*10)/ PWM_ANGLE_RANGE;  //放大10倍，保留1位小数
	G_angle = (angle_temp*angle_n)/10 + PWM_ANGLE_BASE;
	printf("set_angle %d !\n",G_angle);
    return 0;
}

 int read_odom_raw(unsigned char n_need)
{
	int value =0;
	unsigned char G_out_buf[MAX_OUT_BUF_LEN];
	out_buf G_cmd_out; 
	G_out_buf[0] = CMD_ODOM_RAW;
	G_out_buf[1] = G_walks_count & 0xff;
	G_out_buf[2] = (G_walks_count >> 8) & 0xff;
	G_cmd_out.buf = G_out_buf;
	G_cmd_out.len = 3;
	send_data_cb(&G_cmd_out);
	return 0;
}
  int simple_read_all(unsigned char n_need)
 {
 	 unsigned char G_out_buf[MAX_OUT_BUF_LEN];
	 out_buf G_cmd_out;
	 unsigned int value =imu_read_azimuth()/10;
	 unsigned char azimuth_data = (unsigned char)(value/2);
	 G_out_buf[0] = CMD_SIMPLE_READ_ALL;
	 G_out_buf[1] = G_walks_count;
	 G_out_buf[2] = azimuth_data;
	 G_cmd_out.buf = G_out_buf;
	 G_cmd_out.len = 3;
	 send_data_cb(&G_cmd_out);
	 return 0;
 }

  int simple_read_walk(unsigned char n_need)
 {         
         send_ack(CMD_SIMPLE_READ_WALK,G_walks_count);
	 return 0;
 }


 void walk_count(void)
{
	G_walks_count++;
	if(G_walks_count == 0xff)
		G_walks_count = 0;
}

int speed_feedback_cb(void* data)
{
	int *temp_d=(int*)data;
	Gspeed_Real = *temp_d;
	caculate_speed_pwm();
	return 0;
}


void capration_imu(void){
    capration_imu_trigger();
}

void capration_ret(void){
	unsigned char G_out_buf[MAX_OUT_BUF_LEN];
	out_buf G_cmd_out;
	G_out_buf[0] = CMD_IMU_CAPRATION_STATUS;
	G_out_buf[1] = capration_flag;
	G_out_buf[2] = 0;
	G_cmd_out.buf = G_out_buf;
	G_cmd_out.len = 3;
	send_data_cb(&G_cmd_out);

}

int init_contrl(void)
{
    PID_init();
    set_angle(43);
    return 0;
}

