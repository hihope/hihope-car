/*订阅 cmd_vel 
*将角速度和线速度转换成轮速和舵机角度
*上报里程计数据
*通过car_if 发送控制命令，读取imu数据和车轮行程
*根据车轮行程,舵机角度，以及IMU 数据计算odom数据
*/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

//#include "boost/thread/mutex.hpp"
#include "nodelet/nodelet.h"

#include "message_filters/subscriber.h"
//#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include "car_if.h"

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

typedef struct _Car_Status{
	float rotation;
	float wheel_speed;
	float steering_engine_angle;
	float position_x;
	float position_y;
	float relatively_rotaion;
	unsigned char direction;
	unsigned int walks_count;

}Car_Status;

 Car_Status G_car_last_status;

 //每一步的距离 1.1 cm
#define WALK_COUNT_UNIT  1.1 

 #define KEY_TOPIC   "/cmd_vel"
 
#define MAX_ANGLE_VALUE 24*PI_VALUE/180
#define PI_VALUE 3.1415926

// 小车前后轮的距离15.7cm
#define CAR_LONG_CM  15.7
//小车轮子到中心轴的距离8.0cm
#define CAR_HALF_WIDTH_CM 8.0
#define MAX_TRY_TIMES 10
Car_interface *car_if;

#define FAKE_HW_DATA
#define FAKE_IMU
#ifdef FAKE_HW_DATA
unsigned int fake_walk_acc = 0;
#endif


void int_car_status(void)
{
	G_car_last_status.rotation = 0;
	G_car_last_status.wheel_speed = 0;
	G_car_last_status.position_x = 0;
	G_car_last_status.position_y = 0;
	G_car_last_status.direction = 0;
	G_car_last_status.walks_count = 0;	
	G_car_last_status.steering_engine_angle = 0;
	G_car_last_status.relatively_rotaion = 0;
}
void reconfig_car_odom(float rotation , float position_x, float position_y)
{
	G_car_last_status.rotation = rotation;
	G_car_last_status.position_x = position_x;
	G_car_last_status.position_y = position_y;
	G_car_last_status.relatively_rotaion = rotation;
}

void update_car_odom(float x_add , float y_add, float rt_add)
{
	G_car_last_status.position_x += x_add;
	G_car_last_status.position_y += y_add;
	G_car_last_status.relatively_rotaion += rt_add;
  //  ROS_ERROR("calculate_odom raw_rt = %f raw_x=%f raw_y=%f g_position_x %f g_position_y %f g_angular %f \n",rt_add,x_add,y_add,G_car_last_status.position_x,G_car_last_status.position_y,G_car_last_status.relatively_rotaion);	
}

float get_steering_engine_angle_sw(void)
{
	return G_car_last_status.steering_engine_angle;
}

unsigned char get_car_direction_sw(void)
{
	return G_car_last_status.direction;
}

float get_car_rotation(void)
{
#ifdef FAKE_IMU
	return G_car_last_status.relatively_rotaion;
#else
	return G_car_last_status.rotation;
#endif
}

float get_car_rotation_sw(void)
{
	return G_car_last_status.relatively_rotaion;
}


float get_car_speed(void)
{
	return G_car_last_status.wheel_speed;
}

void update_steering_engine_angle_sw(float angle)
{
	G_car_last_status.steering_engine_angle = angle;
}


void update_car_speed(float speed)
{
	G_car_last_status.wheel_speed = speed;
}

void update_car_direction(unsigned char dir)
{
	G_car_last_status.direction = dir;
}


void update_car_roation_hw(float rotation)
{
	G_car_last_status.rotation = rotation;
}

 unsigned int get_fake_walks()
{
	unsigned int fake_walks = G_car_last_status.walks_count + fake_walk_acc;
	if(fake_walks >= 255)
		fake_walks = fake_walks -255;
	return fake_walks;
}

  unsigned int caculate_delta_walks(unsigned int hw_count)
 {
 	unsigned int dt = 0;
	 if(hw_count >= G_car_last_status.walks_count)
	 dt = hw_count - G_car_last_status.walks_count;
	 else
	 dt = 0xff - G_car_last_status.walks_count + hw_count;
	 if((dt < 0) || (dt > 300)) {
		 ROS_ERROR("WALKS ERROR! %d \n",dt);
		 dt = 0;
	 }	 
	 G_car_last_status.walks_count = hw_count; //update count
	 return dt;
 }


int calculate_odom()
{
	unsigned int delta_walks = 0;
	unsigned int walks_count_hw = 0;
	unsigned char dir  = get_car_direction_sw();
	float last_steering_engine_angle = get_steering_engine_angle_sw();
	unsigned int rotation_temp_hw = 0;
	float last_rotation = get_car_rotation();
	int err = -1;
	int try_times = 0;
	unsigned int v_temp = 0;
	unsigned char buf[2];
	float st_angle = PI_VALUE/2 - fabs(last_steering_engine_angle);
	float req_r = 0.0;
	float R0_t = 0.0;
	float L0 = 0.0; // cm
	float L = 0.0;
	float L_n= 0.0;

#ifdef FAKE_HW_DATA
	walks_count_hw = get_fake_walks();
#else
	while(err) { //从硬件读imu和轮速计的值
		if(car_if)
	    	err = car_if->simple_read(buf);

		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}
	v_temp = buf[1];
	rotation_temp_hw = v_temp * 2;
	walks_count_hw = buf[0];
#endif
	delta_walks = caculate_delta_walks(walks_count_hw);
	L0 = delta_walks * WALK_COUNT_UNIT; // 单位 cm
  
	//已知角度，求轨迹半径:
	//(tan(@)*dc) * (tan(@)*dc) = r*r + dc*dc/4
	// (tan(@)*dc) * (tan(@)*dc) - dc*dc/4 = r*r;

   req_r = sqrt(tan(st_angle)*CAR_LONG_CM*tan(st_angle)*CAR_LONG_CM + CAR_LONG_CM*CAR_LONG_CM/4);

//   ROS_ERROR("calculate_odom req_r %f delta_walks %d  walks_count_hw %d \n",req_r,delta_walks,walks_count_hw);

//   ROS_ERROR("calculate_odom st_angle %f L0 %f %d",st_angle,L0,dir);
  //求R0 :R0 = tan(angle)*dc – dw
   R0_t = tan(st_angle)*CAR_LONG_CM - CAR_HALF_WIDTH_CM;

//过滤半径太大的轨迹
   if(R0_t > 10000)
   		R0_t = 0;

  //求车心所经过的距离L : L/L0 = req_r/R0
  if(R0_t == 0)
  	L = L0;
  else
  	L = (req_r * L0) / R0_t;

  //求小车经过的角度
  //周长C = 2PIr
  //走过的角度 n = ( L/C ) * 2PI
	if(R0_t==0)
		L_n = 0;
	else
	L_n = (L/(2*req_r));  //

  float  x =  sin(L_n) * req_r;
  float  y = req_r - cos(L_n) * req_r;
  float raw_x = 0.0;
  float raw_y = 0.0;

  if(R0_t==0)
 	 x=L;

//坐标系旋转 一个导航角后再将增量累加上去。
//	ROS_ERROR("calculate_odom R0_t %f Ln = %f x= %f y=%f \n",R0_t,L_n,x,y);
	raw_x = x*cos(last_rotation) + y*sin(last_rotation);
	raw_y = x*sin(last_rotation)- y*cos(last_rotation);
    if((last_steering_engine_angle >= 0)&&(dir == 0)) {
	update_car_odom(raw_x/100, raw_y/100, L_n);

	} else if((last_steering_engine_angle >= 0)&&(dir == 1))  {
	update_car_odom((-raw_x/100), (-raw_y/100), (-L_n));

	}else if((last_steering_engine_angle < 0)&&(dir == 0)) {
	update_car_odom((raw_x/100), (raw_y/100), -L_n);
	
	}else if((last_steering_engine_angle < 0)&&(dir == 1)) {
	update_car_odom((-raw_x/100), (-raw_y/100), (L_n));
	}
#ifndef FAKE_IMU
	update_car_roation_hw(2*PI_VALUE - (rotation_temp_hw * PI_VALUE/180));
#endif
  return 0;
}

//c = 2pir
//v_l = 2pir/t
//v_a = 2pi/t
//v_l = r*v_a
//r = v_l/v_a
//底层的速度单位为cm/s

void set_linear_speed(float c_speed)
{
	int err = -1;
	int try_times = 0;
	int speed_out = (int)(c_speed*100);
	float last_speed = get_car_speed();
	unsigned char last_dir = get_car_direction_sw();
	unsigned char req_direction = 0;
	unsigned char speed_temp;

	if(speed_out < 0){
		req_direction = 1;
		speed_out = -speed_out;
	}
	else
		req_direction = 0;
	
	if((last_speed == c_speed) &&(last_dir == req_direction))
	{
	//	ROS_ERROR("speed no change!\n");
		return;
	}	
	ROS_ERROR("set_linear_speed %d cm/s  req_direction %d!\n",speed_out,req_direction);

#ifdef FAKE_HW_DATA //模拟模式不设置真实速度
	fake_walk_acc = speed_out/2;
	update_car_speed(c_speed);
	update_car_direction(req_direction);
#else
	speed_temp = speed_out;
//	ROS_ERROR("set_linear_speed %d  %d cm/s !\n",speed_out,speed_temp);
	while(err){
		if(car_if)
			err=car_if->set_speed((char)(speed_temp));
		try_times++;
		ROS_ERROR("set speed %d\n",speed_temp);
		if(try_times > MAX_TRY_TIMES)
		break;
	}
	if(err)
		ROS_ERROR("set_speed error!");
	else		
 		update_car_speed(c_speed);
	
	err =-1;
	while(err){
		if(car_if)
			err=car_if->set_direction(req_direction);
		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}

	if(err)
		ROS_ERROR("set_dir error!");
	else
		update_car_direction(req_direction);
#endif
}

void set_steering_engine_angle(float angle)
{
	int err = -1;
	int try_times=0;
	int angle_hw = (int)(angle*180/PI_VALUE);
	float last_angle = get_steering_engine_angle_sw();
	if(last_angle == angle)
	{
	//	ROS_ERROR("angle not change !\n");
		return;
	}	
#ifdef FAKE_HW_DATA
	update_steering_engine_angle_sw(angle);
#else
	angle_hw = 45 - angle_hw; //hw在45度时为0位置. 
	while(err){
	if(car_if)
	err=car_if->set_wheel_angle((char)(angle_hw));
	try_times++;
	if(try_times > MAX_TRY_TIMES)
	break;
	}

	if(err)
	ROS_ERROR("set_wheel_angle error!");
	else
		update_steering_engine_angle_sw(angle);
#endif
}


void chatterCallback(const geometry_msgs::Twist& twistMsg)
{
	float r = 0.0;
	float linear_speed_temp = twistMsg.linear.x;
	float angular_speed_temp = twistMsg.angular.z;
	float angular_st_temp = 0.0;
    float half_dc_temp = CAR_LONG_CM/200;
	float dc_temp = CAR_LONG_CM/100;
	//ROS_ERROR("chatterCallback in %f %f %f \n",linear_speed_temp,angular_speed_temp,r);

	if(angular_speed_temp == 0) //角速度为0:
	{
		angular_st_temp = 0.0;
	} else if((angular_speed_temp != 0)&&(linear_speed_temp == 0)){ 
		//线速度为0时无法转弯，不支持这样的命令
		ROS_ERROR("not support this cmd! as %f ls %f \n",angular_speed_temp,linear_speed_temp);
	} else {
		//线速度角速度都不为0
		r = linear_speed_temp/angular_speed_temp;
		//根据转向半径计算舵机转向的角度
		angular_st_temp  = PI_VALUE/2 - atan(sqrt(r*r + half_dc_temp*half_dc_temp)/dc_temp);
		
#if 0
		//判断舵机转向的方向
		if(angular_speed_temp < 0){
			if(linear_speed_temp > 0)
				angular_st_temp = -angular_st_temp;
		}

		if(angular_speed_temp > 0){
			if(linear_speed_temp < 0)
				angular_st_temp = -angular_st_temp;
		}
#else
//判断舵机转向的方向

if(linear_speed_temp > 0){
	if(angular_speed_temp > 0)
		angular_st_temp = angular_st_temp;
}

if(linear_speed_temp > 0){
	if(angular_speed_temp < 0)
		angular_st_temp = -angular_st_temp;
}

if(linear_speed_temp < 0){
	if(angular_speed_temp < 0)
		angular_st_temp = angular_st_temp;
}


if(linear_speed_temp < 0){
	if(angular_speed_temp > 0)
		angular_st_temp = -angular_st_temp;
}


#endif
		//舵机角度的限制
		if(angular_st_temp > MAX_ANGLE_VALUE){
			angular_st_temp = MAX_ANGLE_VALUE;
		} else if (angular_st_temp < -MAX_ANGLE_VALUE){
			angular_st_temp = -MAX_ANGLE_VALUE;
		}
	}

	//limit the speed;nor allow speed lower than 0.3 m/s
	if((linear_speed_temp > 0)&&(linear_speed_temp < 0.3))
	{
	   linear_speed_temp = 0.3;
	}else if((linear_speed_temp < 0)&&(linear_speed_temp > -0.3))
	{
	   linear_speed_temp = -0.3;
	}

//	ROS_ERROR("chatterCallback %f %f %f %f \n",linear_speed_temp,angular_speed_temp,r,angular_st_temp);
	
	set_linear_speed(linear_speed_temp);
	set_steering_engine_angle(angular_st_temp);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "minicar");
	ros::NodeHandle n;
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
    ros::Time current_time;
	
	geometry_msgs::TransformStamped odom_trans;
	tf::TransformBroadcaster broadcaster;
	
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	nav_msgs::Path path;
	path.header.stamp=current_time;
	path.header.frame_id="odom";

	int_car_status();
	reconfig_car_odom(0,10,5);
#ifdef FAKE_HW_DATA
	//no need hw
#else
	car_if = new Car_interface("/dev/ttyUSB0");
#endif		
  ros::Subscriber sub = n.subscribe(KEY_TOPIC, 1000, chatterCallback);
  ROS_INFO("minicar start \n");
  ros::Rate loop_rate(10);
  float rotation=0;
  while (ros::ok())
  {
   	calculate_odom();
	current_time = ros::Time::now();
	odom_trans.header.stamp = current_time;
	odom_trans.transform.translation.x = G_car_last_status.position_x;
	odom_trans.transform.translation.y = G_car_last_status.position_y;
	odom_trans.transform.translation.z = 0.0;
	rotation = get_car_rotation();
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rotation);
	broadcaster.sendTransform(odom_trans);


	geometry_msgs::PoseStamped this_pose_stamped;
	this_pose_stamped.pose.position.x = G_car_last_status.position_x;
	this_pose_stamped.pose.position.y = G_car_last_status.position_y;

	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(rotation);
	this_pose_stamped.pose.orientation.x = goal_quat.x;
	this_pose_stamped.pose.orientation.y = goal_quat.y;
	this_pose_stamped.pose.orientation.z = goal_quat.z;
	this_pose_stamped.pose.orientation.w = goal_quat.w;

	this_pose_stamped.header.stamp=current_time;
	path_pub.publish(path);
	this_pose_stamped.header.frame_id="odom";
	path.poses.push_back(this_pose_stamped);
	
  	ros::spinOnce();
	loop_rate.sleep();
  }
  set_linear_speed(0);
  return 0;
}

