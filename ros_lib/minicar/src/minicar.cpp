/*订阅 cmd_vel 
*将角速度和线速度转换成轮速和舵机角度
*上报里程计数据
*通过car_if 发送控制命令，读取imu数据和车轮行程
*根据车轮行程,舵机角度，以及IMU 数据计算odom数据
*/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>


//#include "boost/thread/mutex.hpp"
#include "nodelet/nodelet.h"

#include "message_filters/subscriber.h"
//#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include "car_if.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>


#include <math.h>

//#define USE_CONTRL_FILLTER

//用kf filter 优化里程计数据
//#define FAKE_HW_DATA
#define FAKE_IMU
//#define USE_KF_FILTER
//#define SIM_DEBUG

ros::Publisher fusion_odompub;
ros::Time pub_odom_anchor;

#define KF_ODOM
bool kf_odom_init_flag = false;

bool vision_start = false;

#define MAX_LINE_SPEED 0.1

#include "kffilter.h"

#define HW_MID_ANGLE 43
typedef struct _Car_Status{
	float rotation;
	float wheel_speed;
	float steering_engine_angle;
	float position_x;
	float position_y;
	float relatively_rotaion;
	float line_x_speed;
	float angular_z_speed;
	double time_anchor;
	float dl_count;
	float dw_count;
	unsigned char direction;
	unsigned char anchor_line_dir;
	unsigned char anchor_angle_dir;
	unsigned int walks_count;

}Car_Status;


 typedef struct _raw_status{
	 float vx;         //测量到的线速度
	 float vt;         //测量到的角速度
	 float vx_e;       //理论线速度
	 float vt_e;       //理论角速度
	 float walk_long;  //实际经过的步长
	 ros::Time time_stamp;  //该时刻的实际戳
 }raw_status;

 double sensor_status_time_anchor=0;

 std::vector<raw_status> Status_Recorder;
// std::vector<raw_status> Status_Recorder_Vision;
 void get_sensor_status_by_time(raw_status *ret_status);


 Car_Status G_car_last_status;

 //每一步的距离 1.1 cm
#define WALK_COUNT_UNIT  1.1 

 #define KEY_TOPIC   "/cmd_vel"

 #define RTABMAP_ODOM_TOPIC   "/rtabmap/odom" 
 #define HW_ODOM_TOPIC        "rtabmap/odom_hw"
 #define OUT_ODOM_TOPIC       "rtabmap/odom_hw1"

#define RTABMAP_LASTFRAME_TOPIC       "rtabmap/odom_last_frame"
#define RTABMAP_ERROR_CLOUD_TOPIC       "rtabmap/camera_raw_cloud"

 
#define MAX_ANGLE_VALUE 24*PI_VALUE/180
#define PI_VALUE 3.1415926

// 小车前后轮的距离15.7cm
#define CAR_LONG_CM  15.7
//小车轮子到中心轴的距离8.0cm
#define CAR_HALF_WIDTH_CM 8.0
#define MAX_TRY_TIMES 10
#define CAR_DIR_F 0
#define CAR_DIR_B 1

Car_interface *car_if;

ros::Publisher path_pub;
nav_msgs::Path path;
tf::TransformBroadcaster *pub_broadcaster;

void pub_pose(raw_odom *odom){
	geometry_msgs::PoseStamped this_pose_stamped;
	this_pose_stamped.pose.position.x = odom->p_x;
	this_pose_stamped.pose.position.y = odom->p_y;
	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(odom->yaw);

	this_pose_stamped.pose.orientation.x = goal_quat.x;
	this_pose_stamped.pose.orientation.y = goal_quat.y;
	this_pose_stamped.pose.orientation.z = goal_quat.z;
	this_pose_stamped.pose.orientation.w = goal_quat.w;

	this_pose_stamped.header.stamp=odom->time_stamp;
	path_pub.publish(path);
	this_pose_stamped.header.frame_id="odom";
	path.poses.push_back(this_pose_stamped);

}


void int_car_status(void)
{
	//读取当前的状态 
		int walks_temp =-1;
		int try_times = 0;
		while(walks_temp <0) { //从硬件读轮速计的值
			if(car_if)
				walks_temp = car_if->get_walks();
		
			if(try_times > 0)
			ROS_ERROR("READ ODOM DATA ERROR! %d %d ",try_times,walks_temp);
		
			try_times++;
			if(try_times > MAX_TRY_TIMES)
				break;
		}
		if(walks_temp < 0){
			 walks_temp = 0;
		}
	
	G_car_last_status.rotation = 0;
	G_car_last_status.wheel_speed = 0;
	G_car_last_status.position_x = 10;
	G_car_last_status.position_y = 10;
	G_car_last_status.direction = 0;
	G_car_last_status.walks_count = walks_temp;	
	G_car_last_status.steering_engine_angle = 0;
	G_car_last_status.relatively_rotaion = 0;
	G_car_last_status.time_anchor = -1;
	G_car_last_status.line_x_speed = 0;
	G_car_last_status.angular_z_speed = 0;

	raw_status status_now;
	status_now.vt = 0;
	status_now.vx = 0;
	status_now.vx_e = 0;
	status_now.vt_e = 0;
	status_now.time_stamp = ros::Time::now();
    Status_Recorder.push_back(status_now);	
}
void reconfig_car_odom(float rotation , float position_x, float position_y, int walks)
{
	G_car_last_status.rotation = rotation;
	G_car_last_status.position_x = position_x;
	G_car_last_status.position_y = position_y;
	G_car_last_status.relatively_rotaion = rotation;
	G_car_last_status.walks_count = walks;
}

//更新条件:
//1.距离锚点时间间隔500 ms,也就是5次进入该函数
//锚点更新条件
//1.完成一次速度计算
//2.线速度方向改变
//3.角速度方向改变
unsigned char get_current_line_dir()
{
	return G_car_last_status.direction;
}


unsigned char get_current_angle_dir()
{
	if(G_car_last_status.direction == CAR_DIR_F)
	{
		if(G_car_last_status.steering_engine_angle >= 0)
			return CAR_DIR_F;
		else
			return CAR_DIR_B;
	}

	if(G_car_last_status.direction == CAR_DIR_B)
	{
		if(G_car_last_status.steering_engine_angle >= 0)
			return CAR_DIR_B;
		else
			return CAR_DIR_F;
	}
}

void update_anchor(double now_secs)
{
	G_car_last_status.time_anchor = now_secs;
	G_car_last_status.anchor_line_dir = get_current_line_dir();
	G_car_last_status.anchor_angle_dir = get_current_angle_dir();
	G_car_last_status.dl_count = 0;
	G_car_last_status.dw_count = 0;
}

void update_car_speed(float dl, float dw,double now_secs)
{
	double t_delta = 0;
	t_delta = now_secs - G_car_last_status.time_anchor;		
	if(dl == 0) {
		G_car_last_status.line_x_speed = 0;
	} else {
		if(G_car_last_status.anchor_line_dir)
			G_car_last_status.line_x_speed = -dl/t_delta;
		else			
		G_car_last_status.line_x_speed = dl/t_delta;
	}

	if(dw == 0) {
		G_car_last_status.angular_z_speed = 0;
	}
	else {
		if(G_car_last_status.anchor_angle_dir)
			G_car_last_status.angular_z_speed = -dw/t_delta;
		else
		G_car_last_status.angular_z_speed = dw/t_delta;
	}
	update_anchor(now_secs);
	ROS_ERROR("calculate_odom line_x_speed = %f angular_z_speed=%f \n",G_car_last_status.line_x_speed,G_car_last_status.angular_z_speed);
}
void update_car_position(float x_add , float y_add, float rt_add)
{
	G_car_last_status.position_x += x_add;
	G_car_last_status.position_y += y_add;
	
#ifdef FAKE_IMU
	G_car_last_status.relatively_rotaion += rt_add;
#endif
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

  unsigned int caculate_delta_walks(unsigned int hw_count,unsigned int walks_req)
 {
 	unsigned int dt = 0;
	 if(hw_count >= G_car_last_status.walks_count)
	 	dt = hw_count - G_car_last_status.walks_count;
	 else
	 	dt = 0xff - G_car_last_status.walks_count + hw_count;

	 /*
	 if(dt < walks_req/2 ) {
		 dt = (walks_req/2);
	 }else
	 */
	 
	 if(dt > walks_req * 3)
	 {
			dt = walks_req * 3;
	 }
	 
	// G_car_last_status.walks_count += dt; //update count

	 G_car_last_status.walks_count = hw_count;

	 G_car_last_status.walks_count = G_car_last_status.walks_count % 255;

	 // 当累计误差超过5步时，需要停下来等待
	 // 当累计误差大于-5步时，需要追进？
	 ROS_ERROR("walks_req %d speed %f hw_count %d walks_count %d \n",walks_req,G_car_last_status.wheel_speed,hw_count,G_car_last_status.walks_count);
	 return dt;
 }  
  void pub_odom(float px,float py,float yaw,float vx,float vt,ros::Time time_stamp)
  {
	  nav_msgs::Odometry odom;
	  geometry_msgs::Quaternion goal_quat;
	  goal_quat = tf::createQuaternionMsgFromYaw(yaw);
	  pub_odom_anchor = ros::Time::now();

	  //发布里程计
	  odom.header.stamp = time_stamp; // use corresponding time stamp to image
	  odom.header.frame_id = "odom";
	  odom.child_frame_id = "base_link";
	  odom.pose.pose.position.x = px;
	  odom.pose.pose.position.y = py;
	  odom.pose.pose.position.z = 0;
	  odom.pose.pose.orientation = goal_quat;
	  
	  odom.twist.twist.linear.x = vx;
	  odom.twist.twist.linear.y = 0;
	  odom.twist.twist.linear.z = 0;
	  odom.twist.twist.angular.x = 0;
	  odom.twist.twist.angular.y = 0;
	  odom.twist.twist.angular.z = vt;
	  fusion_odompub.publish(odom);

	//发布tf 变换
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.header.stamp = time_stamp;
	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.translation.z = 0;
	odom_trans.transform.rotation = goal_quat;
	pub_broadcaster->sendTransform(odom_trans);
	//发布轨迹
	geometry_msgs::PoseStamped this_pose_stamped;
	this_pose_stamped.header.frame_id="odom";
	this_pose_stamped.header.stamp=time_stamp;   
	this_pose_stamped.pose.position.x = odom.pose.pose.position.x;
	this_pose_stamped.pose.position.y = odom.pose.pose.position.y;	  
	this_pose_stamped.pose.orientation.x = goal_quat.x;
	this_pose_stamped.pose.orientation.y = goal_quat.y;
	this_pose_stamped.pose.orientation.z = goal_quat.z;
	this_pose_stamped.pose.orientation.w = goal_quat.w;
	path.header.stamp=time_stamp;
	path.header.frame_id="odom";	
	path_pub.publish(path);
	path.poses.push_back(this_pose_stamped);
  }


bool need_update_sensor_status(void)
{
	ros::Time current_time = ros::Time::now();
	raw_status status_last = Status_Recorder.back();
	double delta_time=0;
	delta_time = current_time.toSec() - status_last.time_stamp.toSec();
	if(delta_time >= 0.2)
		return true;
	else
		return false;
}


void read_delta_walks(unsigned int *delta_walks, ros::Time *current_time)
{
	int try_times = 0;
	int walks_count_hw = -1;
	while(walks_count_hw < 0) { //从硬件读轮速计的值
		if(car_if)
			walks_count_hw = car_if->get_walks();
	
		if(try_times > 0)
		ROS_ERROR("read_delta_walks! error ! %d %d ",try_times,walks_count_hw);
	
		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}
	if(walks_count_hw < 0){
		  walks_count_hw = G_car_last_status.walks_count;
	}
	
	 if(try_times > 1) {
		  ROS_ERROR("read_delta_walks has failed %d times! walks_count_hw %d ",try_times,walks_count_hw);
	 }

	 *current_time = ros::Time::now();
	 raw_status status_last = Status_Recorder.back();
	 double delta_time=0;
	 delta_time = current_time->toSec() - status_last.time_stamp.toSec();

	 //根据当前的系统状态，计算出一个期望的值？这样做的效果不清楚，但是应该会避免一些异常
	 float req_walks_f = (100*get_car_speed()/1.1)*delta_time;
	 unsigned int reqwalks_i = fabs(req_walks_f);
	 *delta_walks = caculate_delta_walks(walks_count_hw,reqwalks_i);
}

void record_sensor_status(void)
{
	raw_status status_now;
    raw_status status_last = Status_Recorder.back();	
	ros::Time time_stamp;
	double delta_t = 0;
	unsigned int delta_walks = 0;
	unsigned char dir  = get_car_direction_sw();
	unsigned char angle_dir = get_current_angle_dir();
	float last_steering_engine_angle = get_steering_engine_angle_sw();

	float vx;
	float vt;
	float st_angle = PI_VALUE/2 - fabs(last_steering_engine_angle);
	float req_r = 0.0;
	float R0_t = 0.0;
	float L0 = 0.0; // cm
	float L = 0.0;
	float L_n= 0.0;	
	
    read_delta_walks(&delta_walks , &time_stamp);
	L0 = delta_walks * WALK_COUNT_UNIT; // 单位 cm

	status_now.walk_long = L0; //单位 cm
	status_now.time_stamp = time_stamp;

	
    delta_t = time_stamp.toSec() - status_last.time_stamp.toSec();

	vx = (delta_walks * WALK_COUNT_UNIT) / (delta_t * 100);

	if(dir == CAR_DIR_B)
		vx = -vx;

	//先求半径 
	req_r = sqrt(tan(st_angle)*CAR_LONG_CM*tan(st_angle)*CAR_LONG_CM + CAR_LONG_CM*CAR_LONG_CM/4); //80

	
	//求R0 :R0 = tan(angle)*dc – dw
	R0_t = tan(st_angle)*CAR_LONG_CM - CAR_HALF_WIDTH_CM; //71

	//过滤半径太大的轨迹
	if(R0_t > 10000)
		R0_t = 0;

	//求车心所经过的距离L : L/L0 = req_r/R0
	if(R0_t == 0)
		L = L0;
	else
	    L = (req_r * L0) / R0_t;  //L0 = 558.92   L= 614.12197530864197530864197530864

	//求小车经过的角度
	//周长C = 2PIr
	//走过的角度  N = (L / C)
	if(R0_t == 0)
		L_n = 0;
	else
		L_n = (L/(req_r));	//
	
	vt = L_n / delta_t;
	ROS_ERROR("record_sensor_status st_angle %f L0 %f ln %f dt %f vt %f",st_angle,L,L_n,delta_t,vt);

    if(angle_dir == CAR_DIR_B)
		vt = -vt;

	status_now.vx = vx;
	status_now.vt = vt;

    status_now.vx_e = get_car_speed();
	
    if(R0_t==0)
		status_now.vt_e = 0;
	else
	status_now.vt_e = (status_now.vx_e * delta_t)*100/req_r;

	Status_Recorder.push_back(status_now);
	ROS_ERROR("record_sensor_status vx %f vt %f vx_e %f vt_e %f timestamp %f \n",
		status_now.vx,status_now.vt,
		status_now.vx_e,status_now.vt_e,
		status_now.time_stamp.toSec());
}

void set_linear_speed(float c_speed)
{
	int err = -1;
	int try_times = 0;
	int speed_out = (int)(c_speed*100);
	float last_speed = get_car_speed();
	unsigned char last_dir = get_car_direction_sw();
	unsigned char req_direction = 0;
	unsigned char speed_temp;
	int err_flag =0;

	if(speed_out < 0){
		req_direction = 1;
		speed_out = -speed_out;
	}
	else
		req_direction = 0;
	
	if((last_speed == c_speed) &&(last_dir == req_direction))
	{
		//ROS_ERROR("speed no change!\n");
		return;
	}	
	ROS_ERROR("set_linear_speed %d cm/s  req_direction %d!\n",speed_out,req_direction);
	speed_temp = speed_out;
//	ROS_ERROR("set_linear_speed %d  %d cm/s !\n",speed_out,speed_temp);
	while(err){
		if(car_if)
			err=car_if->set_speed((char)(speed_temp));
		
		if(try_times)
		{
			ROS_ERROR("set speed error! count %d \n",try_times);
			
		}
		try_times++;
		//ROS_ERROR("set speed %d\n",speed_temp);
		if(try_times > MAX_TRY_TIMES)
		break;
	}
	if(err)
		ROS_ERROR("set_speed error!");
	else{
		if(try_times >1)
			ROS_ERROR("set_speed ok at %d times!",try_times);
		
		update_car_speed(c_speed);
	}
	
	err =-1;
	while(err){
		if(car_if)
			err=car_if->set_direction(req_direction);
		
		if(try_times)
		{
			ROS_ERROR("set dir error! count %d \n",try_times);
			
		}
		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}

	if(err)
		ROS_ERROR("set_dir error!");
	else {
		if(try_times >1)
		ROS_ERROR("set dir ok at %d times!",try_times);
	
		update_car_direction(req_direction);
	}
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
	angle_hw = HW_MID_ANGLE - angle_hw; //hw在45度时为0位置. 
	while(err){
	if(car_if)
	err=car_if->set_wheel_angle((char)(angle_hw));
	
	if(try_times)
	{
		ROS_ERROR("set_steering_engine_angle error! count %d \n",try_times);
	}
	
	try_times++;
	if(try_times > MAX_TRY_TIMES)
	break;
	}

	if(err)
		ROS_ERROR("set_wheel_angle error!");
	else{
		if(try_times >1)
		ROS_ERROR("set set_wheel_angle ok at %d times!",try_times);
		
		update_steering_engine_angle_sw(angle);
	}
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
	unsigned char l_dir,a_dir;
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
		angular_st_temp  = PI_VALUE/2 - atan(sqrt(r*r - half_dc_temp*half_dc_temp)/dc_temp);

		//判断舵机转向的方向
		if(linear_speed_temp >= 0){
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

		//舵机角度的限制
		if(angular_st_temp > MAX_ANGLE_VALUE){
			angular_st_temp = MAX_ANGLE_VALUE;
		} else if (angular_st_temp < -MAX_ANGLE_VALUE){
			angular_st_temp = -MAX_ANGLE_VALUE;
		}
	}

	//limit the speed;nor allow speed lower than 0.3 m/s
	if((linear_speed_temp > 0)&&(linear_speed_temp < MAX_LINE_SPEED))
	{
	   linear_speed_temp = MAX_LINE_SPEED;
	}else if((linear_speed_temp < 0)&&(linear_speed_temp > -MAX_LINE_SPEED))
	{
	   linear_speed_temp = -MAX_LINE_SPEED;
	}else if(linear_speed_temp > MAX_LINE_SPEED)
	{
		linear_speed_temp = MAX_LINE_SPEED;
	}else if(linear_speed_temp < -MAX_LINE_SPEED)
	{
		linear_speed_temp = -MAX_LINE_SPEED;
	}

	ROS_ERROR("chatterCallback %f %f %f %f \n",linear_speed_temp,angular_speed_temp,r,angular_st_temp);

	#ifdef USE_CONTRL_FILLTER
		float last_angle = get_steering_engine_angle_sw();
		float last_speed = get_car_speed();	
		if((last_angle!=angular_st_temp) || (last_speed != linear_speed_temp)){
		record_sensor_status(); //更新完状态后再设置新的状态
		set_linear_speed(0);
		set_steering_engine_angle(angular_st_temp);
		ROS_ERROR("sleep start!");
	    ros::Duration(0.2).sleep();
		ROS_ERROR("sleep end!");
		set_linear_speed(linear_speed_temp);	
		}
	#else	
	set_steering_engine_angle(angular_st_temp);
	set_linear_speed(linear_speed_temp);
	#endif
}

#define BAD_DATA 0xffff

void get_sensor_status_by_time(raw_status *ret_status)
{
	double req_time = ret_status->time_stamp.toSec();
	
	unsigned int i =0;
	for(i=Status_Recorder.size()-1;i>0;i--)
	{
		double t_0 = Status_Recorder[i-1].time_stamp.toSec();
		double t_1 = Status_Recorder[i].time_stamp.toSec();
		ROS_ERROR("get_sensor_status_by_time  %f %f %f\n",t_0,t_1,req_time);
		if((req_time>=t_0)&&(req_time<=t_1))
		{
			//计算加速度
			double delta_time = t_1 - t_0;
			double dt_src = req_time - t_0;
			float delta_vx = Status_Recorder[i].vx - Status_Recorder[i-1].vx;
			float delta_vt = Status_Recorder[i].vt - Status_Recorder[i-1].vt;
			
			float delta_vx_e = Status_Recorder[i].vx_e - Status_Recorder[i-1].vx_e;
			float delta_vt_e = Status_Recorder[i].vt_e - Status_Recorder[i-1].vt_e;

			//计算速度
			//(tb-ta)/(tc -ta)*delta_vx +vx_src
			double k = dt_src/delta_time;
			float  vx = k * delta_vx + Status_Recorder[i-1].vx;
			float  vt = k * delta_vt + Status_Recorder[i-1].vt;
			float  vx_e = k * delta_vx + Status_Recorder[i-1].vx_e;
			float  vt_e = k * delta_vt + Status_Recorder[i-1].vt_e;
			float  walk_long = k * Status_Recorder[i].walk_long;
			ret_status->vx = vx;
			ret_status->vt = vt;
			ret_status->vx_e = vx_e;
			ret_status->vt_e = vt_e;
			ret_status->walk_long = walk_long;
			ROS_ERROR("get_sensor_status_by_time: \n vx: %f \n vt: %f \n vx_e: %f \n vt_e : %f walk_long %f \n",
				vx,vt,vx_e,vt_e,walk_long);
			break;
		}
	}
}

//现在有3组数据 理论值，轮速计的值，视觉的值
//视觉里程计正确率概率:
//正确时测到正确值的概率 vison_T_p       0.95
//错误时测到正确值的概率 vison_F_p  0.5
//正确的概率是vison_W               数据丢失/直行/转弯 

//某值正确的概率是？

//正确值的全概率： T_p x W + F_p x (1 - W)

//正确时测到正确值的概率：Tp x W

//该值正确的概率： (tp x w) / (T_p x W + F_p x (1 - W))

//轮速计正确率概率:
//正确时测到正确值的概率 sensor_T_p  0.8  
//错误时测到正确值的概率 sensor_F_p  0.2
//正确的概率是sensor_W   0.9

//(tp x w) / (T_p x W + F_p x (1 - W))


//融合前数据应该是线速度和角速度

//对于视觉里程计，线速度和角速度应该是在该传感器上相对上一次的观测值之间推算出来的值。

//通过线速度和角速度得到半径r

//将视觉和sensor的正确概率归一: exp( vision_p) / [exp( vision_p) + exp( sensor_p)]
//                              exp( sensor_p) / [exp( vision_p) + exp( sensor_p)]

//在有数据丢失情况下视觉里程计的正确率需要降低 interval

#define VISION_P_INC_INTERVAL 0.2
#define VISION_P_REDUCE_INTERVAL 0.3

#define VISION_MAX_T_P 0.98
#define VISION_MAX_P   0.9

float Vison_T_p = 0.95;
float Vison_F_p = 0.05;
float Vison_T   = 0.9;

float Sensor_T_p = 0.4;
float Sensor_F_p = 0.2;
float Sensor_T   = 0.3;

float E_T_p = 0.4;
float E_F_p = 0.2;
float E_T   = 0.3;

void calculate_odom_and_pub(float vx,float vt,float dt,ros::Time pub_time_stamp)
{
	float last_rotation = get_car_rotation();;
	float last_x = G_car_last_status.position_x;
	float last_y = G_car_last_status.position_y;
	float L = vx * dt;
	float W = vt * dt;
	float req_r = 0;
	float x = 0;
	float y =0;
	if(W == 0){
	   x = L;
	   y = 0;
	}else{
		 req_r = vx/vt;
		 x =  sin(W) * req_r;
		 y = req_r - cos(W) * req_r;
	}
	
	//坐标系旋转 一个导航角后再将增量累加上去。
	float raw_x = x*cos(last_rotation) + y*sin(last_rotation);
	float raw_y = x*sin(last_rotation)- y*cos(last_rotation);
    update_car_position(raw_x , raw_y, W);
	ROS_ERROR("calculate_odom_and_pub position_x %f position_y %f rotaion:%f vx %f vt %f dt %f\n",
		G_car_last_status.position_x,
			G_car_last_status.position_y,
			G_car_last_status.relatively_rotaion,
			vx,
			vt,
			dt);	
	pub_odom(G_car_last_status.position_x,G_car_last_status.position_y,
		G_car_last_status.relatively_rotaion,vx,vt,pub_time_stamp);
}

#define VX_Q_SENSOR 0.15
#define VT_Q_SENSOR 0.4

void fusion_data(raw_status vision_status)
{
	static double last_update_time=0;
	float vx = 0;
	float vt = 0;
	//0.如果vision_status 是无效的，则仅根据sensor 的最新状态来更新，
	double delta_time = vision_status.time_stamp.toSec() - last_update_time;
	raw_status sensor_status;//= Status_Recorder.back();
	//如果vo 的时间大于当前sensor status 的时间，则更新一次sensor  时间.
	if(vision_status.time_stamp.toSec() > sensor_status.time_stamp.toSec())
		record_sensor_status();

	if(vision_status.vx != BAD_DATA)
	{
		//为了避免回跳，需要过滤时间戳小于上次更新时间戳的调用...
		if(delta_time > 0)
		{
		//	raw_status sensor_status;
			sensor_status.time_stamp = vision_status.time_stamp;
			//1.找到vision_status 时刻的sensor record 中的系统状态.
			get_sensor_status_by_time(&sensor_status);
			//根据理论速度和轮速计的值来评估vision的vx 和vt 是否在合理范围内.
			float vx_bq = (vision_status.vx - sensor_status.vx)*(vision_status.vx - sensor_status.vx);
			float vt_bq = (vision_status.vt - sensor_status.vt)*(vision_status.vt - sensor_status.vt);
			float vt_bq_2 = (vision_status.vt + sensor_status.vt)*(vision_status.vt + sensor_status.vt);
			if((vx_bq > VX_Q_SENSOR)||(vt_bq > VT_Q_SENSOR))
			{
				if((vt_bq_2 < VT_Q_SENSOR)&&(Vison_T_p > 0.9)){
					ROS_ERROR("fusion_data vision data has  Negative Dir,we belive it when  Vison_T_p is good!!vx %f vt%f \n",vision_status.vx,vision_status.vt);	
				}else{
					ROS_ERROR("fusion_data vision data is error!vx %f vt%f \n",vision_status.vx,vision_status.vt);
					vision_status.vx = sensor_status.vx;
					vision_status.vt = sensor_status.vt;
				}
			}
			
			//2.根据置信度来估算当时的系统状态
			float vision_kp = (Vison_T_p * Vison_T) / (Vison_T_p * Vison_T + Vison_F_p * (1 - Vison_T));
			float sensor_kp = (Sensor_T_p * Sensor_T) / (Sensor_T_p * Sensor_T + Sensor_F_p * (1 - Sensor_T));
			float sensor_ep = (E_T_p * E_T) / (E_T_p * E_T + E_F_p * (1 - E_T));

			//概率归一化
				float full_p = exp(vision_kp) +  exp(sensor_kp) + exp(sensor_ep);
				vision_kp  = exp(vision_kp)/full_p;
				sensor_kp  = exp(sensor_kp)/full_p;
			    sensor_ep  = exp(sensor_ep)/full_p;
			//3.根据概率更新状态
			  vx = vision_kp * vision_status.vx + sensor_kp*sensor_status.vx + sensor_ep*sensor_status.vx_e;
			  vt = vision_kp * vision_status.vt + sensor_kp*sensor_status.vt + sensor_ep*sensor_status.vt_e;
			  calculate_odom_and_pub(vx,vt,delta_time,vision_status.time_stamp);			  
			  last_update_time = vision_status.time_stamp.toSec();
			  ROS_ERROR("fusion_data with vo vision_kp %f sensor_kp %f sensor_ep:%f \n",vision_kp,
			sensor_kp,sensor_ep);	  
		}
		//4.更新视觉里程计的概率分布
		Vison_T_p += VISION_P_INC_INTERVAL;
		Vison_T += VISION_P_INC_INTERVAL;
		if(Vison_T_p > VISION_MAX_T_P)
			Vison_T_p = VISION_MAX_T_P;

		if(Vison_T > VISION_MAX_P)
			Vison_T = VISION_MAX_P;
		
		ROS_ERROR("fusion_data with vo vx %f vt %f delta_time:%f Vison_T_p %f Vison_T %f \n",vx,
			vt,delta_time,Vison_T_p,Vison_T);
	} else {
		//4.还是从vision的时间戳处继续更新.
		sensor_status.time_stamp = vision_status.time_stamp;
		//5.找到vision_status 时刻的sensor record 中的系统状态.
		get_sensor_status_by_time(&sensor_status);
		float sensor_kp = (Sensor_T_p * Sensor_T) / (Vison_T_p * Sensor_T + Sensor_F_p * (1 - Sensor_T));
		float sensor_ep = (E_T_p * E_T) / (E_T_p * E_T + E_F_p * (1 - E_T));
		float full_p = exp(sensor_kp) + exp(sensor_ep);
		sensor_kp  = exp(sensor_kp)/full_p;
		sensor_ep  = exp(sensor_ep)/full_p;
		//3.根据概率更新状态
		vx =sensor_kp*sensor_status.vx + sensor_ep*sensor_status.vx_e;
		vt =sensor_kp*sensor_status.vt + sensor_ep*sensor_status.vt_e;
		calculate_odom_and_pub(vx,vt,delta_time,sensor_status.time_stamp);
		last_update_time = vision_status.time_stamp.toSec();


		//4.更新视觉里程计的概率分布
		Vison_T_p -= VISION_P_INC_INTERVAL;
		Vison_T -= VISION_P_INC_INTERVAL;
		if(Vison_T_p < 0.3)
			Vison_T_p = 0.3;

		if(Vison_T < 0.2)
			Vison_T = 0.2;
		
		ROS_ERROR("fusion_data no vo vx %f vt %f delta_time:%f Vison_T_p %f Vison_T %f \n",vx,
			vt,delta_time,Vison_T_p,Vison_T);
	}
}

void rtabmap_odom_chatterCallback(const nav_msgs::Odometry& rtabmap_odom)
{
	//首先需要解状态
	double roll, pitch, yaw;
	raw_status current_status;
	float vx = 0;
	float vt = 0;
	float current_yaw  = 0;
	float current_x    = 0;
	float current_y    = 0;
	double current_time = 0;

    tf::Quaternion quat;
	tf::quaternionMsgToTF(rtabmap_odom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);	
	current_yaw   =  yaw;
	current_x		 =	rtabmap_odom.pose.pose.position.x; 
	current_y		 =	rtabmap_odom.pose.pose.position.y;	
	current_time  =  rtabmap_odom.header.stamp.toSec();		
#if 0
	//根据x,y,yaw的offset，来计算半径,弧长 ，再计算出 vx 和 vt
	//角速度就等于 delta_rotation / dt;
	static float last_yaw  = 0;
	static float last_x	  =0; 
	static float last_y	  = 0;  
	static double last_time = 0;
	float delta_time   = 0;
	float delta_yaw    = 0;
	float delta_x	  = 0; 
	float delta_y	  = 0; 
	float x 		  = 0; 
	float req_r 	  = 0;

#endif	
	if((current_x != 0) && (current_y != 0 )&& isnormal(current_x)&&isnormal(current_y)&&isnormal(current_yaw)
		&& isnormal(rtabmap_odom.twist.twist.linear.x) && isnormal(rtabmap_odom.twist.twist.angular.z))
	{	
#if 0
		delta_time     = current_time - last_time;
		delta_yaw      = current_yaw - last_yaw;
		delta_x        = current_x - last_x;
		delta_y        = current_y - last_y;
		x             = delta_x * cos(last_yaw) + delta_y*sin(last_yaw);
		req_r         = x / sin(delta_yaw);

		last_yaw = current_yaw;
		last_x = current_x;
		last_y = current_y;
		last_time = current_time;

		vt = delta_yaw/delta_time;
		vx = req_r * vt;

#else
		vt = rtabmap_odom.twist.twist.angular.z;
		vx = rtabmap_odom.twist.twist.linear.x;
#endif
		ROS_ERROR("vx %f vt %f l_speed %f  w_speed %f current_time:%f \n",vx,vt,rtabmap_odom.twist.twist.linear.x,
		rtabmap_odom.twist.twist.angular.z,current_time);
		vision_start = true;
	}else{
		//丢弃错误数据的方式就是不更新last 的值.	
		vx = BAD_DATA;
		vt = BAD_DATA;
	}
	current_status.time_stamp = rtabmap_odom.header.stamp;
	current_status.vx = vx;
	current_status.vt = vt;
	fusion_data(current_status);
}

//#define ONLY_SENSOR
int main(int argc, char **argv)
{
	ros::init(argc, argv, "minicar");
	ros::NodeHandle n;
	bool Mapping_Mode = false;
	 //该topic 发布融合后的odom
	fusion_odompub = n.advertise<nav_msgs::Odometry>(OUT_ODOM_TOPIC, 1);
	// 该广播发布tf数据
	tf::TransformBroadcaster broadcaster;	// 发布tf数据
	pub_broadcaster = &broadcaster;

	//记录和发布航迹
	path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true); //记录path
//	path.header.stamp=current_time;
//	path.header.frame_id="odom";

    //解析参数，初始化car_if
	std::string port_path;

	n.getParam("/minicar/port_path", port_path);

	n.getParam("/minicar/port_path", port_path);

	
	car_if = new Car_interface(port_path.c_str());
	set_steering_engine_angle(1);
	set_steering_engine_angle(0);
	//状态初始化
	int_car_status();
 //订阅cmd_vel
  ros::Subscriber sub = n.subscribe(KEY_TOPIC, 3, chatterCallback);
 //订阅vision odom数据
 //if(!Mapping_Mode)
  ros::Subscriber sub_odom = n.subscribe(RTABMAP_ODOM_TOPIC, 1, rtabmap_odom_chatterCallback);
 
  ros::Rate loop_rate(10);
  float rotation=0;
  while (ros::ok())
  {
    //是否更新轮速计状态
	if(need_update_sensor_status()){
//#ifdef ONLY_SENSOR
	  if(!vision_start){
		raw_status status_now;
		raw_status status_last = Status_Recorder.back();
		nav_msgs::Odometry odom;
		odom.header.stamp = status_last.time_stamp; // use corresponding time stamp to image
		odom.pose.pose.position.x = 0;
		odom.pose.pose.position.y = 0;
		rtabmap_odom_chatterCallback(odom);
	}
//#endif
//	record_sensor_status();
	}


  	ros::spinOnce();
	loop_rate.sleep();
  }
  set_linear_speed(0);
  return 0;
}

