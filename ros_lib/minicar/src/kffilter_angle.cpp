#include "ros/ros.h"
#include "kffilter.h"
extern void pub_pose(raw_odom *odom);
float xhatminus,Pminus,Kg;
float Q= 0.001;
float P = 1.0;
float R = 0.05;
float xhat = 0.0;

void init_kffilter(float x0)
{
 xhat = x0;
}

float angle_filter(float angle,float dt_angle)
{
	float diff = 0;
	xhatminus = xhat + dt_angle;
	Pminus = P + Q;
	Kg = Pminus/(Pminus + R);
 
	diff = (angle - xhatminus) + CV_PI;
	if(diff < 0 )
		diff = diff + CV_PI;
	else if (diff > 2*CV_PI )
	    diff = diff - 3* CV_PI;
	else
	    diff = diff - CV_PI;

	xhat = xhatminus + Kg*diff;

	if(xhat >= 2*CV_PI)
    	xhat = xhat - (2*CV_PI);
	if(xhat <= -2*CV_PI)
		xhat = xhat + 2*CV_PI;
	
	P = (1-Kg)*Pminus;
	return xhat;
}



float xhatminus_vx,Pminus_vx,Kg_vx;
float Q_vx= 0.01;
float P_vx = 1.0;
float R_vx = 0.05;
float xhat_vx = 0.0;

void init_kffilter_vx(float x0)
{
 xhat_vx = x0;
}

float vx_filter(float speed)
{
	xhatminus_vx = xhat_vx;
	Pminus_vx = P_vx + Q_vx;
	Kg_vx = Pminus_vx/(Pminus_vx + R_vx);
	xhat_vx = xhatminus_vx + Kg_vx*(speed - xhatminus_vx);
	P_vx = (1-Kg_vx)*Pminus_vx;
	return xhat_vx;
}


float xhatminus_vt,Pminus_vt,Kg_vt;
float Q_vt= 0.01;
float P_vt = 1.0;
float R_vt = 0.05;
float xhat_vt = 0.0;

void init_kffilter_vt(float x0)
{
 xhat_vt = x0;
}

float vt_filter(float speed)
{
	xhatminus_vt = xhat_vt;
	Pminus_vt = P_vt + Q_vt;
	Kg_vt = Pminus_vt/(Pminus_vt + R_vt);
	xhat_vt = xhatminus_vt + Kg_vt*(speed - xhatminus_vt);
	P_vt = (1-Kg_vt)*Pminus_vt;
	return xhat_vt;
}


//input:当前pose(x,y,yaw), 当前vx,vt,timestamp
//output:最优pose(x,y,yaw),vx,vt,timestamp
//计算过程:对 vx 和vt 做kf?
//x1 = x0 + delta_x
//y1 = y0 + delta_y
//yaw1 = yaw0 + delta_r


float xhatminus_px,xhatminus_py,xhatminus_pyaw;
float Pminus_px,Kg_px;
float Pminus_py,Kg_py;
float Pminus_pyaw,Kg_pyaw;

float Q_px= 0.01;
float P_px = 1.0;
float R_px = 0.05;
float xhat_px = 0.0;


float Q_py= 0.01;
float P_py = 1.0;
float R_py = 0.05;
float xhat_py = 0.0;


float Q_pyaw= 0.5;
float P_pyaw = 1.0;
float R_pyaw = 0.05;
float xhat_pyaw = 0.0;
ros::Time last_time_stamp;

void init_kffilter_odom(float px,float py,float pyaw,float vx,float vt,ros::Time timestamp)
{
 	xhat_px = px;
	xhat_py = py;
	xhat_pyaw = pyaw;
	last_time_stamp = timestamp;
	init_kffilter_vt(vt);
	init_kffilter_vt(vx);
}


void kf_odom(raw_odom *odom)
{
	float vx,vt;
	vx = vx_filter(odom->vx);
	vt = vt_filter(odom->vt);
	double dt = (odom->time_stamp.toSec() - last_time_stamp.toSec());
	last_time_stamp = odom->time_stamp;
	float delta_yaw = fabs(dt * vt);
	float delta_x	= fabs(dt * vx);
	float req_r = vx/vt;


//	ROS_ERROR("kf_odom vt %f vx %f dt %f delta_yaw %f delta_x %f \n",vt,vx,dt,delta_yaw,delta_x);

	if(vt == 0)
		req_r = 0;

	float	x =  sin(delta_yaw) * req_r;
	float	y = req_r - cos(delta_yaw) * req_r;
	float raw_x = 0.0;
	float raw_y = 0.0;

	float diff=0;

	if(fabs(delta_yaw) <= 0.00000001){
	 raw_x = delta_x;
	 raw_y = 0;
	} else {
		raw_x = x*cos(xhat_pyaw) + y*sin(xhat_pyaw);
		raw_y = x*sin(xhat_pyaw)- y*cos(xhat_pyaw);
	}

	if(dt < 0){
		raw_x = -raw_x;
		raw_y = -raw_y;
		delta_yaw = -delta_yaw;
		ROS_ERROR("DT IS ERROR!!");
	}
	
	if(vx >= 0){
	 xhatminus_px = xhat_px + raw_x;
	 xhatminus_py = xhat_py + raw_y;
	}else{
	 xhatminus_px = xhat_px - raw_x;
	 xhatminus_py = xhat_py - raw_y;
	}

	if(vt >= 0){
	 xhatminus_pyaw = xhat_pyaw + delta_yaw;
	}else{
	 xhatminus_pyaw = xhat_pyaw - delta_yaw;
	}

	Pminus_px = P_px + Q_px;
	Pminus_py = P_py + Q_py;
	Pminus_pyaw = P_pyaw + Q_pyaw;

	Kg_px = Pminus_px/(Pminus_px + R_px);
	Kg_py = Pminus_py/(Pminus_py + R_py);
	Kg_pyaw = Pminus_pyaw/(Pminus_pyaw + R_pyaw);

	xhat_px = xhatminus_px + Kg_px*(odom->p_x - xhatminus_px);
	xhat_py = xhatminus_py + Kg_py*(odom->p_y - xhatminus_py);


	diff = (odom->yaw - xhatminus_pyaw) + CV_PI;
	if(diff < 0 )
		diff = diff + CV_PI;
	else if (diff > 2*CV_PI )
	    diff = diff - 3* CV_PI;
	else
	    diff = diff - CV_PI;

	xhat_pyaw = xhatminus_pyaw + Kg_pyaw*(diff);


	if(xhat >= 2*CV_PI)
    	xhat = xhat - (2*CV_PI);
	if(xhat <= -2*CV_PI)
		xhat = xhat + 2*CV_PI;


	P_px = (1-Kg_px)*Pminus_px;
	P_py = (1-Kg_py)*Pminus_py;
	P_pyaw = (1-Kg_pyaw)*Pminus_pyaw;

	odom->p_x = xhat_px;
	odom->p_y = xhat_py;	
	odom->yaw = xhat_pyaw;
	odom->vx = vx;
	odom->vt = vt;
	pub_pose(odom);
}




