#ifndef KF_FILTER_H
#define KF_FILTER_H
#define CV_PI 3.14
typedef struct _raw_odom{
	float p_x;
	float p_y;
	float yaw;
	float vx;
	float vt;
	ros::Time time_stamp;
}raw_odom;

extern void init_kffilter(float x0);
extern float angle_filter(float angle,float dt_angle);
extern void init_kffilter_odom(float px,float py,float pyaw,float vx,float vt,ros::Time timestamp);
extern void kf_odom(raw_odom *odom);
#endif

