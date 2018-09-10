#ifndef DEBUG_H  
#define DEBUG_H
#include "ros/ros.h"
//#define DEBUG_LEVEL_ERR   1
//#define DEBUG_LEVEL_INFO   0

//#define ROS_ERROR(format, ...) do { \
	if(DEBUG_LEVEL_ERR) \
		printf ("Error:" format "\n", ##__VA_ARGS__); \
		} while(0)

//#define ROS_INFO(format, ...) do{ \
	if(DEBUG_LEVEL_INFO) \
		printf ("Info:" format "\n", ##__VA_ARGS__); \
		} while(0)

#endif


