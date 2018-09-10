#ifndef CONTRL_H
#define CONTRL_H

#define FULL_PWM_VALUE  0X271
#define PWM_SPEED_RANGE 255
#define PWM_ANGLE_RANGE 180
#define PWM_ANGLE_BASE 0x7d

#define FULL_SPEED_PWM_VALUE  0X1388

#define SPEED_RANGE_BIG 20 
#define SPEED_RANGE_MID 10 
#define SPEED_RANGE_SMALL 5 
#define SPEED_RANGE_SSMALL 2

#define BIG_JUMP_VALUE   1000
#define MID_JUMP_VALUE   300
#define SMALL_JUMP_VALUE   50
#define SSMALL_JUMP_VALUE   6

typedef unsigned int (*get_angle_func)(void) ;
typedef int (*set_dir_func)(char dir)  ;
typedef unsigned int(*get_speed_func)(void)  ;
typedef  int(*get_azimuth_func)(void)  ;
typedef  void(*walk_count_func)(void)  ;


typedef struct Contrl_Hw_Ptr{
	get_angle_func get_angle;
	get_speed_func get_speed;
	walk_count_func walk_count;
	set_dir_func set_dir;
    get_azimuth_func get_azimuth;
}Contrl_hw;

extern char capration_flag;

extern int init_contrl(void);
extern void capration_imu_trigger(void);
extern int hw_set_direction(unsigned char dir);
extern int send_data_cb(void *data);
extern void send_ack(char cmd,char ret);


#endif
