#ifndef CTL_HW_H
#define CTL_HW_H
#include "contrl.h"
#include "core.h"
#define BV(x) (1<<(x))
extern uint16 speed_pwm;
extern char capration_flag;
#define SH_CP                 P1_4
#define ST_CP                 P1_3
#define DATA_P                P0_0
#define N_OE                  P0_1
#define CODE_DIR_F   0x6
#define CODE_DIR_B   0x18
#define ROTATION_F  0x14
#define ROTATION_B  0xA
extern uint16 Gspeed_Real;
extern uint16 G_angle;
extern uint16 speed_pwm;

extern  void walk_count(void);
extern  void caculate_speed_pwm(void);
extern int ctl_hw_init(void);
extern void capration_imu_op(void);
#endif
