#ifndef COMMAND_H
#define COMMAND_H
#include "core.h"

#define MAX_CMD_LIST 255
#define CMD_LEN     			  4
#define CMD_HEAD               	0xaa
#define CMD_END					0xbb

#define CMD_ACK_HEAD               	0xae
#define CMD_ACK_END               	0xea

#define CMD_DATA_HEAD               0x35
#define CMD_DATA_END               	0x53

//(3 * sizeof(int) + 2)
#define MAX_OUT_BUF_LEN  8
enum{
	CMD_HEAD_INDEX=0,
	CMD_ACK_HEAD_INDEX = CMD_HEAD_INDEX,
	CMD_TYPE_INDEX,
	CMD_ACK_TYPE_INDEX = CMD_TYPE_INDEX,
	CMD_VALUE_INDEX,
	CMD_ACK_RET_VALUE_INDEX = CMD_VALUE_INDEX,
	CMD_AZIMUTH_VALUE_L = CMD_VALUE_INDEX,
	CMD_END_INDEX,
	CMD_ACK_END_INDEX = CMD_END_INDEX,
	CMD_AZIMUTH_VALUE_H = CMD_END_INDEX,
	CMD_MAX_READ_INDEX,
};


#define CMD_SPEED               0x41
#define CMD_ANGLE               0x42
#define CMD_DIR               	0x43	
#define CMD_AZIMUTH             0x44   /*HEAD TYPE VALUE[L] VALUE[H] END*/
#define CMD_ODOM                0X45 
#define CMD_ODOM_RAW            0x46
#define CMD_IMU_CAPRATION       0x47
#define CMD_SIMPLE_READ_ALL            0x48
#define CMD_IMU_CAPRATION_STATUS       0x49

#define CMD_SIMPLE_READ_WALK            0x50


#define CMD_ERROR_VALUE 0x1
#define CMD_ERROR_TYPE  0x2

struct Ack_Cmd{
	char head;
	char type;
	char value;
	char end;
};


typedef int (*cmd_call_back) (unsigned char cmd_value);
typedef int (*send_func) (unsigned char *send_buf,int len);
typedef void (*recv_func) (unsigned char in_data);
typedef int (*hw_init_func) (void);


typedef struct Cmd_Element{
		char cmd;
		char need_ack;
		cmd_call_back cmd_cb;
	}Cmd_Elem;

typedef struct Hw_Ptr{
		send_func send_f;
		recv_func recv_f;
	}hw_func_ptr;

typedef struct Out_Buf{
		unsigned char *buf;
		int len;
	}out_buf;

extern int command_init(void);
extern  int set_speed(unsigned char speed_n);
extern int set_direction(unsigned char dir);
extern int read_azimuth(unsigned char n_need);
extern  int set_angle(unsigned char angle_n);
extern  int read_odom_raw(unsigned char n_need);
extern int simple_read_all(unsigned char n_need);
extern void capration_ret(void);
extern int simple_read_walk(unsigned char n_need);
#endif

