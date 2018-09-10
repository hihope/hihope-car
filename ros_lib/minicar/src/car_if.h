#ifndef CAR_IF_H  
#define CAR_IF_H
#include<stdio.h>      /*标准输入输出定义*/    
#include<stdlib.h>     /*标准函数库定义*/    
#include<unistd.h>     /*Unix 标准函数定义*/    
#include<sys/types.h>     
#include<sys/stat.h>       
#include<fcntl.h>      /*文件控制定义*/    
#include<termios.h>    /*PPSIX 终端控制定义*/    
#include<errno.h>      /*错误号定义*/    
#include<string.h>
#include <sstream>
#include "uart.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include "debug.h"

#define FALSE  -1    
#define TRUE   0    
#define OPENDED 1
#define CLOSED 0
using namespace std;  


#define CMD_LEN     			  4
#define CMD_HEAD               	0xaa
#define CMD_SPEED               0x41
#define CMD_ANGLE               0x42
#define CMD_DIR               	0x43

#define CMD_AZIMUTH             0x44   /*HEAD TYPE VALUE[L] VALUE[H] END*/

#define CMD_ODOM             0x45   /*HEAD TYPE VALUE[L] VALUE[H] END*/
#define CMD_ODOM_RAW             0x46   /*HEAD TYPE VALUE[L] VALUE[H] END*/
#define CMD_IMU_CAPRATION             0x47
#define CMD_SIMPLE_READ             0X48
#define CMD_CAPRATION_STATUS        0X49
#define CMD_SIMPLE_READ_WALK        0X50

#define CMD_END					0xbb

#define CMD_ACK_HEAD             0xae
#define CMD_ACK_END               	0xea

#define CMD_DATA_HEAD               0x35
#define CMD_DATA_END               	0x53

#define QUEUEBUF_SNED_CMD   0x11
#define QUEUEBUF_READI_CMD   0x22
#define QUEUEBUF_READM_CMD   0x33



enum{
	CMD_HEAD_INDEX=0,
	CMD_TYPE_INDEX,
	CMD_VALUE_INDEX,
	CMD_END_INDEX,
	CMD_MAX_INDEX,
};

struct Car_Cmd {
	char cmd_head;
	char cmd_type;
	char cmd_value;
	char cmd_end;
};


struct Cmd_queue_buf {
	char buf_type;
	void* out_buf;
	void* in_buf;
	std::condition_variable *convar;
	int ret;
};


struct Ack_Cmd{
	char ack_head;
	char ack_type;
	char ack_value;
	char ack_end;
};

  struct Read_Int_Data{
	 char head;
	 char type;
	 char value_l;
	  char value_h;
	 char end;
 };
 
  struct Read_Multi_Data{
	char head;
	char type;
	char value0_l;
	char value0_h;
	char value1_l;
	char value1_h;
	char end;
 };
 
 typedef struct Odom_Data{
	int x;
	int y;
}odom_d;

class Car_interface
{ 
typedef std::lock_guard<mutex>			  MutexLockGuard;
typedef std::unique_lock<mutex>           MutexUniqueLock;

private:
	int Speed;
	int Angle;
	Uart *G_Uport;
	std::mutex Transfer_Mutex;
	std::mutex Loop_Mutex;
        std::mutex QCmd_lock;
	int check_ack(char cmd);
	unsigned int  readi_and_check_data(struct Read_Int_Data *data_buf,char cmd);
	unsigned int  readm_and_check_data(struct Read_Multi_Data *data_buf,char cmd);
	unsigned int  read2c_and_check_data(struct Read_Int_Data *data_buf,char cmd);

public: 
	Car_interface(const char* ser_dev);
	~Car_interface(void);
	int set_speed(char speed);
	int set_wheel_angle(char angle);
	int set_direction(char dir);
	int get_rotation_data(float * ret_p);
	int get_odom_data(odom_d *data);
	int get_walk_count_data(unsigned int *walk_count);
	int simple_read(unsigned char * out_buf);
	int capration(void);
	char capration_status_read(void);
    int get_walks(void);
	int readc_and_check_data(char cmd);
//	int get_position_data(void);
//	int set_position(int x,int y,int z);
}; 
#endif
