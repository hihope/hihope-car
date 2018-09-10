#ifndef UART_H  
#define UART_H 
//串口相关的头文件    
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
#include "debug.h"

#define FALSE  -1    
#define TRUE   0    
#define OPENDED 1
#define CLOSED 0
using namespace std;  
class Uart
{  
private:
	int Speed;
	char Dev_path[255];
	int Fd;
	int Dev_status;
	int Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
	int Init(int speed);
public: 
	Uart(int speed_,const char* dev_path_);
	int Open();
	void Close();
	int Recv(char *rcv_buf,int data_len);
	int Send(char *send_buf,int data_len);
	void Flush(void);
}; 
#endif
