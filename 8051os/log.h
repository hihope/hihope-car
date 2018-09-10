#ifndef LOG_H
#define LOG_H
#include "uart.h"
#include "core.h"

extern  int get_log_len(void);
extern unsigned char log_buf[MAX_LOG_BUF_LEN];
#define Log(format , ...)  do{       \
  sprintf(log_buf,format "\n", ##__VA_ARGS__); \
  UartSendString(log_buf,get_log_len()); \
}while(0)
  
#endif