#ifndef UART_H
#define UART_H
#include "command.h"

typedef struct Log_Data{
	unsigned char *buf;
	int len;
}log_data;
#define MAX_LOG_BUF_LEN 255

extern int InitUart(void);
extern int UartSendString(unsigned char *Data, int len);
extern void recv_cmd(unsigned char cmd_in);
#endif

