//
#ifndef CORE_H
#define CORE_H
#include "stdio.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_EVENT 255
#define NULL 0

  
#define  printf(format, ...)  
enum{
	TEST_EVENT1,
	TEST_EVENT2,
	REGISTER_CMD_EVENT,
	REGISTER_COMM_HW_EVENT,
	REGISTER_FAKE_UART_DEBUG_EVENT,
	REGISTER_CTL_HW_EVENT,
	SPEED_FEEDBACK_EVENT,
    SEND_DATA_EVENT,
    DEBUG_LOG_EVENT,
};

typedef signed   char   int8;
typedef unsigned char   uint8;

typedef signed   short  int16;
typedef unsigned short  uint16;

typedef signed   long   int32;
typedef unsigned long   uint32;

typedef unsigned char   bool;

typedef int (*init_func) (void);
typedef int (*event_func) (void* data);
typedef void (*event_cb) (void* data);
typedef struct Event_Element {
	int event;
	event_func func;
	} Event_Elem;

typedef struct Event_Public {
	int event;
	void* data;
	event_cb cb;
	} Event_Pub;


extern int register_event(Event_Elem *event);
extern int pub_event(Event_Pub *event);
extern int unpub_event(Event_Pub *event);
extern int Run_Init(void);
#endif


