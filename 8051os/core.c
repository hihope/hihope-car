#include "core.h"
#include "command.h"
#include "uart.h"
#include "contrl.h"
#include "ctl_hw.h"
#include "Common.h"
Event_Elem *E_events[MAX_EVENT];
Event_Pub *P_events[MAX_EVENT];
int init_event(void);

init_func init_table[] = {
   HAL_BOARD_INIT,
   init_event,
   command_init,
   InitUart,
   init_contrl,
   ctl_hw_init,
};

int init_event(void)
{
	int i = 0;
	for(i=0;i<MAX_EVENT;i++)
	{
		E_events[i]=NULL;
		P_events[i]=NULL;
	}
	return 0;
}

int register_event(Event_Elem *event){
	int i = 0;
	for(i=0;i<MAX_EVENT;i++)
	{
		if(E_events[i]==NULL) {
			printf("reg_event %d \n",i);
			E_events[i] = event;
			break;
		}
	}
	if(i == MAX_EVENT)
		return -1;
	return 0;
}

int pub_event(Event_Pub *event){
	int i = 0;
	for(i=0;i<MAX_EVENT;i++)
	{
		if(P_events[i]==NULL) {
			printf("pub_event %d \n",i);
			P_events[i] = event;
			break;
		}
	}
	if(i == MAX_EVENT)
		return -1;
	return 0;
}


int unpub_event(Event_Pub *event){
	int i = 0;
	for(i=0;i<MAX_EVENT;i++)
	{
		if(P_events[i]==event) {
			P_events[i] = NULL;
			break;
		}
	}
	if(i == MAX_EVENT)
		return -1;
	return 0;
}

void pop_event(Event_Pub ** ret) {
	int i =0;
	*ret = P_events[0];
	for(i=0;i<(MAX_EVENT-1);i++)
	{
		P_events[i] = P_events[i+1];
	}
}

int run_event(Event_Pub * event_p){
	int i =0;	
	for(i = 0;i<MAX_EVENT;i++)
	{
		if(E_events[i]!=NULL)
		{
			if(E_events[i]->event == event_p->event){
				printf("run_event find event %d \n",event_p->event);
				E_events[i]->func(event_p->data);
				if(event_p->cb)
					event_p->cb(event_p->data);
			}
		}
	}
	return 0;
}

int run_all_event(void)
{
	Event_Pub *ev;
	pop_event(&ev);
	while(ev!=NULL)
	{
		run_event(ev);
		pop_event(&ev);
	}
	return 0;
}

int Run_Init(void){
	int i =0;
	for(i=0;i<sizeof(init_table)/sizeof(init_func);i++)
	{
		if(init_table[i]())
			return -1;
	}
	return 0;
}


