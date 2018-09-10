/*
CC2540 Common  header
ghostyu
20130204
*/

/*
为CC2540EM所有基础实验提供共同使用的宏定义或者函数声明。
*/
#ifndef _COMMON_H_
#define _COMMON_H_

typedef signed   char   int8;
typedef unsigned char   uint8;

typedef signed   short  int16;
typedef unsigned short  uint16;

typedef signed   long   int32;
typedef unsigned long   uint32;

typedef unsigned char   bool;

typedef unsigned char   halDataAlign_t;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif


/*极性设置，即高有效还是低有效*/
#define ACTIVE_LOW  !
#define ACTIVE_HIGH  !!
/*位操作*/
#define BV(n) (1<<(n))
/*常规宏定义，即x可包含多条语句*/
#define st(x)      do { x } while (__LINE__ == -1)

void HalHW_WaitUs(uint16 microSecs);
void HalHW_WaitMS(uint16 ms);
int HAL_BOARD_INIT(void);

#endif