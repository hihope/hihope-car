/*
CC2540 Common Source
ghostyu
20130204
*/

/*
为CC2540EM所有基础实验提供共同使用的函数。
*/
#include<ioCC2540.h>
#include"Common.h"
/**************************************************************************************************
软件延时
这里要注意了，若使用uint32类型，时间将延长5倍！！！
如
void HalHW_WaitUs(uint32 microSecs)
 **************************************************************************************************/
void HalHW_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 3 NOPs == 1 usecs */
        asm("NOP");
        asm("NOP");
        asm("NOP");

  }
}

void HalHW_WaitMS(uint16 ms)
{
  while(ms--){
    int i=0;
    for(i=20; i>0; i--)
      HalHW_WaitUs(50);
  }
}
/*晶振初始化*/
int HAL_BOARD_INIT(void)
{
  CLKCONCMD &= ~(BV(6));//使能32M 外部晶振
  while(!(SLEEPSTA & (BV(6))));  //等待晶振设置完成，SLEEPSTA是CLKCONCMD的一个镜像
  CLKCONCMD &= ~(BV(5)|BV(4)|BV(3)|BV(2)|BV(1)|BV(0));
  SLEEPCMD |= 0x04;      
  return 0; 
}


//