#include "uart.h"
#include "core.h"
#include <ioCC2540.h>
#include "Common.h"
unsigned char log_buf[MAX_LOG_BUF_LEN];
int IntToStr_len(char* buf, int m)
{
    char tmp[16];
    int isNegtive = 0;
    int index;

    if(m < 0)
    {
        isNegtive = 1;
        m = - m;
    }

    tmp[15] = '\0';
    index = 14;
    do 
    {
        tmp[index--] = m % 10 + '0';
        m /= 10;
    } while (m > 0);

    if(isNegtive)
        tmp[index--] = '-';
    
    

    return (14-index);
}


char* IntToStr(char* buf, int m,int len)
{
    int isNegtive = 0;
    int index;

    if(m < 0)
    {
        isNegtive = 1;
        m = - m;
    }

    //tmp[15] = '\0';
    index = len-1;
    do 
    {
        buf[index--] = m % 10 + '0';
        m /= 10;
    } while (m > 0);

    if(isNegtive)
        buf[index--] = '-';
	
    return buf;
}


int UartSendString(unsigned char *Data, int len)
{
  int i;
  
  for(i=0; i<len; i++)
  {
    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

int InitUart(void)
{ 
    P0SEL |= BV(2) | BV(3);//配置P0.2和P0.3为外设，非GPIO
    U0CSR |= BV(7); //配置当前为UART，非SPI
    U0GCR |= 11; //根据上述波特率设置表格设置115200波特率
    U0BAUD |= 216;// 根据上述波特率设置表格设置115200波特率
    UTX0IF = 0;//位寄存器，直接操作，清除中断标志

    U0CSR |= BV(6);//允许接收数据
    IEN0 |= BV(2);//打开接收中断
    EA=1;//打开总中断
    return 0;
}


void UartSendByte(int8 byte)
{
      U0DBUF = byte;
      while(UTX0IF == 0);
   //   HalHW_WaitMS(1);
      UTX0IF = 0;
}

#pragma vector = URX0_VECTOR  
__interrupt void UART0_ISR(void)  
{  
        URX0IF = 0;
        EA = 0;
	recv_cmd(U0DBUF);
        EA = 1;
}

