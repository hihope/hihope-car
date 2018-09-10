#include "i2c.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define I2C_ENS1            BV(6)  // I2C总线使能位
#define I2C_STA             BV(5)  // 开始标志位
#define I2C_STO             BV(4)  // 停止标志位
#define I2C_SI              BV(3)  // 中断信号位
#define I2C_AA              BV(2)  // 设置为AA时，I2C模块的确认标志位
#define I2C_MST_RD_BIT      BV(0)  // 主机RD / WRn位与从机地址进行或运算。

#define I2C_CLOCK_MASK      0x83   // 初始设置111，默认为保留

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum
{
  // I2C总线作为主设备时，所有的返回状态
  mstStarted   = 0x08,
  mstRepStart  = 0x10,
  mstAddrAckW  = 0x18,
  mstAddrNackW = 0x20,
  mstDataAckW  = 0x28,
  mstDataNackW = 0x30,
  mstLostArb   = 0x38,
  mstAddrAckR  = 0x40,
  mstAddrNackR = 0x48,
  mstDataAckR  = 0x50,
  mstDataNackR = 0x58,
} i2cStatus_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define I2C_WRAPPER_DISABLE() st( I2CWC    =     0x00;              )
#define I2C_CLOCK_RATE(x)     st( I2CCFG  &=    ~I2C_CLOCK_MASK;    \
                                  I2CCFG  |=     x;                 )
#define I2C_SET_NACK()        st( I2CCFG &= ~I2C_AA; )
#define I2C_SET_ACK()         st( I2CCFG |=  I2C_AA; )

// 启用I2C总线
#define I2C_ENABLE()          st( I2CCFG |= (I2C_ENS1); )
#define I2C_DISABLE()         st( I2CCFG &= ~(I2C_ENS1); )

// 必须在设置STA之前清除SI，然后必须手动清除STA。
#define I2C_STRT() st (             \
  I2CCFG &= ~I2C_SI;                \
  I2CCFG |= I2C_STA;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  I2CCFG &= ~I2C_STA; \
)

// 必须在清除SI前设置STO
#define I2C_STOP() st (             \
  I2CCFG |= I2C_STO;                \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_STO) != 0);  \
)

// 停止时钟拉伸，然后在到达时获取数据
#define I2C_READ(_X_) st (          \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  (_X_) = I2CDATA;                  \
)

// 首先写新数据，然后停止时钟拉伸
#define I2C_WRITE(_X_) st (         \
  I2CDATA = (_X_);                  \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
)


/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
 uint8 i2cAddr;  // 目标从设备地址预先向左移动一个，使RD / WRn LSB最低位保持为零。
//uint8 reg_addr;

/**************************************************************************************************
 * @fn          i2cMstStrt
 *
 * @brief       作为I2C总线主机尝试发送I2C总线起始信号和从机地址
 *
 * @param       RD_WRn -> RW 读写位，也就是从机地址的最低位
 *
 * @return      起始信号请求及从机地址确认状态返回
 */
static uint8 i2cMstStrt(uint8 RD_WRn)
{
  I2C_STRT();                         // 发送起始信号

  if (I2CSTAT == mstStarted)          // 起始信号已被送达
  {
    I2C_WRITE(i2cAddr | RD_WRn);      // 这边的i2cAddr是从机地址事先左移了一位，RD_WRn为读写位
  }

  return I2CSTAT;                     // 返回起始信号请求及从机地址确认状态
}


static uint8 i2cMst_reStrt(uint8 RD_WRn)
{
  I2C_STRT();                         // 发送起始信号

  if (I2CSTAT == mstRepStart)          // 起始信号已被送达
  {
    I2C_WRITE(i2cAddr | RD_WRn);      // 这边的i2cAddr是从机地址事先左移了一位，RD_WRn为读写位
  }

  return I2CSTAT;                     // 返回起始信号请求及从机地址确认状态
}


/**************************************************************************************************
 * @fn          HalI2CInit
 *
 * @brief       初始化I2C总线作为主机
 *
 * @param       address -> I2C从机地址
 * @param       clockRate -> I2C时钟频率
 *
 * @return      None.
 */
void HalI2CInit(uint8 address, i2cClock_t clockRate)
{
  i2cAddr = address << 1;             // 从机地址向左移一位

  I2C_WRAPPER_DISABLE();              // 关闭I2C
  I2CADDR = 0;                        // 不支持多主机模式
  I2C_CLOCK_RATE(clockRate);          // 设置时钟频率
  I2C_ENABLE();                       // 开启I2C
}

uint8 HalI2CRead(uint8 len,uint8 addr,uint8 *pBuf)
{
  uint8 cnt = 0;
  uint8 stat = 0;
//write dummy
while(!stat) {

  if (i2cMstStrt(0) != mstAddrAckW)
  {
    len = 0;
  }

  I2C_WRITE(addr);
  stat = I2CSTAT;
  if (stat == mstDataAckW)	{  // 数据已经发送，返回ACK
        stat = 1;
  }
  else if (stat == mstDataNackW) {
  		I2C_STOP();  
		return 0;
  }else {
  I2C_STOP();  
 	return 0;
  }
}

stat = i2cMst_reStrt(I2C_MST_RD_BIT);

//restart 
  // 作为I2C总线主机尝试发送I2C总线起始信号和从机地址
  // 从机地址+READ已经被传输，且返回ACK
  if ( stat != mstAddrAckR)   
  {
    len = 0;
  }

  // 除了最后一个NACK，所有字节都是ACK。 
  // 如果只读1个字节，则会发送一个NACK。 
  // 因此，如果要读取多于1个字节，我们只要启用ACK。
  if (len > 1)
 {
    I2C_SET_ACK();
  }

  while (len > 0)
  {
    // 从设备在读完最后一个字节后需要发送NACK
    if (len == 1)
    {
      I2C_SET_NACK();
    }

    // 从I2C总线读取一个字节
    I2C_READ(*pBuf++);
    cnt++;
    len--;

    if (I2CSTAT != mstDataAckR)     // 数据已经接收，返回ACK
    {
      if (I2CSTAT != mstDataNackR)  // 数据已经接收，返回NACK
      {
        // NACK，所以不要计算最后一个字节
        cnt--;
      }
      break;
    }
  }
  
  //I2C_SET_NACK();
  
  I2C_STOP();                        // 发送停止信号

  return cnt;
}

/**************************************************************************************************
 * @fn          HalI2CWrite
 *
 * @brief       作为I2C主机向从机写数据
 *
 * @param       len -> 要写的数据长度
 * @param       pBuf -> 指向数据缓冲区pBuf的指针，用于写字节
 *
 * @return      成功写入的字节数
 */
uint8 HalI2CWrite(uint8 len, uint8 *pBuf)
{
  // 作为I2C总线主机尝试发送I2C总线起始信号和从机地址
  // 从机地址+WRITE已经被传输，且返回ACK
  if (i2cMstStrt(0) != mstAddrAckW)
  {
    len = 0;
  }

  for (uint8 cnt = 0; cnt < len; cnt++)
  {
    // 向I2C总线写一个字节
    I2C_WRITE(*pBuf++);

    if (I2CSTAT != mstDataAckW)     // 数据已经发送，返回ACK
    {
      if (I2CSTAT == mstDataNackW)  // 数据已经发送，返回NACK
      {
        len = cnt + 1;
      }
      else
      {
        len = cnt;
      }
      break;
    }
  }

  I2C_STOP();                         // 发送停止信号

  return len;
}

void WriteData(uint8 Address,uint8 Dat){
  uint8 buf[2];
  buf[0] = Address;
  buf[1] = Dat;
  HalI2CWrite(2,buf);
}


void ReadData(uint8 addr, uint8 *Pbuf,uint8 Num){
  HalI2CRead(Num,addr,Pbuf);
}

/**************************************************************************************************
 * @fn          HalI2CDisable
 *
 * @brief       将I2C总线置于非活动状态
 *
 * @param       None.
 *
 * @return      None.
 */
void HalI2CDisable(void)
{
  I2C_DISABLE();             // 关闭I2C总线功能
}

/*********************************************************************
*********************************************************************/
