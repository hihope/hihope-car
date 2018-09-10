#include "uart.h"
/*******************************************************************  
* 名称：                  Open  
* 功能：                打开串口并返回串口设备文件描述  
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)  
* 出口参数：        正确返回为1，错误返回为0  
*******************************************************************/
Uart::Uart(int speed_,const char* dev_path_)
{
	Speed = speed_;
    sprintf(Dev_path,"%s",dev_path_);
}

int Uart::Open()
{    
    if (Dev_status == OPENDED)
	{
		return -1;
	}
    Fd = open(Dev_path, O_RDWR|O_NOCTTY|O_NDELAY);
    if (FALSE == Fd)
    {    
        ROS_ERROR("Can't Open Serial Port %s",Dev_path);
        return -1;
    }    
    //恢复串口为阻塞状态                                   
   if(fcntl(Fd, F_SETFL, 0) < 0)
    {
        ROS_ERROR("fcntl failed!\n");
        return -1;
    }
    else    
    {
        ROS_INFO("fcntl=%d\n",fcntl(Fd, F_SETFL,0));
    }
    //测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))
    {
        ROS_ERROR("standard input is not a terminal device\n");
        return -1;
    }
    else
    {
        ROS_INFO("isatty success!\n");
    }
    ROS_INFO("fd->open=%d\n",Fd);
    Dev_status = OPENDED;
	Init(Speed);
	return 0;
}    
/*******************************************************************  
* 名称：                Close  
* 功能：                关闭串口并返回串口设备文件描述  
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)  
* 出口参数：        void  
*******************************************************************/    
     
void Uart::Close()
{   
	if (Dev_status == CLOSED)
	{
		return;
	}
    close(Fd);
	Dev_status = CLOSED;
}
     
/*******************************************************************  
* 名称：                Set  
* 功能：                设置串口数据位，停止位和效验位  
* 入口参数：        fd        串口文件描述符  
*                              speed     串口速度  
*                              flow_ctrl   数据流控制  
*                           databits   数据位   取值为 7 或者8  
*                           stopbits   停止位   取值为 1 或者2  
*                           parity     效验类型 取值为N,E,O,,S  
*出口参数：          正确返回为1，错误返回为0  
*******************************************************************/    
int Uart::Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)    
{
    int   i;    
    int   status;    
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};    
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};    
             
    struct termios options;    
       
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  
    */    
    if( tcgetattr( fd,&options)  !=  0)    
    {    
        ROS_ERROR("SetupSerial !");        
        return(FALSE);     
    }    
      
    //设置串口输入波特率和输出波特率    
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)    
    {    
        if  (speed == name_arr[i])    
        {                 
            cfsetispeed(&options, speed_arr[i]);     
            cfsetospeed(&options, speed_arr[i]);      
        }    
    }         
       
    //修改控制模式，保证程序不会占用串口    
    options.c_cflag |= CLOCAL;    
    //修改控制模式，使得能够从串口中读取输入数据    
    options.c_cflag |= CREAD;    
      
    //设置数据流控制    
    switch(flow_ctrl)    
    {    
          
        case 0 ://不使用流控制    
              options.c_cflag &= ~CRTSCTS;    
              break;       
          
        case 1 ://使用硬件流控制    
              options.c_cflag |= CRTSCTS;    
              break;    
        case 2 ://使用软件流控制    
              options.c_cflag |= IXON | IXOFF | IXANY;    
              break;    
    }    
    //设置数据位    
    //屏蔽其他标志位    
    options.c_cflag &= ~CSIZE;    
    switch (databits)    
    {      
        case 5    :    
                     options.c_cflag |= CS5;    
                     break;    
        case 6    :    
                     options.c_cflag |= CS6;    
                     break;    
        case 7    :        
                 options.c_cflag |= CS7;    
                 break;    
        case 8:        
                 options.c_cflag |= CS8;    
                 break;      
        default:       
                 ROS_ERROR("Unsupported data size\n");    
                 return (FALSE);     
    }    
    //设置校验位    
    switch (parity)    
    {      
        case 'n':    
        case 'N': //无奇偶校验位。    
                 options.c_cflag &= ~PARENB;     
                 options.c_iflag &= ~INPCK;        
                 break;     
        case 'o':      
        case 'O'://设置为奇校验        
                 options.c_cflag |= (PARODD | PARENB);     
                 options.c_iflag |= INPCK;                 
                 break;     
        case 'e':     
        case 'E'://设置为偶校验      
                 options.c_cflag |= PARENB;           
                 options.c_cflag &= ~PARODD;           
                 options.c_iflag |= INPCK;          
                 break;    
        case 's':    
        case 'S': //设置为空格     
                 options.c_cflag &= ~PARENB;    
                 options.c_cflag &= ~CSTOPB;    
                 break;     
        default:      
                 ROS_ERROR("Unsupported parity\n");        
                 return (FALSE);     
    }     
    // 设置停止位     
    switch (stopbits)    
    {      
        case 1:       
                 options.c_cflag &= ~CSTOPB; break;     
        case 2:       
                 options.c_cflag |= CSTOPB; break;    
        default:       
                       ROS_ERROR("Unsupported stop bits\n");     
                       return (FALSE);    
    }    
       
    //修改输出模式，原始数据输出    
    options.c_oflag &= ~OPOST;    
     options.c_iflag &= ~OPOST;
    options.c_iflag &= ~ICRNL; 
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    options.c_lflag &= ~(ISIG | ICANON);    
       
    //设置等待时间和最小接收字符    
    options.c_cc[VTIME] = 5; /* 读取一个字符等待1*(1/10)s */      
    options.c_cc[VMIN] = 5; /* 读取字符的最少个数为1 */    
       
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读    
    tcflush(fd,TCIFLUSH);    
       
    //激活配置 (将修改后的termios数据设置到串口中）    
    if (tcsetattr(fd,TCSANOW,&options) != 0)      
    {    
        ROS_ERROR("com set error!\n");      
        return (FALSE);     
    }    
    return (TRUE);     
}    
/*******************************************************************  
* 名称：                Init()  
* 功能：                串口初始化  
* 入口参数：        fd       :  文件描述符     
*               speed  :  串口速度  
*                              flow_ctrl  数据流控制  
*               databits   数据位   取值为 7 或者8  
*                           stopbits   停止位   取值为 1 或者2  
*                           parity     效验类型 取值为N,E,O,,S  
*                        
* 出口参数：        正确返回为1，错误返回为0  
*******************************************************************/    
int Uart::Init(int speed)
{    
    int err;
    //设置串口数据帧格式    
    if (Set(Fd,speed,0,8,1,'N') == FALSE)
    {                                                             
        return FALSE;    
    }    
    else    
    {    
        return  TRUE;    
    }    
}    
     
/*******************************************************************  
* 名称：                  Recv  
* 功能：                接收串口数据  
* 入口参数：        fd                  :文件描述符      
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中  
*                              data_len    :一帧数据的长度  
* 出口参数：        正确返回为1，错误返回为0  
*******************************************************************/    
int Uart::Recv(char *rcv_buf,int data_len)    
{    
    int len,fs_sel;
    struct timeval time;    
	fd_set fs_read;
    FD_ZERO(&fs_read);    
    FD_SET(Fd,&fs_read);    
	
    time.tv_sec = 2;
    time.tv_usec = 0;    
	
    //使用select实现串口的多路通信    
    fs_sel = select(Fd+1,&fs_read,NULL,NULL,&time);    
    if(fs_sel)    
    {    
        len = read(Fd,rcv_buf,data_len);    
        return len;    
    }    
    else    
    {    
        ROS_ERROR("Sorry,I am wrong!");    
        return FALSE;    
    }         
}    
/********************************************************************  
* 名称：                 Send  
* 功能：                发送数据  
* 入口参数：        fd                  :文件描述符      
*                              send_buf    :存放串口发送数据  
*                              data_len    :一帧数据的个数  
* 出口参数：        正确返回为1，错误返回为0  
*******************************************************************/    
int Uart::Send(char *send_buf,int data_len)    
{    
    int len = 0;
       
    len = write(Fd,send_buf,data_len);
    if (len == data_len ) 
    {    
        //ROS_INFO("send cmd is  0x%x \n",send_buf[2]);  
        return len;
    }         
    else       
    {    
                   
        tcflush(Fd,TCOFLUSH);    
        return FALSE;    
    }    
       
}


void Uart::Flush(void)
{
	tcflush(Fd,TCIFLUSH); // 清除输入缓存
	tcflush(Fd,TCOFLUSH); // 清除输出缓存
}
