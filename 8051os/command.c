#include "command.h"
#include "core.h"

char CmdBuffer[CMD_LEN];
int send_data_cb(void *data)
{
	int i =0;
	int len=0;
	out_buf *data_temp = (out_buf *)data; 
	unsigned char out[MAX_OUT_BUF_LEN];
	out[0] = CMD_DATA_HEAD;
	for(i=1;i<data_temp->len+1;i++)
	{
	      out[i] = data_temp->buf[i-1];
	}
	out[i] = CMD_DATA_END;
	len = data_temp->len +2;
	UartSendString(out,len);
	return 0;
}


 void send_ack(char cmd,char ret)
{
	struct Ack_Cmd ack_buf;

	ack_buf.head = CMD_ACK_HEAD;
	ack_buf.type= cmd;
	ack_buf.value = ret;
	ack_buf.end = CMD_ACK_END;
	UartSendString((unsigned char *)(&ack_buf),4);
}

void  process_cmd(void)
{
	int ret = 0;
	int i=0;
    if(((CmdBuffer[CMD_HEAD_INDEX]&0xff) == CMD_HEAD)&&((CmdBuffer[CMD_END_INDEX]&0xff) == CMD_END))
    {
    	switch(CmdBuffer[CMD_TYPE_INDEX]) {
			case CMD_SPEED:
				set_speed(CmdBuffer[CMD_VALUE_INDEX]);
				send_ack(CmdBuffer[CMD_TYPE_INDEX],0);
				break;
			case CMD_ANGLE:
				set_angle(CmdBuffer[CMD_VALUE_INDEX]);
				send_ack(CmdBuffer[CMD_TYPE_INDEX],0);
				break;
			case CMD_DIR:
				set_direction(CmdBuffer[CMD_VALUE_INDEX]);
				send_ack(CmdBuffer[CMD_TYPE_INDEX],0);
				break;
			case CMD_AZIMUTH:
				read_azimuth(CmdBuffer[CMD_VALUE_INDEX]);
				break;
			case CMD_ODOM_RAW:
				read_odom_raw(CmdBuffer[CMD_VALUE_INDEX]);
				break;
                     case CMD_IMU_CAPRATION:
                    capration_imu();
                    break;
			case CMD_SIMPLE_READ_ALL:
                                simple_read_all(CmdBuffer[CMD_VALUE_INDEX]);
				    break;
			case CMD_IMU_CAPRATION_STATUS:
				capration_ret();
				break;
                        case CMD_SIMPLE_READ_WALK:
                                simple_read_walk(CmdBuffer[CMD_VALUE_INDEX]);
                                break;
            default:
                    break;
          
		}
    }
}

void recv_cmd(unsigned char cmd_in)
{
	int i=0;
	for(i=0;i<CMD_END_INDEX;i++)
	{
		CmdBuffer[i]=CmdBuffer[i+1];
	}
	CmdBuffer[CMD_END_INDEX]=cmd_in;
	process_cmd();
}

int command_init(void)
{
	//do nothing
}

