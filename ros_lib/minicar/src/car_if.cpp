#include "car_if.h"
using namespace std;
Car_interface::Car_interface(const char* ser_dev)
{
	G_Uport =new Uart(115200,ser_dev);
	if(G_Uport->Open())
	{
		ROS_ERROR("not find uart port %s !",ser_dev);
		G_Uport = NULL;
		return;
	}
	G_Uport->Flush();

}
Car_interface::~Car_interface(void){
	MutexLockGuard lk(Transfer_Mutex);

	if(G_Uport!=NULL)
		G_Uport->Close();
	

}


int Car_interface::check_ack(char cmd)
{
	struct Ack_Cmd ack_buf;	
	int ret =0;
	G_Uport->Recv((char*)(&ack_buf),sizeof(struct Ack_Cmd));
	if(((ack_buf.ack_head&0xff) == CMD_ACK_HEAD) && ((ack_buf.ack_end&0xff) == CMD_ACK_END)){
			if(((ack_buf.ack_type &0xff)== cmd) &&((ack_buf.ack_value &0xff)== 0)) {
					ROS_INFO("check ack ok !: %x ",cmd);
			}else{
				ret = -1;
			}
	} else{
		ret =-1;
                G_Uport->Flush();
		ROS_ERROR("check ack error  !: %x %x %x %x %x \n ",cmd ,ack_buf.ack_head,ack_buf.ack_type,ack_buf.ack_value,ack_buf.ack_end);
	}
	return ret;
}


unsigned int  Car_interface::read2c_and_check_data(struct Read_Int_Data *data_buf,char cmd)
{	
	int ret =0;
	G_Uport->Recv((char*)data_buf,sizeof(struct Read_Int_Data));
	if(((data_buf->head & 0xff) == CMD_DATA_HEAD) && ((data_buf->end & 0xff)== CMD_DATA_END)){
			if((data_buf->type &0xff)== cmd) {
					ROS_INFO("read azimuth ok !: %x %x %x ",data_buf->type,data_buf->value_l,data_buf->value_l);
			}
	} else {
		ret = -1;
                G_Uport->Flush();
		ROS_ERROR("read azimuth error !: %x %x %x %x \n ",data_buf->head,data_buf->type,data_buf->value_l,data_buf->end);
		}
	
	if(!ret)
		return 0;
	
	return -1;
}

unsigned int  Car_interface::readi_and_check_data(struct Read_Int_Data *data_buf,char cmd)
{	
	int ret =0;
	G_Uport->Recv((char*)data_buf,sizeof(struct Read_Int_Data));
	if(((data_buf->head & 0xff) == CMD_DATA_HEAD) && ((data_buf->end & 0xff)== CMD_DATA_END)){
			if((data_buf->type &0xff)== cmd) {
					ROS_INFO("read azimuth ok !: %x %x %x ",data_buf->type,data_buf->value_l,data_buf->value_l);
			}
	} else {
		ret = -1;
                G_Uport->Flush();
		ROS_ERROR("read azimuth error !: %x %x %x %x \n ",data_buf->head,data_buf->type,data_buf->value_l,data_buf->end);
		}
	
	if(!ret)
		return ((data_buf->value_l&0xff) | ((data_buf->value_h&0xff) << 8));
	
	return -1;
}

unsigned int  Car_interface::readm_and_check_data(struct Read_Multi_Data *data_buf,char cmd)
{	
	int ret =0;
	G_Uport->Recv((char*)data_buf,sizeof(struct Read_Multi_Data));
	if(((data_buf->head & 0xff) == CMD_DATA_HEAD) && ((data_buf->end & 0xff)== CMD_DATA_END)){
			if((data_buf->type &0xff)== cmd) {
					ROS_INFO("read multi data ok !: %x %x %x ",data_buf->type,data_buf->value0_l,data_buf->value1_l);
			}
	} else {
		ret = -1;
		ROS_ERROR("read multi data error !: %x %x %x %x \n ",data_buf->head,data_buf->type,data_buf->value0_l,data_buf->end);
		}
	return ret;
}



int Car_interface::set_speed(char speed){
	int ret =0;
	struct Car_Cmd out_buf;
	MutexUniqueLock lk(Transfer_Mutex);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_SPEED;
	out_buf.cmd_value = speed;
	out_buf.cmd_end = CMD_END;
      	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	ret = check_ack(CMD_SPEED);
	return ret;
}

int Car_interface::set_wheel_angle(char angle){
	int ret =0;
	struct Car_Cmd out_buf;
	MutexUniqueLock lk(Transfer_Mutex);
	
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_ANGLE;
	out_buf.cmd_value = angle;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	ret = check_ack(CMD_ANGLE);
	return ret;
}


int Car_interface::set_direction(char dir){
	int ret =0;
	struct Car_Cmd out_buf;
	MutexUniqueLock lk(Transfer_Mutex);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_DIR;
	out_buf.cmd_value = dir;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	ret = check_ack(CMD_DIR);
	return ret;
}


int Car_interface::get_rotation_data(float *rotation_ret){
	int ret =0;
	struct Car_Cmd out_buf;
	struct Read_Int_Data in_buf;
	
	MutexUniqueLock lk(Transfer_Mutex);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_AZIMUTH;
	out_buf.cmd_value = 1;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	*rotation_ret = readi_and_check_data(&in_buf,out_buf.cmd_type)/10;
         if(*rotation_ret < 0)
		return -1;
	return 0;
}

int Car_interface::get_odom_data(odom_d *buf){

	return 0;
}


int Car_interface::get_walk_count_data(unsigned int *walk_count){
	int ret =0;
	struct Car_Cmd out_buf;
	struct Read_Int_Data in_buf;
	MutexUniqueLock lk(Transfer_Mutex);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_ODOM_RAW;
	out_buf.cmd_value = 0;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	*walk_count = readi_and_check_data(&in_buf,out_buf.cmd_type);
         if(*walk_count < 0)
		return -1;
	return 0;
}

int Car_interface::capration(void){
	int ret =0;
	struct Car_Cmd out_buf;
	struct Read_Int_Data in_buf;
	set_direction(0);
	set_wheel_angle(21);
	set_speed(100);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_IMU_CAPRATION;
	out_buf.cmd_value = 0;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
        ret = capration_status_read();
	while(ret)
	{
		printf("wait for capration!\n");
		ret = capration_status_read();
	}
	return 0;
}

int Car_interface::simple_read(unsigned char * ret_buf){
	int ret =0;
	struct Car_Cmd out_buf;
	struct Read_Int_Data in_buf;
	MutexUniqueLock lk(Transfer_Mutex);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_SIMPLE_READ;
	out_buf.cmd_value = 0;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	ret = read2c_and_check_data(&in_buf,out_buf.cmd_type);
	ret_buf[0] = in_buf.value_l;
	ret_buf[1] = in_buf.value_h;
	return ret;
}

char Car_interface::capration_status_read(void){
	int ret =0;
	struct Car_Cmd out_buf;
	struct Read_Int_Data in_buf;
	MutexUniqueLock lk(Transfer_Mutex);
	out_buf.cmd_head = CMD_HEAD;
	out_buf.cmd_type = CMD_CAPRATION_STATUS;
	out_buf.cmd_value = 0;
	out_buf.cmd_end = CMD_END;
	G_Uport->Send((char*)(&out_buf),sizeof(struct Car_Cmd));
	ret = read2c_and_check_data(&in_buf,out_buf.cmd_type);
	return in_buf.value_l;
}




