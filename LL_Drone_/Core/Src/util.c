/*
 * util.c
 *
 *  Created on: 2020. 7. 23.
 *      Author: perSec
 */


#include "util.h"

#include "stdlib.h"


void init_COMMAND(UART_COMMAND* command,USART_TypeDef* uart){
	command->UART=uart;
	command->command_length=0;
	command->command_receive=0;
	LL_USART_EnableIT_RXNE(uart);
	LL_USART_EnableIT_IDLE(uart);
}


//=============================================================================================================================== hexa pid
void save_pid_para(){
	uint32_t temp;
	uint8_t save_buf[pid_flash_size];
	uint8_t i,j;

	//----------------------------------------------------------- pitch
	//stabilize para
	for(i=0;i<2;i++){
		temp=*(unsigned*)&(pitch_para.stabilize_pid_para[i]);
		for(j=0;j<4;j++){
			save_buf[4*i+j]=0x000000ff&temp;
			temp=temp>>8;
		}
	}
	//rate pra
	for(i=0;i<3;i++){
		temp=*(unsigned*)&(pitch_para.rate_pid.rate_pid_para[i]);
		for(j=0;j<4;j++){
			save_buf[4*i+j + 8]=0x000000ff&temp;
			temp=temp>>8;
		}
	}
	//----------------------------------------------------------- roll
	//stabilize para
	for(i=0;i<2;i++){
		temp=*(unsigned*)&(roll_para.stabilize_pid_para[i]);
		for(j=0;j<4;j++){
			save_buf[4*i+j + 20]=0x000000ff&temp;
			temp=temp>>8;
		}
	}
	//rate para
	for(i=0;i<3;i++){
		temp=*(unsigned*)&(roll_para.rate_pid.rate_pid_para[i]);
		for(j=0;j<4;j++){
			save_buf[4*i+j + 28]=0x000000ff&temp;
			temp=temp>>8;
		}
	}
	//----------------------------------------------------------- yaw
	for(i=0;i<3;i++){
		temp=*(unsigned*)&(yaw_para.rate_pid_para[i]);
		for(j=0;j<4;j++){
			save_buf[4*i+j + 40]=0x000000ff&temp;
			temp=temp>>8;
		}
	}
	//-----------------------------------------------------------


	flash_sector_erase(SPI1,pid_flash_address);
	flash_write(SPI1,pid_flash_address,save_buf,pid_flash_size);
}

void load_pid_para(){
	uint32_t temp;
	uint8_t load_buf[pid_flash_size];
	uint8_t i,j;

	flash_read(SPI1,pid_flash_address,load_buf,pid_flash_size);
	//----------------------------------------------------------- pitch
	//stabilize para
	for(i=0;i<2;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j]<<(j*8);
		}
		pitch_para.stabilize_pid_para[i]=*(float*)&temp;
	}
	//rate para
	for(i=0;i<3;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j+8]<<(j*8);
		}
		pitch_para.rate_pid.rate_pid_para[i]=*(float*)&temp;
	}
	//----------------------------------------------------------- roll
	//stabilize para
	for(i=0;i<2;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j+20]<<(j*8);
		}
		roll_para.stabilize_pid_para[i]=*(float*)&temp;
	}
	//rate para
	for(i=0;i<3;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j+28]<<(j*8);
		}
		roll_para.rate_pid.rate_pid_para[i]=*(float*)&temp;
	}
	//----------------------------------------------------------- yaw
	for(i=0;i<3;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j+40]<<(j*8);
		}
		yaw_para.rate_pid_para[i]=*(float*)&temp;
	}
	//-----------------------------------------------------------
}

void show_pid_para(){
	float data[pid_para_size];
	uint32_t temp;
	uint8_t load_buf[pid_flash_size];
	uint8_t i,j;

	flash_read(SPI1,pid_flash_address,load_buf,pid_flash_size);
	for(i=0;i<pid_para_size;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j]<<(j*8);
		}
		data[i]=*(float*)&temp;
	}

	printf("\n\rHexa_PID_Check\r\n");
	printf("=================================\r\n");
	printf("hpsp:%.4f\r\n",data[0]);
	printf("hpsi:%.4f\r\n",data[1]);
	printf("hprp:%.4f\r\n",data[2]);
	printf("hpri:%.4f\r\n",data[3]);
	printf("hprd:%.4f\r\n",data[4]);
	printf("=================================\r\n");
	printf("hrsp:%.4f\r\n",data[5]);
	printf("hrsi:%.4f\r\n",data[6]);
	printf("hrrp:%.4f\r\n",data[7]);
	printf("hrri:%.4f\r\n",data[8]);
	printf("hrrd:%.4f\r\n",data[9]);
	printf("=================================\r\n");
	printf("hyp:%.4f\r\n",data[10]);
	printf("hyi:%.4f\r\n",data[11]);
	printf("hyd:%.4f\r\n",data[12]);
	printf("=================================\n\n\r");
}

void print_pid_para(){
	printf("\n\rHexa_PID\r\n");
	printf("=================================\r\n");
	printf("hpsp:%.4f\r\n",pitch_para.stabilize_p);
	printf("hpsi:%.4f\r\n",pitch_para.stabilize_i);
	printf("hprp:%.4f\r\n",pitch_para.rate_p);
	printf("hpri:%.4f\r\n",pitch_para.rate_i);
	printf("hprd:%.4f\r\n",pitch_para.rate_d);
	printf("=================================\r\n");
	printf("hrsp:%.4f\r\n",roll_para.stabilize_p);
	printf("hrsi:%.4f\r\n",roll_para.stabilize_i);
	printf("hrrp:%.4f\r\n",roll_para.rate_p);
	printf("hrri:%.4f\r\n",roll_para.rate_i);
	printf("hrrd:%.4f\r\n",roll_para.rate_d);
	printf("=================================\r\n");
	printf("hyp:%.4f\r\n",yaw_para.para_p);
	printf("hyi:%.4f\r\n",yaw_para.para_i);
	printf("hyd:%.4f\r\n",yaw_para.para_d);
	printf("=================================\n\n\r");
}

//=============================================================================================================================== magnetometer
void save_mag_para(){
	uint32_t temp;
	uint8_t save_buf[mag_flash_size];
	uint8_t i,j;

	for(i=0;i<mag_para_size;i++){
		temp=*(unsigned*)&(hmc5883.mag_para_arr[i]);
		for(j=0;j<4;j++){
			save_buf[4*i+j]=0x000000ff&temp;
			temp=temp>>8;
		}
	}
	flash_sector_erase(SPI1,mag_flash_address);
	flash_write(SPI1,mag_flash_address,save_buf,mag_flash_size);

}

void load_mag_para(){
	uint32_t temp;
	uint8_t load_buf[mag_flash_size];
	uint8_t i,j;

	flash_read(SPI1,mag_flash_address,load_buf,mag_flash_size);
	for(i=0;i<mag_para_size;i++){
		temp=0;
		for(j=0;j<4;j++){
			temp+=load_buf[4*i+j]<<(j*8);
		}
		hmc5883.mag_para_arr[i]=*(float*)&temp;
	}
}

void print_mag_para(){
	printf("\n\rMag_PID\r\n");
	printf("=================================\r\n");
	printf("mag_offset_x:%.4f\r\n",hmc5883.offset_x);
	printf("mag_offset_y:%.4f\r\n",hmc5883.offset_y);
	printf("mag_offset_z:%.4f\r\n",hmc5883.offset_z);
	printf("=================================\r\n");
	printf("mag_scale_x:%.4f\r\n",hmc5883.scale_x);
	printf("mag_scale_y:%.4f\r\n",hmc5883.scale_y);
	printf("mag_scale_z:%.4f\r\n",hmc5883.scale_z);
	printf("=================================\n\n\r");
}

//===============================================================================================================================


void COMMAND_Decode(UART_COMMAND* command){
	float temp_float;
	uint8_t error_value=0;
	uint8_t error_command=0;
//========================================================================================================================== hexa decoding
	if(command->command_buf[0]=='h'){

		//------------------------------------------------------------------------------------------- pitch
		if(command->command_buf[1]=='p'){
			temp_float=atoff(&command->command_buf[4]);
			if(command->command_buf[2]=='s'){
				if(command->command_buf[3]=='p'){
					if(temp_float<bound_pitch_stabilize_p && temp_float>-bound_pitch_stabilize_p){
						pitch_para.stabilize_p=temp_float;
					}
					else
						error_value=1;
				}
				else if(command->command_buf[3]=='i'){
					if(temp_float<bound_pitch_stabilize_i && temp_float>-bound_pitch_stabilize_i){
						pitch_para.stabilize_i=temp_float;
					}
					else
						error_value=1;
				}
				else{
					error_command=1;
				}
			}
			else if(command->command_buf[2]=='r'){
				if(command->command_buf[3]=='p'){
					if(temp_float<bound_pitch_rate_p && temp_float>-bound_pitch_rate_p){
						pitch_para.rate_p=temp_float;
					}
					else
						error_value=1;
				}
				else if(command->command_buf[3]=='i'){
					if(temp_float<bound_pitch_rate_i && temp_float>-bound_pitch_rate_i){
						pitch_para.rate_i=temp_float;
					}
					else
						error_value=1;
				}
				else if(command->command_buf[3]=='d'){
					if(temp_float<bound_pitch_rate_d && temp_float>-bound_pitch_rate_d){
						pitch_para.rate_d=temp_float;
					}
					else
						error_value=1;
				}
				else{
					error_command=1;
				}
			}
			else{
				error_command=1;
			}
			if(!error_command){
				command->command_buf[4]=0x00;
				printf("%s: %.4f\n\r",command->command_buf,temp_float);
			}
		}

		//------------------------------------------------------------------------------------------- roll
		else if(command->command_buf[1]=='r'){
			temp_float=atoff(&command->command_buf[4]);
			if(command->command_buf[2]=='s'){
				if(command->command_buf[3]=='p'){
					if(temp_float<bound_roll_stabilize_p && temp_float>-bound_roll_stabilize_p){
						roll_para.stabilize_p=temp_float;
					}
					else
						error_value=1;
				}
				else if(command->command_buf[3]=='i'){
					if(temp_float<bound_roll_stabilize_i && temp_float>-bound_roll_stabilize_i){
						roll_para.stabilize_i=temp_float;
					}
					else
						error_value=1;
				}
				else{
					error_command=1;
				}
			}
			else if(command->command_buf[2]=='r'){
				if(command->command_buf[3]=='p'){
					if(temp_float<bound_roll_rate_p && temp_float>-bound_roll_rate_p){
						roll_para.rate_p=temp_float;
					}
					else
						error_value=1;
				}
				else if(command->command_buf[3]=='i'){
					if(temp_float<bound_roll_rate_i && temp_float>-bound_roll_rate_i){
						roll_para.rate_i=temp_float;
					}
					else
						error_value=1;
				}
				else if(command->command_buf[3]=='d'){
					if(temp_float<bound_roll_rate_d && temp_float>-bound_roll_rate_d){
						roll_para.rate_d=temp_float;
					}
					else
						error_value=1;
				}
				else{
					error_command=1;
				}
			}
			else{
				error_command=1;
			}
			if(!error_command){
				command->command_buf[4]=0x00;
				printf("%s: %.4f\n\r",command->command_buf,temp_float);
			}
		}

		//------------------------------------------------------------------------------------------- yaw
		else if(command->command_buf[1]=='y'){
			temp_float=atoff(&command->command_buf[3]);
			if(command->command_buf[2]=='p'){
				if(temp_float<bound_yaw_p && temp_float>-bound_yaw_p){
					yaw_para.para_p=temp_float;
				}
				else
					error_value=1;
			}
			else if(command->command_buf[2]=='i'){
				if(temp_float<bound_yaw_i && temp_float>-bound_yaw_i){
					yaw_para.para_i=temp_float;
				}
				else
					error_value=1;
			}
			else if(command->command_buf[2]=='d'){
				if(temp_float<bound_yaw_d && temp_float>-bound_yaw_d){
					yaw_para.para_d=temp_float;
				}
				else
					error_value=1;
			}
			else{
				error_command=1;
			}
			if(!error_command){
				command->command_buf[3]=0x00;
				printf("%s: %.4f\n\r",command->command_buf,temp_float);
			}
		}

		//------------------------------------------------------------------------------------------- other commands
		//print all
		else if(command->command_buf[1]=='A'){
			printf("hA\r\n");
			print_pid_para();
		}
		//check flash data
		else if(command->command_buf[1]=='C'){
			printf("hC\r\n");
			show_pid_para();
		}
		//load flash to memory
		else if(command->command_buf[1]=='L'){
			printf("hL\r\n");
			load_pid_para();
			print_pid_para();
		}
		//save data
		else if(command->command_buf[1]=='S'){
			printf("hS\r\n");
			save_pid_para();
			print_pid_para();
		}
		//error
		else{
			error_command=1;
		}

		//-------------------------------------------------------------------------------------------
	}

//========================================================================================================================== mag decoding
	else if(command->command_buf[0]=='m'){
		if(command->command_buf[1]=='A'){
			printf("mA\r\n");
			print_mag_para();
		}
		else{
			error_command=1;
		}
	}

//========================================================================================================================== gps decoding
	else if(command->command_buf[0]=='g'){
		temp_float=atoff(&command->command_buf[3]);
		if(command->command_buf[1]=='p'){

		}
		else if(command->command_buf[1]=='i'){

		}
		else if(command->command_buf[1]=='d'){

		}
		else{
			error_command=1;
		}
	}
//========================================================================================================================== baro decoding
	else if(command->command_buf[0]=='b'){
		temp_float=atoff(&command->command_buf[3]);
		if(command->command_buf[1]=='p'){

		}
		else if(command->command_buf[1]=='i'){

		}
		else if(command->command_buf[1]=='d'){

		}
		else{

		}
	}
	else{
		error_command=1;
	}
//========================================================================================================================== error_checking

	if(error_command){
		printf("\r\n----------Wrong Command----------\n\n\r");
	}
	else if(error_value){
		printf("\r\n----------Wrong value----------\n\n\r");
	}
	else{
		if(command->command_buf[0]=='h'){

		}
		else if(command->command_buf[0]=='g'){

		}
		else if(command->command_buf[0]=='b'){

		}
	}
//==========================================================================================================================
}
