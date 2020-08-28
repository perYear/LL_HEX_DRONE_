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
	float buf[pid_para_size];

	buf[0]=pitch_para.stabilize_p;
	buf[1]=pitch_para.stabilize_i;
	buf[2]=pitch_para.rate_p;
	buf[3]=pitch_para.rate_i;
	buf[4]=pitch_para.rate_d;

	buf[5]=roll_para.stabilize_p;
	buf[6]=roll_para.stabilize_i;
	buf[7]=roll_para.rate_p;
	buf[8]=roll_para.rate_i;
	buf[9]=roll_para.rate_d;

	buf[10]=yaw_para.para_p;
	buf[11]=yaw_para.para_i;
	buf[12]=yaw_para.para_d;

	flash_sector_erase(SPI1,pid_flash_address);
	flash_write(SPI1,pid_flash_address,(uint8_t*)buf,pid_flash_size);
}

void load_pid_para(){
	float buf[pid_para_size];

	flash_read(SPI1,pid_flash_address,(uint8_t*)buf,pid_flash_size);

	pitch_para.stabilize_p=buf[0];
	pitch_para.stabilize_i=buf[1];
	pitch_para.rate_p=buf[2];
	pitch_para.rate_i=buf[3];
	pitch_para.rate_d=buf[4];

	roll_para.stabilize_p=buf[5];
	roll_para.stabilize_i=buf[6];
	roll_para.rate_p=buf[7];
	roll_para.rate_i=buf[8];
	roll_para.rate_d=buf[9];

	yaw_para.para_p=buf[10];
	yaw_para.para_i=buf[11];
	yaw_para.para_d=buf[12];
}

void show_pid_para(){
	float data[pid_para_size];

	flash_read(SPI1,pid_flash_address,(uint8_t*)data,pid_flash_size);

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
	flash_sector_erase(SPI1,mag_flash_address);
	flash_write(SPI1,mag_flash_address,(uint8_t*)hmc5883.mag_para_arr,mag_flash_size);

}

void load_mag_para(){
	flash_read(SPI1,mag_flash_address,(uint8_t*)hmc5883.mag_para_arr,mag_flash_size);
}

void print_mag_para(){
	printf("\n\rMag_Para\r\n");
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

//=============================================================================================================================== GPS
void save_GPS_para(){
	flash_sector_erase(SPI1,gps_flash_address);
	flash_write(SPI1,gps_flash_address,(uint8_t*)gps_para,gps_flash_size);

}

void load_GPS_para(){
	flash_read(SPI1,gps_flash_address,(uint8_t*)gps_para,gps_flash_size);
}
void show_GPS_para(){
	float data[gps_para_size];
	flash_read(SPI1,gps_flash_address,(uint8_t*)data,gps_flash_size);


	printf("\n\rGPS_PID_Check\r\n");
	printf("=================================\r\n");
	printf("gp:%.4f\r\n",data[0]);
	printf("gi:%.4f\r\n",data[1]);
	printf("gd:%.4f\r\n",data[2]);
	printf("=================================\n\n\r");
}

void print_GPS_para(){
	printf("\n\rGPS_PID\r\n");
	printf("=================================\r\n");
	printf("gp:%.4f\r\n",gps_p);
	printf("gi:%.4f\r\n",gps_i);
	printf("gd:%.4f\r\n",gps_d);
	printf("=================================\n\n\r");
}

//=============================================================================================================================== barometer
void save_baro_para(){
	flash_sector_erase(SPI1,baro_flash_address);
	flash_write(SPI1,baro_flash_address,(uint8_t*)baro_para,baro_flash_size);
}

void load_baro_para(){
	flash_read(SPI1,baro_flash_address,(uint8_t*)baro_para,baro_flash_size);
}

void show_baro_para(){
	float data[baro_para_size];
	flash_read(SPI1,baro_flash_address,(uint8_t*)data,baro_flash_size);

	printf("\n\rbaro_PID_Check\r\n");
	printf("=================================\r\n");
	printf("bp:%.4f\r\n",data[0]);
	printf("bi:%.4f\r\n",data[1]);
	printf("bd:%.4f\r\n",data[2]);
	printf("=================================\n\n\r");
}

void print_baro_para(){
	printf("\n\rbaro_PID\r\n");
	printf("=================================\r\n");
	printf("bp:%.4f\r\n",baro_p);
	printf("bi:%.4f\r\n",baro_i);
	printf("bd:%.4f\r\n",baro_d);
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
		temp_float=atoff(&command->command_buf[2]);
		if(command->command_buf[1]=='p'){
			gps_p=temp_float;
			printf("gp: %.4f\r\n",gps_p);
		}
		else if(command->command_buf[1]=='i'){
			gps_i=temp_float;
			printf("gi: %.4f\r\n",gps_i);
		}
		else if(command->command_buf[1]=='d'){
			gps_d=temp_float;
			printf("gd: %.4f\r\n",gps_d);
		}

		//print all
		else if(command->command_buf[1]=='A'){
			printf("gA\r\n");
			print_GPS_para();
		}
		//check flash data
		else if(command->command_buf[1]=='C'){
			printf("gC\r\n");
			show_GPS_para();
		}
		//load flash to memory
		else if(command->command_buf[1]=='L'){
			printf("gL\r\n");
			load_GPS_para();
			print_GPS_para();
		}
		//save data
		else if(command->command_buf[1]=='S'){
			printf("gS\r\n");
			save_GPS_para();
			print_GPS_para();
		}
		else{
			error_command=1;
		}
	}
//========================================================================================================================== baro decoding
	else if(command->command_buf[0]=='b'){
		temp_float=atoff(&command->command_buf[2]);
		if(command->command_buf[1]=='p'){
			baro_p=temp_float;
			printf("bp: %.4f\r\n",baro_p);
		}
		else if(command->command_buf[1]=='i'){
			baro_i=temp_float;
			printf("bi: %.4f\r\n",baro_i);
		}
		else if(command->command_buf[1]=='d'){
			baro_d=temp_float;
			printf("bd: %.4f\r\n",baro_d);
		}
		else if(command->command_buf[1]=='A'){
			printf("bA\r\n");
			print_baro_para();
		}
		//check flash data
		else if(command->command_buf[1]=='C'){
			printf("bC\r\n");
			show_baro_para();
		}
		//load flash to memory
		else if(command->command_buf[1]=='L'){
			printf("bL\r\n");
			load_baro_para();
			print_baro_para();
		}
		//save data
		else if(command->command_buf[1]=='S'){
			printf("bS\r\n");
			save_baro_para();
			print_baro_para();
		}
		else{
			error_command=1;
		}
	}

//========================================================================================================================== error_command
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
