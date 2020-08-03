/*
 * PID.c
 *
 *  Created on: Jul 23, 2020
 *      Author: perSec
 */


#include "PID.h"

void NORMAL_PID(RATE_PID* PID,float set_angle, float input_angle){
	float temp_error;
	float temp_p;
	float temp_d;

	temp_error = set_angle - input_angle;

	temp_p = (PID->para_p) * temp_error;
	temp_d = (PID->para_d) * (temp_error=PID->pre_rate_d_error);
	PID->pre_rate_d_error = temp_error;

	if( (PID->rate_i_mem) < 200 && (PID->rate_i_mem>-200) ){
		PID->rate_i_mem += (PID->para_i) * temp_error * pid_dt;
	}
	else{
		if(PID->rate_i_mem >199.5 && temp_error<0.0){
			PID->rate_i_mem += (PID->para_i) * temp_error * pid_dt;
		}
		else if(PID->rate_i_mem <-199.5 && temp_error>0.0){
			PID->rate_i_mem += (PID->para_i) * temp_error * pid_dt;
		}
	}

	PID->pid_result = temp_p + temp_d +PID->rate_i_mem;
}


void DUAL_PID(STABILIZE_PID* PID,float set_angle, float input_angle,float input_rate){
	float temp_error;
	float temp_p;
	float temp_d;


	temp_error = set_angle - input_angle;

	temp_p = (PID->stabilize_p) * temp_error;

	if( (PID->stabilize_i_mem < 270) && (PID->stabilize_i_mem > -270) ){
		PID->stabilize_i_mem += (PID->stabilize_i) * temp_error *pid_dt;
	}
	else{
		if(PID->stabilize_i_mem > 269.5 && temp_error<0.0){
			PID->stabilize_i_mem += (PID->stabilize_i) *temp_error *pid_dt;
		}
		else if(PID->stabilize_i_mem < -269.5 && temp_error>0.0){
			PID->stabilize_i_mem += (PID->stabilize_i) * temp_error *pid_dt;
		}
	}


	temp_error = temp_p - input_rate;
	temp_p = PID->rate_p * temp_error;

	temp_d = PID->rate_d * (temp_error - PID->dual_pre_rate_d_error);
	PID->dual_pre_rate_d_error = temp_error;

	if( (PID->dual_rate_i_mem) < 300 && (PID->dual_rate_i_mem>-300) ){
		PID->dual_rate_i_mem += PID->rate_i * temp_error * pid_dt;
	}
	else{
		if(PID->dual_rate_i_mem >299.5 && temp_error<0.0){
			PID->dual_rate_i_mem += PID->rate_i * temp_error * pid_dt;
		}
		else if(PID->dual_rate_i_mem <-299.5 && temp_error>0.0){
			PID->dual_rate_i_mem += PID->rate_i * temp_error * pid_dt;
		}
	}

	PID->dual_pid_result = temp_p + temp_d + (PID->dual_rate_i_mem) + (PID->stabilize_i_mem);
}
