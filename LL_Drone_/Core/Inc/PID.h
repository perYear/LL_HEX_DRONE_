/*
 * PID.h
 *
 *  Created on: Jul 23, 2020
 *      Author: perSec
 */

#include "main.h"
#include "init.h"

#ifndef INC_PID_H_
#define INC_PID_H_

#define pid_para_size 13

#define bound_pitch_stabilize_p	70
#define bound_pitch_stabilize_i	30
#define bound_pitch_rate_p	70
#define bound_pitch_rate_i	20
#define bound_pitch_rate_d	100

#define bound_roll_stabilize_p	70
#define bound_roll_stabilize_i	30
#define bound_roll_rate_p	70
#define bound_roll_rate_i	20
#define bound_roll_rate_d	100

#define bound_yaw_p 70
#define bound_yaw_i 20
#define bound_yaw_d 1500

#define stabilize_p stabilize_pid_para[0]
#define stabilize_i stabilize_pid_para[1]
#define	rate_p rate_pid.rate_pid_para[0]
#define	rate_i rate_pid.rate_pid_para[1]
#define	rate_d rate_pid.rate_pid_para[2]

#define para_p rate_pid_para[0]
#define para_i rate_pid_para[1]
#define para_d rate_pid_para[2]


#define dual_rate_i_mem rate_pid.rate_i_mem
#define dual_pre_rate_d_error rate_pid.pre_rate_d_error
#define dual_pid_result rate_pid.pid_result


typedef struct{
	float rate_pid_para[3];

	float rate_i_mem;
	float pre_rate_d_error;

	float pid_result;
}RATE_PID;

typedef struct{
	RATE_PID rate_pid;
	float stabilize_pid_para[2];

	float stabilize_i_mem;
}STABILIZE_PID;


STABILIZE_PID pitch_para,roll_para;

RATE_PID yaw_para;

void NORMAL_PID(RATE_PID* PID,float set_angle, float input_angle);


void DUAL_PID(STABILIZE_PID* PID,float set_angle, float input_angle,float input_rate);


#endif /* INC_PID_H_ */
