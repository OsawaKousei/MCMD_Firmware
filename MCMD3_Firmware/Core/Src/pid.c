/*
 * pid.c
 *
 *  Created on: 2024/04/27
 *      Author: tetud
 */

#include "pid.h"

float pure_PID(float error, PID_StructTypedef *param){

}

float PID_ctrl(float target, float current_val, PID_StructTypedef *param, PID_CtrlTypedef *ctrl_param){
	float error=target_duty-current_val;
	if(if_error_linit && error > ctrl_param->max_error){
		error=ctrl_param->max_error;
	}
	float ctrl_val=pure_PID(error, param);
	if(if_accel_linit && ctrl_val > ctrl_param->max_duty){
		ctrl_val=ctrl_param->max_duty;
	}
	return ctrl_val;
}
