/*
 * pid.c
 *
 *  Created on: 2024/04/27
 *      Author: tetud
 */

#include "pid.h"
#include "math_utils.h"

float pure_PID(float error, PID_StructTypedef *param){
	float ctrl_val;
	param->integral+=(error+param->prev_error)*param->ctrl_period/2.0;
	param->diff=(error - param->prev_error+param->td*param->prev_diff)/(param->ctrl_period+param->td);
	ctrl_val=param->kp*error+param->ki*param->integral+param->kd*param->diff;
	param->prev_error=error;
	param->prev_diff=diff;
	return ctrl_val;
}

float PID_ctrl(float target, float current_val, PID_StructTypedef *param, PID_CtrlTypedef *ctrl_param){
	float error=target_duty-current_val;
	if(ctrl_param->if_error_limit && error > ctrl_param->max_error){
		error=ctrl_param->max_error;
	}
	if(ctrl_param->integral_reset_frag){
		param->integral=0.0;
	}
	float ctrl_val=pure_PID(error, param);
	if(ctrl_param->use_rerative_ctrl){
		ctrl_val+=ctrl_param->prev_val;
	}
	if(ctrl_param->if_max_duty && ctrl_val > ctrl_param->max_duty){
		ctrl_val=ctrl_param->max_duty;
	}
	if(ctrl_param->if_accel_limit){

	}
	ctrl_param->prev_val=ctrl_val;
	return ctrl_val;
}
