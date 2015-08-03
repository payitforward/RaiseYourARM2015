/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	PID.c
 * @brief	PID wall controller
 */

#include "../include.h"
#include "PID.h"
//* Private variables -------------------------------------------------------*/
static PID_PARAMETERS pid_parameter;

void pid_init(void)
{
	memset(&pid_parameter, 0, sizeof(PID_PARAMETERS));

}

void pid_set_parameters(PID_PARAMETERS pid_param)
{
	pid_parameter = pid_param;
}

float pid_process(float error)
{
	pid_parameter.e__ = pid_parameter.e_;
	pid_parameter.e_ = pid_parameter.e;
	pid_parameter.e = error;
	pid_parameter.u_ = pid_parameter.u;
	pid_parameter.u = pid_parameter.u_ + pid_parameter.Kp * (pid_parameter.e - pid_parameter.e_)
			+ pid_parameter.Ki * pid_parameter.Ts * pid_parameter.e
			+ (pid_parameter.Kd / pid_parameter.Ts) * (pid_parameter.e - 2 * pid_parameter.e_ + pid_parameter.e__);

	return pid_parameter.u;
}
