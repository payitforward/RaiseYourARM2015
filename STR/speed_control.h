/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *	version 0.0.1
 */

/**
 * @file	speed_control.h
 * @brief	speed control
 */


#ifndef STR_SPEED_CONTROL_H_
#define STR_SPEED_CONTROL_H_

#define PWM_MOTOR_LEFT			TIMER0_BASE, TIMER_A
#define PWM_MOTOR_RIGHT			TIMER3_BASE, TIMER_A

typedef enum
{
	MOTOR_LEFT = 0,
	MOTOR_RIGHT
} MOTOR_SELECT;

extern void speed_control_init(void);
extern void ProcessSpeedControl(void);
extern void speed_set(MOTOR_SELECT Select, int32_t speed);
extern void speed_Enable_Hbridge(bool Enable);
extern void speed_SetMotorModel(MOTOR_SELECT select, real_T Theta[4]);
extern void speed_GetMotorModel(MOTOR_SELECT select, real_T Theta[4]);

#endif /* STR_SPEED_CONTROL_H_ */
