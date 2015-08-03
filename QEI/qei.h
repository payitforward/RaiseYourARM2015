/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *	version 0.0.1
 */

/**
 * @file	qei.h
 * @brief	QEI module controller
 */

#ifndef QEI_QEI_H_
#define QEI_QEI_H_

#define MOTOR_SELECT_LEFT 1
#define MOTOR_SELECT_RIGHT 0

extern void qei_init(uint16_t ms_Timebase);
extern bool qei_getVelocity(bool Select, int32_t *Velocity);


#endif /* QEI_QEI_H_ */
