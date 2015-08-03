/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	timer.h
 * @brief	timer event managment
 */

#ifndef TIMER_H_
#define TIMER_H_

typedef void (*TIMER_CALLBACK_FUNC)();

typedef unsigned char TIMER_ID;

#define INVALID_TIMER_ID 0xff

void Timer_Init(void);
TIMER_ID TIMER_RegisterEvent(TIMER_CALLBACK_FUNC callback, unsigned long ms);
bool TIMER_UnregisterEvent(TIMER_ID timer_id);

#endif /* TIMER_H_ */
