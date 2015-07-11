/*
 * qei.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef QEI_QEI_H_
#define QEI_QEI_H_


extern void qei_init(uint16_t ms_Timebase);
extern bool qei_getVelocity(bool Select, int32_t *Velocity);


#endif /* QEI_QEI_H_ */
