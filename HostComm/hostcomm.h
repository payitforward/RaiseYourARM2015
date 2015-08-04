/*
 * HostComm.h
 *
 *  Created on: Jul 15, 2015
 *      Author: Huan
 */

#ifndef HOSTCOMM_H_
#define HOSTCOMM_H_

enum MSG_CODE
{
    MSG_UPDATE_POS=0,
    MSG_TIME,
    MSG_BOX,
    MSG_WAYPOINT,
    MSG_TRACK_BOT,
    MSG_UPDATE_BOT,
    MSG_SPEED_SET,
    MSG_START_TEST,
};

void HostCommInit();
void HostComm_process(void);
uint16_t HostCommCalCheckSum(uint8_t *data, uint16_t len);

#endif /* HOSTCOMM_H_ */
