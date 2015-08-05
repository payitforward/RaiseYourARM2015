/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

#ifndef __HOST_COMM_H_
#define __HOST_COMM_H_

#define MAX_HOSTCOMM_RX_BUF_SIZE		1024
#define MAX_HOSTCOMM_TX_BUF_SIZE		200
#define HOSTCOMM_MSG_START_CODE			0xA5
#define HOSTCOMM_MSG_END_CODE			0x0D

#define HOSTCOMM_MSG_ID_SIZE			1
#define HOSTCOMM_DEST_ID_SIZE			1
#define HOSTCOMM_SRC_ID_SIZE			1
#define HOSTCOMM_DATE_TIME_SIZE		0
#define HOSTCOMM_MSG_CODE_SIZE		1
#define HOSTCOMM_MSG_HEADER_SIZE 	(HOSTCOMM_MSG_ID_SIZE + HOSTCOMM_DEST_ID_SIZE \
		+ HOSTCOMM_SRC_ID_SIZE + HOSTCOMM_DATE_TIME_SIZE + HOSTCOMM_MSG_CODE_SIZE)

typedef enum
{
    MSG_UPDATE_POS=0,
    MSG_TIME,
    MSG_BOX,
    MSG_WAYPOINT,
    MSG_TRACK_BOT,
    MSG_UPDATE_BOT,
    MSG_SPEED_SET,
    MSG_START_TEST,
    MSG_SET_PID,
}MSGCODE;

typedef struct
{
    unsigned char dst_id;
    unsigned char src_id;
    unsigned long date_time;
    unsigned char msg_id;
    unsigned char msg_code;
    unsigned short msg_len;
    unsigned char * data_ptr;
    unsigned char  data_len;
}HOSTCOMM_SRV_MSG;

typedef unsigned char Callback_ID;
typedef void (*HOSTCOMM_CALLBACK_FUNC)(uint8_t *data,uint16_t len);
#define INVALID_HANDLER_ID 0xff

void HostCommInit();
Callback_ID HostCommHandlerRegister(HOSTCOMM_CALLBACK_FUNC callback,MSGCODE code);
bool HostCommHandlerUnregister(Callback_ID id);
uint16_t HostCommCalCheckSum(uint8_t *data, uint16_t len);
void HostComm_EventProcessing(void);
#endif
