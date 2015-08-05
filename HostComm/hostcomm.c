/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

#include "../include.h"

#define MAX_HANDLER_EVT 10
typedef struct
{
  HOSTCOMM_CALLBACK_FUNC callback;
  uint8_t code;
}HOSTCOMM_EVT;
uint16_t HostCommCalCheckSum(uint8_t *data, uint16_t len);
void HostComm_HandleRxData(unsigned char *ptr, unsigned long len);
void HostComm_CallHandler();
void HostComm_HandleBluetoothEvent();

static unsigned char HostComm_RxBuf[MAX_HOSTCOMM_RX_BUF_SIZE];
static unsigned char HostComm_TxBuf[MAX_HOSTCOMM_TX_BUF_SIZE];
static uint32_t txHead, rxHead;
static uint32_t txTail, rxTail;
static HOSTCOMM_EVT hostcom_event_list[MAX_HANDLER_EVT];
static uint32_t rxFrameStart, rxFrameEnd;
static bool HostComm_HC05_Timer_IsTimeout = false;
uint32_t recvGetBuff[];
uint16_t HostComm_calcCheckSum(uint8_t *data, uint8_t len)
{
    uint16_t sum=0;
    int i;
    for (i=0; i<len; i++)
    {
        sum+=data[i];
    }
    return sum;
}

void HostCommInit()
{
	int i;
	bluetooth_init(115200);
	HC05_RegisterEvtNotify(&HostComm_HandleBluetoothEvent);
    for(i=0; i< MAX_HANDLER_EVT; i++)
    {
    	hostcom_event_list[i].code=0xFF;
    }
}

uint16_t HostCommCalCheckSum(uint8_t *data, uint16_t len)
{
	uint16_t sum=0,index;
	for (index=0;index<len;index++)
	{
		sum += data[index];
	}
	return sum;
}

void HostComm_HandleBluetoothEvent(void)
{
  HC05_SYSTEM_INFO_TYPES InfoType;
  HC05_SYSTEM_INFO_ID InfoId;
	static uint16_t datalen;

  InfoType = HC05_GetSystemInfoType();
  InfoId = HC05_GetSystemInfoID();

  if(InfoType == HC05_OPERATION_ERROR)
  {

  }

  switch(InfoId)
  {
    case HC05_RX_AVAILABLE:			//there is Rx data
		HC05_QueryRxData();
    break;
    case HC05_READ_DONE:			//done TCP reading
		datalen = HC05_GetRxSize();
		HC05_GetRxData(HostComm_RxBuf, datalen);
		HostComm_HandleRxData(HostComm_RxBuf, datalen);
    break;
    case HC05_WRITE_DONE:			//done TCP writing
    break;
    default: break;
  }
}

void HostComm_HandleRxData(unsigned char *ptr, unsigned long buffLen)
{
	int i,j;
	uint16_t len;
	for (i=0;i<buffLen;i++)
	{
		HostComm_RxBuf[rxTail++]=ptr[i];
		rxTail %= MAX_HOSTCOMM_RX_BUF_SIZE;
	}
	for (i=rxHead;i!=rxTail;i=(i+1)%MAX_HOSTCOMM_RX_BUF_SIZE)
	{
		if (HostComm_RxBuf[i]==HOSTCOMM_MSG_START_CODE)
		{
            len = HostComm_RxBuf[i+1];
            len |= HostComm_RxBuf[i+2]<<8;
            if (HostComm_RxBuf[i+2+len+3]==HOSTCOMM_MSG_END_CODE)
            {
                uint16_t crc = HostComm_RxBuf[i+len+3];
                crc |= HostComm_RxBuf[i+len+4]<<8;
                if (HostCommCalCheckSum(&HostComm_RxBuf[i+3],len)==crc)
                {
                	rxFrameStart = i+3;
                	rxFrameEnd = i+2+len;
                	rxHead=rxFrameEnd+4;
                	// Call handler
                	for (j=0 ; j<MAX_HANDLER_EVT;j++)
                	{
                		if (hostcom_event_list[i].code==HostComm_RxBuf[rxFrameStart])
                		{
                			(hostcom_event_list[i].callback)(HostComm_RxBuf+rxFrameStart,len);
                		}
                	}
                    break;
                }
            }
		}
	}
}

Callback_ID HostCommHandlerRegister(HOSTCOMM_CALLBACK_FUNC callback,MSGCODE code)
{
    int i;
    for(i=0; i< MAX_HANDLER_EVT; i++)
    {
      if((hostcom_event_list[i].code == 0xFF) && (hostcom_event_list[i].callback == NULL)) break;
    }
    if(i == MAX_HANDLER_EVT)
    	return INVALID_HANDLER_ID;

    hostcom_event_list[i].code = code;
    hostcom_event_list[i].callback = callback;

    return (Callback_ID)i;
}

bool HostCommHandlerUnregister(Callback_ID id)
{
	bool ret = false;
    if(id < MAX_HANDLER_EVT)
    {
    	hostcom_event_list[id].code = 0xFF;
    	hostcom_event_list[id].callback = NULL;
        ret = true;
    }
    return ret;
}

void HostComm_EventProcessing(void)
{
	HC05_EventProcessing();
	if (HostComm_HC05_Timer_IsTimeout == true)
	{

		HostComm_HC05_Timer_IsTimeout = false;
	}
}
