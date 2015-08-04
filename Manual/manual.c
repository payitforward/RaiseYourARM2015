#include "../include.h"
#include "manual.h"
typedef enum
{
	MANUAL_INIT,
	MANUAL_WAIT_NEW,
	MANUAL_SPEED_SET,
}ManualTask_t;
static volatile uint8_t key=0;
static uint8_t lastKey=0;
static ManualTask_t manualTaskPhase=MANUAL_INIT;
static TIMER_ID manual_TimerID = INVALID_TIMER_ID;
static void manual_TimerTimeout(void);
static void manual_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);
void prepareTestBuffer();
void manualRecv(uint8_t c)
{
	//manual_Runtimeout(manual_TimerTimeout,100);
	key=c;
}

void manualTask()
{
	switch (manualTaskPhase) {
		case MANUAL_INIT:
			LED1_ON();
			manualTaskPhase = MANUAL_WAIT_NEW;
			manual_Runtimeout(manual_TimerTimeout,100);
			break;
		case MANUAL_SPEED_SET:
			LED1_ON();
			switch (key) {
				case 0xA1:
					speed_set(MOTOR_LEFT,0);
					speed_set(MOTOR_RIGHT,200);
					break;
				case 0xA2:
					speed_set(MOTOR_LEFT, -200);
					speed_set(MOTOR_RIGHT,-200);
					break;
				case 0xA3:
					speed_set(MOTOR_LEFT, 200);
					speed_set(MOTOR_RIGHT,0);
					break;
				case 0xA4:
					speed_set(MOTOR_LEFT,200);
					speed_set(MOTOR_RIGHT,200);
					break;
				case 0xA5:
					speed_set(MOTOR_LEFT,-200);
					speed_set(MOTOR_RIGHT,200);
					break;
				case 0xA6:
					speed_set(MOTOR_LEFT,0);
					speed_set(MOTOR_RIGHT,0);
					speed_Enable_Hbridge(false);
					break;
				default:
					speed_set(MOTOR_LEFT,0);
					speed_set(MOTOR_RIGHT,0);
					speed_Enable_Hbridge(false);
					break;
			}

			manualTaskPhase = MANUAL_WAIT_NEW;
			break;
		case MANUAL_WAIT_NEW:
			prepareTestBuffer();
			if (key!=lastKey)
			{
				manualTaskPhase = MANUAL_SPEED_SET;
				lastKey=key;
			}
			break;
		default:
			manualTaskPhase = MANUAL_WAIT_NEW;
			break;
	}
}

static void manual_Stoptimeout(void)
{
	if (manual_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(manual_TimerID);
	manual_TimerID = INVALID_TIMER_ID;
}

static void manual_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	manual_Stoptimeout();
	manual_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
}

void manual_TimerTimeout(void)
{
	key=0xA6;
}

void prepareTestBuffer()
{
	int index=0;
	uint16_t crc;
	int32_t v=qei_getPosLeft();
	uint8_t txBuffer[50];
	uint8_t *p = (uint8_t*)&v;
	static int32_t lastVr,lastVl;
	txBuffer[index++]=0xA5;
	txBuffer[index++]=0;
	txBuffer[index++]=0;
	txBuffer[index++]=MSG_UPDATE_BOT;
	txBuffer[index++]=0;
	txBuffer[index++]=1;
	txBuffer[index++]=0;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	v=qei_getPosRight();
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	if (qei_getVelocity(1,&v)==false)
	{
		v=lastVl;
	}
	else
	{
		lastVl=v;
	}
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	if (qei_getVelocity(0,&v)==false)
	{
		v=lastVr;
	}
	else
	{
		lastVr=v;
	}
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	v=IR_GetIrDetectorValue(0);
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	v=IR_GetIrDetectorValue(1);
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	v=IR_GetIrDetectorValue(2);
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	v=IR_GetIrDetectorValue(3);
	p=(uint8_t*)&v;
	txBuffer[index++]=*p++;
	txBuffer[index++]=*p++;
	crc=HostCommCalCheckSum(txBuffer+3,index-3);
	txBuffer[1]=index-3;
	txBuffer[2]=(index-3)>>8;
	txBuffer[index++]=crc;
	txBuffer[index++]=crc>>8;
	txBuffer[index++]=0x0D;
	bluetooth_send(txBuffer,index);
}
