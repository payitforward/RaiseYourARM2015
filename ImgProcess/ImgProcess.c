#include "../include.h"
#include "math.h"

#define ROBOT_LENGTH 10.0
typedef enum
{
	TASK_PHASE_INIT=0,
	TASK_PHASE_RUN,
	TASK_PHASE_RECV,
	TASK_UPDATE_MOTOR,
}TaskPhase_t;

typedef struct
{
	float lastInput;
	float lastOutput;
	int lastTime;
}Integral_t;

static TaskPhase_t taskPhase=TASK_PHASE_INIT;
static Integral_t xCal, yCal, thetaCal;
static volatile uint32_t millis;
static uint8_t trackBuffer[40];
static TIMER_ID millis_TimerID = INVALID_TIMER_ID;
static int32_t vright,vleft;
void TimeCntCb();
void updateTrack();
void intReset(Integral_t *intData);
void intCal(Integral_t *intData, float input);
void positionCal();

void imageProcessInit()
{
	taskPhase = TASK_PHASE_INIT;
	millis = 0;
	if (millis_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(millis_TimerID);
	millis_TimerID = TIMER_RegisterEvent(TimeCntCb,1);
	intReset(&xCal);
	intReset(&yCal);
	intReset(&thetaCal);
	speed_set(MOTOR_LEFT,0);
	speed_set(MOTOR_RIGHT,0);
//	SysCtlDelay(Sy)
	bluetooth_print("ImgProcess Init\n");
}

void TimeCntCb()
{
	millis++;
	if (millis_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(millis_TimerID);
	millis_TimerID = TIMER_RegisterEvent(TimeCntCb,1);
}

void imageProcessTask()
{
	switch (taskPhase)
	{
	case TASK_PHASE_INIT:
		imageProcessInit();
		taskPhase = TASK_PHASE_RUN;
		bluetooth_print("ImgProcess run\n");
		break;
	case TASK_PHASE_RUN:
		speed_set(MOTOR_LEFT,100);
		speed_set(MOTOR_RIGHT,100);
		positionCal();
		updateTrack();
		break;
	default:
		break;
	}
}

void positionCal()
{
	float theta,input;
	bool get = qei_getVelocity(0,&vright);
	get = get && qei_getVelocity(1,&vleft);
	if (get==false) return;
	input = 1/ROBOT_LENGTH*(vright-vleft);
	intCal(&thetaCal,input);
	theta = thetaCal.lastOutput;
	input = 1/2*(vright+vleft)*cos_f32(theta);
	intCal(&xCal,input);
	input = 1/2*(vright+vleft)*sin_f32(theta);
	intCal(&yCal,input);
}

void intCal(Integral_t *intData, float input)
{
	// Euler integral
	float output = intData->lastOutput;
	uint32_t time = millis;
	output = (output + input*(time - intData->lastTime))/1000.0f;
	intData->lastOutput = output;
	intData->lastInput = input;
	intData->lastTime = time;
}

void intReset(Integral_t *intData)
{
	intData->lastInput=0;
	intData->lastOutput=0;
	intData->lastTime=0;
}

void updateTrack()
{
	uint16_t index=0;
	uint16_t crc;
	float temp=xCal.lastOutput;
	uint8_t *p=(uint8_t*)&temp;
	trackBuffer[index++]=0xA5;
	trackBuffer[index++]=0;
	trackBuffer[index++]=0;
	trackBuffer[index++]=MSG_TRACK_BOT;
	trackBuffer[index++]=0;
	trackBuffer[index++]=1;
	trackBuffer[index++]=0;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	temp=yCal.lastOutput;
	p=(uint8_t*)&temp;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	temp=thetaCal.lastOutput;
	p=(uint8_t*)&temp;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	trackBuffer[index++]=*p++;
	crc=HostCommCalCheckSum(trackBuffer+3,index-3);
	trackBuffer[1]=index-3;
	trackBuffer[2]=(index-3)>>8;
	trackBuffer[index++]=crc;
	trackBuffer[index++]=crc>>8;
	trackBuffer[index++]=0x0D;
	bluetooth_send(trackBuffer,index);
	//bluetooth_print("Hieu\r\n");
}
