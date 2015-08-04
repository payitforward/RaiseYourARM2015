#include "../include.h"
#include "math.h"

#define ROBOT_LENGTH 10.0
#define SPEED_FACTOR 0.05
#define THRESH 15
typedef enum
{
	TASK_PHASE_INIT=0,
	TASK_PHASE_RUN,
	TASK_PHASE_RECV,
	TASK_UPDATE_MOTOR,
	TASK_PHASE_WAIT,
}TaskPhase_t;

typedef struct
{
	float lastInput;
	float lastOutput;
	int lastTime;
}Integral_t;

typedef struct __attribute__((packed))
{
	int32_t error;
	uint16_t xr;
	uint16_t yr;
	uint16_t xd;
	uint16_t yd;
}RecvData_t;
static TaskPhase_t taskPhase=TASK_PHASE_INIT;
static Integral_t xCal, yCal, thetaCal;
static volatile uint32_t millis;
//static uint8_t trackBuffer[40];
static TIMER_ID millis_TimerID = INVALID_TIMER_ID;
static int32_t vright,vleft;
void TimeCntCb();
void updateTrack();
void intReset(Integral_t *intData);
void intCal(Integral_t *intData, float input);
void positionCal();
void cmdConvert(int32_t v, float theta);
static bool newDataFlag = false;
static uint8_t imgBuff[12];
static RecvData_t *pRecv=(RecvData_t*)imgBuff;
static PID_PARAMETERS imgPID;
static int32_t set_speed[2];
static PID_PARAMETERS imgPidParam = {.Kp = 1.0, .Kd = 0.0, .Ki = 0.0, .Ts = 20};

void img_pid_set_parameters(PID_PARAMETERS pid_param)
{
	imgPID = pid_param;
}

float img_pid_process(float error)
{
	imgPID.e__ = imgPID.e_;
	imgPID.e_ = imgPID.e;
	imgPID.e = error;
	imgPID.u_ = imgPID.u;
	imgPID.u = imgPID.u_ + imgPID.Kp * (imgPID.e - imgPID.e_)
			+ imgPID.Ki * imgPID.Ts * imgPID.e
			+ (imgPID.Kd / imgPID.Ts) * (imgPID.e - 2 * imgPID.e_ + imgPID.e__);

	return imgPID.u;
}

void TimeCntCb()
{
	millis++;
	if (millis_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(millis_TimerID);
	millis_TimerID = TIMER_RegisterEvent(TimeCntCb,1);
}
void imageProcessInit()
{
	taskPhase = TASK_PHASE_INIT;
	millis = 0;
//	if (millis_TimerID != INVALID_TIMER_ID)
//		TIMER_UnregisterEvent(millis_TimerID);
//	millis_TimerID = TIMER_RegisterEvent(TimeCntCb,1);
	intReset(&xCal);
	intReset(&yCal);
	intReset(&thetaCal);
//	speed_set(MOTOR_LEFT,0);
//	speed_set(MOTOR_RIGHT,0);
	bluetooth_print("ImgProcess Init\n");
	newDataFlag = false;
}

void imageProcessTask()
{
	uint16_t len,i;
	uint16_t recvLen;
	float u;
	switch (taskPhase)
	{
	case TASK_PHASE_INIT:
		imageProcessInit();
		taskPhase = TASK_PHASE_RUN;
		bluetooth_print("ImgProcess run\n");
		memset(&imgPidParam, 0, sizeof(PID_PARAMETERS));
		break;
	case TASK_PHASE_RUN:
//		positionCal();
//		updateTrack();
		taskPhase = TASK_PHASE_WAIT;
		break;
	case TASK_PHASE_WAIT:
		if (newDataFlag)
		{
			if (((pRecv->xr+THRESH>pRecv->xd) && (pRecv->xr-THRESH<pRecv->xd))
					|| ((pRecv->yr+THRESH>pRecv->yd) && (pRecv->yr-THRESH<pRecv->yd)))
			{
				speed_set(MOTOR_LEFT,0);
				speed_set(MOTOR_RIGHT,0);
			}
			else
			{
				//Calc culate
				imgPidParam.Ts = millis;
				img_pid_set_parameters(imgPidParam);
				millis=0;
				u=img_pid_process(pRecv->error/1000.0);
				set_speed[0] = -200 + (int32_t)(u / 2);
				set_speed[1] = -200 - (int32_t)(u / 2);
				speed_set(MOTOR_RIGHT, set_speed[0]);
				speed_set(MOTOR_LEFT, set_speed[1]);
				taskPhase = TASK_PHASE_RUN;
			}
			newDataFlag = false;
		}
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

void imgProcessRecv(uint8_t c)
{
	static bool newFrame = false;
	static uint8_t index = 0;
	if (c==0xA5)
	{
		newFrame=true;
		index=0;
	}
	else if (newFrame==true)
	{
		imgBuff[index++]=c;
	}

	if (c==0x0D)
	{
		newFrame=false;
		if (index==13)
		{
			index=0;
			newDataFlag = true;
		}
		index=0;
	}
}
//
//void updateTrack()
//{
//	uint16_t index=0;
//	uint16_t crc;
//	float temp=xCal.lastOutput;
//	uint8_t *p=(uint8_t*)&temp;
//	trackBuffer[index++]=0xA5;
//	trackBuffer[index++]=0;
//	trackBuffer[index++]=0;
//	trackBuffer[index++]=MSG_TRACK_BOT;
//	trackBuffer[index++]=0;
//	trackBuffer[index++]=1;
//	trackBuffer[index++]=0;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	temp=yCal.lastOutput;
//	p=(uint8_t*)&temp;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	temp=thetaCal.lastOutput;
//	p=(uint8_t*)&temp;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	trackBuffer[index++]=*p++;
//	crc=HostCommCalCheckSum(trackBuffer+3,index-3);
//	trackBuffer[1]=index-3;
//	trackBuffer[2]=(index-3)>>8;
//	trackBuffer[index++]=crc;
//	trackBuffer[index++]=crc>>8;
//	trackBuffer[index++]=0x0D;
//	bluetooth_send(trackBuffer,index);
//	//bluetooth_print("Hieu\r\n");
//}


