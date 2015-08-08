/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	WallFollow.c
 * @brief	wall follow controller
 */


#include "../include.h"
#include "WallFollow.h"
#define AVG_SPEED_FWD 200
#define AVG_SPEED_FWD_SLOW 120
#define AVG_SPEED_BWD 120

//* Private function prototype ----------------------------------------------*/
static void pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
static MOVE getMove(bool isWallLeft,bool isWallFront,bool isWallRight);
static bool TurnBack(int fwdPulse, int avrSpeedLeft,int avrSpeedRight,int turnPulse
		,int bwdPulseLeft, int bwdPulseRight);
static bool TurnLeft(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse, int fwdPulse2);
static bool TurnRight(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse, int fwdPulse2);

//* Private variables -------------------------------------------------------*/
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;
static TIMER_ID pid_TimerID = INVALID_TIMER_ID;
PID_PARAMETERS pid_wall = {.Kp = 0.1, .Kd = 0.0, .Ki = 0.0012,
		.Ts = 0.020, .PID_Saturation = 200, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_posLeft = {.Kp = 0.06, .Kd = 0.0, .Ki = 0.2,
		.Ts = 0.020, .PID_Saturation = 300, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_posRight = {.Kp = 0.035, .Kd = 0.0, .Ki = 0.12,
		.Ts = 0.020, .PID_Saturation = 250, .e_=0, .e__=0, .u_=0};

static int32_t encLeftTmp=0,encRightTmp=0;
static int32_t posLeftTmp=0,posRightTmp=0;
static float leftError, rightError;
static int32_T avrSpeed, avrSpeedTmp;
static bool rqTurnLeft=false,rqTurnRight=false;
static int32_t CtrlStep=1;
static bool isWallLeft, isWallRight, isWallFrontLeft,isWallFrontRight;
static int32_t fwdEnc;
static MOVE eMove=FORWARD;

void clearPosition()
{
	encLeftTmp=0;
	encRightTmp=0;
	qei_setPosLeft(6500);//robot center
	qei_setPosRight(6500);
}

/**
 * @brief Init Wall follow controller
 */
void pid_Wallfollow_init(PID_PARAMETERS pid_param)
{
//	if (pid_TimerID != 0xff)
//	{
//		TIMER_UnregisterEvent(pid_TimerID);
//	}
	pid_Runtimeout(&pid_process_callback, pid_param.Ts);

	ui32_msLoop =  pid_param.Ts;
}

/**
 * @brief Wall follow controller
 */
static bool pid_wallfollow(float delta_IR_left, float delta_IR_right, float averageSpeed,
		WALL_FOLLOW_SELECT wall_follow_select)
{
	static float error, u;
	static WALL_FOLLOW_SELECT preSelect=WALL_FOLLOW_NONE;
	int32_t set_speed[2];

	if (preSelect!=WALL_FOLLOW_NONE)
		if (preSelect!=wall_follow_select)
			pid_reset(&pid_wall);

	switch (wall_follow_select)
	{
	case WALL_FOLLOW_NONE:	//Do nothing
		return true;
	case WALL_FOLLOW_LEFT:
		error = -delta_IR_left;
		break;
	case WALL_FOLLOW_RIGHT:
		error = delta_IR_right;
		break;
	case WALL_FOLLOW_BOTH:
		error = delta_IR_right - delta_IR_left;
		break;
	default:
		return false;
	}

	preSelect = wall_follow_select;

	u = pid_process(&pid_wall,error);
	set_speed[0] = averageSpeed + (int32_t)(u / 2);
	set_speed[1] = averageSpeed - (int32_t)(u / 2);

	speed_set(MOTOR_RIGHT, set_speed[0]);
	speed_set(MOTOR_LEFT, set_speed[1]);

	return true;
}

static void pid_process_callback(void)
{
	pid_TimerID = INVALID_TIMER_ID;
	ControlFlag = true;
	pid_Runtimeout(&pid_process_callback, ui32_msLoop);
}


void pid_Wallfollow_set_follow(WALL_FOLLOW_SELECT follow_sel)
{
	e_wall_follow_select = follow_sel;
}

static void pid_StopTimeout(void)
{
	if (pid_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(pid_TimerID);
	pid_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime)
{
	pid_StopTimeout();
	pid_TimerID = TIMER_RegisterEvent(CallbackFcn, msTime);
	return pid_TimerID;
}
//add your algorithm code and set parameter for movement here
static MOVE getMove(bool isWallLeft,bool isWallFront,bool isWallRight)
{
	MOVE temp;
	if ((!isWallLeft) && (!isWallFront) && (!isWallRight))
	{
		avrSpeed = AVG_SPEED_FWD;
		temp = SIMPLE_FORWARD;
	}
	if (e_wall_follow_select == WALL_FOLLOW_RIGHT)
	{
		if (!isWallRight)
		{
			fwdEnc = 9500;
			return TURN_RIGHT;
		}
		else if (!isWallFront)
		{
			return FORWARD;
		}
		else if (!isWallLeft)
		{
			fwdEnc = 4000;
			return TURN_LEFT;
		}
		else
		{

			return TURN_BACK;
		}
	}
	else if (e_wall_follow_select == WALL_FOLLOW_LEFT)
	{
		if (!isWallLeft)
		{
			fwdEnc = 10500;
			return TURN_LEFT;
		}
		else if (!isWallFront)
		{
			return FORWARD;
		}
		else if (!isWallRight)
		{
			fwdEnc = 4000;
			return TURN_RIGHT;
		}
		else
		{
			return TURN_BACK;
		}
	}
	return FORWARD;
}

bool move(int deltaLeft,int deltaRight,int velLeftMax, int velRightMax)
{
	static int origLeft, origRight;
	static bool done = true;

	int currentLeft=qei_getPosLeft();
	int currentRight=qei_getPosRight();

	if (done)
	{
		done=false;
		origLeft=currentLeft;
		origRight=currentRight;
	}
	//bluetooth_print("move: %5d %5d\r\n",origLeft,currentLeft);
	if (abs(origLeft+deltaLeft-currentLeft)>500)
	{
		pid_posLeft.PID_Saturation = velLeftMax;
		speed_set(MOTOR_LEFT, pid_process(&pid_posLeft,origLeft+deltaLeft-currentLeft));
		pid_posRight.PID_Saturation = velRightMax;
		speed_set(MOTOR_RIGHT, pid_process(&pid_posRight,origRight+deltaRight-currentRight));
		done=false;
	}
	else
	{
		done = true;
		pid_reset(&pid_posLeft);
		pid_reset(&pid_posRight);
	}

	return done;
}


bool TurnRight(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse, int fwdPulse2)
{
	static int vt,vp;
	switch (CtrlStep)
	{
	case 1:
		posLeftTmp=qei_getPosLeft();
		CtrlStep=2;
		vt=1;
		vp=1;
		avrSpeedTmp=avrSpeed;
	case 2://go straight
		if (abs(qei_getPosLeft()-posLeftTmp)<fwdPulse &&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
		{
			avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
					+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			if (isWallLeft)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_LEFT);
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}
		}
		else
		{
			//updatePos();
			CtrlStep++;
		}
		break;
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4://turn 90 degree
		if (abs(qei_getPosLeft()-posLeftTmp) + abs(qei_getPosRight()-posRightTmp) < turnPulse)
		{
			speed_set(MOTOR_LEFT, avrSpeedLeft);
			speed_set(MOTOR_RIGHT, -avrSpeedRight);
			if((abs(qei_getPosLeft()-posLeftTmp)>(turnPulse*0.8*vp/8)) && (vp<9))
			{
				if (avrSpeedLeft>=10)
					avrSpeedLeft-=10;
				vp++;
			}
			if((abs(qei_getPosRight()-posRightTmp)>(turnPulse*0.2*vt/8)) && (vt<9))
			{
				if (avrSpeedLeft>=2)
					avrSpeedLeft-=2;
				vt++;
			}
		}
		else
		{
			//updatePos();
			clearPosition();
			CtrlStep++;
		}
		break;
	case 5://go straight
		if (move(fwdPulse2,fwdPulse2,AVG_SPEED_BWD,AVG_SPEED_BWD))
		{
			CtrlStep++;
		}
		break;
	case 6:
		//updatePos();
		CtrlStep=1;
		pid_reset(&pid_wall);
		return true;
	}
	return false;
}

bool TurnLeft(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse, int fwdPulse2)
{
	static int vt,vp;
	switch (CtrlStep)
	{
	case 1:
		posLeftTmp=qei_getPosLeft();
		CtrlStep++;
		vt=1;
		vp=1;
		avrSpeedTmp=avrSpeed;
	case 2://go straight
		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) &&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
		{
			avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
					+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			if (isWallRight)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_RIGHT);
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}
		}
		else
		{
			//updatePos();
			CtrlStep++;
		}
		break;
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4://turn 90 degree
		if (abs(qei_getPosLeft()-posLeftTmp) + abs(qei_getPosRight()-posRightTmp) < turnPulse)

		{
			speed_set(MOTOR_RIGHT, avrSpeedRight);
			speed_set(MOTOR_LEFT, -avrSpeedLeft);
			if((abs(qei_getPosRight()-posRightTmp)>(turnPulse*0.8*vp/8)) && (vp<9))
			{
				if (avrSpeedRight>=10)
					avrSpeedRight-=10;
				vp++;

			}
			if((abs(qei_getPosLeft()-posLeftTmp)>(turnPulse*0.2*vt/8)) && (vt<9))
			{
				if (avrSpeedLeft>=2)
					avrSpeedLeft-=2;
				vt++;
			}
		}
		else
		{
			//updatePos();
			clearPosition();
			CtrlStep++;
		}
		break;

	case 5://go straight again
		if (move(fwdPulse2,fwdPulse2,AVG_SPEED_BWD,AVG_SPEED_BWD))
		{
			CtrlStep++;
		}
		break;

	case 6:
		//updatePos();
		CtrlStep=1;
		pid_reset(&pid_wall);
		return true;
	}
	return false;
}

bool TurnBack(int fwdPulse, int avrSpeedLeft,int avrSpeedRight,int turnPulse
		,int bwdPulseLeft, int bwdPulseRight)
{
	LED1_ON();LED2_ON();LED3_ON();
	switch (CtrlStep)
	{
	case 1:
	{
		posLeftTmp = qei_getPosLeft();
		avrSpeedTmp = avrSpeed;
		CtrlStep++;
	}
	case 2://go forward a litte bit
	{

		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) &&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
		{
			avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
					+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			if (isWallRight)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_RIGHT);
			else if (isWallLeft)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_LEFT);
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}
		}
		else
		{
			//updatePos();
			CtrlStep++;
		}
		break;
	}
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4:
	{
		if ((abs(qei_getPosLeft()-posLeftTmp)+abs(qei_getPosRight()-posRightTmp))<turnPulse)
		{
			speed_set(MOTOR_RIGHT, avrSpeedRight);
			speed_set(MOTOR_LEFT, avrSpeedLeft);
		}
		else
		{
			CtrlStep++;
		}
		break;
	}
	case 5:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 6:
	{

		if (((abs(qei_getPosLeft()-posLeftTmp)+abs(qei_getPosRight()-posRightTmp))<turnPulse) &&
				(isWallFrontLeft|isWallFrontRight))
		{
			speed_set(MOTOR_RIGHT, -avrSpeedLeft);
			speed_set(MOTOR_LEFT, -avrSpeedRight);
		}
		else
		{
			CtrlStep++;
		}
		break;
	}

	case 7:
	{
		if (move(-bwdPulseLeft,-bwdPulseRight,AVG_SPEED_BWD+20,AVG_SPEED_BWD-20))
		{
			CtrlStep++;
		}
		break;
	}
	case 8:
	{
		//updatePos();
		CtrlStep=1;
		clearPosition();
		pid_reset(&pid_wall);
		speed_set(MOTOR_RIGHT, AVG_SPEED_FWD_SLOW);
		speed_set(MOTOR_LEFT, AVG_SPEED_FWD_SLOW);
		return true;
	}
	}
	return false;
}


void pid_Wallfollow_process(void)
{
	if (ControlFlag)
	{
		static int i;
		pid_Runtimeout(&pid_process_callback, ui32_msLoop);
		ControlFlag = false;

		leftError=(float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IR_GetIrDetectorValue(1);
		rightError=(float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IR_GetIrDetectorValue(2);
		isWallLeft = IR_GetIrDetectorValue(1)<IR_get_calib_value(IR_CALIB_MAX_LEFT);
		isWallRight = IR_GetIrDetectorValue(2)<IR_get_calib_value(IR_CALIB_MAX_RIGHT);
		isWallFrontLeft = IR_GetIrDetectorValue(0)<IR_get_calib_value(IR_CALIB_MAX_FRONT_LEFT);
		isWallFrontRight = IR_GetIrDetectorValue(3)<IR_get_calib_value(IR_CALIB_MAX_FRONT_RIGHT);


		switch(eMove)
		{
		case FORWARD:

			LED1_OFF();LED2_ON();LED3_OFF();
			//updatePos();
			switch (CtrlStep)
			{
			case 1:
				if (avrSpeed<=AVG_SPEED_FWD-30)
					avrSpeed+=30;
				else if (avrSpeed>=AVG_SPEED_FWD+30)
					avrSpeed-=30;
				else
					avrSpeed=AVG_SPEED_FWD;

				if (isWallRight)
				{
					pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_RIGHT);
				}
				else if (isWallLeft)
				{
					pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_LEFT);
				}
				else
				{
					speed_set(MOTOR_RIGHT, avrSpeed);
					speed_set(MOTOR_LEFT, avrSpeed);
				}
				if (!isWallLeft)
				{
					rqTurnLeft=true;
					//bluetooth_print("left\r\n");
				}
				if (!isWallRight)
				{
					rqTurnRight=true;
					//bluetooth_print("right\r\n");
				}
				if (rqTurnLeft || rqTurnRight)
					CtrlStep++;
				if (isWallFrontLeft | isWallFrontRight)
				{
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					rqTurnLeft=false;
					rqTurnRight=false;
					CtrlStep=1;
				}
				break;
			case 2:
				posLeftTmp=qei_getPosLeft();
				i=1;
				CtrlStep++;
			case 3://slow down
				if (!isWallLeft)
				{
					rqTurnLeft=true;
					//bluetooth_print("left\r\n");
				}
				if (!isWallRight)
				{
					rqTurnRight=true;
					//bluetooth_print("right\r\n");
				}
				if ((abs(qei_getPosLeft()-posLeftTmp)<5000)//tu vi tri xd ko tuong trai/phai->vi tri
					&& (!isWallFrontLeft) && (!isWallFrontRight))
						//xd tuong phia truoc
				{
					if (abs(qei_getPosLeft()-posLeftTmp)>i*1000)
					{
						if (avrSpeed>160)
							avrSpeed -= 10;
						i++;
						if (isWallRight)
						{
							pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_RIGHT);
						}
						else if (isWallLeft)
						{
							pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_LEFT);
						}
						else
						{
							speed_set(MOTOR_RIGHT, avrSpeed);
							speed_set(MOTOR_LEFT, avrSpeed);
						}
					}
				}
				else
				{
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					rqTurnLeft=false;
					rqTurnRight=false;
					CtrlStep=1;
				}
				break;
			}
			break;

		case TURN_LEFT:
			if (TurnLeft(5500,40,160,8500,1500))
				eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
			break;

		case TURN_RIGHT:
			if (TurnRight(6000,160,40,8500,2000))
				eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
			break;
		case TURN_BACK:
			if (TurnBack(7000,-70,60,10500,6000,6000))
				eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
			break;
		}
	}
}

