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

//#define TEST_FORWARD_MOVE
//#define TEST_TURNLEFT_MOVE1
//#define TEST_TURNLEFT_TURN
//#define TEST_TURNLEFT_MOVE2
//#define TEST_TURNLEFT_MOVE3
//#define TEST_TURNRIGHT_MOVE1
//#define TEST_TURNRIGHT_TURN
//#define TEST_TURNRIGHT_MOVE2
//#define TEST_TURNRIGHT_MOVE3
//#define TEST_TURNBACK_FWD
//#define TEST_TURNBACK_TURN1
//#define TEST_TURNBACK_TURN2
//#define TEST_TURNBACK_BACKWARD

#define CELL_ENC 13500

#define AVG_SPEED_FWD_FAST 250
#define AVG_SPEED_FWD 200
#define AVG_SPEED_FWD_SLOW 120
#define AVG_SPEED_BWD 120

//* Private function prototype ----------------------------------------------*/
static void pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
static MOVE getMove(bool isWallLeft,bool isWallFront,bool isWallRight);
static bool TurnBack(int fwdPulse, int avrSpeedLeft,int avrSpeedRight,int turnPulse,
		int resetEnc);
static bool TurnLeft(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse, int fwdPulse2);
static bool TurnRight(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse, int fwdPulse2);

//* Private variables -------------------------------------------------------*/
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;
static TIMER_ID pid_TimerID = INVALID_TIMER_ID;
PID_PARAMETERS pid_wall_right = {.Kp = 0.13, .Kd = 0.0, .Ki = 0.05,
		.Ts = 0.020, .PID_Saturation = 250, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_wall_left = {.Kp = 0.25, .Kd = 0.0, .Ki = 0.05,
		.Ts = 0.020, .PID_Saturation = 250, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_posLeft = {.Kp = 0.06, .Kd = 0.0, .Ki = 0.2,
		.Ts = 0.020, .PID_Saturation = 300, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_posRight = {.Kp = 0.035, .Kd = 0.0, .Ki = 0.12,
		.Ts = 0.020, .PID_Saturation = 250, .e_=0, .e__=0, .u_=0};

static int32_t robotX=0,robotY=0;
static DIRECTION currentDir=UP_DIR;
static int32_t encLeftTmp=0,encRightTmp=0;
static int32_t posLeftTmp=0,posRightTmp=0;
static float leftError, rightError;
static int32_T avrSpeed, avrSpeedTmp;
static bool rqTurnLeft=false,rqTurnRight=false;
static int32_t CtrlStep=1,moveStage=1;
static int32_t fwdPulse,turnPulse;
static bool isWallLeft, isWallRight, isWallFrontLeft,isWallFrontRight;
static MOVE eMove=FORWARD,preMove=NONE;

static void clearPosition()
{
	encLeftTmp=0;
	encRightTmp=0;
	qei_setPosLeft(8800);//distance from robot head to start of current cell in pulses of encoder
	qei_setPosRight(8800);
}
static void initPos()
{
	robotX=0;
	robotY=0;
	clearPosition();
}
//*****************************************************************************
//
//! Update robot position when moving in straight line
//!
//! \param none
//!
//!
//! \return none
//
//*****************************************************************************
static void forwardUpdate()
{
	if (qei_getPosLeft()-encLeftTmp>CELL_ENC)
	{
		encLeftTmp += CELL_ENC;
		switch (currentDir)
		{
		case 0:
			robotY++;
			break;
		case 1:
			robotX++;
			break;
		case 2:
			robotY--;
			break;
		case 3:
			robotX--;
			break;
		}
#ifdef _DEBUG_POS_
		bluetooth_print("pos %d %d %d",robotX,robotY,currentDir);
#endif
	}
}
//*****************************************************************************
//
//! Update robot position when detecting right/left wall
//!
//! \param none
//!
//!
//! \return none
//
//*****************************************************************************
static void forwardUpdateByWall()
{
	//11300:distance from head of robot to the start of current cell
	//when detecting right/left wall in pulses of encoder
	if (qei_getPosLeft()-encLeftTmp<11300-CELL_ENC/2)
	{
		encLeftTmp-=CELL_ENC;
		switch (currentDir)
		{
		case 0:
			robotY--;
			break;
		case 1:
			robotX--;
			break;
		case 2:
			robotY++;
			break;
		case 3:
			robotX++;
			break;
		}
	}
	qei_setPosLeft(11300);
}
/**
 * @brief Init Wall follow controller
 */
static void pid_Wallfollow_init()
{
	ui32_msLoop =  pid_wall_left.Ts * 1000;
	pid_Runtimeout(&pid_process_callback, ui32_msLoop);

}
/**
 * @brief Init Wall follow controller
 */
void wallFollow_init()
{
	pid_Wallfollow_init();
	initPos();
	eMove=FORWARD;
	currentDir=UP_DIR;
	avrSpeed=AVG_SPEED_FWD;
}
/**
 * @brief Wall follow controller
 */
static bool pid_wallfollow(float delta_IR_left, float delta_IR_right, float averageSpeed,
		WALL_FOLLOW_SELECT wall_follow_select)
{
	static float error, u, rightFirst=1;
	static WALL_FOLLOW_SELECT preSelect=WALL_FOLLOW_NONE;
	int32_t set_speed[2];

	if (preSelect!=WALL_FOLLOW_NONE)
		if (preSelect!=wall_follow_select)
		{
			if (preSelect==WALL_FOLLOW_RIGHT)
				pid_reset(&pid_wall_right);
			else if (preSelect==WALL_FOLLOW_LEFT)
				pid_reset(&pid_wall_left);
		}

	switch (wall_follow_select)
	{
	case WALL_FOLLOW_NONE:	//Do nothing
		return true;
	case WALL_FOLLOW_LEFT:
		{
			error = -delta_IR_left;
			u = pid_process(&pid_wall_left,error);
		}
		break;
	case WALL_FOLLOW_RIGHT:
		{
			error = delta_IR_right;
			u = pid_process(&pid_wall_right,error);
		}
		break;
	case WALL_FOLLOW_BOTH:
		{
			error = delta_IR_right - delta_IR_left;
			u = pid_process(&pid_wall_right,error);
		}
		break;
	case WALL_FOLLOW_AUTO:
		if (rightFirst)
		{
			if (isWallRight)
			{

				error = delta_IR_right;
				u = pid_process(&pid_wall_right,error);
			}
			else if (isWallLeft)
			{
				pid_reset(&pid_wall_right);
				error = -delta_IR_left;
				u = pid_process(&pid_wall_left,error);
				rightFirst=0;
			}
		}
		else
		{
			if (isWallLeft)
			{
				error = -delta_IR_left;
				u = pid_process(&pid_wall_left,error);
			}
			else if (isWallRight)
			{
				pid_reset(&pid_wall_left);
				error = delta_IR_right;
				u = pid_process(&pid_wall_right,error);
				rightFirst=1;
			}
		}
		break;
	default:
		return false;
	}

	preSelect = wall_follow_select;

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
//add your algorithm code here
static MOVE getMove(bool isWallLeft,bool isWallFront,bool isWallRight)
{

	if (e_wall_follow_select == WALL_FOLLOW_RIGHT)
	{
		if (!isWallRight)
		{
			return TURN_RIGHT;
		}
		else if (!isWallFront)
		{
			return FORWARD;
		}
		else if (!isWallLeft)
		{
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
			return TURN_LEFT;
		}
		else if (!isWallFront)
		{
			return FORWARD;
		}
		else if (!isWallRight)
		{
			return TURN_RIGHT;
		}
		else
		{
			return TURN_BACK;
		}
	}
	return FORWARD;
}

//*****************************************************************************
//
//! Move robot a little bit (error +-500 pulses). Robot velocity is equal 0 after moving.
//! Make sure this function return true before calling it again.
//!
//! \param deltaLeft distance left motor will go
//! \param deltaRight distance right motor will go
//! \param velLeftMax max left velocity
//! \param velRightMax max right velocity
//!
//! \return true if done
//
//*****************************************************************************
static bool move(int deltaLeft,int deltaRight,int velLeftMax, int velRightMax)
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
#ifdef _DEBUG_MOVE
	bluetooth_print("move: %5d %5d %5d %5d\r\n",(int)left, (int)right,(int)pid_posLeft.u,(int)pid_posRight.u);
#endif

	return done;
}


static bool TurnRight(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse,
		int resetEnc)
{
	static int vt,vp;
	LED1_OFF();LED2_OFF();LED3_ON();
	switch (CtrlStep)
	{
	case 1:
		posLeftTmp=qei_getPosLeft();
		CtrlStep=2;
		vt=1;
		vp=1;
		avrSpeedTmp=avrSpeed;
	case 2://go straight

		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) ||
				(isWallFrontLeft && isWallFrontRight &&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT))))
		{
			if (qei_getPosLeft()<fwdPulse+posLeftTmp)
				avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
				+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			else
				avrSpeed = (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
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
#ifdef TEST_TURNRIGHT_MOVE1
		speed_Enable_Hbridge(false);
#endif
			pid_reset(&pid_wall_left);
			pid_reset(&pid_wall_right);
			forwardUpdate();
			CtrlStep++;
			avrSpeed=avrSpeedTmp;
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
				if (avrSpeedLeft>=24)
					avrSpeedLeft-=24;
				vp++;
			}
			if((abs(qei_getPosRight()-posRightTmp)>(turnPulse*0.2*vt/8)) && (vt<9))
			{
				if (avrSpeedRight>=4)
					avrSpeedRight-=4;
				vt++;
			}
		}
		else
		{
#ifdef TEST_TURNRIGHT_TURN
		speed_Enable_Hbridge(false);
#endif
			currentDir=(currentDir+1)%4;
			clearPosition();
			qei_setPosLeft(resetEnc);
			qei_setPosRight(resetEnc);
			forwardUpdate();
			CtrlStep=1;
			pid_reset(&pid_wall_right);
			pid_reset(&pid_wall_left);
			speed_set(MOTOR_LEFT, avrSpeed);
			speed_set(MOTOR_RIGHT, avrSpeed);
			return true;
		}
		break;
	}
	return false;
}

//*****************************************************************************
//
//! Control two motor to make robot turn left 90 degree
//!
//! \param fwdPulse is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param avrSpeedLeft is the speed of left motor.
//! \param avrSpeedRight is the speed of right motor.
//! \param turnPulse is the total pulse of two encoder after turn
//! \param resetEnc is reset value for encoder after turning 90 degree, ignore this if you don't want to estimate position
//! \return true if finish
//!			false if not
//
//*****************************************************************************
static bool TurnLeft(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse,
		int resetEnc)
{
	static int vt,vp;
	LED1_ON();LED2_OFF();LED3_OFF();
//	bluetooth_print("LS %d\r\n",CtrlStep);
	switch (CtrlStep)
	{
	case 1:
		posLeftTmp=qei_getPosLeft();
		CtrlStep++;
		avrSpeedTmp=avrSpeed;
	case 2://go straight
		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) ||
				(isWallFrontLeft && isWallFrontRight &&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT))))

		{
			if (qei_getPosLeft()<fwdPulse+posLeftTmp)
				avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
					+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			else
				avrSpeed = (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;

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
#ifdef TEST_TURNLEFT_MOVE1
		speed_Enable_Hbridge(false);
#endif

			pid_reset(&pid_wall_right);
			pid_reset(&pid_wall_left);
			forwardUpdate();
			CtrlStep++;
			avrSpeed=avrSpeedTmp;
		}
		break;
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
		vp=1;
		vt=1;
	case 4://turn 90 degree

		if (abs(qei_getPosLeft()-posLeftTmp) + abs(qei_getPosRight()-posRightTmp) < turnPulse)

		{
			speed_set(MOTOR_RIGHT, avrSpeedRight);
			speed_set(MOTOR_LEFT, -avrSpeedLeft);
			if((abs(qei_getPosRight()-posRightTmp)>(turnPulse*0.8*vp/8)) && (vp<9))
			{
				if (avrSpeedRight>=24)
					avrSpeedRight-=24;
				vp++;

			}
			if((abs(qei_getPosLeft()-posLeftTmp)>(turnPulse*0.2*vt/8)) && (vt<9))
			{
				if (avrSpeedLeft>=4)
					avrSpeedLeft-=4;
				vt++;
			}
		}
		else
		{
#ifdef TEST_TURNLEFT_TURN
		speed_Enable_Hbridge(false);
#endif
			currentDir=(currentDir+3)%4;
			clearPosition();
			qei_setPosLeft(resetEnc);
			qei_setPosRight(resetEnc);
			forwardUpdate();
			CtrlStep=1;
			pid_reset(&pid_wall_left);
			pid_reset(&pid_wall_right);
			speed_set(MOTOR_LEFT, avrSpeed);
			speed_set(MOTOR_RIGHT, avrSpeed);
			return true;
		}
		break;
	}
	return false;

}

//*****************************************************************************
//
//! Control two motor to make robot turn back 180 degree.
//!
//! \param fwdPulse is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param avrSpeedLeft is the speed of left motor.
//! \param avrSpeedRight is the speed of left motor.
//! \param NumPulse is the total pulse of two encoder after turn
//! \param resetEnc is the reset value for encoder after turning back
//! \return true if finish
//!			false if not
//
static bool TurnBack(int fwdPulse, int avrSpeedLeft,int avrSpeedRight,int turnPulse,
		int resetEnc)
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

		if (abs(qei_getPosLeft()-posLeftTmp)<fwdPulse)
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
			pid_reset(&pid_wall_left);
			pid_reset(&pid_wall_right);
			forwardUpdate();
			CtrlStep++;
			avrSpeed = avrSpeedTmp;
		}
		break;
	}
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4://turing 90 degree
	{
#ifdef TEST_TURNBACK_FWD
		speed_Enable_Hbridge(false);
#endif
		if ((abs(qei_getPosLeft()-posLeftTmp)+abs(qei_getPosRight()-posRightTmp))<turnPulse)
		{
			speed_set(MOTOR_RIGHT, avrSpeedRight);
			speed_set(MOTOR_LEFT, avrSpeedLeft);
		}
		else
		{
			currentDir=(currentDir+3)%4;
			CtrlStep++;
		}
		break;
	}
	case 5:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 6://turning another 90 degree
	{
#ifdef TEST_TURNBACK_TURN1
		speed_Enable_Hbridge(false);
#endif

		if ((abs(qei_getPosLeft()-posLeftTmp)+abs(qei_getPosRight()-posRightTmp))<turnPulse)
		{
			speed_set(MOTOR_RIGHT, -avrSpeedLeft);
			speed_set(MOTOR_LEFT, -avrSpeedRight);
		}
		else
		{
#ifdef TEST_TURNBACK_TURN2
		speed_Enable_Hbridge(false);
#endif
			currentDir=(currentDir+3)%4;
			clearPosition();
			qei_setPosLeft(resetEnc);
			qei_setPosRight(resetEnc);
			forwardUpdate();
			CtrlStep=1;
			return true;
		}
		break;
	}
	}
	return false;
}
//*****************************************************************************
//
//! Control robot to go straight forward by following wall
//!
//!
//! \return true if left/right wall is detected
//!			false if no left/right wall is detected
//
static bool Forward()
{
	LED1_OFF();LED2_ON();LED3_OFF();
	//bluetooth_print("%d\r\n",avrSpeed);
	if ((!isWallLeft) || (!isWallRight))
	{
		forwardUpdateByWall();
		return true;
	}
	forwardUpdate();
	if (avrSpeed<AVG_SPEED_FWD_FAST-20)
		avrSpeed+=20;
	else if (avrSpeed>AVG_SPEED_FWD_FAST)
		avrSpeed=AVG_SPEED_FWD_FAST;

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
			switch (moveStage)
			{
			case 1:
				if (Forward())
					moveStage++;
				if (isWallFrontLeft| isWallFrontRight)
				{
					preMove=eMove;
					eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
				}
				break;
			case 2:
				posLeftTmp=qei_getPosLeft();
				moveStage++;
				i=1;
				avrSpeedTmp=avrSpeed;
			case 3://slow down
				forwardUpdate();
				if (!isWallLeft)
				{
					rqTurnLeft=true;
				}
				if (!isWallRight)
				{
					rqTurnRight=true;
				}
				if ((abs(qei_getPosLeft()-posLeftTmp)<5000)
						&& (!isWallFrontLeft) && (!isWallFrontRight))
				{
					if ((abs(qei_getPosLeft()-posLeftTmp)>i*500) && (avrSpeed>AVG_SPEED_FWD-40))
					{
						avrSpeed -= 10;
						i++;
					}
					if (isWallLeft|isWallRight)
						pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_AUTO);
					else
					{
						pid_reset(&pid_wall_left);
						pid_reset(&pid_wall_right);
						speed_set(MOTOR_RIGHT, avrSpeed);
						speed_set(MOTOR_LEFT, avrSpeed);
					}
				}
				else
				{
#ifdef TEST_FORWARD_MOVE
					speed_Enable_Hbridge(false);
#endif
					preMove=eMove;
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					if (eMove==FORWARD)
						avrSpeed=AVG_SPEED_FWD;
					rqTurnLeft=false;
					rqTurnRight=false;
					moveStage=1;
				}
				break;
			}
			break;

		case TURN_LEFT:
			switch (moveStage)
			{
			case 1:
				if (preMove!=FORWARD)//after turning left or right
					//test
					// ____
					// |   |
					// | | |
					fwdPulse=6000;
				else if ((preMove==FORWARD) && (avrSpeed<AVG_SPEED_FWD_FAST))
					//after turning back
					//test
					// ___
					// |__   |
					//    |__|
					fwdPulse=5500;
				else//after moving forward
					//test
					// ___
					// ___  |
					//    | |
					//    |_|
					fwdPulse=4500;
				moveStage++;

			case 2:
				if (TurnLeft(fwdPulse,60,240,7800,1700+CELL_ENC))
				{
					moveStage++;
				}
				break;
			case 3:
				posLeftTmp=qei_getPosLeft();
				moveStage++;
			case 4:
				//go straight a little bit to check wall
				forwardUpdate();
				if (abs(qei_getPosLeft()-posLeftTmp)<2000)
				{
					if (abs(qei_getPosLeft()-posLeftTmp)>1500)
					{
						if (!isWallRight)
							rqTurnRight=1;
						if (!isWallLeft)
							rqTurnLeft=1;
					}
					avrSpeed=AVG_SPEED_FWD_SLOW;
					if (isWallLeft|isWallRight)
						pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_AUTO);
					else
					{
						pid_reset(&pid_wall_left);
						pid_reset(&pid_wall_right);
						speed_set(MOTOR_RIGHT, avrSpeed);//left motor is faster
						speed_set(MOTOR_LEFT, avrSpeed);
					}
				}
				else
				{
#ifdef TEST_TURNLEFT_MOVE3
					speed_Enable_Hbridge(false);
#endif
					//time to check front wall
					preMove=eMove;
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					rqTurnLeft=false;
					rqTurnRight=false;
					moveStage=1;
					pid_reset(&pid_wall_left);
					pid_reset(&pid_wall_right);
				}
			}
			break;

		case TURN_RIGHT:
			switch (moveStage)
			{
			case 1:
				if (preMove!=FORWARD)//after turning left or right
					//test
					// ____
					// |   |
					// | | |
					fwdPulse=6000;
				else if ((preMove==FORWARD) && (avrSpeed<AVG_SPEED_FWD_FAST))
					//after turning back
					//test
					// 	  ____
					// |  ____
					// |__|
					fwdPulse=5500;
				else//after moving forward
					//test
					//   _____
					// | _____
					// | |
					// |_|
					fwdPulse=4500;
				moveStage++;
			case 2:
				if (TurnRight(fwdPulse,200,40,8000,1700+CELL_ENC))
				{
					moveStage++;
				}
				break;
			case 3:
				posLeftTmp=qei_getPosLeft();
				moveStage++;
			case 4:
				forwardUpdate();
				if (abs(qei_getPosLeft()-posLeftTmp)<1000)
				{
					if (abs(qei_getPosLeft()-posLeftTmp)>500)
					{
						if (!isWallRight)
							rqTurnRight=1;
						if (!isWallLeft)
							rqTurnLeft=1;
					}
					avrSpeed=AVG_SPEED_FWD_SLOW;
					if (isWallLeft|isWallRight)
						pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_AUTO);
					else
					{
						pid_reset(&pid_wall_left);
						pid_reset(&pid_wall_right);
						speed_set(MOTOR_RIGHT, avrSpeed);
						speed_set(MOTOR_LEFT, avrSpeed);
					}
				}
				else
				{
#ifdef TEST_TURNRIGHT_MOVE3
					speed_Enable_Hbridge(false);
#endif
					preMove=eMove;
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					rqTurnLeft=false;
					rqTurnRight=false;
					moveStage=1;
					pid_reset(&pid_wall_left);
					pid_reset(&pid_wall_right);
				}
			}
			break;
		case TURN_BACK:
			switch (moveStage)
			{
			case 1:
				if (preMove==FORWARD)
					fwdPulse=8000;
				else
					fwdPulse=9000;
				moveStage++;
			case 2:
				if (TurnBack(fwdPulse,-140,60,8000,13000))
				{
					//rotate more if we still detect front wall: do it yourself ;D
					moveStage++;
				}
				break;
			case 3:
				if (move(-9000,-9000,AVG_SPEED_BWD,AVG_SPEED_BWD))
				{
#ifdef TEST_TURNBACK_BACKWARD
					speed_Enable_Hbridge(false);
#endif
					forwardUpdate();
					avrSpeed = AVG_SPEED_FWD_SLOW;
					preMove=eMove;
					eMove=FORWARD;
					moveStage = 1;
				}
			}



			break;
		}
	}
}

