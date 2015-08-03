/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	Button.h
 * @brief	Push button driver
 */

#include "../include.h"
#include "Buzzer.h"

//* Private function prototype ----------------------------------------------*/
static void buzzer_lowbat_notify_handler(void);
static void Buzzer_lowbat_shutdown_handler(void);
static void buzzer_on_timeout_handler(void);
static void buzzer_Stoptimeout(void);
static TIMER_ID buzzer_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);

//* Private variables -------------------------------------------------------*/
static TIMER_ID buzzer_TimerID = INVALID_TIMER_ID;

static const BUZZER_PULSE Lowbat_notify_profile[2] = {
	{.Freq = 2000, .msTime = 200},
	{.Freq = 0, .msTime = 3000}
};

static const BUZZER_PULSE Lowbat_shutdown_profile[2] = {
	{.Freq = 2000, .msTime = 500},
	{.Freq = 0, .msTime = 200}
};

//* Function declaration ----------------------------------------------------*/
/**
 * @brief Buzzer init
 */
void buzzer_init(void)
{
	BUZZER_GPIO_PERIPHERAL_ENABLE();
	BUZZER_TIMER_PERIPHERAL_ENABLE();

	ROM_GPIOPinConfigure(BUZZER_TIMER_PIN_AF);
	ROM_GPIOPinTypeTimer(BUZZER_PORT, BUZZER_PIN);

	ROM_TimerConfigure(BUZZER_TIMER, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
//	ROM_TimerPrescaleSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, 10);
	ROM_TimerLoadSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, ROM_SysCtlClockGet());
	ROM_TimerMatchSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, ROM_SysCtlClockGet()); // PWM
	ROM_TimerEnable(BUZZER_TIMER, BUZZER_TIMER_CHANNEL);
}

/**
 * @brief Set buzzer frequency
 */
void buzzer_setsound(uint32_t ulFrequency)
{
	uint32_t ulPeriod;

	if (ulFrequency == 0)
	{
		ROM_TimerLoadSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, ROM_SysCtlClockGet());
		ROM_TimerMatchSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, ROM_SysCtlClockGet());
	}
	else
	{
		ulPeriod = ROM_SysCtlClockGet() / ulFrequency;
		ROM_TimerLoadSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, ulPeriod);
		ROM_TimerMatchSet(BUZZER_TIMER, BUZZER_TIMER_CHANNEL, ulPeriod / 2);
	}
}

/**
 * @brief low battery notify
 */
bool buzzer_low_battery_notify(void)
{
	buzzer_setsound(Lowbat_notify_profile[0].Freq);
	if (buzzer_Runtimeout(&buzzer_lowbat_notify_handler, Lowbat_notify_profile[0].msTime) == INVALID_TIMER_ID)
		return false;
	return true;
}

/**
 * @brief low battery shutdown notify
 */
bool buzzer_low_battery_shutdown(void)
{
	buzzer_setsound(Lowbat_shutdown_profile[0].Freq);
	if (buzzer_Runtimeout(&Buzzer_lowbat_shutdown_handler, Lowbat_shutdown_profile[0].msTime) == INVALID_TIMER_ID)
		return false;
	return true;
}

/**
 * @brief low battery shutdown notify
 */
static void buzzer_lowbat_notify_handler(void)
{
	static uint8_t step = 0, repeat = 0;

	buzzer_TimerID = INVALID_TIMER_ID;

	if (step == 0)
	{
		buzzer_setsound(Lowbat_notify_profile[1].Freq);
		buzzer_Runtimeout(&buzzer_lowbat_notify_handler, Lowbat_notify_profile[1].msTime);
	}
	else
	{
		if (repeat < 20)
		{
			buzzer_setsound(Lowbat_notify_profile[0].Freq);
			buzzer_Runtimeout(&buzzer_lowbat_notify_handler, Lowbat_notify_profile[0].msTime);
			repeat++;
		}
		else
		{
			buzzer_setsound(0);
			repeat = 0;
		}
	}
	step++;
	step %= 2;
}

/**
 * @brief low battery shutdown handler
 */
static void Buzzer_lowbat_shutdown_handler(void)
{
	static uint8_t step = 0, repeat = 0;

	buzzer_TimerID = INVALID_TIMER_ID;

	if (step == 0)
	{
		buzzer_setsound(Lowbat_shutdown_profile[1].Freq);
		buzzer_Runtimeout(&Buzzer_lowbat_shutdown_handler, Lowbat_shutdown_profile[1].msTime);
	}
	else
	{
		if (repeat < 10)
		{
			buzzer_setsound(Lowbat_shutdown_profile[0].Freq);
			buzzer_Runtimeout(&Buzzer_lowbat_shutdown_handler, Lowbat_shutdown_profile[0].msTime);
			repeat++;
		}
		else
		{
			buzzer_setsound(0);
			repeat = 0;
//			ROM_IntMasterDisable();
			/* TODO
			 * Turn-off Boost circuit, H-Bridge, LED, Bluetooth, ...
			 */
		}
	}
	step++;
	step %= 2;
}

/**
 * @brief turn on buzzer
 */
void buzzer_on(uint32_t Freq, uint32_t msTime)
{
	buzzer_setsound(Freq);
	buzzer_Runtimeout(&buzzer_on_timeout_handler, msTime);
}

static void buzzer_on_timeout_handler(void)
{
	buzzer_TimerID = INVALID_TIMER_ID;
	buzzer_setsound(0);
}

static void buzzer_Stoptimeout(void)
{
	if (buzzer_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(buzzer_TimerID);
	buzzer_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID buzzer_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	buzzer_Stoptimeout();
	buzzer_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
	return buzzer_TimerID;
}
