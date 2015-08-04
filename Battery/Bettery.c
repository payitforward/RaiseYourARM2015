/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *	version 0.0.1
 */

/**
 * @file	Battery.c
 * @brief	Battery sense
 */
#include "../include.h"
#include "Battery.h"

//* Private function prototype ----------------------------------------------*/
void BattSenseTimerTimeout(void);
void BattSenseISR(void);
static void battery_Stoptimeout(void);
static void battery_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);

//* Public variables -------------------------------------------------------*/
volatile float BatteryVoltage = 0; //<! Battery voltage
//* Private variables ------------------------------------------------------*/
static TIMER_ID battery_TimerID = INVALID_TIMER_ID;

//* Function declaration ---------------------------------------------------*/
/**
 * @brief Init battery sense
 */
void BattSense_init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
	ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64);
	ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_END | ADC_CTL_CH10 | ADC_CTL_IE);
	ROM_ADCSequenceEnable(ADC1_BASE, 3);
 	ADCIntRegister(ADC1_BASE, 3, &BattSenseISR);
 	ROM_ADCIntEnable(ADC1_BASE, 3);

 	// Register schedule for battery measurement
 	battery_Runtimeout(&BattSenseTimerTimeout, 10000);
}

/**
 * @brief Battery measurement trigger
 */
void BattSenseTimerTimeout(void)
{
	battery_TimerID = INVALID_TIMER_ID;
	ROM_ADCProcessorTrigger(ADC1_BASE, 3);
}

/**
 * @brief Battery measurement convertion done callback
 */
void BattSenseISR(void)
{
	uint32_t ADCResult;
	ROM_ADCIntClear(ADC1_BASE, 3);
	battery_Runtimeout(&BattSenseTimerTimeout, 10000);
	ROM_ADCSequenceDataGet(ADC1_BASE, 3, (uint32_t *)&ADCResult);
	BatteryVoltage = ((float)ADCResult) / 4096 * 3.3 * (100 + 470) / 100 + 0.3;

	if (BatteryVoltage < (float)7.6)
	{
		//shutdown robot here to protect battery
		buzzer_low_battery_shutdown();
	}
	else if (BatteryVoltage < (float)7.4)
	{
		//Notify user to shutdown robot
		buzzer_low_battery_notify();
	}
}

static void battery_Stoptimeout(void)
{
	if (battery_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(battery_TimerID);
	battery_TimerID = INVALID_TIMER_ID;
}

static void battery_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	battery_Stoptimeout();
	battery_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
}
