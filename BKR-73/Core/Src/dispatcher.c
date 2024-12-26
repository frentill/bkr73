/*
 * dispatcher.h
 *
 *  Created on: Dec 24, 2024
 *      Author: Theta
 */
#include "dispatcher.h"
#include "app.h"

void (*Dispatcher_Thread_Ptr)(void) = &Dispatcher_Thread_A0;

//==============================================================================
// A - TASK (executed in every 250 usec)
//==============================================================================
void Dispatcher_Thread_A0(void)
{
    // loop rate synchronizer for A-tasks
	if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
	{
		LL_TIM_ClearFlag_UPDATE(TIM1);

		// @TODO: read DIGITAL inputs
    }

	Dispatcher_Thread_Ptr = &Dispatcher_Thread_B0;
}

//==============================================================================
//  B - TASK (executed in every 10 ms)
//==============================================================================
void Dispatcher_Thread_B0(void)
{
	static LedControl_t *status_led;
    // loop rate synchronizer for B-tasks
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);

		AppReadRemoteData();

		AppSendRemoteData();

		status_led = (AppState.Remote.IsOK) ? &AppState.Led1s : &AppState.Led100ms;

		if(status_led->state)    LL_GPIO_SetOutputPin(GPIOC, SD_Pin);
		else                     LL_GPIO_ResetOutputPin(GPIOC, SD_Pin);

		AppUpdateTimers();
    }

	Dispatcher_Thread_Ptr = &Dispatcher_Thread_A0;
}



void DispatcherInit(void)
{

}



void DispatcherRun(void)
{

}
