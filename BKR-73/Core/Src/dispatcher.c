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
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

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
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

		AppReadRemoteData();

		AppSendRemoteData();

		status_led = (!AppState.Remote.IsOK) ? &AppState.Led1s : &AppState.Led100ms;

		if(status_led->state)    HAL_GPIO_WritePin(GPIOC, SD_Pin, GPIO_PIN_SET);
		else                     HAL_GPIO_WritePin(GPIOC, SD_Pin, GPIO_PIN_RESET);

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
