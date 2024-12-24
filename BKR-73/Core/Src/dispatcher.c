/*
 * dispatcher.h
 *
 *  Created on: Dec 24, 2024
 *      Author: Theta
 */
#include "dispatcher.h"

#ifdef RELEASE
static uint16_t warmup_timer = 3000; // cycles
#else
static uint16_t warmup_timer = 300; // cycles
#endif

static uint16_t ledCnt = 0, ledPeriod = 200;
static const uint16_t Tauz = 10; // ms

void DispatcherInit(void)
{
	while (1)
	{
		if (LL_TIM_IsActiveFlag_UPDATE(TIM6))
		{
			LL_TIM_ClearFlag_UPDATE(TIM6);

			warmup_timer--;
			if(warmup_timer == 0) break;


			TOGGLE_BOARD_LED();

		}
	}

	ledPeriod = 50;
	warmup_timer = 200;

	while (1)
	{
		if (LL_TIM_IsActiveFlag_UPDATE(TIM6))
		{
			LL_TIM_ClearFlag_UPDATE(TIM6);

			warmup_timer--;
			if(warmup_timer == 0) break;


			TOGGLE_BOARD_LED();

		}
	}
}

void DispatcherRun(void)
{

}
