/*
 * dispatcher.h
 *
 *  Created on: Dec 24, 2024
 *      Author: Theta
 */

#ifndef INC_DISPATCHER_H_
#define INC_DISPATCHER_H_

#define TOGGLE_BOARD_LED()       ledCnt += Tauz;                          \
			                     if (ledCnt >= ledPeriod)                 \
			                     {                                        \
			                         LL_GPIO_TogglePin(GPIOC, SD_Pin);    \
			                         ledCnt = 0;                          \
			                     }

#include "main.h"

void DispatcherInit(void);

void DispatcherRun(void);

#endif /* INC_DISPATCHER_H_ */
