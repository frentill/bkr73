/*
 * dispatcher.h
 *
 *  Created on: Dec 24, 2024
 *      Author: Theta
 */

#ifndef INC_DISPATCHER_H_
#define INC_DISPATCHER_H_


#include "main.h"

void Dispatcher_Thread_A0(void);
void Dispatcher_Thread_B0(void);

extern void (*Dispatcher_Thread_Ptr)(void);

void DispatcherInit(void);

void DispatcherRun(void);
#endif /* INC_DISPATCHER_H_ */
