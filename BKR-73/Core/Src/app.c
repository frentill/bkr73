/*
 * app.c
 *
 *  Created on: Dec 25, 2024
 *      Author: nichikov
 */

#include "app.h"

ApplicationState_t AppState =  {
	.T = 0,
	.Tauz = 10, // ms
	.Remote = {0,0,0,},
	.CH1 = {0,0,0.0f,0.0f,0.0f,},
	.CH2 = {0,0,0.0f,0.0f,0.0f,},
	.Board = {0,0,0.0f,0.0f,0.0f,},
	.Led1s = {0,0,1000,},
	.Led500ms = {0,0,500,},
	.Led100ms = {0,0,100,},
};

void AppUpdateTimers()
{
	AppState.T += AppState.Tauz;

	AppState.Led1s.i += AppState.Tauz;
	if(AppState.Led1s.i >= AppState.Led1s.max) {
		AppState.Led1s.state = (AppState.Led1s.state == 0) ? 1 : 0;
		AppState.Led1s.i = 0;
	}


	AppState.Led500ms.i += AppState.Tauz;
	if(AppState.Led500ms.i >= AppState.Led500ms.max) {
		AppState.Led500ms.state = (AppState.Led500ms.state == 0) ? 1 : 0;
		AppState.Led500ms.i = 0;
	}

	AppState.Led100ms.i += AppState.Tauz;
	if(AppState.Led100ms.i >= AppState.Led100ms.max) {
		AppState.Led100ms.state = (AppState.Led100ms.state == 0) ? 1 : 0;
		AppState.Led100ms.i = 0;
	}

	AppState.Remote.Timer += AppState.Tauz;
	if(AppState.Remote.Timer >= AppState.Remote.TimeToError) {
		AppState.Remote.Timer = AppState.Remote.TimeToError;
		AppState.Remote.IsOK = 0;
	} else {
		AppState.Remote.IsOK = 1;
	}
}
