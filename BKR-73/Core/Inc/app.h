/*
 * app.h
 *
 *  Created on: Dec 25, 2024
 *      Author: nichikov
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"

typedef struct RocketChannel_s {
    char Ready;
    char HeadCapture;
    float Omega;
    float Phi;
    float Phi_cor;
}RocketChannel_s;


typedef struct BoardChannel_s {
    char Fire;
    char EmergencyFire;
    float Distance;
    float Azimuth;
    float Epsilon;
}RocketChannel_t;

typedef struct RemoteControl_s {
	char IsOK;
	uint32_t Timer;
	uint32_t TimeToError;
}RemoteControl_t;

typedef struct LedControl_s {
	char state;
	uint32_t i;
	uint32_t max;
}LedControl_t;

typedef struct ApplicationState_s {
	uint32_t T;
	uint32_t Tauz;
	RemoteControl_t Remote;
	RocketChannel_s CH1;
	RocketChannel_s CH2;
	RocketChannel_t Board;
	LedControl_t Led1s;
	LedControl_t Led500ms;
	LedControl_t Led100ms;
}ApplicationState_t;

extern ApplicationState_t AppState;

void AppUpdateTimers();

#endif /* INC_APP_H_ */
