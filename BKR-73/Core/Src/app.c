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
	.Remote = {0,0,0,0,0,0,0,0,0,0,0,0,},
	.CH1 = {0,0,0.0f,0.0f,0.0f,},
	.CH2 = {0,0,0.0f,0.0f,0.0f,},
	.Board = {0,0,0.0f,0.0f,0.0f,},
	.Led1s = {0,0,1000,},
	.Led500ms = {0,0,500,},
	.Led100ms = {0,0,100,},
	.Leds = {0,0,0,0,0,},
};
void AppReadRemoteData()
{
	while (LL_USART_IsActiveFlag_RXNE(UART4) && AppState.Remote.rxIndex < REMOTE_RX_BUFFER_SIZE) {
		// Read the received byte and store it in the buffer
		AppState.Remote.rxBuffer[AppState.Remote.rxIndex++] = LL_USART_ReceiveData8(UART4);
	}

	if(AppState.Remote.rxIndex >= 1)
	{
		uint8_t cmd = AppState.Remote.rxBuffer[0];

		AppState.Remote.Abort   = (cmd | REMOTE_ABORT)   ? 1 : 0;
		AppState.Remote.Prepare = (cmd | REMOTE_PREPARE) ? 1 : 0;
		AppState.Remote.MC      = (cmd | REMOTE_MC_VC)   ? 1 : 0;
		AppState.Remote.VC      = (cmd | REMOTE_MC_VC)   ? 0 : 1;
		AppState.Remote.R1      = (cmd | REMOTE_R1)      ? 1 : 0;
		AppState.Remote.R1_2    = (cmd | REMOTE_R1_2)    ? 1 : 0;
		AppState.Remote.R2      = (cmd | REMOTE_R2)      ? 1 : 0;
		AppState.Remote.PPZ     = (cmd | REMOTE_PPZ)     ? 1 : 0;

		AppState.Remote.rxIndex = 0;
		AppState.Remote.Timer = 0;   // reset error timer countdown
	}
}

void AppSendRemoteData()
{
	uint8_t data = 0;

	UPDATE_LED_DATA(data, AppState.Leds.Allow, REMOTE_ALLOW);
	UPDATE_LED_DATA(data, AppState.Leds.Capture, REMOTE_CAPTURE);
	UPDATE_LED_DATA(data, AppState.Leds.CH1, REMOTE_CH1);
	UPDATE_LED_DATA(data, AppState.Leds.CH2, REMOTE_CH2);
	UPDATE_LED_DATA(data, AppState.Leds.Working, REMOTE_WORKING);

	if (LL_USART_IsActiveFlag_TXE(UART4)) {
		LL_USART_TransmitData8(UART4, data);
	}
}

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
