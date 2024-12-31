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
	.Leds = {0,0,LED_MODE_FOREVER,LED_MODE_FOREVER,LED_MODE_1s,},
	.DigitalAddress = 0,
};
void AppReadRemoteData()
{
	while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) && AppState.Remote.rxIndex < REMOTE_RX_BUFFER_SIZE) {
	    // Read the received byte and store it in the buffer
	    AppState.Remote.rxBuffer[AppState.Remote.rxIndex++] = (uint8_t)huart4.Instance->DR;


	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
			__HAL_UART_CLEAR_OREFLAG(&huart4); // Clear overrun error flag
		}
	}


	if(AppState.Remote.rxIndex >= 1)
	{
		uint8_t cmd = AppState.Remote.rxBuffer[0];

		AppState.Remote.Abort   = (cmd & REMOTE_ABORT)   ? 1 : 0;
		AppState.Remote.Prepare = (cmd & REMOTE_PREPARE) ? 0 : 1;
		AppState.Remote.MC      = (cmd & REMOTE_MC_VC)   ? 1 : 0;
		AppState.Remote.VC      = (cmd & REMOTE_MC_VC)   ? 0 : 1;
		AppState.Remote.R1      = (cmd & REMOTE_R1)      ? 1 : 0;
		AppState.Remote.R1_2    = (cmd & REMOTE_R1_2)    ? 1 : 0;
		AppState.Remote.R2      = (cmd & REMOTE_R2)      ? 1 : 0;
		AppState.Remote.PPZ     = (cmd & REMOTE_PPZ)     ? 0 : 1;
		AppState.Remote.Timer = 0;   // reset error timer countdown
		AppState.Remote.IsOK = 1;
	}

	AppState.Remote.rxIndex = 0;
}

void AppSendRemoteData()
{
	HAL_GPIO_WritePin(GPIOB, REMOTE_DE1_Pin, GPIO_PIN_SET);

	uint8_t data = 0;

	UPDATE_LED_DATA(data, AppState.Leds.Allow, REMOTE_ALLOW);
	UPDATE_LED_DATA(data, AppState.Leds.Capture, REMOTE_CAPTURE);
	UPDATE_LED_DATA(data, AppState.Leds.CH1, REMOTE_CH1);
	UPDATE_LED_DATA(data, AppState.Leds.CH2, REMOTE_CH2);
	UPDATE_LED_DATA(data, AppState.Leds.Working, REMOTE_WORKING);

	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE)) {
	    HAL_UART_Transmit(&huart4, &data, 1, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(GPIOB, REMOTE_DE1_Pin, GPIO_PIN_RESET);
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


// Acquiring and glitch filtering binary data connected to gpio via digital muxes
void AppAcqireDigitalData(void){

	static volatile char filters[8][3] = {
		    {0, 0, 0},
		    {0, 0, 0},
		    {0, 0, 0},
		    {0, 0, 0},
		    {0, 0, 0},
		    {0, 0, 0},
		    {0, 0, 0},
		    {0, 0, 0}
		};

	const char TSHi = 9;
	const char TSLo = 1;

	uint16_t a = AppState.DigitalAddress;

	switch( a ){
	case 0 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.NP);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.NP);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.CH1.OPC);
		break;
	}
	case 1 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.Fire);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.Fire);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.CH1.VSK27);
		break;
	}
	case 2 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.ZD);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.ZD);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.CH1.V115);
		break;
	}
	case 3 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.Ready);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.Ready);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.CH2.OPC);
		break;
	}
	case 4 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.Health);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.Health);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.CH2.VSK27);
		break;
	}
	case 5 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.HeadCapture);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.HeadCapture);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.CH2.V115);
		break;
	}
	case 6 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.OK);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.OK);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.Board.APZ);
		break;
	}
	case 7 :
	{
		DIGITAL_MUX_PROCESS_PIN(Q0, 0, &AppState.CH1.PPS);
		DIGITAL_MUX_PROCESS_PIN(Q1, 1, &AppState.CH2.PPS);
		DIGITAL_MUX_PROCESS_PIN(Q2, 2, &AppState.Board.EmergencyFire);
		break;
	}
	}

	AppState.DigitalAddress ++;
	if(AppState.DigitalAddress > 7) AppState.DigitalAddress = 0;
    AppSetDigitalMux(AppState.DigitalAddress);
}
