/*
 * app.h
 *
 *  Created on: Dec 25, 2024
 *      Author: nichikov
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"

#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000

                                //    Таблиця. Разові команди вхідні з пульта
                                //    ________________________________________________
                                //   |Біт | Назва  |  Опис                            |
                                //   |----|--------|----------------------------------|
#define REMOTE_ABORT    (BIT0 ) //   | 0  |        |  Скидання                        |
#define REMOTE_PREPARE  (BIT1 ) //   | 1  |        |  Підготовка                      |
#define REMOTE_MC_VC    (BIT2 ) //   | 2  | МЦ/ВЦ  |  МЦ/ВЦ                           |
#define REMOTE_R1       (BIT3 ) //   | 3  | 1      |  1                               |
#define REMOTE_R1_2     (BIT4 ) //   | 4  | 1+2    |  1+2                             |
#define REMOTE_PPZ      (BIT5 ) //   | 5  | ППЗ    |  ППЗ                             |
#define REMOTE_R2       (BIT6 ) //   | 6  | 2      |  2                               |
#define REMOTE_RESERVE  (BIT7 ) //   | 7  |        |  Резерв                          |
                                //   |____|________|__________________________________|


                                //    Таблиця. Разові команди вихідні до пульта
                                //    ________________________________________________
                                //   |Біт | Назва  |  Опис                            |
                                //   |----|--------|----------------------------------|
#define REMOTE_ALLOW    (BIT0 ) //   | 0  |        |  Пуск дозволено                  |
#define REMOTE_CAPTURE  (BIT1 ) //   | 1  |        |  Захоплення                      |
#define REMOTE_CH1      (BIT2 ) //   | 2  |        |  1 кан.                          |
#define REMOTE_CH2      (BIT3 ) //   | 3  |        |  2 кан.                          |
#define REMOTE_WORKING  (BIT4 ) //   | 4  |        |  Справна                         |
#define REMOTE_RESERVE1 (BIT5 ) //   | 5  |        |  Резерв                          |
#define REMOTE_RESERVE2 (BIT6 ) //   | 6  |        |  Резерв                          |
#define REMOTE_RESERVE3 (BIT7 ) //   | 7  |        |  Резерв                          |
                                //   |____|________|__________________________________|

#define REMOTE_RX_BUFFER_SIZE 16


typedef struct RocketChannel_s {
    char Ready;         // Висит
    char HeadCapture;   // ЗГ
    float Omega;
    float Phi;
    float Phi_cor;
    char NP;            // НП  Неуправляемый пуск = АП (Аварийный
    char Fire;          // Пуск
    char ZD;            // ЗД   Запуск двигателя ракеты
    char Health;        // Испр
    char OK;            // ОК Открытие клапана = Подготовка
    char PPS;           // ППС

    char OPC;           // ОПЦ  Отмена ПЦ ?? Будет КЦУ?
    char VSK27;         // +27ВСК
    char V115;          // 115V
}RocketChannel_t;


typedef struct BoardChannel_s {
    char EmergencyFire; // Пуск3 ???
    float Distance;
    float Azimuth;
    float Epsilon;
    char APZ;           // АПЗ
}BoardChannel_t;

typedef struct RemoteControl_s {
	char IsOK;
	uint32_t Timer;
	uint32_t TimeToError;
	char Abort;
	char Prepare;
	char MC;
	char VC;
	char R1;
	char R1_2;
	char R2;
	char PPZ;
	uint8_t rxIndex;
	uint8_t rxBuffer[REMOTE_RX_BUFFER_SIZE];
}RemoteControl_t;

typedef struct LedControl_s {
	char state;
	uint32_t i;
	uint32_t max;
}LedControl_t;


typedef enum LedItem_e {
    LED_MODE_DISABLED = 0,
    LED_MODE_FOREVER  = 1,
    LED_MODE_1s       = 2,
    LED_MODE_500ms    = 3,
    LED_MODE_100ms    = 4,
} LedItem_e_t;


#define UPDATE_LED_DATA(data, led_mode, led_flag)                 \
    do {                                                          \
        switch (led_mode) {                                       \
            case LED_MODE_FOREVER:                                \
                data |= led_flag;                                 \
                break;                                            \
            case LED_MODE_1s:                                     \
                if(AppState.Led1s.state)        data |= led_flag; \
                break;                                            \
            case LED_MODE_500ms:                                  \
                if(AppState.Led500ms.state)     data |= led_flag; \
                break;                                            \
            case LED_MODE_100ms:                                  \
                if(AppState.Led100ms.state)     data |= led_flag; \
                break;                                            \
        }                                                         \
    } while(0)

#define DIGITAL_MUX_PROCESS_PIN(pin, filter_idx, var_ptr)             \
    if (HAL_GPIO_ReadPin(pin##_GPIO_Port, pin##_Pin)) {               \
        if (filters[a][filter_idx] <= TSHi) filters[a][filter_idx]++; \
        else *(var_ptr) = 1;                                          \
    } else {                                                          \
        if (filters[a][filter_idx] >= TSLo) filters[a][filter_idx]--; \
        else *(var_ptr) = 0;                                          \
    }


typedef struct LedList_s {
	char Allow;
	char Capture;
	char CH1;
	char CH2;
	char Working;
}LedList_t;

typedef struct ApplicationState_s {
	uint32_t T;
	uint32_t Tauz;
	RemoteControl_t Remote;
	RocketChannel_t CH1;
	RocketChannel_t CH2;
	BoardChannel_t Board;
	LedControl_t Led1s;
	LedControl_t Led500ms;
	LedControl_t Led100ms;
	LedList_t Leds;
	char DigitalAddress;
}ApplicationState_t;

extern ApplicationState_t AppState;

void AppReadRemoteData();
void AppSendRemoteData();
void AppUpdateTimers();
void AppAcqireDigitalData();



//*************************************************************************
// Function set addresses on external digital multiplexer                 *
// By gpio. Argument range is 0...8                                       *
//                                                                        *
//*************************************************************************
static inline void AppSetDigitalMux(uint16_t channel)
{
    if(channel & BIT0) HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_SET);
    else               HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_RESET);


    if(channel & BIT1) HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
    else               HAL_GPIO_WritePin(A1_GPIO_Port, A2_Pin, GPIO_PIN_RESET);


    if(channel & BIT2) HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
    else               HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
}

#endif /* INC_APP_H_ */
