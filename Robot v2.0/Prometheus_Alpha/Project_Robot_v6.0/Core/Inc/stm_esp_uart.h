/*
 * stm_esp_uart.h
 *
 *  Created on: Mar 1, 2025
 *      Author: Piotr Wu
 */

#ifndef INC_STM_ESP_UART_H_
#define INC_STM_ESP_UART_H_

#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart_ex.h"

#define STM_RX_BUFFER_SIZE 16

// Flags for stm32 - esp8266 communication
typedef union {
    struct {
        uint8_t RECEIVED_MSG : 1;
        uint8_t RESERVED : 7;
    };
    uint8_t all_flags;
} Flag_stm_esp_communication;

extern volatile Flag_stm_esp_communication Flg_stm_esp_connect;

typedef enum{
	SM_STM_UART_HALT,
	SM_STM_UART_START,
	SM_STM_UART_AUTO,
	SM_STM_UART_MANUAL,
	SM_STM_UART_MANUAL_FORWARD,
	SM_STM_UART_MANUAL_BACKWARD,
	SM_STM_UART_MANUAL_LEFT,
	SM_STM_UART_MANUAL_RIGHT,
	SM_STM_UART_MANUAL_STOP
}SM_STM_ESP_UART;

typedef struct {
	const char* cmd;
	SM_STM_ESP_UART state;
} CommandMap;

void stm__start_UART_DMA();
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void stm__parser_uart(SM_STM_ESP_UART *SM_STM_ESP_RXTX);
void stm__sm_uart_stm_esp(SM_STM_ESP_UART *SM_STM_ESP_RXTX);

#endif /* INC_STM_ESP_UART_H_ */
