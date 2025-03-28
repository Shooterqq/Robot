/*
 * stm_esp_uart.c
 *
 *  Created on: Mar 1, 2025
 *      Author: Piotr Wu
 */
#include <string.h>
#include "stm_esp_uart.h"
#include "step_motor_driver.h"
#include "step_motor_driver.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

uint8_t RxBuffer[STM_RX_BUFFER_SIZE];

volatile Flag_stm_esp_communication Flg_stm_esp_connect = {0};

void stm__start_UART_DMA()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, RxBuffer, STM_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);  // Turn off half buffer callback
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4)
    {
    	Flg_stm_esp_connect.RECEIVED_MSG = 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, RxBuffer, STM_RX_BUFFER_SIZE);	 // Wait for next command
    }
}


void stm__sm_uart_stm_esp(SM_STM_ESP_UART *SM_STM_ESP_RXTX)
{
	switch (*SM_STM_ESP_RXTX)
	{
	case SM_STM_UART_HALT:
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_STOP;
		break;
	case SM_STM_UART_START:		// do wywalenia?
		break;
	case SM_STM_UART_AUTO:
		step_auto_drive_motor();
		break;
	case SM_STM_UART_MANUAL:	// do wywalenia?
		break;
	case SM_STM_UART_MANUAL_FORWARD:
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_FORWARD;
		break;
	case SM_STM_UART_MANUAL_BACKWARD:
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_BACKWARD;
		break;
	case SM_STM_UART_MANUAL_LEFT:
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_RIGHT;
		break;
	case SM_STM_UART_MANUAL_RIGHT:
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_LEFT;
		break;
	case SM_STM_UART_MANUAL_STOP:
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_STOP;
		break;
	default:
		SM_STP_MTR_DIRECTION = SM_STM_UART_HALT;
		break;
	}
}

const CommandMap commandTable[] =
{
	{"STM_HALT", SM_STM_UART_HALT},
	{"STM_START", SM_STM_UART_START},
	{"STM_MANUAL", SM_STM_UART_MANUAL},
	{"STM_FORW", SM_STM_UART_MANUAL_FORWARD},
	{"STM_BACK", SM_STM_UART_MANUAL_BACKWARD},
	{"STM_LEFT", SM_STM_UART_MANUAL_LEFT},
	{"STM_RIGHT", SM_STM_UART_MANUAL_RIGHT},
	{"STM_STOP", SM_STM_UART_MANUAL_STOP},
};

void stm__parser_uart(SM_STM_ESP_UART *SM_STM_ESP_RXTX)
{
	if (Flg_stm_esp_connect.RECEIVED_MSG)
	{
		*SM_STM_ESP_RXTX = SM_STP_MTR_DIR_STOP;   // domyślny (jeśli nie znajdzie)

		for (int i = 0; i < sizeof(commandTable) / sizeof(commandTable[0]); i++)
		{
			if (strncmp((char*)RxBuffer, commandTable[i].cmd, strlen(commandTable[i].cmd)) == 0)
			{
				*SM_STM_ESP_RXTX = commandTable[i].state;
				break;
			}
		}

		memset(RxBuffer, 0, STM_RX_BUFFER_SIZE);
		Flg_stm_esp_connect.RECEIVED_MSG = 0;
	}
}







