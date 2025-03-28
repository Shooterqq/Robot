/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "step_motor_driver.h"
#include "audio_save.h"
#include "adc_processing.h"
#include "audio_save.h"
#include "hc_sr_04.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
inline static void calc_motor_ticks(void);

struct us_sensor_str distance_sensor;

static volatile uint_fast32_t mms_ticks = 0;
static volatile uint_fast16_t ms_ticks = 0;

volatile uint16_t adc_buffer[5] = {0};
static volatile float filtered_values[5] = {0};

static volatile bool motor_enable_flag = false;			// flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	ms_ticks++;

	if(ms_ticks == TIM_SYS_TICK_6_7HZ)		// 6,6(7) Hz
	{
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer, ADC_2_NR_OF_CONV);
		ms_ticks = 0;
		pid_flag = 1;
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update and TIM16 interrupts.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger, commutation and TIM17 interrupts.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
	distance_sensor.distance_cm = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
	distance_sensor.distance_cm = hc_sr04_convert_us_to_cm(distance_sensor.distance_cm);
  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel1 global interrupt.
  */
void DMA2_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel1_IRQn 0 */

  /* USER CODE END DMA2_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA2_Channel1_IRQn 1 */

  /* USER CODE END DMA2_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel3 global interrupt.
  */
void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */

  /* USER CODE END DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA2_Channel3_IRQn 1 */

  /* USER CODE END DMA2_Channel3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16)	// 10 kHz for step motor
	{
		static uint32_t cycle_counter = 0;
		cycle_counter++;

        // Motors turned on
        if (motor_enable_flag) {
            if (cycle_counter >= MOTOR_TURN_TIM_ON) {
                cycle_counter = 0;
                motor_enable_flag = false;
            }
        }
        // Motors turned off
        else {
            if (cycle_counter >= MOTOR_TURN_TIM_OFF) {
                cycle_counter = 0;
                motor_enable_flag = true;
            }
        }

		if (motor_enable_flag) {
			switch (SM_STP_MTR_DIRECTION)
			{
			case SM_STP_MTR_DIR_STOP:
				calc_motor_ticks();
				step_wave_motor_stop();
				break;
			case SM_STP_MTR_DIR_FORWARD:
				calc_motor_ticks();
				step_wave_motor_forward();
				break;
			case SM_STP_MTR_DIR_BACKWARD:
				calc_motor_ticks();
				step_wave_motor_backward();
				break;
			case SM_STP_MTR_DIR_LEFT:
				calc_motor_ticks();
				step_wave_motor_left();
				break;
			case SM_STP_MTR_DIR_RIGHT:
				calc_motor_ticks();
				step_wave_motor_right();
				break;
			default:
				calc_motor_ticks();
				step_wave_motor_stop();
				break;
			}
		} else {
			step_wave_motor_stop();		// Break for motors
		}
	}

	if (htim->Instance == TIM17)	// 8 kHz for ADC
	{
//		if(audio_Iterator >= HALF_SEC_SAMPLES)
//		{
//			audio_Iterator = 0;
//		}
//
//		HAL_ADC_Start(&hadc1);
//		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//		{
//			adcValue[audio_Iterator] = HAL_ADC_GetValue(&hadc1);
//		}
//		++audio_Iterator;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
    	// Low pass filtration
        for (uint_fast8_t i = 0; i < ADC_2_NR_OF_CONV; i++)
        {
            filtered_values[i] = LOW_PASS_FILTER((float)adc_buffer[i], filtered_values[i], ALPHA);
        }

    	max_Val = filtered_values[0];
    	second_max_Val = filtered_values[0];
    	min_Val = filtered_values[0];

        for(uint_fast8_t i = 1; i < ADC_2_NR_OF_CONV; i++)		// from i = 1 because upper values are initialised as filtered_values[0] and we skip 1 iteration
        {
        	if (filtered_values[i] > max_Val)
        	{
        	    second_max_Val = max_Val;
        	    max_Val = filtered_values[i];

        	    robot_direction_1 = robot_direction_0;
        	    robot_direction_0 = i;
        	}
        	else if (filtered_values[i] > second_max_Val)
        	{
        	    second_max_Val = filtered_values[i];
        	    robot_direction_1 = i;
        	}

        	if(filtered_values[i] < min_Val)
        	{
        		min_Val = filtered_values[i];
        	}
        }
    }
}

inline static void calc_motor_ticks(void)
{
	mms_ticks++;

	if (mms_ticks == MIN_STEP_TIM_BIG) {
		Flg_Mtr_Drv_Seq.FLAG_D = 0;
		Flg_Mtr_Drv_Seq.FLAG_A = 1;
	} else if (mms_ticks == (MIN_STEP_TIM_BIG * 2)) {
		Flg_Mtr_Drv_Seq.FLAG_A = 0;
		Flg_Mtr_Drv_Seq.FLAG_B = 1;
	} else if (mms_ticks == (MIN_STEP_TIM_BIG * 3)) {
		Flg_Mtr_Drv_Seq.FLAG_B = 0;
		Flg_Mtr_Drv_Seq.FLAG_C = 1;
	} else if (mms_ticks == (MIN_STEP_TIM_BIG * 4)) {
		Flg_Mtr_Drv_Seq.FLAG_C = 0;
		Flg_Mtr_Drv_Seq.FLAG_D = 1;
		mms_ticks = 0;
	}
}



/* USER CODE END 1 */
