/*
 * step_motor_driver.c
 *
 *  Created on: Jan 17, 2025
 *      Author: Piotr Wu
 */

//typedef struct {
//	GPIO_TypeDef *port; // Wskaźnik na port (np. GPIOA, GPIOB)
//	uint16_t pin;       // Numer pinu (np. GPIO_PIN_0, GPIO_PIN_1)
//} GPIO_Pins;
//
//GPIO_Pins pins[] =
//{
//    {GPIOA, GPIO_PIN_0}, // Pin PA0
//    {GPIOA, GPIO_PIN_1}, // Pin PA1
//    {GPIOB, GPIO_PIN_0}, // Pin PB0
//    {GPIOB, GPIO_PIN_1}  // Pin PB1
//};

#include "hc_sr_04.h"
#include "adc_processing.h"
#include"step_motor_driver.h"
#include "i2c-lcd.h"
#include <main.h>
#include <math.h>

volatile Flag_Motor_Drive_Seq Flg_Mtr_Drv_Seq = {0};

// Flags for driving step motors
volatile SM_STP_MTR_CONTROLL SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_STOP;


void step_algorithm_motor(void)	//full step
{
	if(Flg_Mtr_Drv_Seq.FLAG_A)
	{
		HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_SET);
	}
	else if(Flg_Mtr_Drv_Seq.FLAG_B)
	{
		HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_SET);
	}
	else if(Flg_Mtr_Drv_Seq.FLAG_C)
	{
		HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	}
	else if(Flg_Mtr_Drv_Seq.FLAG_D)
	{
		HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
	}
}

void step_wave_motor_left(void)	// wave drive
{
	  if(Flg_Mtr_Drv_Seq.FLAG_A){
		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_SET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_B){
		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_C){
		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_D){
		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_SET);
	  }
}


void step_wave_motor_right(void)	// wave drive
{
	  if(Flg_Mtr_Drv_Seq.FLAG_A){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_SET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_B){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_SET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_C){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_D){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
}

void step_wave_motor_forward(void)	// wave drive
{
	  if(Flg_Mtr_Drv_Seq.FLAG_A){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_SET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_SET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_B){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_SET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_C){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_D){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
}

void step_wave_motor_backward(void)	// wave drive
{
	  if(Flg_Mtr_Drv_Seq.FLAG_A){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_SET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_SET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_B){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_SET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_C){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  }
	  else if(Flg_Mtr_Drv_Seq.FLAG_D){
		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_SET);
	  }
}

void step_wave_motor_left_2_test(void)	// wave drive
{
	  if(Flg_Mtr_Drv_Seq.FLAG_A)
	  	{
	  		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
	  	}
	  	else if(Flg_Mtr_Drv_Seq.FLAG_B)
	  	{
	  		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
	  	}
	  	else if(Flg_Mtr_Drv_Seq.FLAG_C)
	  	{
	  		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
	  	}
	  	else if(Flg_Mtr_Drv_Seq.FLAG_D)
	  	{
	  		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_SET);
	  	}
}
void step_wave_motor_stop(void)	// wave drive
{
		  HAL_GPIO_WritePin(Mtr_2_Green_GPIO_Port, Mtr_2_Green_Pin, GPIO_PIN_RESET);		// Right
		  HAL_GPIO_WritePin(Mtr_2_Black_GPIO_Port, Mtr_2_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Blue_GPIO_Port, Mtr_2_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_2_Red_GPIO_Port, Mtr_2_Red_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(Mtr_1_Green_GPIO_Port, Mtr_1_Green_Pin, GPIO_PIN_RESET);		// Left
		  HAL_GPIO_WritePin(Mtr_1_Black_GPIO_Port, Mtr_1_Black_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Blue_GPIO_Port, Mtr_1_Blue_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Mtr_1_Red_GPIO_Port, Mtr_1_Red_Pin, GPIO_PIN_RESET);
}

float PID_err = 0;
float PID_out_val = 0;
PID pid = PID_INIT;
extern struct us_sensor_str distance_sensor;

void step_auto_drive_motor(void)
{
	if (pid_flag) {
		PID_err = PID_compute_Error(robot_direction_0, robot_direction_1, max_Val, second_max_Val);
		PID_out_val = PID_computePID(&pid, 0, PID_err);

		if (fabs(PID_err) < EPSILON) {
			pid.integral *= EXTINCTION_VAL;	// Gradual extinction of the PID controller integral
		}
		pid_flag = 0;
	}

	if (distance_sensor.distance_cm > 20) {
		if ((PID_out_val <= 10) && (PID_out_val >= -10)) {
			SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_FORWARD;
			//			  lcd_clear();
			//			  lcd_send_string("Direction:");
			//			  lcd_put_cur(1, 0);
			//			  lcd_send_string("forward");
			// w zaleznosci od output czas wlaczenia timera
			// albo w petli for
		} else if (PID_out_val < 10) {
			SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_LEFT;
			//			  lcd_clear();
			//			  lcd_send_string("Direction:");
			//			  lcd_put_cur(1, 0);
			//			  lcd_send_string("right");
		} else if (PID_out_val > 10) {
			SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_RIGHT;
			//			  lcd_clear();
			//			  lcd_send_string("Direction:");
			//			  lcd_put_cur(1, 0);
			//			  lcd_send_string("left");
		}
	} else {
		SM_STP_MTR_DIRECTION = SM_STP_MTR_DIR_STOP;

		lcd_clear();			// wyswietlanie w timerze bo za szybko
		lcd_put_cur(0, 0);
		lcd_send_string(" LIGHT FOUNDED!");
		lcd_put_cur(1, 0);
		lcd_send_string("  Gratulation!");
	}
}
