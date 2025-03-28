/*
 * step_motor_driver.h
 *
 *  Created on: Jan 17, 2025
 *      Author: Piotr Wu
 */

#ifndef INC_STEP_MOTOR_DRIVER_H_
#define INC_STEP_MOTOR_DRIVER_H_

#include "stm32f3xx_hal.h"

#define MOTOR_TURN_TIM_ON 2000
#define MOTOR_TURN_TIM_OFF 10000

// state machine for driving step motor
typedef enum {
	SM_STP_MTR_DIR_STOP,
	SM_STP_MTR_DIR_FORWARD,
	SM_STP_MTR_DIR_BACKWARD,
	SM_STP_MTR_DIR_LEFT,
	SM_STP_MTR_DIR_RIGHT
}SM_STP_MTR_CONTROLL;

extern volatile SM_STP_MTR_CONTROLL SM_STP_MTR_DIRECTION;

// Flags for step motor sequence
typedef union {
    struct {
        uint8_t FLAG_A : 1;
        uint8_t FLAG_B : 1;
        uint8_t FLAG_C : 1;
        uint8_t FLAG_D : 1;
        uint8_t RESERVED : 4;
    };
    uint8_t all_flags;
} Flag_Motor_Drive_Seq;

extern volatile Flag_Motor_Drive_Seq Flg_Mtr_Drv_Seq;


void step_algorithm_motor(void);

void step_wave_motor_left(void);
void step_wave_motor_right(void);
void step_wave_motor_forward(void);
void step_wave_motor_backward(void);
void step_wave_motor_left_2_test(void);
void step_wave_motor_stop(void);
void step_auto_drive_motor(void);


#endif /* INC_STEP_MOTOR_DRIVER_H_ */
