/*
 * adc_processing.h
 *
 *  Created on: Jan 30, 2025
 *      Author: Piotr Wu
 */

#ifndef INC_ADC_PROCESSING_H_
#define INC_ADC_PROCESSING_H_

#include "stdint.h"

#define ALPHA 0.1f           	// Filter coefficient
#define ADC_2_NR_OF_CONV 5		// Number of ADC conversions
#define NUM_PHOTOREISTS 5  		// Number of photo resistors
#define MIN_STEP_TIM 14			// Duration of step for motors
#define MIN_STEP_TIM_BIG 26		// Duration of step for motors
#define EPSILON 0.01f			// Minimum error value for PID
#define EXTINCTION_VAL 0.95f	// Value to decreasing pid.integral
#define TIM_SYS_TICK_6_7HZ 150
#define PID_INIT { \
    0.5f,               /* kp */ \
    0.1f,               /* ki */ \
    0.01f,              /* kd */ \
    0.0f,               /* previous_error */ \
    0.0f,               /* integral */ \
    1000.0f,            /* max_output */ \
    -1000.0f,           /* min_output */ \
    200.0f,             /* max_integral */ \
    -200.0f,            /* min_integral */ \
    100.0f,             /* max_derivative */ \
    -100.0f             /* min_derivative */ \
}

#define LPF_SCALE 256  // Skalowanie dla wartości alpha (np. 256 = 1.0)

#define LOW_PASS_FILTER_INT(new_value, prev_filtered_value, alpha) \
    (((alpha) * (new_value) + (LPF_SCALE - (alpha)) * (prev_filtered_value)) / LPF_SCALE)


#define LOW_PASS_FILTER(new_value, prev_filtered_value, alpha) \
    ((alpha) * (new_value) + (1.0f - (alpha)) * (prev_filtered_value))


#define MOVING_AVERAGE_INT(new_value, filtered_value, N)   \
    (filtered_value = (filtered_value * (N - 1) + new_value) / N)


#define MOVING_AVERAGE_FLOAT(new_value, filtered_value, N)   \
    (filtered_value = (filtered_value * (N - 1) + new_value) / (float)(N))

typedef struct  {
    float kp, ki, kd;      // PID coefficients (proportional, integral, derivative)
    float previous_error;  // Error from the previous iteration (used for derivative calculation)
    float integral;        // Accumulated integral term (sum of errors over time)
    float max_output;      // Maximum allowed output value
    float min_output;      // Minimum allowed output value
    float max_integral;    // Maximum limit for the integral term (anti-windup)
    float min_integral;    // Minimum limit for the integral term (anti-windup)
    float max_derivative;  // Maximum limit for the derivative term (to prevent spikes)
    float min_derivative;  // Minimum limit for the derivative term (to prevent noise issues)
} PID;

extern uint8_t pid_flag;								// flag
extern uint8_t robot_direction_0;						// flag
extern uint8_t robot_direction_1;						// flag

extern uint16_t max_Val;
extern uint16_t second_max_Val;
extern uint16_t min_Val;

void PID_ProcessAndFilterADCResults(uint16_t *buffer, uint16_t size, float *filtered_value);
int PID_constrain(int value, int min_value, int max_value);
void PID_init(PID* pid, float p, float i, float d, float max_out, float min_out, float max_int, float min_int, float max_deriv, float min_deriv);
float PID_computePID(PID* pid, float setpoint, float input);
int PID_direction(int sensorIndex);
float PID_compute_Error(int max1, int max2, float val1, float val2);

#endif /* INC_ADC_PROCESSING_H_ */
