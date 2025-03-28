/*
 * adc_processing.c
 *
 *  Created on: Jan 30, 2025
 *      Author: Piotr Wu
 */

#include "adc_processing.h"

uint8_t pid_flag = 0;								// flag
uint8_t robot_direction_0 = 0;						// flag
uint8_t robot_direction_1 = 0;						// flag

uint16_t max_Val = 0;
uint16_t second_max_Val = 0;
uint16_t min_Val = UINT16_MAX;

void PID_ProcessAndFilterADCResults(uint16_t *buffer, uint16_t size, float *filtered_value)
{
    float average = 0.0f;

    // Calculate the average of the data in the buffer
    for (uint16_t i = 0; i < size; i++) {
        average += buffer[i];
    }
    average /= size;

    // Apply low pass filtering
    *filtered_value = ALPHA * average + (1.0f - ALPHA) * (*filtered_value);
}

// Limiter for for 'value' to the range [min_value, max_value].
int PID_constrain(int value, int min_value, int max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    else if (value > max_value)
    {
        return max_value;
    }
    return value;
}


// Initialisation PID by function
void PID_init(PID* pid, float p, float i, float d, float max_out, float min_out, float max_int, float min_int, float max_deriv, float min_deriv)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->previous_error = 0.0f;
	pid->integral = 0.0f;
	pid->max_output = max_out;
	pid->min_output = min_out;
	pid->max_integral = max_int;
	pid->min_integral = min_int;
	pid->max_derivative = max_deriv;
	pid->min_derivative = min_deriv;
}

// Calculate PID error
float PID_computePID(PID* pid, float setpoint, float input)
{
	// Calculating the real part of error
    float error = setpoint - input;

    // Integral component
    pid->integral += error;
    pid->integral = PID_constrain(pid->integral, pid->min_integral, pid->max_integral);

    // Derivative component
    float derivative = error - pid->previous_error;
    derivative = PID_constrain(derivative, pid->min_derivative, pid->max_derivative);

    // Calculating PID output
    float pid_output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid_output = PID_constrain(pid_output, pid->min_output, pid->max_output);

    // Save current error for next iteration
    pid->previous_error = error;

    return pid_output;
}

// Returns the PID direction based on the sensor index: +1 for right, -1 for left, 0 for center.
int PID_direction(int sensorIndex)
{
    if (sensorIndex == 1 || sensorIndex == 2) return 1;  // Right site → plus
    if (sensorIndex == 3 || sensorIndex == 4) return -1; // Left site → minus
    return 0;  // Mid
}

// Calculates the PID error from the difference between 'val1' and 'val2', taking into account the PID direction.
float PID_compute_Error(int max1, int max2, float val1, float val2)
{
    return (val1 - val2) * PID_direction(max1);
}























//
//
//
//
//
//// Zmienne PID
//float Kp = 1.0f;  // Wzmocnienie proporcjonalne
//float Ki = 0.1f;  // Wzmocnienie całkowe
//float Kd = 0.05f; // Wzmocnienie różniczkowe
//
//float previous_error = 0.0f;
//float integral = 0.0f;
//
//float PID(int error)
//{
//    // Obliczenie składników PID
//    integral += error;
//    float derivative = error - previous_error;
//    previous_error = error;
//
//    // Wyliczenie wyjścia PID
//    return Kp * error + Ki * integral + Kd * derivative;
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//typedef struct {
//    float estimate;      // Aktualne oszacowanie stanu
//    float error_cov;     // Aktualny błąd kowariancji
//    float process_noise; // Szum procesu
//    float measurement_noise; // Szum pomiaru
//} KalmanFilter;
//
///**
// * @brief Inicjalizacja filtra Kalmana
// *
// * @param filter Wskaźnik na strukturę filtra Kalmana
// * @param initial_estimate Początkowe oszacowanie stanu
// * @param process_noise Szum procesu
// * @param measurement_noise Szum pomiaru
// */
//void Kalman_Init(KalmanFilter *filter, float initial_estimate, float process_noise, float measurement_noise) {
//    filter->estimate = initial_estimate;
//    filter->error_cov = 1.0f;  // Domyślnie ustawiamy 1.0 (początkowy błąd kowariancji)
//    filter->process_noise = process_noise;
//    filter->measurement_noise = measurement_noise;
//}
//
///**
// * @brief Zastosowanie filtra Kalmana do aktualizacji stanu na podstawie nowego pomiaru
// *
// * @param filter Wskaźnik na strukturę filtra Kalmana
// * @param measurement Nowy pomiar
// * @return Przefiltrowane oszacowanie stanu
// */
//float Kalman_Update(KalmanFilter *filter, float measurement) {
//    // Przewidywanie
//    float predicted_estimate = filter->estimate;
//    float predicted_error_cov = filter->error_cov + filter->process_noise;
//
//    // Obliczenie współczynnika Kalmana
//    float kalman_gain = predicted_error_cov / (predicted_error_cov + filter->measurement_noise);
//
//    // Korekta stanu i kowariancji błędu
//    filter->estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate);
//    filter->error_cov = (1.0f - kalman_gain) * predicted_error_cov;
//
//    return filter->estimate;
//}














