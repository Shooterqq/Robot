/*
 * audio_save.c
 *
 *  Created on: Jan 21, 2025
 *      Author: Piotr Wu
 */
#include "audio_save.h"

uint16_t adcValue[HALF_SEC_SAMPLES] = {0};			// not used
volatile uint_fast32_t audio_Iterator = 0;	// not used
