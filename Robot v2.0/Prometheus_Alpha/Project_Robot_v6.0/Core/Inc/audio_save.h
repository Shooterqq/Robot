/*
 * audio_save.h
 *
 *  Created on: Jan 21, 2025
 *      Author: Piotr Wu
 */

#ifndef INC_AUDIO_SAVE_H_
#define INC_AUDIO_SAVE_H_

#include <stdint.h>

#define HALF_SEC_SAMPLES 4000

extern uint16_t adcValue[HALF_SEC_SAMPLES];			// not used
extern volatile uint_fast32_t audio_Iterator;	// not used

#endif /* INC_AUDIO_SAVE_H_ */
