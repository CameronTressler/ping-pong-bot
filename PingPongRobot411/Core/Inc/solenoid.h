#ifndef INC_SOLENOID_H_
#define INC_SOLENOID_H_

#include <stdint.h>
#include "main.h"

// Define
#define SOL_DELAY 75

// Solenoid
typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} solenoid_t;

extern solenoid_t solenoid;

// Initializes the solenoid
void solenoid_init();

// Moves solenoid in accordance with SOL_DELAY
void solenoid_actuate();

#endif /* INC_SOLENOID_H_ */
