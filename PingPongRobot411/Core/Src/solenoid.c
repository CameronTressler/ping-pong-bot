#include "main.h"
#include "solenoid.h"

solenoid_t solenoid;

void solenoid_init() {
	// Set the GPIO information
	solenoid.GPIOx = GPIOA;
	solenoid.GPIO_Pin = 10;

	// Make sure it isn't actuated
	HAL_GPIO_WritePin(solenoid.GPIOx, solenoid.GPIO_Pin, 0);
}

void solenoid_actuate() {
	// Turn on, wait, and then turn off
	HAL_GPIO_WritePin(solenoid.GPIOx, solenoid.GPIO_Pin, 1);
	HAL_Delay(SOL_DELAY);
	HAL_GPIO_WritePin(solenoid.GPIOx, solenoid.GPIO_Pin, 0);

}
