#include "main.h"
#include "solenoid.h"
#include <stdint.h>
#include "stm32f4xx_hal_gpio.h"

solenoid_t solenoid;

void solenoid_init() {
	// Set the GPIO information
	solenoid.GPIOx = GPIOC;
	solenoid.GPIO_Pin = GPIO_PIN_5;

	// Make sure it isn't actuated
	HAL_GPIO_WritePin(solenoid.GPIOx, solenoid.GPIO_Pin, 0);
}

void solenoid_actuate() {
	// Turn on, wait, and then turn off
	decrement_ball_count();
	HAL_GPIO_WritePin(solenoid.GPIOx, solenoid.GPIO_Pin, 1);
	HAL_Delay(SOL_DELAY);
	HAL_GPIO_WritePin(solenoid.GPIOx, solenoid.GPIO_Pin, 0);


}
