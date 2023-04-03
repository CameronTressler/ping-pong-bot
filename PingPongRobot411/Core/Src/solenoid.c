#include "main.h"
#include "solenoid.h"
#include <stdint.h>
#include "stm32f4xx_hal_gpio.h"

solenoid_t solenoid;

void solenoid_init() {
	// Set the GPIO information
	solenoid.GPIOx = GPIOA;
	solenoid.GPIO_Pin = GPIO_PIN_10;

	// Make sure it isn't actuated
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10, 0);
}

void solenoid_actuate() {
	// Turn on, wait, and then turn off
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	HAL_Delay(SOL_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

}
