#include "hbridge.h"

/* HBridge Configuration IDs
 *
 *  0 Drive Left
 *  1 Drive Right
 *  2 Launcher Left
 *  3 Launcher Right
 *
 */
hbridge_t hbridges[4];


// init all hbridges with PWM of 0
void init_hbridges() {
	hbridges[0].CCR = (uint32_t*)(TIM4_OFFSET + TIM_CCR1_OFFSET);
	hbridges[0].GPIOx = GPIOA;
	hbridges[0].dir = GPIO_PIN_12;
	hbridges[0].ndir = GPIO_PIN_11;
	hbridges[0].PinState = 0;
	hbridges[0].PWM = 0;

	hbridges[1].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR2_OFFSET);
	hbridges[1].GPIOx = GPIOC;
	hbridges[1].dir = GPIO_PIN_12;
	hbridges[1].ndir = GPIO_PIN_11;
	hbridges[1].PinState = 0;
	hbridges[1].PWM = 0;

	hbridges[2].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR3_OFFSET);
	hbridges[2].GPIOx = GPIOC;
	hbridges[2].dir = GPIO_PIN_10;
	hbridges[2].ndir = GPIO_PIN_8;
	hbridges[2].PinState = 0;
	hbridges[2].PWM = 0;

	hbridges[3].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR4_OFFSET);
	hbridges[3].GPIOx = GPIOC;
	hbridges[3].dir = GPIO_PIN_7;
	hbridges[3].ndir = GPIO_PIN_6;
	hbridges[3].PinState = 0;
	hbridges[3].PWM = 0;
}

void set_PWM(hbridge_t* hbridge, float PWM) {
	if (hbridge->PWM == PWM) {
		return;
	}

	hbridge->PWM = PWM;

	// If we go forwards
	if (PWM >= 0) {
		*(hbridge->CCR) = (uint32_t)(((float) 255)* PWM);
		HAL_GPIO_WritePin(hbridge->GPIOx, hbridge->ndir, 0);
		HAL_GPIO_WritePin(hbridge->GPIOx, hbridge->dir, 1);
		hbridge->PinState = 0;
	} else {
		*(hbridge->CCR) = (uint32_t)(((float) 255)* PWM * -1);
		HAL_GPIO_WritePin(hbridge->GPIOx, hbridge->ndir, 1);
		HAL_GPIO_WritePin(hbridge->GPIOx, hbridge->dir, 0);
		hbridge->PinState = 1;
	}
}

float get_PWM(hbridge_t* hbridge) {
	return hbridge->PWM;
}

int get_dir(hbridge_t* hbridge) {
	return hbridge->PinState;
}


