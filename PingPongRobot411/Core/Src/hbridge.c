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
	hbridges[0].GPIOx = GPIOB;
	hbridges[0].GPIO_Pin = 4;
	hbridges[0].PinState = 0;
	hbridges[0].PWM = 0;

	hbridges[1].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR2_OFFSET);
	hbridges[1].GPIOx = GPIOB;
	hbridges[1].GPIO_Pin = 5;
	hbridges[1].PinState = 0;
	hbridges[1].PWM = 0;

	hbridges[2].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR3_OFFSET);
	hbridges[2].GPIOx = GPIOC;
	hbridges[2].GPIO_Pin = 11;
	hbridges[2].PinState = 0;
	hbridges[2].PWM = 0;

	hbridges[3].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR4_OFFSET);
	hbridges[3].GPIOx = GPIOC;
	hbridges[3].GPIO_Pin = 12;
	hbridges[3].PinState = 0;
	hbridges[3].PWM = 0;
}

void set_PWM(hbridge_t hbridge, float PWM) {
	hbridge.PWM = PWM;

	// If we go forwards
	if (PWM >= 0) {
		*(hbridge.CCR) = (uint32_t)((float) 65335)*PWM;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 1);
	}
}

float get_PWM(hbridge_t hbridge) {
	return hbridge.PWM;
}


