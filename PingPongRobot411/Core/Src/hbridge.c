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
	hbridges[0].PWM = 0;
	hbridge_t hbridgelol = hbridges[0];

	hbridges[1].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR2_OFFSET);
	hbridges[1].PWM = 0;
	hbridgelol = hbridges[1];

	hbridges[2].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR3_OFFSET);
	hbridges[2].PWM = 0;
	hbridgelol = hbridges[2];

	hbridges[3].CCR = (uint32_t*) (TIM4_OFFSET + TIM_CCR4_OFFSET);
	hbridges[3].PWM = 0;
	hbridgelol = hbridges[3];
}

void set_PWM(hbridge_t hbridge, float PWM) {
	hbridge.PWM = PWM;

	*(hbridge.CCR) = (uint32_t)((float) 65335)*PWM;
}

float get_PWM(hbridge_t  hbridge) {
	return hbridge.PWM;
}


