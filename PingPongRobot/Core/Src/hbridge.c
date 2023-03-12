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



void init_hbridge(hbridge_t hbridge, uint16_t CCR) {
	hbridge->CCR = CCR;
	hbridge->PWM = 0;
}

void set_PWM(hbridge_t hbridge, float PWM) {

}


