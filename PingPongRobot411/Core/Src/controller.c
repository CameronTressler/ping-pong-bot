#include "n64.h"
#include "controller.h"
#include "drive.h"
#include "hbridge.h"
#include "serialDisplay.h"
#include "main.h"
#include "solenoid.h"

// External variables
extern hbridge_t hbridges[4];
extern solenoid_t solenoid;

// Launch one ball
void controller_launch_ball() {
	// Adjust display
	decrement_ball_count();

	// Start launcher
	set_PWM(hbridges[0], LAUNCH_PWM);
	set_PWM(hbridges[1], -LAUNCH_PWM);

	// Maybe delay slightly to get up to speed?
	//HAL_Delay(LAUNCH_DELAY);

	// Actuate solenoid
	solenoid_actuate();

	// Turn off hbridges
	set_PWM(hbridges[0], 0);
	set_PWM(hbridges[1], 0);
}

void controller_drive(n64_t *n64_state) {
}



