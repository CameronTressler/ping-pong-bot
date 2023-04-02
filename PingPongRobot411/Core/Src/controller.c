#include "n64.h"
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
	set_PWM(hbridge[0], LAUNCH_PWM);
	set_PWM(hbridge[1], -LAUNCH_PWM);

	// Maybe delay slightly to get up to speed?
	HAL_Delay(LAUNCH_DELAY);

	// Actuate solenoid
	solenoid_actuate();

	// Turn off hbridges
	set_PWM(hbridge[0], 0);
	set_PWM(hbridge[1], 0);
}



