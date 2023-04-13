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
	set_PWM(hbridges + 2, LAUNCH_PWM);
	set_PWM(hbridges + 3, -LAUNCH_PWM);

	// Maybe delay slightly to get up to speed?
	HAL_Delay(LAUNCH_DELAY);

	// Actuate solenoid
	solenoid_actuate();
	HAL_Delay(LAUNCH_DELAY/3);

	// Turn off hbridges
	set_PWM(hbridges + 2, 0);
	set_PWM(hbridges + 3, 0);
}

void controller_drive(n64_t *n64_state) {
	float forward, left;
	  if (n64_state->button_status[N64_DU] == n64_state->button_status[N64_DD]) {
		  forward = 0;
	  }
	  else if (n64_state->button_status[N64_DU]) {
		  forward = 1;
	  }
	  else if (n64_state->button_status[N64_DD]){
		  forward = -1;
	  }

	  if (n64_state->button_status[N64_L] == n64_state->button_status[N64_R]) {
		  left = 0;
	  }
	  else if (n64_state->button_status[N64_L]) {
		  left = 1;
	  }
	  else if (n64_state->button_status[N64_R]) {
		  left = -1;
	  }

	  // TODO: consider using constant values for forward/backward + turning at the same time
	  // could have smooth predetermined values instead of drive logic.

	  safe_drive(forward, left);
}



