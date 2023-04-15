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

	// Actuate solenoid
	solenoid_actuate();
	HAL_Delay(1000);

}

void controller_start_launcher(float start_pwm, float launch_pwm) {
	// Start launcher
		set_PWM(hbridges + 2, -1 * start_pwm);
		set_PWM(hbridges + 3, start_pwm);

		// Delay slightly to gather inertia.
		HAL_Delay(LAUNCH_START_DELAY);


		// Spin launcher to final speed
		set_PWM(hbridges + 2, -1 * launch_pwm);
		set_PWM(hbridges + 3, launch_pwm);

		// Delay slightly to get up to speed?
		HAL_Delay(LAUNCH_DELAY);
}

void controller_drive(n64_t *n64_state) {
	float forward = 0.0;
	float left = 0.0;
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



