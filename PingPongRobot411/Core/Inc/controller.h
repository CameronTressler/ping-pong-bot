#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "n64.h"

/*
 * arrows for menu
 * A for some select and launch
 * START for starting
 * B for back
 * JOYSTICK for drive
 *
 * */

// External values

// PWM and delay values. Start value is for generating inertia, other values for actual launch.
#define LAUNCH_START_PWM 0.75f
#define FREEPLAY_LAUNCH_PWM 0.675f
#define INTERVALS_PWM_HIGH 0.615f
#define INTERVALS_PWM_MEDIUM 0.60f
#define INTERVALS_PWM_LOW 0.58f


#define LAUNCH_START_DELAY 1000
#define LAUNCH_DELAY 3000

// Functions
void controller_launch_ball();


void controller_start_launcher(float start_pwm, float launch_pwm);

void controller_drive(n64_t *n64_state);

#endif /* INC_CONTOLLER_H_ */
