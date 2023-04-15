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

// Defined values
#define LAUNCH_PWM 0.75f
#define LAUNCH_DELAY 3000

// Functions
void controller_launch_ball();

void controller_drive(n64_t *n64_state);

#endif /* INC_CONTOLLER_H_ */
