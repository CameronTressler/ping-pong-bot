#ifndef CONTROLLER_H_
#define CONTROLLER_H_

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
#define LAUNCH_PWM 0.5f
#define LAUNCH_DELAY 50

// Functions
void controller_launch_ball();

void controller_drive(); // TODO: need clearer interface with n64

#endif /* INC_CONTOLLER_H_ */
