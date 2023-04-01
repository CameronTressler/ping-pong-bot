#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "n64.h"
#include "drive.h"
#include "serialDisplay.h"

/*
 * arrows for menu
 * A for some select and launch
 * START for starting
 * B for back
 * JOYSTICK for drive
 *
 * */

void controller_launch_ball();

void controller_drive();

void controller_menu();

#endif /* INC_CONTOLLER_H_ */
