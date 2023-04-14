#include "drive.h"
#include "hbridge.h"
#include "ultrasonic.h"

#include <math.h>
#include <stdio.h>

float max_magnitude(float a, float b) {
	if (fabs(a) > fabs(b)) {
		return fabs(a);
	}
	return fabs(b);
}

#define DEADZONE_THRESHOLD 0.05

float adjust_deadzone(float joystick) {
	float mag = fabs(joystick);

	if (mag < DEADZONE_THRESHOLD) {
		return 0.0;
	}

	float new_mag = (mag - DEADZONE_THRESHOLD) / (1 - DEADZONE_THRESHOLD);
	return new_mag * (joystick / mag);
}

void safe_drive(float lin_forward, float rot_left) {
	lin_forward = adjust_deadzone(lin_forward);
	rot_left = adjust_deadzone(rot_left);

	// Stop the robot from driving off the table.
	if (lin_forward > 0 && is_off_table(ultras + FRONT_ULTRA)) {
		lin_forward = 0;
	}
	else if (lin_forward < 0 && is_off_table(ultras + REAR_ULTRA)) {
		lin_forward = 0;
	}

	// Convert to l/r drive.
	float left_wheel = lin_forward - rot_left / 1.5;
	float right_wheel = lin_forward + rot_left / 1.5;

	// Adjust values if conversion to l/r drive pushed magnitude over 1.0.
	float max_mag = max_magnitude(left_wheel, right_wheel);
	if (max_mag > 1.0) {
		left_wheel /= max_mag;
		right_wheel /= max_mag;
	}

	left_wheel *= 0.35;
	right_wheel *= 0.35;

	//printf("LW: %.3f\t\tRW: %.3f\n\r", left_wheel, right_wheel);

	// Command HBridges.
	set_PWM(hbridges + 0, left_wheel);
	set_PWM(hbridges + 1, right_wheel);
}
