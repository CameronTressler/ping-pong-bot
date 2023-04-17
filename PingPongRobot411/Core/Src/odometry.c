#include "odometry.h"

#include <math.h>
#include <stdlib.h>
#include "stdio.h"
#include "util.h"

void init_imu() {
	// Set to config operational mode.
	i2c_mask_write(IMU_ADDR, OPR_MODE_REG, 4, 0, 0b0000);
	HAL_Delay(20);

	// Set to normal power mode.
	i2c_mask_write(IMU_ADDR, PWR_MODE_REG, 2, 0, 0b00);

	// Set to use radians and mg.
	i2c_mask_write(IMU_ADDR, UNIT_SEL_REG, 3, 0, 0b111);

	// Set to NDOF operational mode.
	i2c_mask_write(IMU_ADDR, OPR_MODE_REG, 4, 0, 0b1000);
	HAL_Delay(20);
}


int is_imu_calibrated(imu_calib_t* calib) {
	if (calib->age > CALIB_LIFE) {
		uint8_t data;
		i2c_read(IMU_ADDR, CALIB_STAT_REG, &data, 1);

		calib->data = data;
		calib->age = 0;

		//printf("Calibration: %d\n\r", data);
	}

	++calib->age;

	// Return whether accelerometer and gyro are calibrated.
	// Magnetometer calibration is very flaky and doesn't seem to matter.

	if ((calib->data & 0b11111111) >= 60) {
		return 1;
	}
	return 0;
}

void init_odom(odom_t* odom) {
	odom->cur_pos.x = 0.0;
	odom->cur_pos.y = 0.0;
	odom->cur_pos.heading = 0.0;
	odom->velocity = 0.0;

	for (int i = 0; i < 4; ++i) {
		odom->corners[i].x = 0.0;
		odom->corners[i].y = 0.0;
	}

}

void update_position(odom_t* odom, double dist) {
	odom->cur_pos.x += cos(odom->cur_pos.heading) * dist;
	odom->cur_pos.y += sin(odom->cur_pos.heading) * dist;
}

void reset_velocity(odom_t* odom, uint8_t num_iterations) {
	// Backtrack the distance covered in the last num_iterations.
	if (num_iterations > 0) {
		double dist = -1 * odom->velocity * num_iterations / IMU_UPDATE_RT;
		update_position(odom, dist);
	}

	odom->velocity = 0.0;
}

double predict_velocity(double prev_vel, double left_cmd, double right_cmd) {
	// Find the average wheel command.
	double avg_cmd = (left_cmd + right_cmd) / 2.0;

	// Approximate the velocity we asymptotically would reach.
	// TODO: Check if positive is forward.
	double target_vel = avg_cmd * CMD_TO_VEL;

	// Find a predicted velocity, factoring in acceleration.
	double new_vel;
	if (target_vel > prev_vel) {
		if (prev_vel > 0.0) {
			new_vel = prev_vel + DELTA_ACCEL;
		}
		else {
			new_vel = prev_vel + DELTA_DECCEL;
		}

		if (new_vel > target_vel) {
			return target_vel;
		}
	}
	else {
		if (prev_vel < 0.0) {
			new_vel = prev_vel - DELTA_ACCEL;
		}
		else {
			new_vel = prev_vel - DELTA_DECCEL;
		}

		if (new_vel < target_vel) {
			return target_vel;
		}
	}

	return new_vel;
}

void update_odom(odom_t* odom, hbridge_t* hbridges, ultra_t* ultras) {
//	if (!is_imu_calibrated(&calibration)) {
//		return;
//	}
	is_imu_calibrated(&calibration);

	imu_raw_data_t yaw;
	// imu_raw_data_t lin_accel;

	// Update heading in radians.
	i2c_read(IMU_ADDR, EUL_DATA_X, yaw.buf, 2);
	odom->cur_pos.heading = yaw.data.datum / 900.0;

	// If commanded velocity is zero, assume actual velocity is zero.
//	if (fabs(get_PWM(hbridges[0])) < 0.001 && fabs(get_PWM(hbridges[1])) < 0.001) {
//		reset_velocity(odom, 0);
//		return;
//	}

	// Get acceleration.
	// i2c_read(IMU_ADDR, LIA_DATA_X, lin_accel.buf, 2);

	// Update acceleration in m/s^2.
	// double accel = lin_accel.data.datum / 101.971621;
	// double measured_velocity = odom->velocity + (accel / IMU_UPDATE_RT);

	// Given odom->velocity as our previous velocity, use measured acceleration and predicted
	// velocity to obtain a new velocity estimate.
	double predicted_velocity =
			predict_velocity(odom->velocity, get_PWM(hbridges + 0), get_PWM(hbridges + 1));

	// odom->velocity = measured_velocity;
	odom->velocity = predicted_velocity;
//	odom->velocity = (PREDICTED_RATIO * predicted_velocity) +
//			   	     ((1 - PREDICTED_RATIO) * measured_velocity);

	update_position(odom, odom->velocity / IMU_UPDATE_RT);

	// Correct position.
//	if (ultras_off_table()) {
//
//	}

	++(odom->i);
	if (odom->i % 25 == 0) {
		printf("%d : %f : %f : %f : %f\n\r", calibration.data, odom->cur_pos.heading, odom->velocity, odom->cur_pos.x, odom->cur_pos.y);
	}
}

//void calibrate_corner(odom_t* odom, uint8_t corner_num) {
//	odom->x_corner[corner_num] = odom->x_rel;
//	odom->y_corner[corner_num] = odom->y_rel;
//}
//
//double distance_to_line(odom_t* odom, uint8_t c_1, uint8_t c_2) {
//	double numerator = fabs(
//		(odom->x_corner[c_2] - odom->x_corner[c_1]) *
//		(odom->y_corner[c_1] - odom->y_rel) -
//		(odom->x_corner[c_1] - odom->x_rel) *
//		(odom->y_corner[c_2] - odom->y_corner[c_1])
//	);
//
//	double denominator = pow((odom->x_corner[c_2] - odom->x_corner[c_1]), 2);
//
//
//}
//
//void adjust_off_table(odom_t* odom) {
//	// Find closest edge of table.
//
//
//}

odom_t odometry;
imu_calib_t calibration;

void reset_path(path_t* path) {
	path->num_valid = 0;
	path->current_setpoint = 0;
	path->pb_state = TURN;
}

void add_setpoint(odom_t* odom, path_t* path) {
	path->setpoints[path->num_valid].x = odom->cur_pos.x;
	path->setpoints[path->num_valid].y = odom->cur_pos.y;
	path->setpoints[path->num_valid].heading = odom->cur_pos.heading;
	++path->num_valid;
}

void set_playback_cmds(odom_t* odom, path_t* path, display_t* display) {
	switch(path->pb_state) {
		case TURN: {
			// Get angle to point relative to current heading, from 0 to 2PI.
			double angle_diff = get_angle_to_setpoint(odom, path);

			// If we are close to angle, move on to DRIVE
			if (angle_diff < ANGLE_THRESHOLD || angle_diff > 2 * PI - ANGLE_THRESHOLD) {
				path->pb_state = DRIVE;
			}
			// If we need to turn counter clockwise.
			else if (
				(0 < angle_diff && angle_diff < PI / 2) ||
				(PI < angle_diff && angle_diff < 3 * PI / 2)
			) {
				safe_drive(0.0f, 1.0f);
			}
			// Else we need to turn clockwise.
			else {
				safe_drive(0.0f, -1.0f);
			}

			
			break;
		}
		case DRIVE: {
			// Get angle to point relative to current heading, from 0 to 2PI.
			double angle_diff = get_angle_to_setpoint(odom, path);

			safe_drive(1, 0);

			if (get_distance_to_setpoint(odom, path) < DIST_THRESHOLD) {
				safe_drive(0, 0);
				path->pb_state = LAUNCH;
			}
			else if (!(angle_diff < MAX_ACCEPTABLE_ANGLE ||
					   angle_diff > 2 * PI - MAX_ACCEPTABLE_ANGLE)) {
				safe_drive(0, 0);
				path->pb_state = TURN;
			}

			break;
		}
		case LAUNCH: {
			double d_theta = path->setpoints[path->current_setpoint].heading -
							 odom->cur_pos.heading;

			// If we still need to turn.
			if (fabs(d_theta) > ANGLE_THRESHOLD) {
				// If we need to turn counterclockwise.
				if ((0.0 < d_theta && d_theta < PI) || d_theta < -1 * PI) {
					safe_drive(0.0f, 1.0f);
				}
				// Else we need to turn clockwise.
				else {
					safe_drive(0.0f, -1.0f);
				}
			}
			// If we are in position.
			else {
				safe_drive(0.0f, 0.0f);

				if (display->ball_count > 0) {
					solenoid_actuate();

					++(path->current_setpoint);
					if (path->current_setpoint >= path->num_valid) {
						path->current_setpoint = 0;
					}

					path->pb_state = TURN;
				}
			}

			break;
		}
	}
}

double get_angle_to_setpoint(odom_t* odom, path_t* path) {
	double d_x = path->setpoints[path->current_setpoint].x - odom->cur_pos.x;
	double d_y = path->setpoints[path->current_setpoint].y - odom->cur_pos.y;

	double theta = atan(d_y / d_x);

	double angle_diff = theta - odom->cur_pos.heading;

	while (angle_diff < 0.0) {
		angle_diff += 2 * PI;
	}
	while (angle_diff > 2 * PI) {
		angle_diff -= 2 * PI;
	}

	return angle_diff;
}

double get_distance_to_setpoint(odom_t* odom, path_t* path) {
	double d_x = path->setpoints[path->current_setpoint].x - odom->cur_pos.x;
	double d_y = path->setpoints[path->current_setpoint].y - odom->cur_pos.y;

	return sqrt(pow(d_x, 2.0) + pow(d_y, 2.0));
}

path_t path;
