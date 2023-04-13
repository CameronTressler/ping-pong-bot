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
	i2c_mask_write(IMU_ADDR, OPR_MODE_REG, 4, 0, 0b1100);
	HAL_Delay(20);
}


int is_imu_calibrated(imu_calib_t* calib) {
	if (calib->age > CALIB_LIFE) {
		uint8_t data;
		i2c_read(IMU_ADDR, CALIB_STAT_REG, &data, 1);

		calib->data = data;
		calib->age = 0;

		printf("Calibration: %d\n\r", data);
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
	odom->x_rel = 0.0;
	odom->y_rel = 0.0;
	odom->velocity = 0.0;
	odom->heading = 0.0;

	for (int i = 0; i < 4; ++i) {
		odom->x_corner[i] = 0.0;
		odom->y_corner[i] = 0.0;
	}

	odom->calib_age = 0;
}

void update_position(odom_t* odom, float dist) {
	odom->x_rel += cos(odom->heading) * dist;
	odom->y_rel += sin(odom->heading) * dist;
}

void reset_velocity(odom_t* odom, uint8_t num_iterations) {
	// Backtrack the distance covered in the last num_iterations.
	if (num_iterations > 0) {
		float dist = -1 * odom->velocity * num_iterations / IMU_UPDATE_RT;
		update_position(odom, dist);
	}

	odom->velocity = 0.0;
}

float predict_velocity(float prev_vel, float left_cmd, float right_cmd) {
	// Find the average wheel command.
	float avg_cmd = (left_cmd + right_cmd) / 2.0;

	// Approximate the velocity we asymptotically would reach.
	// TODO: Check if positive is forward.
	float target_vel = avg_cmd * CMD_TO_VEL;

	// Find a predicted velocity, factoring in acceleration.
	float new_vel;
	if (target_vel > prev_vel) {
		new_vel = prev_vel + DELTA_VEL;

		if (new_vel > target_vel) {
			return target_vel;
		}
	}
	else {
		new_vel = prev_vel - DELTA_VEL;

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

	imu_raw_data_t yaw, lin_accel;

	// Update heading in radians.
	i2c_read(IMU_ADDR, EUL_DATA_X, yaw.buf, 2);
	odom->heading = yaw.data.datum / 900.0;

	// If commanded velocity is zero, assume actual velocity is zero.
	if (fabs(get_PWM(hbridges[0])) < 0.001 && fabs(get_PWM(hbridges[1])) < 0.001) {
		reset_velocity(odom, 0);
		return;
	}

	// Get acceleration.
	i2c_read(IMU_ADDR, LIA_DATA_X, lin_accel.buf, 2);

	// Update acceleration in m/s^2.
	float accel = lin_accel.data.datum / 101.971621;
	float measured_velocity = odom->velocity + (accel / IMU_UPDATE_RT);

	// Given odom->velocity as our previous velocity, use measured acceleration and predicted
	// velocity to obtain a new velocity estimate.
	float predicted_velocity =
			predict_velocity(odom->velocity, get_PWM(hbridges[0]), get_PWM(hbridges[1]));

	odom->velocity = (PREDICTED_RATIO * predicted_velocity) +
			   	     ((1 - PREDICTED_RATIO) * measured_velocity);

	update_position(odom, odom->velocity / IMU_UPDATE_RT);

	// Correct position.
	if (ultras_off_table()) {

	}

	++(odom->i);
	if (odom->i % 25 == 0) {
		printf("%f : %f : %f : %f\n\r", odom->heading, odom->velocity, odom->x_rel, odom->y_rel);
	}
}

//void calibrate_corner(odom_t* odom, uint8_t corner_num) {
//	odom->x_corner[corner_num] = odom->x_rel;
//	odom->y_corner[corner_num] = odom->y_rel;
//}
//
//float distance_to_line(odom_t* odom, uint8_t c_1, uint8_t c_2) {
//	float numerator = fabs(
//		(odom->x_corner[c_2] - odom->x_corner[c_1]) *
//		(odom->y_corner[c_1] - odom->y_rel) -
//		(odom->x_corner[c_1] - odom->x_rel) *
//		(odom->y_corner[c_2] - odom->y_corner[c_1])
//	);
//
//	float denominator = pow((odom->x_corner[c_2] - odom->x_corner[c_1]), 2);
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

