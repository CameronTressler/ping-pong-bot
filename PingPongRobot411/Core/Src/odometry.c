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

uint8_t get_imu_calib() {
	uint8_t data;
	i2c_read(IMU_ADDR, CALIB_STAT_REG, &data, 1);

	return data;
}

int is_calibrated(uint8_t data) {
	return (data & 0b00111111) >= 60;
}

void init_odom(odom_t* odom) {
	odom->x_rel = 0.0;
	odom->y_rel = 0.0;
	odom->velocity = 0.0;
	odom->heading = 0.0;
}

void update_odom(odom_t* odom) {
	uint8_t calibration = get_imu_calib();
	if (!is_calibrated(calibration)) {
		printf("Not calibrated :(\n\r");
		return;
	}

	imu_raw_data_t yaw, lin_accel;

	// Update heading in radians.
	i2c_read(IMU_ADDR, EUL_DATA_X, yaw.buf, 2);
	odom->heading = yaw.data.datum / 900.0;

	i2c_read(IMU_ADDR, LIA_DATA_X, lin_accel.buf, 2);

	// Update acceleration in m/s^2. Filter out near-zero values.
	double accel;
	if (abs(lin_accel.data.datum) < ACCEL_ZERO_THRESHOLD) {
		accel = 0.0;

		if (odom->iterations_no_accel < VELOCITY_ZERO_THRESHOLD) {
			++(odom->iterations_no_accel);
		}
	}
	else {
		// Convert mg to m/s^2.
		accel = lin_accel.data.datum / 101.971621;
		odom->iterations_no_accel = 0;
	}

	if (odom->iterations_no_accel < VELOCITY_ZERO_THRESHOLD) {
		odom->velocity += accel / IMU_UPDATE_RT;
		update_position(odom, odom->velocity / IMU_UPDATE_RT);
	}
	else {
		odom->velocity = 0.0;
	}

	++(odom->i);
	if (odom->i % 10 == 0) {
		printf("%f : %f : %f : %f\n\r", accel, odom->heading, odom->x_rel, odom->y_rel);
	}
}

void update_position(odom_t* odom, double dist) {
	odom->x_rel += cos(odom->heading) * dist;
	odom->y_rel += sin(odom->heading) * dist;
}

odom_t odometry;
