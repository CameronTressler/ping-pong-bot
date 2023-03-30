#include "odometry.h"
#include "util.h"
#include <math.h>

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
	return data == 255;
}

void init_odom(odom_t* odom) {
	odom->x_rel = 0.0;
	odom->y_rel = 0.0;
	odom->velocity = 0.0;
	odom->heading = 0.0;
}

void update_odom(odom_t* odom) {
	uint8_t calibration = get_imu_calib();

	imu_raw_data_t yaw, lin_accel;

	i2c_read(IMU_ADDR, EUL_DATA_X, yaw.buf, 2);
	i2c_read(IMU_ADDR, LIA_DATA_X, lin_accel.buf, 2);

	// Update heading in degrees.
	odom->heading = yaw.data.datum / 900.0;
//	odom->heading = (yaw.bytes.msb * 16.0) +
//					(yaw.bytes.lsb / 16.0);

	// Update velocity in m/s.
	double accel = lin_accel.data.datum / 9806.65;
//	double accel = (lin_accel.bytes.y_msb * 0.284444) +
//				   (lin_accel.bytes.y_lsb / 900.0);
	odom->velocity += accel / IMU_UPDATE_RT;

	double delta_pos = odom->velocity / IMU_UPDATE_RT;

	odom->x_rel += cos(odom->heading) * delta_pos;
	odom->y_rel += sin(odom->heading) * delta_pos;
}

odom_t odometry;
