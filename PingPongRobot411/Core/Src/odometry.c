#include "odometry.h"
#include "util.h"

void init_imu() {
	// Set to normal power mode.
	i2c_mask_write(IMU_ADDR, PWR_MODE_REG, 2, 0, 0b00);

	// Set to NDOF operational mode.
	i2c_mask_write(IMU_ADDR, OPR_MODE_REG, 4, 0, 0b1100);
}

void update_odom(odom_t* odom) {
	imu_raw_data_t eul_angles, lin_accel;

	i2c_read(IMU_ADDR, EUL_DATA, eul_angles.buf, 6);
	i2c_read(IMU_ADDR, LIA_DATA, lin_accel.buf, 6);
}

odom_t odometry;
