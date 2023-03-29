#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"

// The IMU communicates over I2C with the following address.
#define IMU_ADDR 0x28

// Register address for setting power mode.
#define PWR_MODE_REG 0x3E

// Register address for setting operation mode.
#define OPR_MODE_REG 0x3D

// Register address for setting units of measurements.
#define UNIT_SEL_REG 0x3B

// Register for reading calibration status
#define CALIB_STAT_REG 0x35

void init_imu();
uint8_t get_imu_calib();
int is_calibrated(uint8_t data);

// Output rate for fusion data is 100Hz.
#define IMU_UPDATE_RT 100.0

#define EUL_DATA_Z 0x1E

#define LIA_DATA_X 0x28

typedef union {
	struct {
		uint8_t lsb;
		uint8_t msb;
	} bytes;

	struct {
		int16_t datum;
	} data;

	uint8_t buf[2];
} imu_raw_data_t;

typedef struct {
	double x_rel;
	double y_rel;

	double velocity;
	double heading;
} odom_t;

void init_odom(odom_t* odom);
void update_odom(odom_t* odom);

extern odom_t odometry;

#endif
