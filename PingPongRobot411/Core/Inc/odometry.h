#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"

// The IMU communicates over I2C with the following address.
#define IMU_ADDR 0x28

// Register address for setting power mode.
#define PWR_MODE_REG 0x3E

// Register address for setting operation mode.
#define OPR_MODE_REG 0x3D

// Register address for Remapping axis names.
#define AXIS_MAP_CONFIG_REG 0x41

// Register address for setting units of measurements.
#define UNIT_SEL_REG 0x3B
// Uses degrees.
#define UNIT_SEL_DEG 0b0 // 0bxxxxx0xx

void init_imu();

// Output rate for fusion data is 100Hz.

#define EUL_DATA 0x1A

#define LIA_DATA 0x28

typedef union {
	struct {
		uint8_t x_lsb;
		uint8_t x_msb;
		uint8_t y_lsb;
		uint8_t y_msb;
		uint8_t z_lsb;
		uint8_t z_msb;
	} bytes;

	uint8_t buf[6];
} imu_raw_data_t;

typedef struct {
	double x_rel;
	double y_rel;
	double yaw;
} odom_t;

void update_odom(odom_t* odom);

extern odom_t odometry;

#endif
