#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"
#include "ultrasonic.h"
#include "hbridge.h"

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

typedef struct {
	uint8_t data;
	uint8_t age;
} imu_calib_t;

void init_imu();
int is_imu_calibrated(imu_calib_t* calib);

// Output rate for fusion data is 100Hz.
#define IMU_UPDATE_RT 100.0

#define EUL_DATA_X 0x1A
#define LIA_DATA_X 0x28

// The ratio of hbridge commands to expected velocity in m/s.
#define CMD_TO_VEL 0.75

// The expected max amount of change in velocity per iteration.
#define DELTA_ACCEL 0.02
#define DELTA_DECCEL 0.05

#define PREDICTED_RATIO 1.0


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
	float x_rel;
	float y_rel;

	float velocity;
	float heading;

	float x_corner[4];
	float y_corner[4];

	uint8_t calib_age;

	uint32_t i;
} odom_t;

#define CALIB_LIFE 100

void init_odom(odom_t* odom);
void update_odom(odom_t* odom, hbridge_t* hbridges, ultra_t* ultras);

void calibrate_corner(odom_t* odom, uint8_t corner_num);
void adjust_off_table(odom_t* odom);
float predict_velocity(float prev_vel, float left_cmd, float right_cmd);

extern imu_calib_t calibration;
extern odom_t odometry;

#endif
