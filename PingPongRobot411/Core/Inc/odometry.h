#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"
#include "ultrasonic.h"
#include "hbridge.h"
#include "drive.h"
#include "serialDisplay.h"
#include "solenoid.h"
#include <stdbool.h>
#include <stdint.h>

#define PI 3.1415926535

///////////////////////////////////////////////////////////////////////////////
// ------- BNO ----------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////

// The BNO communicates over I2C with the following address.
#define BNO_ADDR 0x28

// Register address for setting power mode.
#define BNO_PWR_MODE_REG 0x3E

// Register address for setting operation mode.
#define BNO_OPR_MODE_REG 0x3D

// Register address for setting units of measurements.
#define BNO_UNIT_SEL_REG 0x3B

// Register for reading calibration status
#define BNO_CALIB_STAT_REG 0x35

typedef struct {
	uint8_t data;
	uint8_t age;
} imu_calib_t;

void init_bno();
bool check_bno_calibration(imu_calib_t* calib, double heading);

#define EUL_DATA_X 0x1A
#define LIA_DATA_X 0x28

///////////////////////////////////////////////////////////////////////////////
// ------- ODOM ---------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////

// Output rate for fusion data is in Hz.
#define ODOM_UPDATE_RT 100.0

#define MG_TO_MS2 101.971621

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

// The ratio of hbridge commands to expected velocity in m/s.
#define MAX_THEORETICAL_SPEED 2.0

// The expected max amount of change in velocity per iteration.
#define ACCEL_RATIO 0.2
#define DELTA_DECCEL 0.05

#define PREDICTED_RATIO 1.0

typedef struct {
	double x;
	double y;
	double heading;
} coord_t;


typedef struct {
	coord_t cur_pos;
	double velocity;

	coord_t corners[4];

	uint32_t i;
	uint8_t zero_count;

	bool bno_calibrated;
	bool adxl_calibrated;
	int16_t adxl_offset;
} odom_t;

#define CALIB_LIFE 100
#define ZERO_THRESHOLD 2

extern imu_calib_t bno_calibration;
extern odom_t odometry;

#define MAX_SETPOINTS 10

#define ANGLE_THRESHOLD 0.2
#define KP_TURN_ADJUST 5
#define MAX_ACCEPTABLE_ANGLE 2
#define DIST_THRESHOLD 0.0005

typedef enum {
	TURN,
	DRIVE,
	LAUNCH
} play_back_state_t;

typedef struct {
	coord_t setpoints[MAX_SETPOINTS];
	uint8_t num_valid;
	uint8_t current_setpoint;

	play_back_state_t pb_state;

	float forward_cmd, left_cmd;
	bool cmds_active;
} path_t;

void init_odom(odom_t* odom);
void update_odom(odom_t* odom, hbridge_t* hbridges,ultra_t* ultras);

void calibrate_corner(odom_t* odom, uint8_t corner_num);
void adjust_off_table(odom_t* odom);
double predict_velocity(double prev_vel, double left_cmd, double right_cmd);

void reset_path(path_t* path);
void add_setpoint(odom_t* odom, path_t* path);
void set_playback_cmds(odom_t* odom, path_t* path, display_t* display);
bool facing_target(double angle_diff, double threshold);
double convert_to_std_rad(double angle);
double get_angle_to_setpoint(odom_t* odom, path_t* path);
double get_distance_to_setpoint(odom_t* odom, path_t* path);

extern path_t path;

///////////////////////////////////////////////////////////////////////////////
// ------- dynamic ------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////

#define DYNAMIC_SPEED 0.3

extern bool dynamic_forward;
extern double dynamic_angle;
extern bool dynamic_cmds_active;

void set_dynamic_angle(double angle);
void move_dynamic(odom_t* odom, ultra_t* ultras);

///////////////////////////////////////////////////////////////////////////////
// ------- ADXL ---------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////

// The ADXL communicates over I2C with the following address.
#define ADXL_ADDR 			0x1D

// register used to enable ADXL measurement mode
#define ADXL_PWR_CTL_REG	0x2D

// register used to format output data from ADXL
#define ADXL_DATA_FMT_REG	0x31

// ADXL data registers
#define ADXL_X_LSB			0x32
#define ADXL_X_MSB			0x33
#define ADXL_Y_LSB			0x34
#define ADXL_Y_MSB			0x35
#define ADXL_Z_LSB			0x36
#define ADXL_Z_MSB			0x37

void init_adxl(void);
double get_accel(void);
void adxl_calibrate(odom_t *odom);


#endif
