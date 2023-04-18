#ifndef __SERIALDISPLAY_H
#define __SERIALDISPLAY_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#define TIM5_OFFSET 0x40000C00

#define TIM_COUNT_OFFSET 0x24
#define TIM_ARR_OFFSET 0x2C

typedef enum
{
    welcome,
    menu_1,
    menu_2,
    menu_3,
    launch,
	pb_not_calibrated,
	pb_calibrate,
	pb_record,
	pb_go,
	intervals,
	intervals_countdown,
	intervals_select_high,
	intervals_select_medium,
	intervals_select_low
} display_state;

typedef struct {
	int ball_count;
	int interval_delay;
	int speed;
	float last_pwm;
	uint32_t * interval_count;
	uint32_t * ARR;
	bool balls_displayed;
	int countdown;
	bool change;
	char *top_text; // must be 40 characters wide!!
	char *bottom_text; // must be 40 characters wide!!
	display_state intervals_distance_last;

} display_t;



extern I2C_HandleTypeDef hi2c3;
extern display_t display;

// Address
#define DISPLAY_ADDR 0x4E

// Ball count
void increment_ball_count(void);

void decrement_ball_count(void);

void display_reset_ball_count(void);

// Menu architecture
void display_playback_record(void);

void display_playback_begin(void);

void display_freeplay(void);

void display_welcome(void);

void display_intervals_begin(void);

void display_intervals_launch(void);

void display_menu_1(void);

void display_menu_2(void);

void display_menu_3(void);

void display_intervals_high(void);

void display_intervals_medium(void);

void display_intervals_low(void);

void display_intervals_countdown(void);

void display_freeplay_speed(void);

// Initialization and commands
void display_init(void);

void display_write_string(char *str);

void display_send_cmd(char cmd);

void display_playback_calibrate(void);
void display_not_calibrated(void);


// Commands
#define SPECIAL_COMMAND 0xFE
#define CLEAR_CMD 0x01
#define ON_OFF_CMD 0x08

// Random
#define MAX_BALL_COUNT 6 // idk about this lol


#endif
