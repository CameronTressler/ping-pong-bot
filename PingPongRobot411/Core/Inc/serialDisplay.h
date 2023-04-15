#ifndef __SERIALDISPLAY_H
#define __SERIALDISPLAY_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

typedef struct {
	int ball_count;
	int countdown;
	bool change;
	char *top_text; // must be 40 characters wide!!
	char *bottom_text; // must be 40 characters wide!!

} display_t;

typedef enum
{
    welcome,
    menu_1,
    menu_2,
    menu_3,
    launch,
	pb_countdown,
	pb_record,
	pb_relocate,
	pb_begin,
	intervals,
	intervals_countdown,
	intervals_select_high,
	intervals_select_medium,
	intervals_select_low
} display_state;

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

void display_playback_relocate(void);

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

// Count down display
void display_pb_countdown(void);

void display_intervals_countdown(void);

// Initialization and commands
void display_init(void);

void display_write_string(char *str);

void display_send_cmd(char cmd);


// Commands
#define SPECIAL_COMMAND 0xFE
#define CLEAR_CMD 0x01
#define ON_OFF_CMD 0x08

// Random
#define BALL_COUNT 20 // idk about this lol


#endif
