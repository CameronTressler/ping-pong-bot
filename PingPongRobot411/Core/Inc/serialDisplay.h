#ifndef __SERIALDISPLAY_H
#define __SERIALDISPLAY_H

#include <stdint.h>
#include "main.h"

typedef struct {
	int ball_count;
	int screen;
} display_t;

extern I2C_HandleTypeDef hi2c3;
extern display_t display;

// Address
#define DISPLAY_ADDR 0x4E

void increment_ball_count();

// Depends what we want messages to be
void decrement_ball_count();

void display_reset_ball_count();

void display_init();

void display_write_string(char *str);

void writeHelloWorld();

void display_send_cmd(char cmd);

// Commands
#define SPECIAL_COMMAND 0xFE
#define CLEAR_CMD 0x01
#define ON_OFF_CMD 0x08


#endif
