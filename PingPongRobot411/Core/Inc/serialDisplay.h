#ifndef __SERIALDISPLAY_H
#define __SERIALDISPLAY_H

typedef struct {
	int ball_count;
	int screen;
} display_t;


// Address
#define DISPLAY_ADDR 0x4E

// Depends what we want messages to be
void display_decrease_ball_count();

void display_reset_ball_count();

void display_init();

void display_write_string(char *str);

void writeHelloWorld();


#endif
