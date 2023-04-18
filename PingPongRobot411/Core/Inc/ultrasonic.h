#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdbool.h>

#define NUM_ULTRAS 2

#define FRONT_ULTRA 0
#define REAR_ULTRA 1

#define OFF_TABLE_THRESHOLD 1
#define DEFINITELY_OFF_TABLE_THRESHOLD 6

#define MAX_ON_TABLE_IN 18

float get_ultra_distance_in(unsigned int count_us);

typedef struct {
	// A count of how many times we've recently seen the ultra off the table.
	unsigned int off_table;

	// The timer tick of when the ultra's echo started.
	unsigned int count_of_echo_start;
} ultra_t;

void init_ultra(ultra_t* ultra);

// Called when a rising or falling echo edge is detected via interrupt.
void update_ultra(ultra_t* ultra, unsigned int current_count);

// Returns whether the ultra deems itself probably off the table.
bool is_off_table(ultra_t* ultra);

bool definitely_off_table(ultra_t* ultra);

// Returns whether any ultrasonic is definitely off the table.
bool ultras_definitely_off_table();

extern ultra_t ultras[NUM_ULTRAS];

#endif
