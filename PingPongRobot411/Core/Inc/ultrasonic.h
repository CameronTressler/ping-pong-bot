#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#define NUM_ULTRAS 2

// TODO: Update to match system
#define FRONT_ULTRA 0
#define REAR_ULTRA 1

#define MAX_ON_TABLE_IN 10

float get_ultra_distance_in(unsigned int count_us);

typedef struct {
	unsigned int off_table;
	unsigned int count_of_echo_start;
} ultra_t;

void init_ultra(ultra_t* ultra);

void update_ultra(ultra_t* ultra, unsigned int current_count);

unsigned int is_off_table(ultra_t* ultra);
unsigned int ultras_off_table();

extern ultra_t ultras[NUM_ULTRAS];

#endif
