#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

float get_ultra_distance_in(unsigned int count_us);

typedef struct {
	float distance_in;
	unsigned int count_of_echo_start;
} ultra_t;

void init_ultra(ultra_t* ultra);

void update_ultra(ultra_t* ultra, unsigned int current_count);

extern ultra_t ultras[2];

#endif
