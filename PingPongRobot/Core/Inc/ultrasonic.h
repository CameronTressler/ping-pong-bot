#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

float get_ultra_distance_in(int count_us);

typedef struct {
	float distance_in;
	int count_of_echo_start;
} ultra_t;

extern ultra_t ultra;

#endif
