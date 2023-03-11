#include "ultrasonic.h"

ultra_t ultra;

float get_ultra_distance_in(int count_us) {
	return ((float) count_us) / 144.0;
}
