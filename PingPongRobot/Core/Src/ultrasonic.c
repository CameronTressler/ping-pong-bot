#include "ultrasonic.h"

float get_ultra_distance_in(unsigned int count_us) {
	return ((float) count_us) / 144.0;
}

void init_ultra(ultra_t* ultra) {
	ultra->count_of_echo_start = 0;
	ultra->distance_in = 0.0;
}

void update_ultra(ultra_t* ultra, unsigned int current_count) {

	// If we're at the beginning of an echo.
	if (ultra->count_of_echo_start == 0) {
		ultra->count_of_echo_start = current_count;
	}

	// Error state: end of echo appears to be before start of echo.
	else if (current_count < ultra->count_of_echo_start) {
		// Accept temporary error state with the hope of getting back on
		// track in the next pulse.
		ultra->distance_in = -1;

		// Reset count so we now assume an echo has started but not finished,
		// and count is low enough to be reset on the next trigger.
		ultra->count_of_echo_start = 1;
	}

	// Else we're at the end of an echo.
	else {
		unsigned int elapsed_counts = current_count - ultra->count_of_echo_start;
		ultra->distance_in = get_ultra_distance_in(elapsed_counts);

		// Reset count for beginning of next echo.
		ultra->count_of_echo_start = 0;
	}
}

ultra_t ultras[2];
