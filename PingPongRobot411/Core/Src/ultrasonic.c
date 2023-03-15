#include "ultrasonic.h"

float get_ultra_distance_in(unsigned int count_us) {
	return ((float) count_us) / 144.0;
}

void init_ultra(ultra_t* ultra) {
	ultra->off_table = 0;
	ultra->count_of_echo_start = 0;
}

void update_ultra(ultra_t* ultra, unsigned int current_count) {

	// If we're at the beginning of an echo.
	if (ultra->count_of_echo_start == 0) {
		ultra->count_of_echo_start = current_count;
	}

	// Error state: end of echo appears to be before start of echo.
	else if (current_count < ultra->count_of_echo_start) {
		// Reset count so we now assume an echo has started but not finished,
		// and count is low enough to be reset on the next trigger.
		ultra->count_of_echo_start = 1;
	}

	// Else we're at the end of an echo.
	else {
		unsigned int elapsed_counts = current_count - ultra->count_of_echo_start;

		if (get_ultra_distance_in(elapsed_counts) > MAX_ON_TABLE_IN) {
			if (ultra->off_table < 3) {
				++ultra->off_table;
			}
		}
		else {
			if (ultra->off_table > 0) {
				--ultra->off_table;
			}
		}

		// Reset count for beginning of next echo.
		ultra->count_of_echo_start = 0;
	}
}

unsigned int is_off_table(ultra_t* ultra) {
	if (ultra->off_table >= 2) {
		return 1;
	}
	return 0;
}

ultra_t ultras[NUM_ULTRAS];
