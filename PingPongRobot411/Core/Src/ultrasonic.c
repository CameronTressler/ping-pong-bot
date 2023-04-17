#include "ultrasonic.h"
#include "hbridge.h"
#include <stdio.h>

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
		// printf("Error state\n\r");
	}

	// Else we're at the end of an echo.
	else {
		unsigned int elapsed_counts = current_count - ultra->count_of_echo_start;

		if (get_ultra_distance_in(elapsed_counts) > MAX_ON_TABLE_IN) {
			if (ultra->off_table < DEFINITELY_OFF_TABLE_THRESHOLD) {
				++ultra->off_table;

//				if (ultra->off_table == 3) {
//					set_PWM(hbridges + 0, -1 * get_PWM(hbridges + 0));
//					set_PWM(hbridges + 1, -1 * get_PWM(hbridges + 1));
//
//					HAL_Delay(20);
//
//					set_PWM(hbridges + 0, 0.0);
//					set_PWM(hbridges + 1, 0.0);
//				}
			}
		}
		else {
			if (ultra->off_table > 0) {
				--ultra->off_table;
			}
		}

		// Reset count for beginning of next echo.
		ultra->count_of_echo_start = 0;
		// printf("%d\n\r", elapsed_counts);
	}
}

bool is_off_table(ultra_t* ultra) {
	if (ultra->off_table >= OFF_TABLE_THRESHOLD) {
		return true;
	}
	return false;
}

bool ultras_definitely_off_table() {
	for (uint8_t i = 0; i < NUM_ULTRAS; ++i) {
		if (ultras[i].off_table >= DEFINITELY_OFF_TABLE_THRESHOLD) {
			return true;
		}
	}

	return false;
}

ultra_t ultras[NUM_ULTRAS];
