#include "n64.h"
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_gcc.h"

void n64_init(n64_t *n64_state) {
	n64_update(0, n64_state);
}

void n64_copy(n64_t *dst, n64_t *src) {
	for (int i = 0; i < N64_NUM_VALUES; ++i) {
		dst->button_status[i] = src->button_status[i];
	}
}


void n64_read(uint32_t tx_msg, n64_t *n64_state) {

	// get interrupt enable status (0: enabled)
	uint32_t prim = __get_PRIMASK();

	// disable interrupts
	__disable_irq();


	uint32_t rx_msg = n64_read_as(tx_msg);


	// re-enable interrupts iff they were enabled before
	if (!prim) {
		__enable_irq();
	}

	// update n64 state
	if (tx_msg == N64_POLL) {
		n64_update(rx_msg, n64_state);
	}

}

void n64_update(uint32_t poll_rx_msg, n64_t *n64_state) {
	// scheme:
	// http://www.qwertymodo.com/hardware-projects/n64/n64-controller

	// not using a for loop because that cements the order of the
	// enum. This is flexible.
	n64_state->button_status[N64_A] = (poll_rx_msg >> 31) & 1;
	n64_state->button_status[N64_B] = (poll_rx_msg >> 30) & 1;
	n64_state->button_status[N64_Z] = (poll_rx_msg >> 29) & 1;
	n64_state->button_status[N64_Start] = (poll_rx_msg >> 28) & 1;
	n64_state->button_status[N64_DU] = (poll_rx_msg >> 27) & 1;
	n64_state->button_status[N64_DD] = (poll_rx_msg >> 26) & 1;
	n64_state->button_status[N64_DL] = (poll_rx_msg >> 25) & 1;
	n64_state->button_status[N64_DR] = (poll_rx_msg >> 24) & 1;
	n64_state->button_status[N64_Reset] = (poll_rx_msg >> 23) & 1;

	// bit 22 is reserved (undefined)

	n64_state->button_status[N64_L] = (poll_rx_msg >> 21) & 1;
	n64_state->button_status[N64_R] = (poll_rx_msg >> 20) & 1;
	n64_state->button_status[N64_CU] = (poll_rx_msg >> 19) & 1;
	n64_state->button_status[N64_CD] = (poll_rx_msg >> 18) & 1;
	n64_state->button_status[N64_CL] = (poll_rx_msg >> 17) & 1;
	n64_state->button_status[N64_CR] = (poll_rx_msg >> 16) & 1;

	// 8 bit joystick values
	n64_state->button_status[N64_X] = (poll_rx_msg >> 8) & 0xFF;
	n64_state->button_status[N64_Y] = poll_rx_msg & 0xFF;
}

bool n64_button_pressed(n64_t *prev, n64_t *curr, n64_button button) {
	// true if button was previously unpressed (0) and is now pressed (1)
	return !prev->button_status[button] && curr->button_status[button];
}

