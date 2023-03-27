#include "n64.h"
#include <stdint.h>
#include "cmsis_gcc.h"

void n64_init(n64_t *n64_state) {
	n64_update(0, n64_state);
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

	n64_state->A = (poll_rx_msg >> 31) & 1;
	n64_state->B = (poll_rx_msg >> 30) & 1;
	n64_state->Z = (poll_rx_msg >> 29) & 1;
	n64_state->Start = (poll_rx_msg >> 28) & 1;
	n64_state->DU = (poll_rx_msg >> 27) & 1;
	n64_state->DD = (poll_rx_msg >> 26) & 1;
	n64_state->DL = (poll_rx_msg >> 25) & 1;
	n64_state->DR = (poll_rx_msg >> 24) & 1;
	n64_state->Reset = (poll_rx_msg >> 23) & 1;

	// bit 22 is reserved (undefined)

	n64_state->L = (poll_rx_msg >> 21) & 1;
	n64_state->R = (poll_rx_msg >> 20) & 1;
	n64_state->CU = (poll_rx_msg >> 19) & 1;
	n64_state->CD = (poll_rx_msg >> 18) & 1;
	n64_state->CL = (poll_rx_msg >> 17) & 1;
	n64_state->CR = (poll_rx_msg >> 16) & 1;

	// 8 bit values
	n64_state->X = (poll_rx_msg >> 8) & 0xFF;
	n64_state->X = poll_rx_msg & 0xFF;
}
