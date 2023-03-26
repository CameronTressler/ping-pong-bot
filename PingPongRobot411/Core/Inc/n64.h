#ifndef __N64_H
#define __N64_H

#include <stdint.h>

/*
 *  N64 commands (as listed at http://www.qwertymodo.com/hardware-projects/n64/n64-controller)
 *  preprocessed to be sent via n64_read according to the n64 communication protocol:
 *  1 --> 0111
 *  0 --> 0001
 *  Reversed because tx occurs from lsb --> msb
 *  E --> 1110 --> 1
 *  8 --> 1000 --> 0
 */

#define N64_STATUS 0x88888888
#define N64_POLL   0xE8888888
#define N64_READ   0x8E888888
#define N64_WRITE  0xEE888888
#define N64_RESET  0xEEEEEEEE

typedef struct {
	// Buttons
	uint8_t A;
	uint8_t B;
	uint8_t Z;
	uint8_t L;
	uint8_t R;
	uint8_t Start;

	// D-Pad
	uint8_t DU;
	uint8_t DD;
	uint8_t DL;
	uint8_t DR;

	// C-Pad
	uint8_t CU;
	uint8_t CD;
	uint8_t CL;
	uint8_t CR;

	// Joystick Axes
	uint8_t X;
	uint8_t Y;

	// Reset condition
	uint8_t Reset;
} n64_t;

void n64_init(n64_t *n64_state);

// n64 controller read button status
void n64_read(uint32_t tx_msg, n64_t *n64_state);	// wrapper
uint32_t n64_read_as(uint32_t tx_msg);				// assembly routine

// parse polling response and update n64 state
void n64_update(uint32_t poll_rx_msg, n64_t *n64_state);

#endif
