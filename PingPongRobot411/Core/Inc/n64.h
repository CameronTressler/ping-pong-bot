#ifndef __N64_H
#define __N64_H

#include <stdint.h>
#include <stdbool.h>

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


// other constants
#define N64_NUM_VALUES 17

// enum for n64 buttons -- used as index to button_status array
typedef enum {
	N64_A, N64_B, N64_Z, N64_L, N64_R, N64_Start,
	N64_DU, N64_DD, N64_DL, N64_DR,
	N64_CU, N64_CD, N64_CL, N64_CR,
	N64_X, N64_Y,
	N64_Reset
} n64_button;

typedef struct {
	uint8_t button_status[N64_NUM_VALUES]; // indices given by n64_button enum
} n64_t;

void n64_init(n64_t *n64_state);

// n64 controller read button status
void n64_read(uint32_t tx_msg, n64_t *n64_state);	// wrapper
uint32_t n64_read_as(uint32_t tx_msg);				// assembly routine

// parse polling response and update n64 state
void n64_update(uint32_t poll_rx_msg, n64_t *n64_state);

// check if button has been pressed
bool n64_button_pressed(n64_t *prev, n64_t *curr, n64_button button);

#endif
