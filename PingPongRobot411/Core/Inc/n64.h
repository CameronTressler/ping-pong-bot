#ifndef __N64_H
#define __N64_H

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

// n64 controller read button status
extern uint32_t n64_read(uint32_t tx_msg); 

#endif
