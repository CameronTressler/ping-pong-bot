	.global	serial_transmit_bb
 	.p2align 2
	.syntax unified
 	.type	serial_transmit_bb,%function
 serial_transmit_bb:
 	.fnstart

	push {lr}
 	push {r4-r7}

 	.equ	GPIOA_ADDR, 		0x48000000
 	.equ	GPIO_ODR_OFFSET, 	0x14
 	.equ	INITIAL_WRITE_VAL,	0xFFFFFFFE

 	// get write address for GPIOA ODR
 	// bit 0 corresponds to PA0
 	movw	r1,	#:lower16:GPIOA_ADDR
 	movt	r1,	#:upper16:GPIOA_ADDR
 	add	r1,	r1, #GPIO_ODR_OFFSET

 	// get initial write value
 	movw	r2,	#:lower16:INITIAL_WRITE_VAL
 	movt	r2,	#:upper16:INITIAL_WRITE_VAL

 	// setup loop variables
 	mov	r3,	#0	// counter
 	mov	r4,	#32	// condition -- number of bits in message

 	/*

 		BEGIN TIMING CRITICAL SECTION

 	*/

 	// transmit start bit (0)
 	str	r2,	[r1]
 	// noops for timing
 	nop
 	nop
 	nop
 	nop

 loop: // TODO: if iteration causes too much delay, unroll loop
 	// check loop condition
 	cmp	r3,	r4
 	bge	end

 	// write a bit to PA0
 	mov	r5,	r2
 	and	r6,	r0,	#1
 	orr	r5,	r5, r6
 	str	r5,	[r1]

 	// shift message right
 	lsr	r0,	r0, #1

 	// ++i
 	add	r3,	r3, #1
 	b loop

 end:
 	// transmit stop bit (1)
 	orr	r7,	r2,	#1
 	str	r7,	[r1]
 	// noops for timing
 	nop
 	nop
 	nop
 	nop

	pop {r4-r7}
	pop {pc}

/*
#define GPIOA_ADDR 0x48000000
#define GPIO_ODR_OFFSET 0x14
#define INITIAL_WRITE_VAL 0xFFFFFFFE

void serial_transmit_bb(uint32_t tx_msg) {
	uint32_t *gpio_write_addr = (uint32_t*) GPIOA_ADDR + GPIO_ODR_OFFSET;

	// timing critical begins here

	*gpio_write_addr = 0xFFFFFFFE; // transmit start bit

	// each iteration must take 1us
	for (int i = 0; i < 32; ++i) {
		uint32_t val_to_write = 0xFFFFFFFE; // final bit is 0
		val_to_write |= (tx_msg &= 1);
		*gpio_write_addr = val_to_write;
		tx_msg >>= 1;
	}

	*gpio_write_addr = 0xFFFFFFFF; // transmit stop bit

}

*/

