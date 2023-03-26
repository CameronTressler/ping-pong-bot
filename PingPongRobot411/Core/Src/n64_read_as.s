	.global	n64_read_as
 	.p2align 2
	.syntax unified
 	.type	n64_read_as,%function
 n64_read_as:
 	.fnstart

	push {lr}
 	push {r4-r11}

 	/*

 		NOTE: 4 NOP == 0.5 us

 	 */

 	.equ	GPIOA_ADDR, 		0x40020000
 	.equ	GPIO_ODR_OFFSET, 	0x14
 	.equ	INITIAL_WRITE_VAL,	0xFFFFFFFE
 	.equ    GPIO_IDR_OFFSET,   0x10
 	.equ 	GPIOB_ADDR, 		0x40020400

 	movw r11, #:lower16:GPIOB_ADDR
 	movt r11, #:upper16:GPIOB_ADDR
 	add	r11, #GPIO_ODR_OFFSET

 	// get write address for GPIOA ODR
 	// bit 0 corresponds to PA0
 	mov r9, r1 //moves receive buffer starting addr into r9
 	movw r1, #:lower16:GPIOA_ADDR
 	movt r1, #:upper16:GPIOA_ADDR

 	// get read and write addresses for GPIO
 	add r8, r1, #GPIO_IDR_OFFSET
 	add	r1,	r1,	#GPIO_ODR_OFFSET

 	// get initial write value
 	movw r2, #:lower16:INITIAL_WRITE_VAL
 	movt r2, #:upper16:INITIAL_WRITE_VAL

 	// setup loop variables
 	mov	r3,	#0	// counter
 	mov	r4,	#32	// condition -- number of bits in message

 	/*********************************
	 *                               *
	 * BEGIN TIMING CRITICAL SECTION *
	 *                               *
 	 *********************************/ 

 	// transmit start bit (0)
 	str	r2,	[r1]
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop


 txLoop: // TODO: if iteration causes too much delay, unroll loop
 	// check loop condition
 	cmp	r3,	r4
 	bge	txDone

 	// write a bit to PA0
 	mov	r5,	r2
 	and	r6,	r0,	#1
 	orr	r5,	r5,	r6
 	str	r5,	[r1]

 	// shift message right
 	lsr	r0,	r0,	#1

 	// ++i
 	add	r3,	r3,	#1

	// timing
 	nop
 	nop
 	nop
 	nop
 	nop

 	b txLoop

 txDone:
 	// transmit stop bits (011)
 	str r2, [r1]
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop

 	orr	r7,	r2,	#1
 	str	r7,	[r1]
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop

 	str r7, [r1]
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
// 	nop
// 	nop

 	
	/*********************************
	 *                               *
	 *   BEGIN DATA RECEIVE SECTION  *
	 *                               *
 	 *********************************/ 

	// loop counts down from 31 --> 0
 	mov r5, #31 //loop iterator
	mov r6, #0 //loop end condition
	mov r0, #0
	mov r7, #1

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

rxLoop:
	cmp r5, r6 //check if iterator = end cond
	blt rxDone // if eq then end loop
	ldr r9, [r8] //read bit

	// TODO: store instructions to r11 (GPIOB) disable the GPIOB port
	// but it does not seem to work without them, even when writing
	// to sp.
	str r6, [r11] // TESTING 
	str r7, [r11] // TESTING
	nop
	nop
	/*
	nop
	nop
	nop
	nop
	nop
	nop
	*/

 	lsr r9, #1 //isolate bit
 	and r9, #1 //isolate bit
 	lsl r9, r5 //shift left by i
 	orr r0, r9 //or with output number
 	sub r5, #1

 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop
 	nop

	b rxLoop

rxDone:

	pop {r4-r11}
	pop {pc}

/*
#define GPIOA_ADDR 0x48000000
#define GPIO_ODR_OFFSET 0x14
#define INITIAL_WRITE_VAL 0xFFFFFFFE
void joybus_communicate(uint32_t tx_msg) {
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

uint32_t n64val;
mv r5, #31
mv r6, #0
for (int i = 31; i >= 0; ++i){
	cmp r6, r5
	blt rxDone
	ldr r1, [r8] //read bit
 	lsr r1, #1 isolate bit
 	and r1, #1 isolate bit
 	lsl r1, #i //sbhift left by i
 	orr r1, #1 //or with output number
 	add r6, #1
	b rxLoop

}
*/
