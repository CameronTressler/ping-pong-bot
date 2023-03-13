#ifndef INC_HBRIDGE_H_
#define INC_HBRIDGE_H_

#include <stdint.h>

#define TIM4_OFFSET 0x40000800 // Timer used for hbridge channels

#define TIM_CCR4_OFFSET 0x40
#define TIM_CCR3_OFFSET 0x3C  //capture/compare register 3
#define TIM_CCR2_OFFSET 0x38  //capture/compare register 2
#define TIM_CCR1_OFFSET 0x34  //capture/compare register 1

/* HBridge Configuration IDs
 *
 *  0 Drive Left
 *  1 Drive Right
 *  2 Launcher Left
 *  3 Launcher Right
 *
 */

typedef struct {
	float PWM;
	uint32_t * CCR;
} hbridge_t;

extern hbridge_t hbridges[4];


// Initializes HBridges with pointer to designated CCR register
void init_hbridges();


// Set PWM (0-1.0)
void set_PWM(hbridge_t hbridge, float PWM);

// Get PWM (0-1.0)
float get_PWM(hbridge_t hbridge);

#endif /* INC_HBRIDGE_H_ */
