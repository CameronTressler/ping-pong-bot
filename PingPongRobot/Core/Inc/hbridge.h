#ifndef INC_HBRIDGE_H_
#define INC_HBRIDGE_H_

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
	uint16_t * CCR;
} hbridge_t;

extern hbridge_t hbridges[4];


// Initializes HBridge with pointer to designated CCR register
void init_hbridge(hbridge_t hbridge, uint16_t * CCR);


// Set PWM (0-100)
void set_pwm(hbridge_t hbridge);

// Get PWM (0-100)
float get_PWM(hbridge_t hbridge);

#endif /* INC_HBRIDGE_H_ */
