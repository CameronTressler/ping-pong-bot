#include "serialDisplay.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "controller.h"

display_t display;

//  Ball count display lookup table
char ball_display_table[21][34] = {
		"0                                ",
		"1                                ",
		"2                                ",
		"3                                ",
		"4                                ",
		"5                                ",
		"6                                ",
		"7                                ",
		"8                                ",
		"9                                ",
		"10                               ",
		"11                               ",
		"12                               ",
		"13                               ",
		"14                               ",
		"15                               ",
		"16                               ",
		"17                               ",
		"18                               ",
		"19                               ",
		"20                               "
};

// Countdown display table
char countdown_display_table[4][28] = {
	"0                          ",
	"1                          ",
	"2                          ",
	"3                          "
};

// Increases ball count by one and updates display
void increment_ball_count(void) {

	// Increment ball count
	if(display.ball_count != MAX_BALL_COUNT)
		++display.ball_count;

	if (display.balls_displayed) {
		// Write to display
		display.top_text = "";
		char top_str[50];
		strcpy(top_str, "Balls: ");

		display.top_text = strcat(top_str, ball_display_table[display.ball_count]);

		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
	}
}

// Decreases ball count by one and updates display
void decrement_ball_count(void) {
	if(display.ball_count != 0)
		--display.ball_count;

	// Update display
	display.top_text = "";

	char top_str[50];
	strcpy(top_str, "Balls: ");

	display.top_text = strcat(top_str, ball_display_table[display.ball_count]);
	display_write_string(display.top_text);
	display_write_string(display.bottom_text);
}


// Resets value of ball count to BALL_COUNT defined in serialDisplay.h
void display_reset_ball_count(void) {
	display.ball_count = MAX_BALL_COUNT;

	// Update display
	display.top_text = "";

	char top_str[50];
	strcpy(top_str, "Balls: ");

	display.top_text = strcat(top_str, ball_display_table[display.ball_count]);
	display_write_string(display.top_text);
	display_write_string(display.bottom_text);

}

void display_freeplay_speed(void) {
	display.top_text = "";

	char top_str[50];
	strcpy(top_str, "Speed: ");

	display.top_text = strcat(top_str, ball_display_table[display.speed]);
	display_write_string(display.top_text);
	display_write_string(display.bottom_text);
}

void display_freeplay(void) {
	if(display.change) {
		display.top_text = "";

		char top_str[50];
		strcpy(top_str, "Balls: ");

		display.top_text = strcat(top_str, ball_display_table[display.ball_count]);
		display_write_string(display.top_text);
		display.bottom_text = "Launch: A                               ";
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_playback_record(void) {
	if(display.change) {
		display.top_text = "Setpoint: A                             ";
		display.bottom_text = "Finish: B                               ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_playback_begin(void) {
	if(display.change) {
		display.top_text = "";
		char top_str[50];
		strcpy(top_str, "Balls: ");
		display.top_text = strcat(top_str, ball_display_table[display.ball_count]);

		display.bottom_text = "Playing back!                           ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_intervals_begin(void) {
	if(display.change) {
		display.top_text = "";
		char top_str[50];
		strcpy(top_str, "Delay: ");
		display.top_text = strcat(top_str, ball_display_table[display.interval_delay]);
		display.bottom_text = "STOP: B                                 ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_menu_1(void) {
	if(display.change) {
		display.top_text = "Free Play                               ";
		display.bottom_text = "Press A                                 ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_menu_2(void) {
	if(display.change) {
		display.top_text = "Play Back                               ";
		display.bottom_text = "Press A                                 ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_menu_3(void) {
	if(display.change) {
		display.top_text = "Intervals                               ";
		display.bottom_text = "Press A                                 ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}

}

void display_menu_4(void) {
	if(display.change) {
		display.top_text = "Dynamic                                 ";
		display.bottom_text = "Press A                                 ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_dynamic_countdown(void) {
	display.top_text = "Dynamic                                 ";
	display.bottom_text = "";

	char bottom_str[50];
	strcpy(bottom_str, "Starting in: ");

	display.bottom_text = strcat(bottom_str, countdown_display_table[display.countdown]);

	display_write_string(display.top_text);
	display_write_string(display.bottom_text);
}

void display_dynamic_calibrate(void) {
	if(display.change) {
		display.top_text = "Align table edge                        ";
		display.bottom_text = "Press A                                 ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_welcome(void) {
	if(display.change) {
		display.top_text = "WELCOME                                 ";
		display.bottom_text = "Press START                             ";
		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}

}

void display_intervals_countdown(void) {
	display.top_text = "Intervals                               ";
	display.bottom_text = "";

	char bottom_str[50];
	strcpy(bottom_str, "Starting in: ");

	display.bottom_text = strcat(bottom_str, countdown_display_table[display.countdown]);

	display_write_string(display.top_text);
	display_write_string(display.bottom_text);
}

void display_intervals_high(void) {
	if(display.change) {
		display.top_text = "Select Range                            ";
		display.bottom_text = "A: Long Range                           ";

		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_intervals_medium(void) {
	if(display.change) {
		display.top_text = "Select Range                            ";
		display.bottom_text = "A: Medium Range                         ";

		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_intervals_low(void) {
	if(display.change) {
		display.top_text = "Select Range                            ";
		display.bottom_text = "A: Short Range                          ";

		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_playback_calibrate(void) {
	if(display.change) {
		display.top_text = "Go to back right                        ";
		display.bottom_text = "A: Ready                                ";

		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}

void display_not_calibrated(void) {
	if (display.change) {
		display.top_text = "Please calibrate                        ";
		display.bottom_text = "B: Back                                 ";

		display_write_string(display.top_text);
		display_write_string(display.bottom_text);
		display.change = false;
	}
}


void display_send_data(char data) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	int err = HAL_I2C_Master_Transmit (&hi2c3, DISPLAY_ADDR,(uint8_t *) data_t, 4, 100);
	if(err != HAL_OK) {
		Error_Handler();
	}
}

void display_init (void)
{
	display.countdown = 3;
	display.speed = 4;
	display.last_pwm = FREEPLAY_LAUNCH_PWM;
	display.change = true;
	display.interval_delay = 5;
	display.ARR = (uint32_t*)(TIM5_OFFSET + TIM_ARR_OFFSET);
	display.interval_count = (uint32_t*)(TIM5_OFFSET + TIM_COUNT_OFFSET);
	display.intervals_distance_last = intervals_select_high;

	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	display_send_cmd(0x30);
	HAL_Delay(5);  // wait for >4.1ms
	display_send_cmd(0x30);
	HAL_Delay(1);  // wait for >100us
	display_send_cmd(0x30);
	HAL_Delay(10);
	display_send_cmd(0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	display_send_cmd(0x28);
	HAL_Delay(1);
	display_send_cmd(ON_OFF_CMD); // display on/off control
	HAL_Delay(1);
	display_send_cmd(CLEAR_CMD);  // clear display
	HAL_Delay(1);
	display_send_cmd(0x06); // entry mode set
	HAL_Delay(1);
	display_send_cmd(0x0C); // display on/off control
	HAL_Delay(1);
}

void display_write_string(char *str){
	__disable_irq();
	while (*str) display_send_data (*str++);
	__enable_irq();
}


void display_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c3, DISPLAY_ADDR,(uint8_t *) data_t, 4, 100);
}
