#include "serialDisplay.h"
#include <stdint.h>
#include "main.h"

display_t display;

/*
 * General TODO;
 *  - Finalize buttons for mn
 */

// Increases ball count by one and updates display
void increment_ball_count() {
	++display.ball_count;

	char text[20];
	sprintf(text, "%d", display.ball_count);
	display_write_string("Balls : " + text);
	display.top_text = "Balls: " + text;
	display_write_string(display.bottom_text);
}

// Decreases ball count by one and updates display
void decrement_ball_count(){
	--display.ball_count;

	// Update display
	char text[20];
	sprintf(text, "%d", display.ball_count);
	display_write_string("Balls : " + text);
	display.top_text = "Balls: " + text;
	display_write_string(display.bottom_text);
}


// Resets value of ball count to BALL_COUNT defined in serialDisplay.h
void display_reset_ball_count(){
	display.ball_count = BALL_COUNT;

	// Update display
	char text[20];
	sprintf(text, "%d", display.ball_count);
	display_write_string("Balls : " + text);
	display.top_text = "Balls: " + text;
	display_write_string(display.bottom_text);
}

void display_freeplay() {
	display.bottom_text = "Free-Play";
	display.top_text = "Launch: A"; // TODO: what button is this
	display_write_string(display.bottom_text);
	display_write_string(display.top_text);
}

void display_playback_record() {
	display.bottom_text = "RECORDING...";
	display.top_text = "RECORDING";
	display_write_string(display.bottom_text);
	display_write_string(display.top_text);
}

void display_playback_relocate(){
	display.bottom_text = "Play Back";
	display.top_text = "Relocate";
	display_write_string(display.bottom_text);
	display_write_string(display.top_text);
}

void display_playback_begin(){
	display.bottom_text = "Play Back";
	display.top_text = "Press A"; // TODO: what button is this
	display_write_string(display.bottom_text);
	display_write_string(display.top_text);
}

void display_

void display_send_data(char data) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c3, DISPLAY_ADDR,(uint8_t *) data_t, 4, 100);
}

void display_init (void)
{
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
	while (*str) display_send_data (*str++);
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
