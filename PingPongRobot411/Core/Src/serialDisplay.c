#include "serialDisplay.h"

display_t display;

void increment_ball_count() {
	++display.ball_count;
}

void display_decrease_ball_count(){

}

void display_reset_ball_count(){

}

void display_init(){
	// Based on sequence described in datasheet
	HAL_Delay(20);
	//HAL_I2C_Master_Transmit(&hi2c1, SAD_W_M, 0x30, 1, 1000);
}

void display_write_string(char *str){

}

void writeHelloWorld(){

}
