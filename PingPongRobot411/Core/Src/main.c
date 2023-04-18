/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdint.h>
#include "ultrasonic.h"
#include "odometry.h"
#include "hbridge.h"
#include "n64.h"
#include "serialDisplay.h"
#include "controller.h"
#include "solenoid.h"
#include "drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern hbridge_t hbridges[4];
extern display_t display;
extern solenoid_t solenoid;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	NVIC_SystemReset();
	n64_t n64_status_prev;
	n64_t n64_status_curr;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Initiliaze ultrasonics.
  for (unsigned int i = 0; i < NUM_ULTRAS; ++i) {
	  init_ultra(ultras + i);
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // Start Ultrasonic timer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Start hbridge timers
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  init_hbridges();
  solenoid_init();
  display_init();

  n64_init(&n64_status_prev);
  n64_init(&n64_status_curr);
  n64_read(N64_RESET, NULL);

  init_odom(&odometry);
  init_bno();
  init_adxl();

  // Start IMU timer
  HAL_TIM_Base_Start_IT(&htim10);

  // State machine
  display_state curr_state = welcome;
  display_state prev_state = welcome;
  float intervals_pwm = 0;
  // Is htim5 interrupt
  bool htim5_int = 0;

  bool set_freeplay_pwm = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int n64_count = 0;


  while (1)
  {
	  if (n64_count == 25) {
		  n64_read(N64_RESET, NULL);
		  HAL_Delay(50); // without some delay the n64 inputs are finnicky
		  n64_count = 0;
	  }
	  ++n64_count;

	  // get input from controller
	  n64_read(N64_POLL, &n64_status_curr);

	  // State machine
	  switch(prev_state) {
		  case welcome: {
			  display.balls_displayed = false;
			  display_welcome();

			  // Go to menu
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_Start)){
				  curr_state = menu_1;
				  display.change = true;
			  }

			  break;
		  }

		  case menu_1: {
			  display.balls_displayed = false;
			  display_menu_1();
			  controller_drive(&n64_status_curr);
			  // Go to next menu
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  curr_state = menu_2;
				  display.change = true;
			  }

			  // Go to previous menu
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = menu_4;
				  display.change = true;
			  }

			  // Or go to launching
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  set_freeplay_pwm = true;
				  curr_state = launch;
				  display.change = true;
			  }

			  break;
		  }

		  case menu_2: {
			  display.balls_displayed = false;
			  display_menu_2();
			  controller_drive(&n64_status_curr);
			  // Go to next menu
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  curr_state = menu_3;
				  display.change = true;
			  }

			  // Go to previous menu
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = menu_1;
				  display.change = true;
			  }

			  // Go to playback record
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  if (!odometry.bno_calibrated) {
					  curr_state = pb_not_calibrated;
				  }
				  else {
					  curr_state = pb_calibrate;
				  }

				  display.change = true;

				  // Reset path before creating new setpoints.
				  reset_path(&path);
			  }

			  break;
		  }

		  case menu_3: {
			  display.balls_displayed = false;
			  display_menu_3();
			  controller_drive(&n64_status_curr);
			  // Go to next menu
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  curr_state = menu_4;
				  display.change = true;
			  }

			  // Go to previous menu
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = menu_2;
				  display.change = true;
			  }

			  // Go to intervals
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  curr_state = display.intervals_distance_last;
				  display.change = true;
			  }

			  break;
		  }

		  case menu_4: {
			  display.balls_displayed = false;
			  display_menu_4();
			  controller_drive(&n64_status_curr);
			  // Go to next menu
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  curr_state = menu_1;
				  display.change = true;
			  }

			  // Go to previous menu
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = menu_3;
				  display.change = true;
			  }

			  // Go to dynamic calibrate
			  else if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  curr_state = dynamic_not_calibrated;
				  display.change = true;
			  }

			  break;
		  }

		  case launch: {
			  display.balls_displayed = true;
			  display_freeplay();

			  if (set_freeplay_pwm) {
				  controller_start_launcher(LAUNCH_START_PWM, display.last_pwm);
				  set_freeplay_pwm = false;
			  }

			  // Launch ball
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  controller_launch_ball();
			  }
			  else {
				  controller_drive(&n64_status_curr);
			  }

			  // Increment and decrement speed
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  if (display.speed < 4) {
					  display.speed++;
				  } else {
					  display.speed = 1;
				  }
			  }

			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  if (display.speed > 1) {
					  display.speed--;
				  } else {
					  display.speed = 4;
				  }
			  }

			  // only adjust if speed has changed
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU) ||
					  n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD) ) {

				  // update display
				  display_freeplay_speed();

				  // Decide speed
				  switch(display.speed) {
					  case 4: {
						  controller_adjust_launch_speed(FREEPLAY_LAUNCH_PWM);
						  display.last_pwm = FREEPLAY_LAUNCH_PWM;
						  break;
					  }
					  case 3: {
						  controller_adjust_launch_speed(INTERVALS_PWM_HIGH);
						  display.last_pwm = INTERVALS_PWM_HIGH;
						  break;
					  }
					  case 2: {
						  controller_adjust_launch_speed(INTERVALS_PWM_MEDIUM);
						  display.last_pwm = INTERVALS_PWM_MEDIUM;
						  break;
					  }
					  case 1: {
						  controller_adjust_launch_speed(INTERVALS_PWM_LOW);
						  display.last_pwm = INTERVALS_PWM_LOW;
						  break;
					  }
				  }
			  }

			  // Exit
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  set_PWM(hbridges + 2, 0);
				  set_PWM(hbridges + 3, 0);
				  curr_state = menu_1;
				  display.change = true;
			  }

			  break;
		  }

		  case dynamic_not_calibrated: {
			  display.balls_displayed = false;
			  controller_drive(&n64_status_curr);

			  display_not_calibrated();

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_4;
				  display.change = true;
			  }
			  if (odometry.bno_calibrated) {
				  curr_state = dynamic_calibrate;
				  display.change= true;
			  }

			  break;
		  }

		  case dynamic_calibrate: {
			  display.balls_displayed = false;
			  controller_drive(&n64_status_curr);

			  display_dynamic_calibrate();

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_4;
				  display.change = true;
			  }
			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  curr_state = dynamic_countdown;
				  display.countdown = 3;
				  display.change = true;
			  }

			  break;
		  }

		  case dynamic_countdown: {
			  display.balls_displayed = false;
			  display_dynamic_countdown();

			  // Delay
			  HAL_Delay(2000);

			  if (display.countdown == 2) {
					// Start launcher
					set_PWM(hbridges + 2, -1 * LAUNCH_START_PWM);
					set_PWM(hbridges + 3, LAUNCH_START_PWM);
					  --display.countdown;
			  }

			  else if (display.countdown == 1) {

					// Spin launcher to final speed
					set_PWM(hbridges + 2, -1 * INTERVALS_PWM_MEDIUM);
					set_PWM(hbridges + 3, INTERVALS_PWM_MEDIUM);
					  --display.countdown;
			  }

			  // If done, exit
			  else if(display.countdown == 0) {
				  display.countdown = 3;
				  curr_state = dynamic;
				  display.change = true;
			  }

			  // If not, decrement
			  else {
				  --display.countdown;
			  }

			  break;
		  }

		  case dynamic: {
			  display.balls_displayed = true;
			  display_interval_begin();

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_4;
				  display.change = true;
			  }

			  move_dynamic(&odometry, &ultras);

			  // Launch at constant interval only if interrupt hasn't been started
			  if(!htim5_int){
				  htim5_int = true;

				  // Start interrupt with default of 5 seconds
				  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK ) {
					  Error_Handler();
				  }
			  }

			  // Increment and decrement interrupt speed by two seconds 1-11 seconds
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  if(display.interval_delay < 11) {
					  display.change = true;
					  display.interval_delay += 2;
					  *(display.ARR) = (uint32_t) (display.interval_delay * 10000);
					  *(display.interval_count) = (uint32_t) 0;
				  }
			  }

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  if(display.interval_delay > 1) {
					  display.change = true;
					  display.interval_delay -= 2;
					  *(display.ARR) = (uint32_t) (display.interval_delay * 10000);
					  *(display.interval_count) = (uint32_t) 0;
				  }
			  }

			  // Exit and cancel interrupt
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
					// Stop launcher
					set_PWM(hbridges + 2, 0.0f);
					set_PWM(hbridges + 3, 0.0f);

				  curr_state = menu_4;
				  display.change = true;
				  htim5_int = false;
				  if(HAL_TIM_Base_Stop_IT(&htim5) != HAL_OK) {
					  Error_Handler();
				  }
			  }

			  break;
		  }

		  case pb_not_calibrated: {
			  display.balls_displayed = false;
			  controller_drive(&n64_status_curr);

			  display_not_calibrated();

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_2;
				  display.change = true;
			  }
			  if (odometry.bno_calibrated) {
				  curr_state = pb_calibrate;
				  display.change= true;
			  }

			  break;
		  }

		  case pb_calibrate: {
			  display.balls_displayed = false;
			  controller_drive(&n64_status_curr);

			  display_playback_calibrate();

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  curr_state = pb_record;
				  adxl_calibrate(&odometry);
				  display.change = true;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_2;
				  display.change = true;
			  }

			  break;
		  }

		  case pb_record: {
			  display.balls_displayed = false;
			  display_playback_record();
			  controller_drive(&n64_status_curr);

					  // A button: drop setpoint
				if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  add_setpoint(&odometry, &path);
				}

			  // B button: finish path
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B) &&
				 path.num_valid > 0) {
				  curr_state = pb_go;
				  display.change = true;

				  // Set commands from path planning to be active.
				  path.cmds_active = true;
			  }

				  break;
			  }

		  case pb_go: {
			  display.balls_displayed = true;
			  display_playback_begin();

				// B button: exit back to menu
				if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_2;
				  display.change = true;

				  // Disable commands from path planning.
				  path.cmds_active = false;
			  }

			  break;
		  }

		  case intervals_countdown: {
			  display.balls_displayed = false;
			  display_intervals_countdown();

			  // Delay
			  HAL_Delay(2000);

			  if (display.countdown == 2) {
					// Start launcher
					set_PWM(hbridges + 2, -1 * LAUNCH_START_PWM);
					set_PWM(hbridges + 3, LAUNCH_START_PWM);
					  --display.countdown;
			  }

			  else if (display.countdown == 1) {

					// Spin launcher to final speed
					set_PWM(hbridges + 2, -1 * intervals_pwm);
					set_PWM(hbridges + 3, intervals_pwm);
					  --display.countdown;
			  }

			  // If done, exit
			  else if(display.countdown == 0) {
				  display.countdown = 3;
				  curr_state = intervals;
				  display.change = true;
			  }

			  // If not, decrement
			  else {
				  --display.countdown;
			  }

			  break;
		  }


		  case intervals: {
			  display.balls_displayed = true;
			  display_intervals_begin();

			  // Launch at constant interval only if interrupt hasn't been started
			  if(!htim5_int){
				  htim5_int = 1;

				  // Start interrupt with default of 5 seconds
				  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK ) {
					  Error_Handler();
				  }
			  }

			  // Increment and decrement interrupt speed by two seconds 1-11 seconds
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  if(display.interval_delay < 11) {
					  display.change = true;
					  display.interval_delay += 2;
					  *(display.ARR) = (uint32_t) (display.interval_delay * 10000);
					  *(display.interval_count) = (uint32_t) 0;
				  }
			  }

			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  if(display.interval_delay > 1) {
					  display.change = true;
					  display.interval_delay -= 2;
					  *(display.ARR) = (uint32_t) (display.interval_delay * 10000);
					  *(display.interval_count) = (uint32_t) 0;
				  }
			  }

			  // Exit and cancel interrupt
			  if(n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
					// Stop launcher
					set_PWM(hbridges + 2, 0.0f);
					set_PWM(hbridges + 3, 0.0f);

				  curr_state = menu_3;
				  display.change = true;
				  htim5_int = 0;
				  if(HAL_TIM_Base_Stop_IT(&htim5) != HAL_OK) {
					  Error_Handler();
				  }
			  }

			  break;
		  }

		  case intervals_select_high: {
			  display.balls_displayed = false;
			  display_intervals_high();
			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  intervals_pwm = INTERVALS_PWM_HIGH;
				  display.intervals_distance_last = intervals_select_high;
				  display.countdown = 3;
				  curr_state = intervals_countdown;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_3;
				  display.change = true;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  display.change = true;
				  curr_state = intervals_select_medium;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = intervals_select_low;
				  display.change = true;
			  }
			  break;
		  }

		  case intervals_select_medium: {
			  display.balls_displayed = false;
			  display_intervals_medium();
			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  intervals_pwm = INTERVALS_PWM_MEDIUM;
				  display.intervals_distance_last = intervals_select_medium;
				  display.countdown = 3;
				  curr_state = intervals_countdown;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_3;
				  display.change = true;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  curr_state = intervals_select_low;
				  display.change = true;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = intervals_select_high;
				  display.change = true;
			  }
			  break;
		  }

		  case intervals_select_low: {
			  display.balls_displayed = false;
			  display_intervals_low();
			  if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_A)) {
				  intervals_pwm = INTERVALS_PWM_LOW;
				  display.intervals_distance_last = intervals_select_low;
				  display.countdown = 3;
				  curr_state = intervals_countdown;

			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_B)) {
				  curr_state = menu_3;
				  display.change = true;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CD)) {
				  curr_state = intervals_select_high;
				  display.change = true;
			  }
			  else if (n64_button_pressed(&n64_status_prev, &n64_status_curr, N64_CU)) {
				  curr_state = intervals_select_medium;
				  display.change = true;
			  }
			  break;
		  }

	  }

	  prev_state = curr_state;

	  n64_copy(&n64_status_prev, &n64_status_curr); // update n64 state
	  HAL_Delay(50); // without some delay the n64 inputs are finnicky


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1599;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, N64_transmit_Pin|LD2_Pin|Hbridge_0_NDIR_Pin|Hbridge_0_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Solenoid_enable__gate0_Pin|Hbridge_3_NDIR_Pin|Hbridge_3_DIR_Pin|Hbridge_2_NDIR_Pin
                          |Hbridge_2_DIR_Pin|Hbridge_1_NDIR_Pin|Hbridge_1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ultrasonic_echo_Pin Ultrasonic_echoC1_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_echo_Pin|Ultrasonic_echoC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : N64_transmit_Pin LD2_Pin Hbridge_0_NDIR_Pin Hbridge_0_DIR_Pin */
  GPIO_InitStruct.Pin = N64_transmit_Pin|LD2_Pin|Hbridge_0_NDIR_Pin|Hbridge_0_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : N64_receive_Pin */
  GPIO_InitStruct.Pin = N64_receive_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(N64_receive_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_breakbeam_sensor_Pin */
  GPIO_InitStruct.Pin = IR_breakbeam_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_breakbeam_sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Solenoid_enable__gate0_Pin Hbridge_3_NDIR_Pin Hbridge_3_DIR_Pin Hbridge_2_NDIR_Pin
                           Hbridge_2_DIR_Pin Hbridge_1_NDIR_Pin Hbridge_1_DIR_Pin */
  GPIO_InitStruct.Pin = Solenoid_enable__gate0_Pin|Hbridge_3_NDIR_Pin|Hbridge_3_DIR_Pin|Hbridge_2_NDIR_Pin
                          |Hbridge_2_DIR_Pin|Hbridge_1_NDIR_Pin|Hbridge_1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	set_PWM(hbridges + 0, 0);
	set_PWM(hbridges + 1, 0);
	set_PWM(hbridges + 2, 0);
	set_PWM(hbridges + 3, 0);
	__disable_irq();
	while (1)
	{

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
