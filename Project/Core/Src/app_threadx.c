/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "motor_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACEX_BUFFER_SIZE		64000
#define TEST_THREAD_SIZE		1024
#define TEST_THREAD_PRIORITY	10

#define CONTROLLER_THREAD_SIZE		1024
#define CONTROLLER_TREAD_PRIORITY	9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];

TX_THREAD test_thread;
TX_THREAD controller_thread;
TX_QUEUE q_motor_ref;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
int sign(int x);
void test_thread_entry(ULONG init);

void controller_thread_entry(ULONG constants);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
	TX_BYTE_POOL *bytePool = (TX_BYTE_POOL *) memory_ptr;
	VOID *pointer;

	// Initialize msg-queue for motor controller
	UINT msg_size = 1;
	UINT num_msg = 10;
	ULONG queue_size = num_msg * (msg_size * 4);
	tx_byte_allocate(bytePool, &pointer, queue_size, TX_NO_WAIT);

	ret = tx_queue_create(&q_motor_ref, "MOTOR_POSITION_REFERENCE", sizeof(UINT), pointer, queue_size);

	// Initialize motor
	if(motor_init() != TX_SUCCESS) {
		return ret;
	};



  // stack allocation for Test thread
  ret = tx_byte_allocate(bytePool, &pointer, TEST_THREAD_SIZE, TX_NO_WAIT);

  if (ret != TX_SUCCESS)
    return ret;

/**
  // Test thread create
  ret = tx_thread_create(&test_thread, "Test thread", test_thread_entry, 1234,
	  pointer, TEST_THREAD_SIZE, TEST_THREAD_PRIORITY, TEST_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);


  // stack allocation for Controller thread
  ret = tx_byte_allocate(bytePool, &pointer, TEST_THREAD_SIZE, TX_NO_WAIT);

  if (ret != TX_SUCCESS)
    return ret;
**/

  // Controller thread create
  uint8_t controller_constants[2] = {1, 1};
  ret = tx_thread_create(&controller_thread, "Controller thread", controller_thread_entry, (ULONG)controller_constants,
	  pointer, CONTROLLER_THREAD_SIZE,  CONTROLLER_TREAD_PRIORITY, CONTROLLER_TREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);



  if (ret != TX_SUCCESS)
    return ret;


  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  tx_trace_enable(&tracex_buffer, TRACEX_BUFFER_SIZE, 30);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

int sign(int x) {
	return x < 0 ? -1 : 1;
}
void test_driver() {

}

void test_thread_entry(ULONG init)
{
	UINT test_vals[] = {10, 0, 50, 100, 200, 500, 400, 200, 0};

	// motor_init();

	while(1)
	{
		printf("---TESTING MOTOR Controller---\n");
		tx_thread_sleep(200);
		for (int i = 0; i < sizeof(test_vals)/sizeof(int); i++) {
			UINT val = test_vals[i];
			printf("position %d\n", val);
			tx_queue_send(&q_motor_ref, &val, TX_NO_WAIT);
			tx_thread_sleep(50000);
		}
	}
}

void controller_thread_entry(ULONG constants) {
	// uint8_t* controller_constants = (uint8_t*)constants;
	// int kp = 1; // controller_constants[0];
	// int ki = 1; // controller_constants[1];
	int min_speed = 15;
	uint32_t pos_curr;
	motor_get_pos(&pos_curr);
	int32_t err_curr, err_prev = 0;	// Might be array for longer filter
	int32_t pos_ref = pos_curr;
	double speed = 0;

	double P, D;
	double I = 0;
	double kp = 1;
	double ki = 0.05;
	double kd = 0.0;

	double Ts = 1.0/TX_TIMER_TICKS_PER_SECOND;

	while (1) {

		tx_queue_receive(&q_motor_ref, &pos_ref, TX_NO_WAIT);
		motor_get_pos(&pos_curr);

		printf("Current position %d\n", pos_curr);

		printf("Reference position %d\n", pos_ref);
		// int32_t err_curr = pos_curr > 0 ? abs(pos_ref) - pos_curr : pos_ref - pos_curr;
		err_curr = pos_ref - pos_curr;
		if (err_curr > MOTOR_MAX_POS/2) err_curr -= MOTOR_MAX_POS;
		if (err_curr < -MOTOR_MAX_POS/2) err_curr += MOTOR_MAX_POS;

		printf("Current error %d\n", err_curr);

		P = kp * err_curr;

		I += ki * Ts * err_curr;

		D = kd * (err_curr - err_prev) / Ts;

		speed = P + I + D;
		if (abs(speed) < min_speed) speed = 0;
		printf("Setting Speed: %d\n", speed);
		motor_set_speed(speed);

		err_prev = err_curr;


		tx_thread_sleep(Ts * TX_TIMER_TICKS_PER_SECOND);
	}

}
/* USER CODE END 1 */
