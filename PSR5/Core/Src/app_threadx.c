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
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "encoder_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACEX_BUFFER_SIZE				64000
#define LED_THREAD_STACK_SIZE			4096
#define POS_RESET_THREAD_STACK_SIZE		1024
#define LED_THREAD_PRIORITY				10
#define POS_RESET_THREAD_PRIORITY		11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];

TX_THREAD led_thread;
TX_THREAD pos_reset_thread;

extern TX_EVENT_FLAGS_GROUP e_driver;
extern TX_SEMAPHORE s_driver_input;
extern TX_SEMAPHORE s_driver_output;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void led_thread_entry(ULONG init);
void pos_reset_thread_entry(ULONG init);
void led_clear_green();
void led_clear_red();
void led_set_speed_low(bool forward);
void led_set_speed_medium(bool forward);
void led_set_speed_high(bool forward);
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

  // stack allocation for LED thread
  ret = tx_byte_allocate(bytePool, &pointer, LED_THREAD_STACK_SIZE, TX_NO_WAIT);

  if (ret != TX_SUCCESS)
	  return ret;


  // LED thread create
  ret = tx_thread_create(&led_thread, "LED thread", led_thread_entry, 1234,
		  pointer, LED_THREAD_STACK_SIZE, LED_THREAD_PRIORITY, LED_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
	  return ret;

  // stack allocation for position reset thread
  ret = tx_byte_allocate(bytePool, &pointer, POS_RESET_THREAD_STACK_SIZE, TX_NO_WAIT);

  if (ret != TX_SUCCESS)
	  return ret;


  // position reset thread create

  ret = tx_thread_create(&pos_reset_thread, "Position reset thread", pos_reset_thread_entry, 1234,
		  pointer, POS_RESET_THREAD_STACK_SIZE, POS_RESET_THREAD_PRIORITY,POS_RESET_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);


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

void led_thread_entry(ULONG init)
{
	encoder_driver_initialize();
	uint32_t position_last;
	uint32_t position_current;

	while(1)
	{
		position_last = position_current;
		encoder_driver_input(&position_current);

		int32_t diff = position_last - position_current;
		printf("CURRENT:%d\r\n", position_current);
		printf("DIFF:%d\r\n", diff);

		led_clear_green();
		led_clear_red();

		if (diff > 100) {led_set_speed_high(true);}
		else if (diff > 50) {led_set_speed_medium(true);}
		else if (diff > 10) {led_set_speed_low(true);}

		else if (diff < -100) {led_set_speed_high(false);}
		else if (diff < -50) {led_set_speed_medium(false);}
		else if (diff < -10) {led_set_speed_low(false);}

		tx_thread_sleep(20);
	}
}

void led_clear_green() {
		HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, SET);
		HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, SET);
		HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, SET);
};

void led_clear_red(){
		HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, SET);
		HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_R_Pin, SET);
		HAL_GPIO_WritePin(LED3_R_GPIO_Port, LED3_R_Pin, SET);
};

void pos_reset_thread_entry(ULONG init){
	ULONG actual;
	while (1) {
		tx_event_flags_get(&e_driver, EV_DRIVER_POSITION_RESET, TX_AND_CLEAR, &actual, TX_NO_WAIT);
		if (actual & EV_DRIVER_POSITION_RESET) {
			printf("---RESET---\r\n");
			encoder_driver_output(0);
		}
	}
};

void led_set_speed_low(bool forward){
	if (forward) {
		HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, RESET);
	} else {
		HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, RESET);
	}
};
void led_set_speed_medium(bool forward){
	led_set_speed_low(forward);

	if (forward) {
		HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, RESET);
	} else {
		HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_R_Pin, RESET);
	}

};

void led_set_speed_high(bool forward){
	led_set_speed_medium(forward);

	if (forward) {
		HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, RESET);
	} else {
		HAL_GPIO_WritePin(LED3_R_GPIO_Port, LED3_R_Pin, RESET);
	}
};


/* USER CODE END 1 */
