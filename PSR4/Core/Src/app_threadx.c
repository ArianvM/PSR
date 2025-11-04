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
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACEX_BUFFER_SIZE		64000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define THREAD_STACK_SIZE 			1024
#define LOW_PRIORITY 				10
#define HIGH_PRIORITY 				9
#define TIMER_SEMAPHORE_INIT_CNT 	0
#define TICK_TIME_NS				4.2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];
TX_THREAD low_thread;
TX_THREAD high_thread;
TX_SEMAPHORE timer_semaphore;
uint32_t t_start;
uint32_t t_end;
extern TIM_HandleTypeDef htim2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void low_thread_entry(ULONG initial_input);
void high_thread_entry(ULONG initial_input);
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
  TX_BYTE_POOL *pool_ptr = (TX_BYTE_POOL *) memory_ptr;
  VOID *thread_memory_ptr;

  printf("Hello World");
  // ---- Low Thread ----

  ret = tx_byte_allocate(pool_ptr, &thread_memory_ptr, THREAD_STACK_SIZE, TX_NO_WAIT);
  ret = tx_thread_create(&low_thread,
	  "Low Thread", low_thread_entry,
	  0x00 , thread_memory_ptr,
	  THREAD_STACK_SIZE, LOW_PRIORITY,
	  LOW_PRIORITY, 0,
	  TX_AUTO_START);

  // ---- High Thread ----

  ret = tx_byte_allocate(pool_ptr, &thread_memory_ptr, THREAD_STACK_SIZE, TX_NO_WAIT);
  ret = tx_thread_create(&high_thread,
	  "High Thread", high_thread_entry,
	  0x00 , thread_memory_ptr,
	  THREAD_STACK_SIZE, HIGH_PRIORITY,
	  HIGH_PRIORITY, 0,
	  TX_AUTO_START);


  // ---- Timer Semaphore ----

  ret = tx_semaphore_create(&timer_semaphore, "Timer Semaphore", TIMER_SEMAPHORE_INIT_CNT);
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
void low_thread_entry(ULONG initial_input){
	while(1) {
		t_start = htim2.Instance->CNT;
		// Put Timer Semaphore
		tx_semaphore_put(&timer_semaphore);
		t_end = htim2.Instance->CNT;
		uint32_t ticks = t_end - t_start;
		float tick_time = ticks * TICK_TIME_NS;
		// printf("put() takes %d ticks\r\n", ticks);
		// printf("put() takes %f ns\r\n", tick_time);
		// Sleep 1s
		tx_thread_sleep(10);
	}
}
void high_thread_entry(ULONG initial_input){
	while(1) {
		// Get Timer Semaphore
		tx_semaphore_get(&timer_semaphore, TX_WAIT_FOREVER);
		t_end = htim2.Instance->CNT;
		// print t_end - t_start
		uint32_t ticks = (t_end - t_start);
		float tick_time = (float)ticks * TICK_TIME_NS;
		printf("\n Context switch takes %d ticks\r\n", ticks);
		printf("\n Context switch takes %f ns\r\n", tick_time);
	}
}
/* USER CODE END 1 */
