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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

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
#define THREAD_STACK_SIZE		1024
#define BLINK_1_PRIORITY		11
#define BLINK_2_PRIORITY		11
#define BLINK_3_PRIORITY		11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];
TX_THREAD blink_1_thread;
TX_THREAD blink_2_thread;
TX_THREAD blink_3_thread;
TX_MUTEX mutex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void blink_1_thread_entry(ULONG initial_input);
void blink_2_thread_entry(ULONG initial_input);
void blink_3_thread_entry(ULONG initial_input);
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

  ULONG initial_pool_size;
  ret = tx_byte_pool_info_get(pool_ptr,
		  TX_NULL,
		  &initial_pool_size,
		  TX_NULL,
		  TX_NULL,
		  TX_NULL,
		  TX_NULL);

  ret = tx_byte_allocate(pool_ptr,
		  &thread_memory_ptr,
		  THREAD_STACK_SIZE,
		  TX_NO_WAIT);
  ret = tx_thread_create(&blink_1_thread,
		  "Blink 1",
		  blink_1_thread_entry,
		  0x0420,
		  thread_memory_ptr,
		  THREAD_STACK_SIZE,
		  BLINK_1_PRIORITY,
		  BLINK_1_PRIORITY,
		  0, TX_AUTO_START);

  ULONG current_pool_size;

  ret = tx_byte_pool_info_get(pool_ptr,
		  TX_NULL,
		  &current_pool_size,
		  TX_NULL,
		  TX_NULL,
		  TX_NULL,
		  TX_NULL);

  ULONG overhead = initial_pool_size - current_pool_size;

  printf("Allocating memory has an overhead of %ld bytes.\n", overhead);

  ret = tx_byte_allocate(pool_ptr,
		  &thread_memory_ptr,
		  THREAD_STACK_SIZE,
		  TX_NO_WAIT);
  ret = tx_thread_create(&blink_2_thread,
		  "Blink 2",
		  blink_2_thread_entry,
		  0x0420,
		  thread_memory_ptr,
		  THREAD_STACK_SIZE,
		  BLINK_2_PRIORITY,
		  BLINK_2_PRIORITY,
		  0, TX_AUTO_START);

  ret = tx_byte_allocate(pool_ptr,
		  &thread_memory_ptr,
		  THREAD_STACK_SIZE,
		  TX_NO_WAIT);
  ret = tx_thread_create(&blink_3_thread,
		  "Blink 3",
		  blink_3_thread_entry,
		  0x0420,
		  thread_memory_ptr,
		  THREAD_STACK_SIZE,
		  BLINK_3_PRIORITY,
		  BLINK_3_PRIORITY,
		  0, TX_AUTO_START);

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

/* USER CODE BEGIN 2 */
void blink_1_thread_entry(ULONG initial_input) {
	tx_mutex_create(&mutex,
			"My Mutex",
			TX_NO_INHERIT);
	while (1) {
		if (tx_mutex_get(&mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
			tx_thread_sleep(100);
			HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
			tx_mutex_put(&mutex);
		}
	}
}
void blink_2_thread_entry(ULONG initial_input) {
	while (1) {
		if (tx_mutex_get(&mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
			tx_thread_sleep(100);
			HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
			tx_mutex_put(&mutex);
		}
	}
}
void blink_3_thread_entry(ULONG initial_input) {
	while (1) {
		if (tx_mutex_get(&mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
		{
			HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
			tx_thread_sleep(100);
			HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
			tx_mutex_put(&mutex);
		}
	}
}
/* USER CODE END 2 */
