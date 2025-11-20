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
#include <motor_driver_set.h>
#include "app_threadx.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACEX_BUFFER_SIZE		64000
#define TEST_THREAD_SIZE		1024
#define TEST_THREAD_PRIORITY	10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];

TX_THREAD test_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void test_thread_entry(ULONG init);
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
  ret = tx_byte_allocate(bytePool, &pointer, TEST_THREAD_SIZE, TX_NO_WAIT);

  if (ret != TX_SUCCESS)
    return ret;


  // LED thread create
  ret = tx_thread_create(&test_thread, "LED thread", test_thread_entry, 1234,
	  pointer, TEST_THREAD_SIZE, TEST_THREAD_PRIORITY, TEST_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

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

void test_thread_entry(ULONG init)
{
	motor_init();
	int test_vals[] = {10, -20, 30, -50, 100, -150, 200, -300, 400, -500};

	while(1)
	{
		printf("---TESTING MOTOR DRIVER---\n");
		tx_thread_sleep(20);
		for (int i = 0; i < sizeof(test_vals)/sizeof(int); i++) {
			int val = test_vals[i];
			printf("Setting position %d\n", val);
			motor_set_speed(val);
			tx_thread_sleep(500);
		}
	}
}

/* USER CODE END 1 */
