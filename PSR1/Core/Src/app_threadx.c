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
#define THREAD_STACK_SIZE		1024
#define LED_THREAD_PRIORITY		12
#define T2_THREAD_PRIORITY		14
#define T3_THREAD_PRIORITY		13

#define TRACEX_BUFFER_SIZE		64000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t led_thread_stack[THREAD_STACK_SIZE];
TX_THREAD led1_thread;

uint8_t t2_thread_stack[THREAD_STACK_SIZE];
TX_THREAD t2_thread;

uint8_t t3_thread_stack[THREAD_STACK_SIZE];
TX_THREAD t3_thread;

uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID led1_thread_entry(ULONG initial_input);
VOID t2_thread_entry(ULONG initial_input);
VOID t3_thread_entry(ULONG initial_input);
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

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  tx_trace_enable(&tracex_buffer, 64000, 30);

  tx_thread_create(&led1_thread, "LED thread", led1_thread_entry, 0x1234, led_thread_stack, THREAD_STACK_SIZE,
		  LED_THREAD_PRIORITY, LED_THREAD_PRIORITY, 0, TX_AUTO_START);
  tx_thread_create(&t2_thread, "T2 thread", t2_thread_entry, 0x0420, t2_thread_stack, THREAD_STACK_SIZE,
		  T2_THREAD_PRIORITY, T2_THREAD_PRIORITY, 0, TX_AUTO_START);
  tx_thread_create(&t3_thread, "T3 thread", t3_thread_entry, 0x4242, t3_thread_stack, THREAD_STACK_SIZE,
		  T3_THREAD_PRIORITY, T3_THREAD_PRIORITY, 0, TX_AUTO_START);

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
VOID led1_thread_entry(ULONG initial_input)
{
	while (1) {
		tx_trace_user_event_insert(4096, 1, 2, 3, 4);

		printf("Hello from LED tread, Initial input is 0x%04X\n", (unsigned int) initial_input);
		HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
		tx_thread_sleep(100);
	}
}

VOID t2_thread_entry(ULONG initial_input)
{
	while (1)
	{
		printf("This is thread 2, Initial input is 0x%04X\n", (unsigned int) initial_input);
		HAL_Delay(300);
		tx_thread_sleep(100);
	}

}

VOID t3_thread_entry(ULONG initial_input)
{
	while (1)
	{
		printf("This is thread 3, Initial input is 0x%04X\n", (unsigned int) initial_input);
		HAL_Delay(1000);
		tx_thread_sleep(100);
	}

}
/* USER CODE END 1 */
