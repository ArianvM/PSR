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
#define LED_THREAD_STACK_SIZE	4096
#define LED_THREAD_PRIORITY		10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];

TX_THREAD led_thread;

TX_QUEUE q_http;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void led_thread_entry(ULONG init);
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

  size_t msg_size = sizeof(UINT) ;
  size_t queue_size = 4*msg_size;

  ret = tx_byte_allocate(bytePool, &pointer, queue_size, TX_NO_WAIT);

  if (ret != TX_SUCCESS) {
	  printf("Allocation Err: %d\n", ret);
	  return ret;
  }

  ret = tx_queue_create(&q_http, "HTTP Queue",
		  msg_size,
		  pointer, queue_size);
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
	UINT msg;
	while(1)
	{
		tx_queue_receive(&q_http, &msg, TX_WAIT_FOREVER);
		printf("LED THREAD: MSG AVAILABLE\n");
		switch (msg) {
		case 1:
			printf("LED THREAD: LED1\n");
			HAL_GPIO_TogglePin(LED1_G_GPIO_Port, LED1_G_Pin);
			tx_thread_sleep(50);
			HAL_GPIO_TogglePin(LED1_G_GPIO_Port, LED1_G_Pin);
			break;
		case 2:
			printf("LED THREAD: LED2\n");
			HAL_GPIO_TogglePin(LED2_G_GPIO_Port, LED2_G_Pin);
			tx_thread_sleep(50);
			HAL_GPIO_TogglePin(LED2_G_GPIO_Port, LED2_G_Pin);
			break;
		case 3:
			printf("LED THREAD: LED3\n");
			HAL_GPIO_TogglePin(LED3_G_GPIO_Port, LED3_G_Pin);
			tx_thread_sleep(50);
			HAL_GPIO_TogglePin(LED3_G_GPIO_Port, LED3_G_Pin);
			break;
		}
		tx_thread_sleep(50);
	}
}

/* USER CODE END 1 */
