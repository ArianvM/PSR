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
#define THREAD_STACK_SIZE		1024
#define THREAD_PRIORITY			10
#define EV_BTN1					(1 << 0)
#define EV_BTN2					(1 << 1)
#define EV_BTN3					(1 << 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tracex_buffer[TRACEX_BUFFER_SIZE];
TX_EVENT_FLAGS_GROUP ev_flags_group;
TX_THREAD led1_thread;
TX_THREAD led2_thread;
TX_THREAD led3_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID led1_thread_entry(ULONG initial_input);
VOID led2_thread_entry(ULONG initial_input);
VOID led3_thread_entry(ULONG initial_input);
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
  VOID* stack_ptr;

  ret = tx_byte_allocate(pool_ptr,
	  &stack_ptr, THREAD_STACK_SIZE,
	  TX_NO_WAIT);

  ret = tx_thread_create(&led1_thread,
	  "LED1 Thread", led1_thread_entry,
	  0x0420, stack_ptr,
	  THREAD_STACK_SIZE, THREAD_PRIORITY,
	  THREAD_PRIORITY, 5,
	  TX_AUTO_START);
/*
  ret = tx_byte_allocate(pool_ptr,
	  &stack_ptr, THREAD_STACK_SIZE,
	  TX_NO_WAIT);

  ret = tx_thread_create(&led2_thread,
	  "LED2 Thread", led2_thread_entry,
	  0x0420, stack_ptr,
	  THREAD_STACK_SIZE, THREAD_PRIORITY,
	  THREAD_PRIORITY, 5,
	  TX_AUTO_START);

  ret = tx_byte_allocate(pool_ptr,
	  &stack_ptr, THREAD_STACK_SIZE,
	  TX_NO_WAIT);

  ret = tx_thread_create(&led3_thread,
	  "LED3 Thread", led3_thread_entry,
	  0x0420, stack_ptr,
	  THREAD_STACK_SIZE, THREAD_PRIORITY,
	  THREAD_PRIORITY, 5,
	  TX_AUTO_START);
	  */
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
VOID led1_thread_entry(ULONG initial_input) {

	UINT ret;
		ret = tx_event_flags_create(&ev_flags_group,
				"My Events");
			ULONG flags;
		while (1) {

			ret = tx_event_flags_get(&ev_flags_group,
									EV_BTN1 | EV_BTN2 | EV_BTN3,
									TX_OR_CLEAR,
									&flags, TX_NO_WAIT);

			if ((flags & EV_BTN1)
					|| (flags & EV_BTN2)
					|| (flags & EV_BTN3)) {	// Any button pressed
				HAL_GPIO_TogglePin(LED1_G_GPIO_Port, LED1_G_Pin);
				if ( (flags & (EV_BTN1 | EV_BTN2))			// BTN1 + BTN2
						|| (flags & (EV_BTN1 | EV_BTN3))	// BTN1 + BTN3
						|| (flags & (EV_BTN2 | EV_BTN3)))	// BTN2 + BTN3
				{
					HAL_GPIO_TogglePin(LED2_G_GPIO_Port, LED2_G_Pin);
				}
			}
		}
		/*
	UINT ret;
	ret = tx_event_flags_create(&ev_flags_group,
			"My Events");
	ULONG flags;
	while (1) {

		ret = tx_event_flags_get(&ev_flags_group,
								EV_BTN1,
								TX_OR_CLEAR,
								&flags, TX_NO_WAIT);

		if ((flags & EV_BTN1))
			HAL_GPIO_TogglePin(LED1_G_GPIO_Port, LED1_G_Pin);

	}
*/
}

VOID led2_thread_entry(ULONG initial_input) {
	UINT ret;
	ULONG flags;
	while (1) {

		ret = tx_event_flags_get(&ev_flags_group,
								EV_BTN2,
								TX_OR_CLEAR,
								&flags, TX_NO_WAIT);

		if ((flags & EV_BTN2))
			HAL_GPIO_TogglePin(LED2_G_GPIO_Port, LED2_G_Pin);

	}
};

VOID led3_thread_entry(ULONG initial_input) {
	UINT ret;
	ULONG flags;
	while (1) {

		ret = tx_event_flags_get(&ev_flags_group,
								EV_BTN3,
								TX_OR_CLEAR,
								&flags, TX_NO_WAIT);

		if ((flags & EV_BTN3))
			HAL_GPIO_TogglePin(LED3_G_GPIO_Port, LED3_G_Pin);

	}
};
/* USER CODE END 1 */
