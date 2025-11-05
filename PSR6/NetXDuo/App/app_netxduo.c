/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
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
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "main.h"
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
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
/* USER CODE BEGIN PV */
NX_UDP_SOCKET UDPSocket;
TX_EVENT_FLAGS_GROUP ev_sw_flags;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry (ULONG thread_input);
/* USER CODE BEGIN PFP */
ULONG send_udp(char* msg, ULONG ipAddress, ULONG port, NX_PACKET **packet, UCHAR *data_buffer);
/* USER CODE END PFP */

/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR *pointer;

  /* USER CODE BEGIN MX_NetXDuo_MEM_POOL */
  /* USER CODE END MX_NetXDuo_MEM_POOL */

  /* USER CODE BEGIN 0 */
  tx_event_flags_create(&ev_sw_flags, "Button Flags");
  /* USER CODE END 0 */

  /* Initialize the NetXDuo system. */
  nx_system_initialize();

    /* Allocate the memory for packet_pool.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the Packet pool to be used for packet allocation,
   * If extra NX_PACKET are to be used the NX_APP_PACKET_POOL_SIZE should be increased
   */
  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_POOL_ERROR;
  }

    /* Allocate the memory for Ip_Instance */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

   /* Create the main NX_IP instance */
  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

    /* Allocate the memory for ARP */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Enable the ARP protocol and provide the ARP cache size for the IP instance */

  /* USER CODE BEGIN ARP_Protocol_Initialization */

  /* USER CODE END ARP_Protocol_Initialization */

  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the ICMP */

  /* USER CODE BEGIN ICMP_Protocol_Initialization */

  /* USER CODE END ICMP_Protocol_Initialization */

  ret = nx_icmp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable TCP Protocol */

  /* USER CODE BEGIN TCP_Protocol_Initialization */

  /* USER CODE END TCP_Protocol_Initialization */

  ret = nx_tcp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the UDP protocol required for  DHCP communication */

  /* USER CODE BEGIN UDP_Protocol_Initialization */

  /* USER CODE END UDP_Protocol_Initialization */

  ret = nx_udp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

   /* Allocate the memory for main thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread */
  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", nx_app_thread_entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* USER CODE BEGIN MX_NetXDuo_Init */

  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/**
* @brief  Main thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID nx_app_thread_entry (ULONG thread_input)
{
  /* USER CODE BEGIN Nx_App_Thread_Entry 0 */
	UINT ret;
	ULONG bytes_read;
	UCHAR data_buffer[128];
	NX_PACKET *incoming_packet;
	NX_PACKET *outcoming_packet;
	ULONG ipAddress = IP_ADDRESS(192, 168, 1, 3);
	UINT port = 5000;
	// ULONG ip_pc = IP_ADDRESS(192, 168, 1, 10);
	// UINT port_pc = 5001;
	// UINT ip_isknown = 0;

	ULONG sw_flags;
	// create UDP socket
	ret = nx_udp_socket_create(&NetXDuoEthIpInstance, &UDPSocket, "UDP Server Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 2);
	if (ret != NX_SUCCESS)
	{
		printf("UDP server create error. %02X\n", ret);
		while(1)
		{
			tx_thread_sleep(100);
		}
	}

	// bind the socket to the port 5000 - this is the nucleo board local port
	ret = nx_udp_socket_bind(&UDPSocket, 5001, TX_WAIT_FOREVER);
	if (ret != NX_SUCCESS)
	{
		printf("Binding error. %02X\n", ret);
		while(1)
		{
			tx_thread_sleep(100);
		}
	}
	else
	{
		printf("UDP Server listening on PORT 5000.\n");
	}

	// GPIO_PinState sw_states[3] = {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};

	// start the loop
	while (1)
	{
		tx_event_flags_get(&ev_sw_flags, 0xff, TX_OR_CLEAR, &sw_flags, TX_NO_WAIT);

		// wait for one second or until the UDP is received
		ret = nx_udp_socket_receive(&UDPSocket, &incoming_packet, 10);

		if (ret == NX_SUCCESS)
		{
			// if packet has been successfully received, then retrieved the data into local buffer
			ret = nx_packet_data_retrieve(incoming_packet, data_buffer, &bytes_read);

			if (ret == NX_SUCCESS)
			{
				// get the source IP address and port
				// nx_udp_source_extract(incoming_packet, &ipAddress, &port);
				// printf("Socket received %d bytes from %d.%d.%d.%d:%d\n",
						// (int) bytes_read, (int) (ipAddress >> 24) & 0xFF, (int) (ipAddress >> 16) & 0xFF,
						// (int) (ipAddress >> 8) & 0xFF, (int) ipAddress & 0xFF, port);
//
				// ip_isknown = 1;

				// allocate packet for reply

				// ret = nx_packet_allocate(&NxAppPool, &outcoming_packet, NX_UDP_PACKET, 100);
				// if (ret != NX_SUCCESS)
				// {
					// // if error has been detected, print the error code and jump to the beginning of the while loop commands
					// printf("Packet allocate error %02x\n", ret);
					// continue;
				// }

				UCHAR cmd[7];
				memcpy(cmd, data_buffer, 7);

				GPIO_PinState state = cmd[6] == '1' ? RESET : SET;
				if ((int)strncmp((CHAR *)cmd, "LED1", 4) == 0) {
					HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, state);
				} else if ((int)strncmp((CHAR *)cmd, "LED2", 4) == 0) {
					HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, state);
				} else if ((int)strncmp((CHAR *)cmd, "LED3", 4) == 0) {
					HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, state);
				}

				/*
				// append data to the packet
				ret = nx_packet_data_append(outcoming_packet, data_buffer,
						bytes_read, &NxAppPool, 100);

				if (ret != NX_SUCCESS)
				{
					// if error has been detected, print the error code and jump to the beginning of the while loop commands
					printf("Packet append error %02x\n", ret);
					continue;
				}

				// send the data to the IP address and port which has been extracted from the incoming packet
				ret = nx_udp_socket_send(&UDPSocket, outcoming_packet,	ipAddress, port);
				if (ret != NX_SUCCESS)
				{
					// in the case of socket send failure we MUST release the outcoming packet!
					printf("UDP send error %02x\n", ret);
					nx_packet_release(outcoming_packet);
				}
				else
				{
					// in the case of socket success we MUST NOT release the outcoming packet!
					printf("UDP send successfully\n");
				}
				*/
			}

			// we MUST always release the incoming packet
			nx_packet_release(incoming_packet);
			printf("Packets available %d\n\n", (int) NxAppPool.nx_packet_pool_available);
		}

		// if(!ip_isknown) {
			// printf("Ip unknown.\n");
			// continue;
		// }

		if(sw_flags & EV_SW_1_DOWN) {
			printf("SW1 DOWN\n");
			ret = send_udp("LED1G=1\n", ipAddress, port, &outcoming_packet, data_buffer);
			if (ret != NX_SUCCESS) {
				printf("Send UDP Error: %02x\n", ret);
			}
		} else if(sw_flags & EV_SW_1_UP) {
			printf("SW1 UP\n");
			ret = send_udp("LED1G=0\n", ipAddress, port, &outcoming_packet, data_buffer);
			if (ret != NX_SUCCESS) {
				printf("Send UDP Error: %02x\n", ret);
			}
		}
		if(sw_flags & EV_SW_2_DOWN) {
			printf("LED2G DOWN\n");
			ret = send_udp("LED2G=1\n", ipAddress, port, &outcoming_packet, data_buffer);
			if (ret != NX_SUCCESS) {
				printf("Send UDP Error: %02x\n", ret);
			}
		} else if(sw_flags & EV_SW_2_UP) {
			printf("SW2 UP\n");
			ret = send_udp("LED2G=0\n", ipAddress, port, &outcoming_packet, data_buffer);
			if (ret != NX_SUCCESS) {
				printf("Send UDP Error: %02x\n", ret);
			}
		}
		if(sw_flags & EV_SW_3_DOWN) {
			printf("SW3 DOWN\n");
			ret = send_udp("LED3G=1\n", ipAddress, port, &outcoming_packet, data_buffer);
			if (ret != NX_SUCCESS) {
				printf("Send UDP Error: %02x\n", ret);
			}
		} else if(sw_flags & EV_SW_3_UP) {
			printf("SW3 UP\n");
			ret = send_udp("LED3G=0\n", ipAddress, port, &outcoming_packet, data_buffer);
			if (ret != NX_SUCCESS) {
				printf("Send UDP Error: %02x\n", ret);
			}
		}


	}
  /* USER CODE END Nx_App_Thread_Entry 0 */

}
/* USER CODE BEGIN 1 */
ULONG send_udp(char* msg, ULONG ipAddress, ULONG port, NX_PACKET **packet, UCHAR *data_buffer){
	ULONG ret;
	ULONG bytes = strlen(msg) + 1;
	// allocate packet for reply
	ret = nx_packet_allocate(&NxAppPool, &packet, NX_UDP_PACKET, 100);
	if (ret != NX_SUCCESS)
	{
		// if error has been detected, print the error code and jump to the beginning of the while loop commands
		printf("Packet allocate error %02x\n", ret);
		return ret;
	}

	memcpy(data_buffer, msg, bytes);
	// append data to the packet
	ret = nx_packet_data_append(packet, data_buffer,
			bytes, &NxAppPool, 100);

	if (ret != NX_SUCCESS)
	{
		// if error has been detected, print the error code and jump to the beginning of the while loop commands
		printf("Packet append error %02x\n", ret);
		return ret;
	}

	// send the data to the IP address and port which has been extracted from the incoming packet
	ret = nx_udp_socket_send(&UDPSocket, packet, ipAddress, port);
	if (ret != NX_SUCCESS)
	{
		// in the case of socket send failure we MUST release the outcoming packet!
		printf("UDP send error %02x\n", ret);
		nx_packet_release(packet);
	}
	else
	{
		// in the case of socket success we MUST NOT release the outcoming packet!
		// printf("UDP send successfully\n");
		printf("UDP sent %d bytes to %d.%d.%d.%d:%d\n",
			(int) bytes, (int) (ipAddress >> 24) & 0xFF, (int) (ipAddress >> 16) & 0xFF,
			(int) (ipAddress >> 8) & 0xFF, (int) ipAddress & 0xFF, port);
	}
	return ret;
};
/* USER CODE END 1 */
