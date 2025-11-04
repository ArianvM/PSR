/*
 * encoder_driver.c
 *
 *  Created on: Oct 19, 2025
 *      Author: Ondrej Teren
 */

#include "encoder_driver.h"

// TX_SEMAPHORE s_driver_input;
// TX_SEMAPHORE s_driver_output;
TX_MUTEX m_driver_position;
TX_EVENT_FLAGS_GROUP e_driver;

UINT encoder_driver_initialize()
{
	// TODO: Place your code here.
	// Make necessary ThreadX component creation (mutexes, semaphores, event flags).
	// You can create even a thread if it is necessary.
	UINT ret;

	// ret = tx_semaphore_create(&s_driver_input, "S_DRIVER_IN", 0);
	// ret = tx_semaphore_create(&s_driver_output, "S_DRIVER_OUT", 0);
	ret = tx_mutex_create(&m_driver_position, "M_DRIVER_POSITION", TX_INHERIT);
	if (ret != TX_SUCCESS) return ret;

	ret = tx_event_flags_create(&e_driver, "E_DRIVER");

	return ret;
}

UINT encoder_driver_input(uint32_t *position)
{
	// TODO: Place your code here.
	// Provide the value corresponding to the shaft rotary position. Store the value to the *position pointer.
	// Do not forget to protect the shared memory with appropriate lock mechanism.

	// To access the encoder value read the register TIM1->CNT, e.g.:
	// *position = TIM1->CNT;

	UINT ret;

	// tx_semaphore_get(&s_driver_input, );
	ret = tx_mutex_get(&m_driver_position, 0x1);
	if (ret != TX_SUCCESS) return ret;

	*position = TIM1->CNT;
	ret = tx_mutex_put(&m_driver_position);
	return ret;
}

UINT encoder_driver_output(uint32_t position)
{
	// TODO: Place your code here.
	// Preset the encoder position to the desired value.
	// Do not forget to protect the shared memory with appropriate lock mechanism.

	// To write the desired position write to the TIM1-CNT register, e.g.:
	// TIM1->CNT = position;

	UINT ret;

	ret = tx_mutex_get(&m_driver_position, 0x1);
	if (ret != TX_SUCCESS) return ret;

	TIM1->CNT = position;
	ret = tx_mutex_put(&m_driver_position);
	if (ret != TX_SUCCESS) return ret;

	//ret = tx_event_flags_set(&e_driver, EV_DRIVER_POSITION_RESET, TX_OR);

	return ret;
}
