/*
 * encoder_driver.c
 *
 *  Created on: Nov 27, 2025
 *      Author: hecto
 */

#include <encoder_driver.h>
#include "tx_api.h"
#include "stm32h7xx_hal.h"

#define ENCODER_MAX_COUNT 256 //range from 0 to 500

TX_MUTEX enc_dr; //declaring mutex

int32_t absolute_position_count = 0;

int16_t previous_count = 0;

UINT encoder_driver_initialize()
{

	previous_count = TIM1->CNT;

	UINT ret;
	ret = tx_mutex_create(&enc_dr, "Encoder Mutex",  TX_INHERIT);


	return ret;
}

int32_t encoder_driver_get_position(void)
{
    UINT ret;
    int32_t return_position = 0;

    // 1. Get the Mutex
    ret = tx_mutex_get(&enc_dr, TX_WAIT_FOREVER);

    // 2. ONLY run calculation if we GOT the mutex (ret == TX_SUCCESS)
    // Your previous code had (ret != TX_SUCCESS), which skipped the logic!
    if (ret == TX_SUCCESS)
    {
        // Read Hardware
        uint16_t current_count = TIM1->CNT % ENCODER_MAX_COUNT;

        // --- CRITICAL FIX HERE ---
        // We cast to (int16_t) so the math handles negative numbers correctly.
        int16_t diff = (int16_t)current_count - (int16_t)previous_count;

        // Check for Forward Wrap (e.g., 490 -> 10)
        // If diff is a huge NEGATIVE number (like -480)
        if (diff < -(ENCODER_MAX_COUNT / 2))
        {
            diff += ENCODER_MAX_COUNT;
        }
        // Check for Backward Wrap (e.g., 10 -> 490)
        // If diff is a huge POSITIVE number (like +480)
        else if (diff > (ENCODER_MAX_COUNT / 2))
        {
            diff -= ENCODER_MAX_COUNT;
        }

        // If diff is small (0, 1, -1), neither IF statement runs.
        // This stops the "+500" runaway bug.

        // Update Global Position
        absolute_position_count += diff;

        // Update History
        previous_count = current_count;

        // Set return value
        //return_position = absolute_position_count;
        return_position = current_count;


        // Release the Mutex
        tx_mutex_put(&enc_dr);
    }

    return return_position;
}
