/*
 * encoder_driver.h
 *
 *  Created on: Oct 19, 2025
 *      Author: Ondrej Teren
 */

#ifndef INC_ENCODER_DRIVER_H_
#define INC_ENCODER_DRIVER_H_

#include "main.h"
#include "tx_api.h"

#define EV_DRIVER_POSITION_RESET (1<<0)

UINT encoder_driver_initialize();
UINT encoder_driver_input(uint32_t *position);
UINT encoder_driver_output(uint32_t position);

#endif /* INC_ENCODER_DRIVER_H_ */
