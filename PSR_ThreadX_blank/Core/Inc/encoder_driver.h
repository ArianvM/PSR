/*
 * driver_encoder_stw.h
 *
 *  Created on: Nov 19, 2025
 *      Author: hecto
 */

#ifndef INC_ENCODER_DRIVER_H_
#define INC_ENCODER_DRIVER_H_

#include "main.h"
#include "tx_api.h"

UINT encoder_driver_initialize();
UINT encoder_driver_input(uint32_t *position);
UINT encoder_driver_output(uint32_t position);


#endif /* INC_ENCODER_DRIVER_H_ */
