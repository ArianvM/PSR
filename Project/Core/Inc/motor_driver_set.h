/*
 * driver_set.h
 *
 *  Created on: Nov 20, 2025
 *      Author: arian
 */

#ifndef INC_MOTOR_DRIVER_SET_H_
#define INC_MOTOR_DRIVER_SET_H_


#include "main.h"
#include "tx_api.h"

UINT motor_init();
UINT motor_set_speed(int speed);
UINT motor_get_pos(uint32_t* pos);


#endif /* INC_MOTOR_DRIVER_SET_H_ */
