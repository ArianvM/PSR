/*
 * driver_set.c
 *
 *  Created on: Nov 20, 2025
 *      Author: arian
 */
#include <motor_driver.h>

TX_MUTEX m_motor_speed;
TX_MUTEX m_motor_pos;

UINT motor_init() {
	UINT ret;

	ret = tx_mutex_create(&m_motor_pos, "MOTOR_POSITION", TX_INHERIT);
	if (ret != TX_SUCCESS) {
		printf("ERROR: mutex_create: %d\n", ret);
		return ret;
	}
	ret = tx_mutex_create(&m_motor_speed, "MOTOR_SPEED", TX_INHERIT);
	if (ret != TX_SUCCESS) {
		printf("ERROR: mutex_create: %d\n", ret);
		return ret;
	}

	if (ret != TX_SUCCESS) {
		printf("ERRLR: queue_create: %d\n", ret);
		return ret;
	}

	printf("Motor initialized...\n");
	return ret;
}

UINT motor_set_speed(int speed) {
	UINT ret;
	if (abs(speed) > 500) {
		printf("ERROR: motor_set_speed: invalid value %d\n", speed);
		ret = -1;
	}
	ret = tx_mutex_get(&m_motor_speed, 0x1);
	if (ret != TX_SUCCESS) {
		printf("ERROR: mutex_get: %d\n", ret);
		return ret;
	}

	if (speed >= 0) {
		TIM2->CCR3 = (uint32_t)speed;
		TIM2->CCR4 = 0;
	} else {
		TIM2->CCR3 = 0;
		TIM2->CCR4 = (uint32_t)abs(speed);
	}

	ret = tx_mutex_put(&m_motor_speed);
	if (ret != TX_SUCCESS) {
		printf("ERROR: mutex_put: %d", ret);
		return ret;
	}
}

UINT motor_get_pos(uint32_t* pos) {
	UINT ret;
	uint32_t pos_tmp;
	ret = tx_mutex_get(&m_motor_pos, 0x1);
	if (ret != TX_SUCCESS) return ret;
	pos_tmp = TIM1->CNT;
	*pos = pos_tmp % MOTOR_MAX_POS;
	ret = tx_mutex_put(&m_motor_pos);
	return ret;
}
