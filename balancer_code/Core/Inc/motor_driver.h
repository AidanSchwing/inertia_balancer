/*
 * motor_driver.h
 *
 *  Created on: Apr 18, 2024
 *      Author: Aidan
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "main.h"

struct motor_struct {

	TIM_HandleTypeDef* mot_timer;

	uint32_t mot_channel1;
	uint32_t mot_channel2;

	// overall duty determines direction, [-100,100]
	float dutyOverall;

} typedef motor_t;


// prototype functions, copy later
void MOTOR_Init(motor_t *p);
void MOTOR_DeInit(motor_t *p);
void MOTOR_SetDuty(motor_t *p);

#endif /* INC_MOTOR_DRIVER_H_ */
