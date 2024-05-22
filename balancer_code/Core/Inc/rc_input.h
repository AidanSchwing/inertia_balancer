/*
 * rc_input.h
 *
 *  Created on: May 19, 2024
 *      Author: Aidan
 */

#ifndef INC_RC_INPUT_H_
#define INC_RC_INPUT_H_

#include "main.h"

struct rc_input_struct {

	uint32_t trig_ch_rise; // channel for trigger rise
	uint32_t trig_ch_fall; // channel for trigger fall

	uint32_t whl_ch_rise; // channel for wheel rise
	uint32_t whl_ch_fall; // channel for wheel fall

	TIM_HandleTypeDef* rc_timer; // timer for rc input, pointer

	// bounding values to the receiver PWM signal length, us
	// values from scope PWM inspection
	uint32_t nom_pwm;
	uint32_t high_pwm;
	uint32_t low_pwm;

	// time in @ relevant points in the PWM cycle
	uint32_t trig_first;
	uint32_t trig_second;
	uint32_t trig_delta;

	uint32_t whl_first;
	uint32_t whl_second;
	uint32_t whl_delta;

} typedef rc_inp_t;

// prototype functions
uint32_t RC_get_trig(rc_inp_t *p);
uint32_t RC_get_whl(rc_inp_t *p);
void RC_start_cap(rc_inp_t *p);
void RC_callback(rc_inp_t*p, TIM_HandleTypeDef*timer);

#endif /* INC_RC_INPUT_H_ */
