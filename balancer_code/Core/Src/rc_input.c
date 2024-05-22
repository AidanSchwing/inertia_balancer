/*
 * rc_input.c
 *
 *  Created on: May 19, 2024
 *      Author: Aidan
 */

#include "main.h"
#include "rc_input.h"

uint16_t trig_val = 0;
uint16_t whl_val = 0;

// start the interrupts on the timers
void RC_start_cap(rc_inp_t *p)
{
	HAL_TIM_IC_Start_IT(p->rc_timer, p->trig_ch_rise);
	HAL_TIM_IC_Start_IT(p->rc_timer, p->trig_ch_fall);
	HAL_TIM_IC_Start_IT(p->rc_timer, p->whl_ch_rise);
	HAL_TIM_IC_Start_IT(p->rc_timer, p->whl_ch_fall);
}

void RC_callback(rc_inp_t*p, TIM_HandleTypeDef*timer)
{
	// loop thru timer chanels to check which called the interrupt
	// & read correct channel given the timer and active channel
	// simply write the correct

	// trig rising
	if (timer->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		//__HAL_TIM_SET_COUNTER(timer, 0);
		p->trig_first = HAL_TIM_ReadCapturedValue(timer, p->trig_ch_rise);
	}

	// trig falling
	else if(timer->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		p->trig_second = HAL_TIM_ReadCapturedValue(timer, p->trig_ch_fall);

		p->trig_delta = p->trig_second - p->trig_first;
	}

	// wheel rising
	else if(timer->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		//__HAL_TIM_SET_COUNTER(timer, 0);
		p->whl_first = HAL_TIM_ReadCapturedValue(timer, p->whl_ch_rise);
	}

	// wheel falling
	else if(timer->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		p->whl_second = HAL_TIM_ReadCapturedValue(timer, p->whl_ch_fall);

		p->whl_delta = p->whl_second - p->whl_first;
	}
}

// returns the most recent value of the time delta between rising and falling
uint32_t RC_get_trig(rc_inp_t *p)
{
	uint32_t new_val = p->trig_delta;

	if (new_val > (p->high_pwm)) {
		return p->nom_pwm; // return the nominal if out of range
	}
	else if (new_val < (p->low_pwm)) {
		return p->nom_pwm; // return the nominal if out of range
	}

	return p->trig_delta;
}


// returns the most recent value of the time delta between rising and falling
uint32_t RC_get_whl(rc_inp_t *p)
{
	uint32_t new_val = p->whl_delta;

	if (new_val > (p->high_pwm)) {
		return p->nom_pwm; // return the nominal if out of range
	}
	else if (new_val < (p->low_pwm)) {
		return p->nom_pwm; // return the nominal if out of range
	}

	return p->whl_delta;
}






