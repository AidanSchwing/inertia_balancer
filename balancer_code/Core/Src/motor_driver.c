/*
 * motor_driver.c
 *
 *  Created on: Apr 18, 2024
 *      Author: Aidan
 */

#include "main.h"
#include "motor_driver.h"

void MOTOR_Init(motor_t *p)
{

	// init timer pwm to zero.
	p->mot_timer->Instance->CCR1 = 0;
	p->mot_timer->Instance->CCR2 = 0;

	// initialize PWM timers. assume that the rest of timer and channel setup is handled by cube
	HAL_TIM_PWM_Start(p->mot_timer, p->mot_channel1);
	HAL_TIM_PWM_Start(p->mot_timer, p->mot_channel2);

	MOTOR_SetDuty(p);

}


void MOTOR_DeInit(motor_t *p)
{

	HAL_TIM_PWM_Stop(p->mot_timer, p->mot_channel1);
	HAL_TIM_PWM_Stop(p->mot_timer, p->mot_channel2);

}


void MOTOR_SetDuty(motor_t *p)
{
	// setting duty on both channels. may need to be changed for motor reversing.

	// define an ARR int to calculate the ticks to count
	uint32_t arr = p->mot_timer->Instance->ARR;

	// calculate new duty cycle to assign to pulsing motor as a function of the arr
	//float dutyMag = fabs(p->dutyOverall); //magnitude of duty

	int32_t new_duty = (int32_t)( ((p->dutyOverall * arr)/100.0f ) ); //duty as value assignable to PWM channel


	// logic for motor directions: (CH1, CH2))
	// (H, H) : brake
	// (L, H) : forward
	// (H, L) : reverse

	// if zero, set both to HI
	if (p->dutyOverall == 0 ){
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel1,arr);
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel2,arr);
		return;
	}

	// if positive, set CH1 to percent on and CH2 to HI
	else if ( (p->dutyOverall > 0) & (p->dutyOverall <= 100) ){

		//assign new duty cycle to channel 1
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel1,arr-new_duty);
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel2,arr);

		return;
	}

	// if negative, set CH1 to HI and CH2 to percent on
	else if ( (p->dutyOverall < 0) & (p->dutyOverall >= -100) ){

		//assign new duty cycle to channel 2
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel1,arr);
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel2,arr-(-new_duty));

		return;
	}



	/*
	if (( p->dutyCycle1 >= 0)&( p->dutyCycle1<=100)) {


		//->mot_timer->Instance->CCR1 = ( arr * p->dutyCycle1 ) / 100;
	}

	if (( p->dutyCycle2 >= 0)&( p->dutyCycle2<=100)) {
		uint32_t new_duty = ( arr * p->dutyCycle2 ) / 100;
		__HAL_TIM_SET_COMPARE(p->mot_timer,p->mot_channel2,new_duty);
		//p->mot_timer->Instance->CCR2 = ( arr * p->dutyCycle2 ) / 100;
	}
	*/

}
