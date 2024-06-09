/*
 * odrive_wrapper.h
 *
 *  Created on: May 20, 2024
 *      Author: Aidan
 */

#ifndef INC_ODRIVE_WRAPPER_H_
#define INC_ODRIVE_WRAPPER_H_

#include "main.h"

struct odrive_struct {

	UART_HandleTypeDef* huart;

} typedef odrive_t;


// following is an implementation of the useful commands from here:
// https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html
// all commands follow as ASCII protocol, formatted as so:
// command *42 ; comment [new line character], where *42 is a non-required checksum

// prototype functions
// commands to odrive controller
void ODRIVE_Reboot(odrive_t *p);
void ODRIVE_ClearErrors(odrive_t *p);

// functions for motor control
void ODRIVE_SetPosition(odrive_t *p, int motor_number, float position, float velocity_lim, float torque_lim);
void ODRIVE_SetPositionFF(odrive_t *p, int motor_number, float position, float velocity_feedforward, float current_feedforward);
void ODRIVE_SetVelocity(odrive_t *p, int motor_number, float velocity);
void ODRIVE_SetTorque(odrive_t *p, int motor_number, float torque);
uint8_t* ODRIVE_GetFeedback(odrive_t *p, int motor_number); // returns "pos vel"

// paramater reading and writing
uint8_t* ODRIVE_GetVBus(odrive_t *p); // functionally equivalent to the read param but used enough that it is its own function
uint8_t* ODRIVE_IsCalibrated(odrive_t *p);
void ODRIVE_WriteParam(odrive_t *p, char param[]);
uint8_t* ODRIVE_ReadParam(odrive_t *p, char param[]);

// generic send / receive functions
uint8_t* getReceivedString(odrive_t *P);
void start_receive_string(odrive_t *p);
void ODRIVE_Receive_Callback (UART_HandleTypeDef *huart, odrive_t *p);

#endif /* INC_ODRIVE_WRAPPER_H_ */
