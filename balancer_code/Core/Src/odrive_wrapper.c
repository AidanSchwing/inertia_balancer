/*
 * odrive_wrapper.c
 *
 *  Created on: May 20, 2024
 *      Author: Aidan
 */
/// @file odrive_wrapper.c
/// @brief Provides functions for interfacing with an external ODrive controller.
///< Implements the ASCII protocol as defined by ODrive: https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html
#include "main.h"
#include "odrive_wrapper.h"

uint8_t rxBuffer[BUFFER_SIZE]; 		// Receive buffer. max length of expected returned values.
uint8_t rxIndex = 0; 				// init index at zero.
uint8_t responseFinishedFlag = 0;
uint8_t receivedData[BUFFER_SIZE];

/**
 * Sends ASCII command to reboot ODrive.
 * @param p Pointer to an ODrive object.
 */
void ODRIVE_Reboot(odrive_t *p)
{
	char message[50];
    int len = sprintf(message, "sr\r\n");
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Sends ASCII command to clear errors on ODrive.
 * @param p Pointer to an ODrive object.
 */
void ODRIVE_ClearErrors(odrive_t *p)
{
	char message[50];
    int len = sprintf(message, "sc\r\n");
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Sends ASCII command to send position setpoint to ODrive.
 * @param p Pointer to an ODrive object.
 * @param motor_number Motor number on ODrive. Typically 0.
 * @param position Position setpoint
 * @param velocity_lim Velocity limit
 * @param torque_lim Torque limit
 */
void ODRIVE_SetPosition(odrive_t *p, int motor_number, float position, float velocity_lim, float torque_lim)
{
	char message[50];
    int len = sprintf(message, "q %d %.3f %.3f %.3f\r\n", motor_number, position, velocity_lim, torque_lim);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Sends ASCII command to send position setpoint with feedforward ODrive.
 * @param p Pointer to an ODrive object.
 * @param motor_number Motor number on ODrive. Typically 0.
 * @param position Position setpoint
 * @param velocity_feedforward Velocity feedforward value.
 * @param current_feedforward Current feedforward value.
 */
void ODRIVE_SetPositionFF(odrive_t *p, int motor_number, float position, float velocity_feedforward, float current_feedforward)
{
	char message[50];
    int len = sprintf(message, "p %d %.3f %.3f %.3f\r\n", motor_number, position, velocity_feedforward, current_feedforward);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Sends ASCII command to send velocity setpoint to ODrive.
 * @param p Pointer to an ODrive object.
 * @param motor_number Motor number on ODrive. Typically 0.
 * @param velocity Velocity setpoint.
 */
void ODRIVE_SetVelocity(odrive_t *p, int motor_number, float velocity)
{
	char message[50];
    int len = sprintf(message, "v %d %.3f\r\n", motor_number, velocity);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Sends ASCII command to send torque setpoint to ODrive.
 * @param p Pointer to an ODrive object.
 * @param motor_number Motor number on ODrive. Typically 0.
 * @param torque Torque setpoint.
 */
void ODRIVE_SetTorque(odrive_t *p, int motor_number, float torque)
{
	char message[50];
    int len = sprintf(message, "c %d %.3f\r\n", motor_number, torque);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Sends ASCII command to get current position and velocity from ODrive.
 * @param p Pointer to an ODrive object.
 * @param motor_number Motor number on ODrive. Typically 0.
 */
uint8_t* ODRIVE_GetFeedback(odrive_t *p, int motor_number)
{
	char message[50];
	int len = sprintf(message, "f %d \r\n", motor_number);
	HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);

	uint8_t* response = 0;
	while((response = getReceivedString(p)) == NULL)
	{
	  // wait for a response to be completed
	  HAL_Delay(1);
	}
	//HAL_UART_Transmit(p->huart, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
	return response;
}

/**
 * Sends ASCII command to get VBUS voltage from ODrive.
 * Used mainly as a test to see if we can send and receive UART commands.
 * @param p Pointer to an ODrive object.
 */
uint8_t* ODRIVE_GetVBus(odrive_t *p)
{
	char message[50];
	int len = sprintf(message, "r vbus_voltage \r\n");
	HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);

	uint8_t* response = 0;
	while((response = getReceivedString(p)) == NULL)
	{
	  // wait for a response to be completed
	  HAL_Delay(1);
	}
	return response;
}

/**
 * Sends ASCII command to check calibrated status ODrive.
 * I think this one is broken.
 * @param p Pointer to an ODrive object.
 */
uint8_t* ODRIVE_IsCalibrated(odrive_t *p)
{
	char message[50];
	int len = sprintf(message, "r axis0.motor.is_calibrated \r\n");
	HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);

	uint8_t* response = 0;
	while((response = getReceivedString(p)) == NULL)
	{
	  // wait for a response to be completed
	  HAL_Delay(1);
	}
	return response;
}

/**
 * Sends ASCII command to ODrive.
 * @param p Pointer to an ODrive object.
 * @param param[] Parameter to write to, in string format.
 */
void ODRIVE_WriteParam(odrive_t *p, char param[])
{
	char message[50];
    int len = sprintf(message, "w %s \r\n", param);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

/**
 * Receives ASCII command from ODrive.
 * @param p Pointer to an ODrive object.
 * @param param[] Parameter to read from, in string format.
 */
uint8_t* ODRIVE_ReadParam(odrive_t *p, char param[])
{
	char message[50];
	int len = sprintf(message, "r %s \r\n", param);
	HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);

	uint8_t* response = 0;
	while((response = getReceivedString(p)) == NULL)
	{
	  // wait for a response to be completed
	  HAL_Delay(1);
	}
	return response;
}

/**
 * Sends ASCII command to reboot ODrive.
 * @param p Pointer to an ODrive object.
 */
uint8_t* getReceivedString(odrive_t *p)
{
	static uint8_t receivedDataCopy[BUFFER_SIZE]; // create a copied variable that is returned

	if (responseFinishedFlag)
	{
		responseFinishedFlag = 0;
		memcpy(receivedDataCopy, receivedData, strlen((char*)receivedData));
		return receivedDataCopy; // return the copied variable
	}
	else
	{
		return NULL;
	}
}

/**
 * Starts receiving a new string via UART.
 * Sets the buffer index to zero and sets up HAL_UART_Receive correctly for the ODrive object.
 * @param p Pointer to an ODrive object.
 */
void start_receive_string(odrive_t *p)
{
	rxIndex = 0; // reset the buffer index
    memset(rxBuffer, 0, sizeof(rxBuffer)); // Clear the received string buffer
    HAL_UART_Receive_IT(p->huart, &rxBuffer[rxIndex], 1); // Start receiving data
}

/**
 * Callback to properly receive strings via UART.
 * @param p Pointer to an ODrive object.
 * @param huart UART object that is compared to ODrive object to confirm they are the same.
 */
void ODRIVE_Receive_Callback (UART_HandleTypeDef *huart, odrive_t *p)
{
    if (huart == p->huart) {
        if (rxBuffer[rxIndex] == '\n')
        {
            // Newline character received, null-terminate the string
            rxBuffer[rxIndex] = '\0';
        	memset(receivedData, 0, sizeof(receivedData)); // clear the previous data
        	memcpy(receivedData, rxBuffer, rxIndex + 1); // copy the buffer to received
            responseFinishedFlag = 1;
            start_receive_string(p); // Start receiving the next string
            return;
        }
        else
        {
        	rxIndex++; //increment position in buffer
        }

        HAL_UART_Receive_IT(p->huart, &rxBuffer[rxIndex], 1); // Start receiving data

    }
}

// for use in decoding the feedback message
/**
 * Extracts floats from a given string of two floats in ASCII protocol format.
 * For use in converting feedback message.
 * @param message Pointer to the received message.
 * @param x Pointer to a first float.
 * @param y Pointer to a second float.
 */
void extractFloats(uint8_t* message, float* x, float* y) {
    if (!message || !x || !y) {
        return -1;  // Invalid input
    }

    // sscanf will parse two float values from the message
    // The format string "%f %f" means:
    // - Read a float (x.xxxx), then skip any whitespace
    // - Read another float (y.yyyy)
    // The actual values replace the x's and y's in the message
    int result = sscanf(message, "%f %f", x, y);

    if (result != 2) {
        return -1;  // Failed to extract both floats
    }

    return 0;  // Success
}


