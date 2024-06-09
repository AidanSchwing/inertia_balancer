/*
 * odrive_wrapper.c
 *
 *  Created on: May 20, 2024
 *      Author: Aidan
 */
#include "main.h"
#include "odrive_wrapper.h"

uint8_t rxBuffer[BUFFER_SIZE]; 		// Receive buffer. max length of expected returned values.
uint8_t rxIndex = 0; 				// init index at zero.
uint8_t responseFinishedFlag = 0;
uint8_t receivedData[BUFFER_SIZE];

void ODRIVE_Reboot(odrive_t *p)
{
	char message[50];
    int len = sprintf(message, "sr\r\n");
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}


void ODRIVE_ClearErrors(odrive_t *p)
{
	char message[50];
    int len = sprintf(message, "sc\r\n");
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}


void ODRIVE_SetPosition(odrive_t *p, int motor_number, float position, float velocity_lim, float torque_lim)
{
	char message[50];
    int len = sprintf(message, "q %d %.3f %.3f %.3f \r\n", motor_number, position, velocity_lim, torque_lim);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}


void ODRIVE_SetPositionFF(odrive_t *p, int motor_number, float position, float velocity_feedforward, float current_feedforward)
{
	char message[50];
    int len = sprintf(message, "p %d %.3f %.3f %.3f \r\n", motor_number, position, velocity_feedforward, current_feedforward);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}


void ODRIVE_SetVelocity(odrive_t *p, int motor_number, float velocity)
{
	char message[50];
    int len = sprintf(message, "v %d %.3f\r\n", motor_number, velocity);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}

//scary!
void ODRIVE_SetTorque(odrive_t *p, int motor_number, float torque)
{
	char message[50];
    int len = sprintf(message, "c %d %.3f \r\n", motor_number, torque);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}


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


void ODRIVE_WriteParam(odrive_t *p, char param[])
{
	char message[50];
    int len = sprintf(message, "w %s \r\n", param);
    HAL_UART_Transmit(p->huart, (uint8_t*)message, len, HAL_MAX_DELAY);
}


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


void start_receive_string(odrive_t *p)
{
	rxIndex = 0; // reset the buffer index
    memset(rxBuffer, 0, sizeof(rxBuffer)); // Clear the received string buffer
    HAL_UART_Receive_IT(p->huart, &rxBuffer[rxIndex], 1); // Start receiving data
}


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





