/*
 * IMU_driver.c
 *
 *  Created on: Jun 2, 2024
 *      Author: sydne
 */
#include "main.h"
#include "IMU_driver.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// buffer for uart communication
char msg[50];

int16_t accel_x_raw, accel_y_raw, accel_z_raw; // raw accelerometer data
float accel_x, accel_y, accel_z;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw; // raw gyro data
float gyro_x, gyro_y, gyro_z;

uint8_t raw_data[6]; //for use with the above


//function definitions for the IMU
/// Checks I2C connection, reads the WHO_AM_I register, and scans device address for verification purposes. Prints debugging messages through uart.
void IMU_init(ICM_20948*p_IMU){

	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(p_IMU->hi2c, ICM20948_I2C_ADDR, 1, 100); // i2c pointer & because we want the address
    	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 // addrr is 11001, added 0 for the function to read
    	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 // trials: number of times to try the connection
    	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 // timout in ms
    if (ret == HAL_OK)
      {
    	  HAL_UART_Transmit(p_IMU->huart, (uint8_t*)"\r\n imu: connected ", 19, HAL_MAX_DELAY); // ensure string length is correct
      }
      else
      {
    	  HAL_UART_Transmit(p_IMU->huart, (uint8_t*)"\r\n imu: no connection", 21, HAL_MAX_DELAY);
      }


    // checking the WHO_AM_I register
    uint8_t check;
    uint8_t data;

    // Check WHO_AM_I register
    HAL_I2C_Mem_Read(p_IMU->hi2c, ICM20948_I2C_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &check, 1, HAL_MAX_DELAY);
    if (check == 0xEA) // WHO_AM_I should return 0xEA for ICM-20948
    {
    	HAL_UART_Transmit(p_IMU->huart, (uint8_t*)"\r\n ICM-20948 found\r\n", 20, HAL_MAX_DELAY);
         sprintf(msg, "WHO_AM_I reads 0x%02X\r\n", check);
         HAL_UART_Transmit(p_IMU->huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
         // Wake up the sensor (clear sleep mode bit)
         data = 0x01;
         HAL_I2C_Mem_Write(p_IMU->hi2c, ICM20948_I2C_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
     }
     else
     {
    	 HAL_UART_Transmit(p_IMU->huart, (uint8_t*)"ICM-20948 not found\r\n", 21, HAL_MAX_DELAY);
      }


     //I2C scanner
     char info[] = "Scanning I2C bus...\r\n";
     HAL_UART_Transmit(p_IMU->huart, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

     for (uint8_t addr = 1; addr < 128; addr++)
     {
    	 if (HAL_I2C_IsDeviceReady(p_IMU->hi2c, addr << 1, 1, 10) == HAL_OK)
         {
    		 char msg[32];
    		 sprintf(msg, "Found device at 0x%02X\r\n", addr);
    		 HAL_UART_Transmit(p_IMU->huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
         } else {
    		 char msg[50];
    		 sprintf(msg, "No device found\r\n");
    		 //HAL_UART_Transmit(p_IMU->huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
         }
     }
}


AccelData IMU_read_accel(ICM_20948*p_IMU){
	AccelData accel_data = {0};

	if (HAL_I2C_Mem_Read(p_IMU->hi2c, ICM20948_I2C_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY) == HAL_OK)
	{
		accel_x_raw = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	    accel_y_raw = (int16_t)(raw_data[2] << 8 | raw_data[3]);
	    accel_z_raw = (int16_t)(raw_data[4] << 8 | raw_data[5]);

	    // take the raw readings, divide by sensitivity, multiply by 9.81 to get reading m/s^2 reading
	    accel_x = (float)accel_x_raw * GRAV / ACCEL_FS;
	    accel_y = (float)accel_y_raw * GRAV / ACCEL_FS;
	    accel_z = (float)accel_z_raw * GRAV / ACCEL_FS;

	    accel_data.accel_x = accel_x_raw * GRAV / ACCEL_FS;
	    accel_data.accel_y = accel_y_raw * GRAV / ACCEL_FS;
	    accel_data.accel_z = accel_z_raw * GRAV / ACCEL_FS;

	    // output the data in m/s^2
	    // took out the \r\n for testing
	    //sprintf(msg, "Accel X: %.4f, Y: %.4f, Z: %.4f     ", accel_data.accel_x, accel_data.accel_y, accel_data.accel_z);
	    //HAL_UART_Transmit(p_IMU->huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


	    }
	    else
	    {
	    	HAL_UART_Transmit(p_IMU->huart, (uint8_t*)"Error reading accel data\r\n", 26, HAL_MAX_DELAY);
	    }
	return accel_data;
}


GyroData IMU_read_gyro(ICM_20948*p_IMU){
	GyroData gyro_data = {0};

	if (HAL_I2C_Mem_Read(p_IMU->hi2c, ICM20948_I2C_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY) == HAL_OK)
	{

		gyro_x_raw = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	    gyro_y_raw = (int16_t)(raw_data[2] << 8 | raw_data[3]);
	    gyro_z_raw = (int16_t)(raw_data[4] << 8 | raw_data[5]);

	    // take the raw readings, divide by sensitivity. data in deg/s
	    gyro_x = (float)gyro_x_raw / GYRO_FS_SEL;
	    gyro_y = (float)gyro_y_raw / GYRO_FS_SEL;
	    gyro_z = (float)gyro_z_raw / GYRO_FS_SEL;

	    gyro_data.gyro_x = gyro_x_raw / GYRO_FS_SEL;
	    gyro_data.gyro_y = gyro_y_raw / GYRO_FS_SEL;
	    gyro_data.gyro_z = gyro_z_raw / GYRO_FS_SEL;

	    // output the data in deg/s
	    //sprintf(msg, "Gyro X: %.4f, Y: %.4f, Z: %.4f      ", gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z);
	    //HAL_UART_Transmit(p_IMU->huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	    }
	    else
	    {
	    	// maybe throw a real error here, lmao
	    	HAL_UART_Transmit(p_IMU->huart, (uint8_t*)"Error reading gyro data\r\n", 25, HAL_MAX_DELAY);
	    }

	return gyro_data;
}

