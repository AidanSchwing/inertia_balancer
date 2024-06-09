/*
 * IMU_driver.h
 *
 *  Created on: Jun 2, 2024
 *      Author: sydne
 */

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_
#include "stm32f4xx_hal.h"

// ICM 20948 register addresses
#define ICM20948_I2C_ADDR (0x69<<1) // 7 bit address shifted 1 for R/W
#define WHO_AM_I_REG 0x00
#define PWR_MGMT_1_REG 0x06
// accel mem locations
#define ACCEL_XOUT_H 0x2D //high byte
#define ACCEL_XOUT_L 0x2E //low byte
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define ACCEL_FS 16384
// gyro mem locations
#define GYRO_XOUT_H 0x33 //high byte
#define GYRO_XOUT_L 0x34 //low byte
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define GYRO_FS_SEL 131

#define GRAV 9.81

// ICM 20948, essential info to define the IMU for use
struct IMU_comm_struct {
    I2C_HandleTypeDef*hi2c; //i2c used
    UART_HandleTypeDef*huart;  //uart used
} typedef ICM_20948;

// struct to hold the acceleration readings
struct accel_struct {
    float accel_x;
    float accel_y;
    float accel_z;
} typedef AccelData;

// struct to hold gyro data
struct gyro_struct {
	float gyro_x;
	float gyro_y;
	float gyro_z;
} typedef GyroData;


//function prototypes
void IMU_init(ICM_20948*p_IMU);
AccelData IMU_read_accel(ICM_20948*p_IMU);
GyroData IMU_read_gyro(ICM_20948*p_IMU);


#endif /* IMU_DRIVER_H_ */
