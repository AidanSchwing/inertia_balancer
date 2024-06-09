/*
 * controller.c
 *
 *  Created on: Jun 8, 2024
 *      Author: Aidan
 */
#include "controller.h"

// derived from MATLAB simulation of discrete time system. here for book-keeping
// format row, col
float Ad[3][3] = {{1.0055,0.0100,-0.0001},
				  {1.0920, 1.0055,-0.0130},
				  {0,0,0.9048}};
float Bd[3] = {0.0001,0.0130,0.0952};

// LQR gains, used to calculte new requested speed
float K[3] = {303.5, 29.2, -1.94};

// state vector
float x[3] = {0,0,0};		// [angle, angular_vel, wheel_spd]
float u = 0; 				// commanded motor speed

char ctrl_msg[50];


// system variables. assume x to be the axis running vertically through the IMU
float angle_x = 0.0f;
float gyro_rate_x = 0.0f;

void update_control(float angle, float angular_velocity, float motor_curr_speed)
{
	x[0] = angle;
	x[1] = angular_velocity;
	x[2] = motor_curr_speed;

	u = -(K[0]*x[0] + K[1]*x[1] + K[2]*x[2]);
	u = fmaxf(fminf(u,30),-30); // limiting requested motor speed

    //set_motor_speed();
}


// reads IMU data and calculates angle of the system
float calculate_IMU_Angle(ICM_20948 *p_IMU)
{

	float ax, ay, az;
	float gx, gy, gz;
	float accel_angle_x;
	float adj_ax;

	// read both of the sensors
	AccelData accel = IMU_read_accel(p_IMU);	// m/s^2
	ax = accel.accel_x;
	ay = accel.accel_y;
	az = accel.accel_z;

	GyroData gyro = IMU_read_gyro(p_IMU);
	gx = gyro.gyro_x;
	gy = gyro.gyro_y;
	gz = gyro.gyro_z;


	float centrifugal_accel = pow(gyro_rate_x * M_PI / 180, 2) * DISPLACEMENT_DIST;

	adj_ax = ax + centrifugal_accel;

	accel_angle_x = atan2f(ay, sqrt(adj_ax * adj_ax + az + az)) * (180 / M_PI);

	gyro_rate_x = gx;

    // Calculate the angle from the adjusted accelerometer reading
    float denominator = sqrt(adj_ax * adj_ax + az * az);
    if (denominator != 0) {
        accel_angle_x = atan2(ay, denominator) * (180.0 / M_PI);
    } else {
        printf("Warning: Denominator is zero, setting accel_angle_x to zero to avoid NaN\n");
        accel_angle_x = 0;
    }

	angle_x = ALPHA * (angle_x + gyro_rate_x * DT) + (1 - ALPHA) * accel_angle_x;

    sprintf(ctrl_msg, "SYS ANGLE: %.4f     ", angle_x);
    HAL_UART_Transmit(p_IMU->huart, (uint8_t*)ctrl_msg, strlen(ctrl_msg), HAL_MAX_DELAY);

	return angle_x;
}






