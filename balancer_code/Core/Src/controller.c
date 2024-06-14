/*
 * controller.c
 *
 *  Created on: Jun 8, 2024
 *      Author: Aidan
 */
/// @file controller.c
/// @brief Provides functions relevant to system control.
#include "controller.h"
#include "main.h"
#include <math.h>

#define MAX_CHANGE 0.1	///< Maximum change in controller speed setpoint per interval

// LQR gains, used to calculte new requested speed
float K[3] = {15, 0.1, -0.3};	///< LQR gain matrix

// state vector
float x[3] = {0,0,0};		///< State vector in form [angle, angular_vel, wheel_spd]
float u = 0; 				///< Commanded motor speed

char ctrl_msg[50];


// system variables. assume x to be the axis running vertically through the IMU
float angle_z = 0.0f;
float angular_velocity = 0.0f;

/**
 * Calculates an updates the motor speed setpoint using an LQR implementation.
 * Calculates the next motor speed setpoint using current state vector and set of LQR gains. LQR gains originally calculated using MATLAB script.
 * @param p_odrive Pointer to an ODrive object.
 * @param angle Current system angle.
 * @param angular_velocity Current system angular velocity.
 * @param motor_curr_speed Current motor speed.
 */
void update_control(odrive_t *p_odrive, float angle, float angular_velocity, float motor_curr_speed)
{
	x[0] = angle;
	x[1] = angular_velocity;
	x[2] = motor_curr_speed;

	float u_prev = u;

	u = -(K[0]*x[0] + K[1]*x[1] + K[2]*x[2]);

	u = u / (2*M_PI);

	u = fmaxf(fminf(u,10),-10); // limiting requested motor speed

	float delta_u = u - u_prev;

	// rate limiting on speed change in a single cycle
	if (abs(delta_u) > MAX_CHANGE ) {
		if (delta_u > 0){
			u = u_prev + MAX_CHANGE;

		} else if (delta_u < 0){
			u = u_prev - MAX_CHANGE;
		}
	}

	ODRIVE_SetVelocity(p_odrive, 0, u);
}


// reads IMU data and calculates angle of the system
/**
 * Initializes the system angle using the IMU using a complementary filter.
 * Calls functions from the IMU driver and calculates the system angle using a complementary filter.
 * Noise was determined to be substantial even in the gyro data, so the gyro output is smoothed by calculating the true change in system angle.
 * This is a little messy.
 * @param ICM_20948 Pointer to an IMU object
 */
pos_spd calculate_IMU_Angle(ICM_20948 *p_IMU)
{

	float ax, ay, az;
	float gx, gy, gz;
	float gyro_angle;
	float accel_angle_z;
	static float previous_angle_z= 0.0000;

	// read both of the sensors
	AccelData accel = IMU_read_accel(p_IMU);	// m/s^2
	ax = accel.accel_x;
	ay = accel.accel_y;
	az = accel.accel_z;

	GyroData gyro = IMU_read_gyro(p_IMU);
	gx = gyro.gyro_x;
	gy = gyro.gyro_y;
	gz = gyro.gyro_z;

	gyro_angle = angle_z + gz*DT;

	accel_angle_z = atan2f(ax,ay);

    if (accel_angle_z > M_PI) accel_angle_z -= 2 * M_PI;
    else if (accel_angle_z < -M_PI) accel_angle_z += 2 * M_PI;

    accel_angle_z = accel_angle_z  * (180.0 / M_PI);

    // complementary filter
	angle_z = ALPHA * gyro_angle  +  (1 - ALPHA) * accel_angle_z;

    // Calculate tangential acceleration (at)
    //float at = ax * cosf(angle_z) + ay * sinf(angle_z);

    // Calculate angular acceleration (alpha)
    //float alpha = at / DISPLACEMENT_DIST;

    // Update angular velocity using angular acceleration
    float angular_velocity = (angle_z-previous_angle_z) / DT;
    previous_angle_z = angle_z;

    pos_spd output = {angle_z, angular_velocity};

	return output;
}






