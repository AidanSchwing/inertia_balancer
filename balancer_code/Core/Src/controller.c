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
float angle_z = 0.0f;
float angular_velocity = 0.0f;

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






