/*
 * controller.h
 *
 *  Created on: Jun 8, 2024
 *      Author: Aidan
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"
#include "IMU_driver.h"
#include <math.h>

// defines for the complementary filter
#define ALPHA 0.98
#define DT 0.01 					// sampling period
#define DISPLACEMENT_DIST 0.09		// displacement distance to the axis of the gyro that is


// prototype functions
// LQR controller, update speed setpoint
void update_control(float angle, float angular_velocity, float motor_curr_speed);
// System angle using complementary filter on gyro and accel data
float calculate_IMU_Angle(ICM_20948 *p_IMU);


#endif /* INC_CONTROLLER_H_ */
