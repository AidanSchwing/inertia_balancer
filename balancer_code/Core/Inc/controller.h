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
#include "odrive_wrapper.h"
#include <math.h>

// defines for the complementary filter
#define ALPHA 0.98
#define DT 0.01 					// sampling period 10ms
#define DISPLACEMENT_DIST 0.07		// displacement distance to the axis of the gyro that is

struct pos_spd {
	float position;
	float speed;
}typedef pos_spd;

// prototype functions
// LQR controller, update speed setpoint
void update_control(odrive_t *p_odrive, float angle, float angular_velocity, float motor_curr_speed);
// System angle using complementary filter on gyro and accel data
pos_spd calculate_IMU_Angle(ICM_20948 *p_IMU);


#endif /* INC_CONTROLLER_H_ */
