# Inertia Balancer Project

**Author:** Aidan Schwing, Sydney Ulvick

https://aidanschwing.github.io/inertia_balancer/



# Introduction
An inverted pendulum is a classic example of an unstable, controllable system in control theory. These systems are typically created using either a movable base or a moving mass to control the position of the system. We set out to implement such a system with reaction wheel-based balancing.  Initial designs of the system included a servo to brake the spinning mass in order to “bounce” up and balance. Our team was mostly interested in the control system implementation, and as such we chose to use an off-the-shelf motor controller. An ODrive derivative was ultimately chosen to serve as the BLDC driver with a 650kv quad-copter motor for price and availability. We wanted to constrain the system size to something that could serve as a display on a desk, approximately 6x6”. The final design is presented below. 

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image.png)

# Hardware

## Mechanical Hardware
The cube has a simple frame composed of two 3D printed sheets, separated by standoffs on four corners, fastened by M5 hardware. The inertia wheel is waterjet 6065 aluminum, post machined on the lathe to ensure concentricity with the motor shaft. The center hole was sized to ensure a light press fit on the motor shaft. The outer end of the motor shaft was turned to slide into a flanged bearing. The reaction wheel pressed into the motor shaft can be seen below, with its outer profile about to be turned to eliminate any eccentricity in the inertia.

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-1.png)
![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-2.png)

Motor selection was paramount for ensuring that the system would be able to output enough torque for balancing from relatively large angles. Referenced projects implement small, gimbal-style motors or extremely low kv quadcopter motors. Motor selection is interconnected with controller selection, as the controller has to be sized appropriately to meet current requirements. We settled on a 650kv quadcopter motor due to the low kv and ability to use an ODrive field-oriented-control (FOC).

Original plans for the project were to directly control wheel torque for the purposes of manipulating system angle. Torque output by the rotating wheel can be directly related to the system angle, as discussed later in the system modeling section. ODrive is an open-source system that has torque control capabilities. The ODrive’s lowest level of control occurs directly on torque, with velocity and position controls being cascaded in exterior loops. Accessing the inner loop allows us to most directly control the system. 

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-3.png)

Upon exploration with real hardware, we discovered that the ODrive’s torque resolution is too low to control the system, driving us to use the velocity controller instead. This could potentially be resolved through use of a heavier flywheel or modifications to the controller hardware, but we did not have time to perfect the controller. 

## Electrical Hardware
To measure the angle of tilt, the ICM 20948 inertial measurement unit (IMU) was used. The imu was placed along the axis that we wanted the cube to balance along, allowing us to calculate system angle from 2 of the IMU axes. The result of this is the x direction pointing right and the y direction pointing up when the cube is upright. A depiction of the IMU orientation on the cube can be seen below. The chosen IMU has both I2C and SPI communication protocols; we chose to use I2C.  

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-4.png)

Interface between the motor controller and MCU PCB was accomplished with UART. This UART connection allows for reading and writing of almost every motor control parameter. Of note are direct speed and position readouts, which can be used for inputs to the LQR controller. 

The system was powered via a 4s1p, with 450 mAh, 75C max LiPO battery. The PCB was designed to interact with required sensors and motor drivers, and needed power provided appropriately. Input battery voltage (14.8V) was stepped to 5V via a buck regulator, then to 3.8V via a linear regulator for the MCU, then to 1.8V via another linear regulator for the IMU. Headers were included to power and interact with the servo, motor, motor driver, encoder, and remote e stop during troubleshooting and for worst-case if we were unable to get board power functional. Status LEDs for all voltage levels and as an MCU output were implemented. 

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-5.png)

# Software Implementation
All code for this project was written in C. Tasks are implemented in a pseudo-state machine. The IMU state generates a new set of state variables that are used for calculation of the controller set-point in the reaction state. While they are drawn as dependent in Figure X, each of the states occurs only when a flag is set by a timer-based interrupt. Timer interrupts exist for both of the critical tasks and only raise a flag indicating that the task should be called. This ensures that the IMU read and reaction set-point recalculation occur at intervals that are approximately the same, but prevents the tasks from interrupting each other. If the set-point receives an invalid input, no speed is set. If the IMU state is missed, the control state simply outputs the same speed as the previous iteration. 

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-6.png)

## IMU
An IMU driver was created from the ground up due to lack of available resources on its use with I2C. The initialization of the IMU checks the connection of the device and reads the device address. It then sends feedback through UART to a laptop. 

To best read the pitch angle, both accelerometer and gyroscope data were read. The gyroscope was read for short term accuracy but drifted through extended durations while accelerometer data was consistent but had significant noise. In order to get an accurate pitch angle over a long period of time, a complementary filter was implemented with an alpha of 0.98 on accelerometer readings. The pitch was read from the accelerometer by taking the inverse tan of the y and x components, knowing that gravity is always in the direction of negative y. The pitch from the gyro was read by integrating the value by multiplying it with the sampling rate. This setup requires the system to be “calibrated” by leaving the system near it’s zero point for approximately 3 seconds, allowing the calculated system angle to settle. 

Ultimately, it would have been easier to use an IMU with built-in euler angle calculations. 

## Motor Driver: ODrive Interface
Communication with the ODrive motor controller from a separate MCU is possible with a wide variety of protocols. The simplest of these is the [ASCII protocol](https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html) , where strings are sent in a specified format and terminated with a new-line character. This protocol allows for both writing and reading of control parameters for the BLDC. 

The ODrive also responds to some commands with specified outputs of known format. Receiving these responses is non-trivial and is performed with a callback on UART inputs that checks the length of a message buffer and for the termination character of the response. A blocking “receive” state is placed on the main code loop during message receives from the external controller, though this response period is usually negligibly short compared to other code critical code execution.  

## Control System
Multiple controllers were considered for optimally balancing. While PID control can be useful for driving the system to zero angle, it does allow for minimization of the spinning mass’s speed. An LQR controller can ideally perform multi-variate control, as the controlled variable can be treated as a function of the chosen state variables. An LQR controller can minimize system angle and wheel speed if optimized correctly. LQR controller implementation is also computationally inexpensive, as once the state variables are determined it is simply a matter of multiplying said state variables by their respective gains. 

The system was modeled using state variables of system angle, angular speed, and wheel angular speed. Due to the torque control issues mentioned in the mechanical hardware section, we switched the BLDC to control motor speed. We then can estimate the change in speed with some small time constant approximated from step tests in speed.

Once the state matrices were determined, MATLAB was used to discretize the system model and determine the LQR gains. The discretized system response appeared reasonable, but ultimately the gains failed to adequately control the system. 

Our largest issue with control was oscillations around zero angle. High penalty on system angle during LQR gain calculation results in very aggressive oscillations around the zero angle as error changes signs, while low penalty fails to adequately generate the torque required beyond a very small angle range. The system can balance for an extremely small range of angles, but further exploration into control strategies could prove beneficial to performance. Direct control over torque as originally planned would be best for system control, as the speed of the motor is not directly coupled with the system angle. 

![alt text](https://github.com/AidanSchwing/inertia_balancer/blob/main/balancer_code/Core/Pages/image-7.png)
