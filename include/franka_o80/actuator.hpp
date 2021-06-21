#pragma once

namespace franka_o80
{
///Actuator number
static const int actuator_number = 40;

///Actuator number corresponding to control mode. Needs to be set to one of `Mode` values
static const int control_mode        = 0;
///Actuator number corresponding to error indicator
static const int control_error       = 3;
///Actuator numbers corresponding to reset
static const int control_reset       = 4;

///Actuator number corresponding to gripper width
static const int gripper_width       = 5;
///Actuator number corresponding to gripper temperature
static const int gripper_temperature = 6;

///Actuator numbers corresponding to robot angular positions
static const int joint_position[7]  = {  7,  8,  9, 10, 11, 12, 13 };
///Actuator numbers corresponding to robot angular velocities
static const int joint_velocity[7]  = { 14, 15, 16, 17, 18, 19, 20 };
///Actuator numbers corresponding to robot torques
static const int joint_torque[7]    = { 21, 22, 23, 24, 25, 26, 27 };

///Actuator numbers corresponding to effector position
static const int cartesian_position[3]      = { 28, 29, 30 };
///Actuator numbers corresponding to effector orientation (Euler angles)
static const int cartesian_orientation[3]   = { 34, 35, 36 };
///Actuator numbers corresponding to effector translation velocity
static const int cartesian_velocity[3]     = { 31, 32, 33 };
///Actuator numbers corresponding to effector rotation velocity (WRT)
static const int cartesian_rotation[3]      = { 37, 38, 39 };

}  // namespace franka_o80