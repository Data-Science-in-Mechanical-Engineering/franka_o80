#pragma once

namespace franka_o80
{
///Actuator number
static const int actuator_number = 63;

///Actuator number corresponding to control mode. Contains `franka_o80::Mode` values
static const int control_mode        = 0;
///Actuator number corresponding to error indicator. Contains `franka_o80::Error` values
static const int control_error       = 1;
///Actuator numbers corresponding to reset
static const int control_reset       = 2;

///Actuator number corresponding to gripper width
static const int gripper_width       = 3;
///Actuator number corresponding to gripper temperature
static const int gripper_temperature = 4;
///Actuator number corresponding to gripper force
static const int gripper_force       = 5;

///Actuator numbers corresponding to robot angular positions
static const int joint_position[7]  = {  6,  7,  8,  9, 10, 11, 12 };
///Actuator numbers corresponding to robot angular velocities
static const int joint_velocity[7]  = { 13, 14, 15, 16, 17, 18, 19 };
///Actuator numbers corresponding to robot torques
static const int joint_torque[7]    = { 20, 21, 22, 23, 24, 25, 26 };

///Actuator numbers corresponding to effector position
static const int cartesian_position[3]      = { 27, 28, 29 };
///Actuator numbers corresponding to effector orientation. Contains quaternion values
static const int cartesian_orientation      = 30;
///Actuator numbers corresponding to effector translation velocity
static const int cartesian_velocity[3]      = { 31, 32, 33 };
///Actuator numbers corresponding to effector rotation velocity (WRT)
static const int cartesian_rotation[3]      = { 34, 35, 36 };

///Actuator numbers corresponding to joint-space stiffness
static const int joint_stiffness[7]         = { 37, 38, 39, 40, 41, 42, 43 };
///Actuator numbers corresponding to joint-space damping
static const int joint_damping[7]           = { 44, 45, 46, 47, 48, 49, 50 };
///Actuator numbers corresponding to cartesian stiffness (for velocity and rotation)
static const int cartesian_stiffness[6]     = { 51, 52, 53, 54, 55, 56 };
///Actuator numbers corresponding to cartesian damping (for velocity and rotation)
static const int cartesian_damping[6]       = { 57, 58, 59, 60, 61, 62 };

}  // namespace franka_o80