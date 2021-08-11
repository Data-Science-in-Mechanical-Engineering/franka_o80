#pragma once

namespace franka_o80
{
///Actuator number
static const int actuator_number = 63;

///Actuator number corresponding to control mode. Needs to be set to one of `Mode` values
static const int control_mode        = 0;
///Actuator number corresponding to error indicator
static const int control_error       = 1;
///Actuator numbers corresponding to reset
static const int control_reset       = 2;

///Actuator number corresponding to gripper width
static const int gripper_width       = 3;
///Actuator number corresponding to gripper temperature
static const int gripper_temperature = 4;
///Actuator number corresponding to gripper force
static const int gripper_force       = 62;

///Actuator numbers corresponding to robot angular positions
static const int joint_position[7]  = {  5,  6,  7,  8,  9, 10, 11 };
///Actuator numbers corresponding to robot angular velocities
static const int joint_velocity[7]  = { 12, 13, 14, 15, 16, 17, 18 };
///Actuator numbers corresponding to robot torques
static const int joint_torque[7]    = { 19, 20, 21, 22, 23, 24, 25 };

///Actuator numbers corresponding to effector position
static const int cartesian_position[3]      = { 26, 27, 28 };
///Actuator numbers corresponding to effector orientation
static const int cartesian_orientation      = 29;
///Actuator numbers corresponding to effector translation velocity
static const int cartesian_velocity[3]      = { 30, 31, 32 };
///Actuator numbers corresponding to effector rotation velocity (WRT)
static const int cartesian_rotation[3]      = { 33, 34, 35 };

///Actuator numbers corresponding to joint-space stiffness
static const int joint_stiffness[7]         = { 36, 37, 38, 39, 40, 41, 42 };
///Actuator numbers corresponding to joint-space damping
static const int joint_damping[7]           = { 43, 44, 45, 46, 47, 48, 49 };
///Actuator numbers corresponding to cartesian stiffness (for velocity and rotation)
static const int cartesian_stiffness[6]     = { 50, 51, 52, 53, 54, 55 };
///Actuator numbers corresponding to cartesian damping (for velocity and rotation)
static const int cartesian_damping[6]       = { 56, 57, 58, 59, 60, 61 };

}  // namespace franka_o80