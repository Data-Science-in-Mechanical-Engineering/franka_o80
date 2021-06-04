#pragma once

namespace franka_o80
{
///Actuator number corresponding to position control. If set to `1`, robot will read position values. Can be combined with torque control
static const int control_positions   = 0;
///Actuator number corresponding to velocity control. If set to `1`, robot will read velocity values. Can be combined with torque control
static const int control_velocities  = 1;
///Actuator number corresponding to torque control. If set to `1`, robot will read torque values. Can be combined with position and velocity control
static const int control_torques     = 2;
///Actuator number corresponding to error indicator
static const int control_error       = 3;
///Actuator numbers corresponding to reset
static const int control_reset       = 4;
///Actuator number corresponding to gripper width
static const int gripper_width       = 5;
///Actuator number corresponding to gripper temperature
static const int gripper_temperature = 6;

///Actuator numbers corresponding to robot angular positions (States only)
static const int robot_positions[7]  = {  7,  8,  9, 10, 11, 12, 13 };
///Actuator numbers corresponding to robot angular velocities (States only)
static const int robot_velocities[7] = { 14, 15, 16, 17, 18, 19, 20 };
///Actuator numbers corresponding to robot torques (States only)
static const int robot_torques[7]    = { 21, 22, 23, 24, 25, 26, 27 };

///Actuator numbers corresponding to effector position (CartesialStates only)
static const int cartesial_positions[3] = {  7,  8,  9 };
///Actuator numbers corresponding to effector velociry (CartesialStates only)
static const int cartesial_velocities[3]= { 10, 11, 12 };
///Actuator numbers corresponding to effector force (CartesialStates only)
static const int cartesial_forces[3]    = { 13, 14, 15 };
///Actuator numbers corresponding to effector forward direction (CartesialStates only)
static const int cartesial_forward[3]   = { 16, 17, 18 };
///Actuator numbers corresponding to effector right direction (CartesialStates only)
static const int cartesial_right[3]     = { 19, 20, 21 };

}  // namespace franka_o80