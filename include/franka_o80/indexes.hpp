#pragma once

namespace franka_o80
{

///Actuator numbers corresponding to robot angular positions
static const int robot_positions[7]  = { 0,  1,  2,  3,  4,  5,  6  };
///Actuator numbers corresponding to robot angular velocities
static const int robot_velocities[7] = { 7,  8,  9,  10, 11, 12, 13 };
///Actuator numbers corresponding to robot torques
static const int robot_torques[7]    = { 14, 15, 16, 17, 18, 19, 20 };
///Actuator number corresponding to gripper width
static const int gripper_width       = 21;
///Actuator number corresponding to gripper temperature
static const int gripper_temperature = 22;
///Actuator number corresponding to position control. If set to `1`, robot will read position values. Can be combined with torque control
static const int control_positions   = 23;
///Actuator number corresponding to velocity control. If set to `1`, robot will read velocity values. Can be combined with torque control
static const int control_velocities  = 24;
///Actuator number corresponding to torque control. If set to `1`, robot will read torque values. Can be combined with position and velocity control
static const int control_torques     = 25;
///Actuator number corresponding to error indicator
static const int control_error       = 26;
///Actuator numbers corresponding to reset
static const int control_reset       = 27;

}  // namespace franka_o80