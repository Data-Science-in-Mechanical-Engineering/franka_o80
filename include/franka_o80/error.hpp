#pragma once

namespace franka_o80 {

///Error
enum class Error
{
    ok,
    robot_command_exception,
    robot_control_exception,
    robot_invalid_operation_exception,
    robot_network_exception,
    robot_realtime_exception,
    robot_invalid_argument_exception,
    robot_other_exception,
    gripper_command_exception,
    gripper_network_exception,
    gripper_invalid_operation_exception,
    gripper_other_exception
};

} // namespace franka_o80