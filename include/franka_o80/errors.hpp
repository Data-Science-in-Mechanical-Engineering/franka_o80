#pragma once

namespace franka_o80 {
	///No error
	static const double error_ok = 0.0;

    ///Robot has thrown `CommandException`
	static const double error_robot_command_exception = 1.0;
	///Robot has thrown `ControlException`
	static const double error_robot_control_exception = 2.0;
	///Robot has thrown `InvalidOperationException`
	static const double error_robot_invalid_operation_exception = 3.0;
	///Robot has thrown `NetworkException`
	static const double error_robot_network_exception = 4.0;
	///Robot has thrown `RealtimeException`
	static const double error_robot_realtime_exception = 5.0;
	///Robot has thrown `std::invalid_argument`
	static const double error_robot_invalid_argument_exception = 6.0;
    ///Robot has thrown other exception
    static const double error_robot_other_exception = 7.0;

    ///Gripper has thrown `CommandException`
	static const double error_gripper_command_exception = 8.0;
    ///Gripper has thrown `NetworkException`
	static const double error_gripper_network_exception = 9.0;
    ///Gripper has thrown `InvalidOperationException`
	static const double error_gripper_invalid_operation_exception = 10.0;
    ///Gripper has thrown other exception
	static const double error_gripper_other_exception = 11.0;
} // namespace franka_o80