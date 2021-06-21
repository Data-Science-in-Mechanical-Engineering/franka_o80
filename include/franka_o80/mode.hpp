#pragma once

namespace franka_o80
{
enum class Mode
{
    invalid,
	torque,
	torque_position,
	torque_velocity,
	torque_cartesian_position,
	torque_cartesian_velocity,
	position,
	velocity,
	cartesian_position,
	cartesian_velocity,
	intelligent_position,
	intelligent_cartesian_position
};

} //namespace franka_o80