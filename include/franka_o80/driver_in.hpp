#pragma once

#include <franka/control_types.h>
#include <iostream>
#include <string>

namespace franka_o80
{
class DriverIn
{
public:
	franka::JointPositions joint_positions{};
	double gripper_width;

	DriverIn();
    void print(bool endl);
    std::string to_string() const;
};
}  // namespace franka_o80