#pragma once

#include <franka/control_types.h>
#include <iostream>
#include <string>

namespace franka_o80
{
class DriverInput
{
public:
	franka::JointPositions joint_positions;
	double gripper_width;

	DriverInput();
    void print(bool endl);
    std::string to_string() const;
};
}  // namespace franka_o80