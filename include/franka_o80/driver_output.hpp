#pragma once

#include <franka/control_types.h>
#include <iostream>
#include <string>

namespace franka_o80
{
class DriverOutput
{
public:
	franka::JointPositions joint_positions;
	double gripper_width;

	DriverOutput();
    void print(bool endl);
};
}  // namespace franka_o80