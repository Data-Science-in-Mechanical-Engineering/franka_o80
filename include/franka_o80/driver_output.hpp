#pragma once

#include <franka/control_types.h>
#include <iostream>
#include <string>

namespace franka_o80
{
///Output of `Driver` or observation
class DriverOutput
{
public:
    ///Joint positions of robot
    franka::JointPositions joint_positions;
    ///Width of gripper
    double gripper_width;
    
    ///Creates driver output
    DriverOutput();
    ///Prints driver input to `std::cout`
    //@param `true` to end output with `std::endl`
    void print(bool endl);
};
}  // namespace franka_o80