#pragma once

#include <franka/control_types.h>
#include <iostream>
#include <string>

namespace franka_o80
{
///Input for `Driver`
class DriverInput
{
public:
    ///Joint positions of robot
    franka::JointPositions joint_positions;
    ///Width of gripper
    double gripper_width;
    
    ///Creates driver input
    DriverInput();
    ///Prints driver input to `std::cout`
    //@param `true` to end output with `std::endl`
    void print(bool endl);
    ///Returs string representation of driver input
    std::string to_string() const;
};
}  // namespace franka_o80