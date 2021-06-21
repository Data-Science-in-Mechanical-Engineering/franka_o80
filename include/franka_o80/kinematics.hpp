#pragma once

#include "states.hpp"
#include "actuator.hpp"
#include <array>
#include <limits>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace franka_o80
{
///Transforms joint state to cartesian state
void joint_to_cartesian(States &states);

///Transforms cartesian state to joint state
void cartesian_to_joint(States &states);

///Transforms cartesian state to joint state
void cartesian_to_joint(States &states, const States &hint);

///Forward and inverse kinematics information
class Kinematics
{
    friend void joint_to_cartesian(States &states);
    friend void cartesian_to_joint(States &states);
    friend void cartesian_to_joint(States &states, const States &hint);

private:
    static bool initialized_;
    static size_t robot_joint_ids_[7];
    static pinocchio::Model model_;
    static pinocchio::Data data_;
    static void initialize_();
};

}  // namespace franka_o80