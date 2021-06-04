#pragma once

#include "states.hpp"
#include "indexes.hpp"
#include <array>
#include <limits>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace franka_o80
{
class CartesialStates;

///Cartesial space state
class CartesialStates
{
private:
    friend CartesialStates to_cartesial(const States &states);
    friend States to_joint(const CartesialStates &states, double position0);
    friend States to_joint(const CartesialStates &states, const States &hint, double position0);

    static bool initialized_;
    static size_t robot_joint_ids_[7];
    static pinocchio::Model model_;
    static pinocchio::Data data_;

public:
    ///Creates cartesial state
    CartesialStates();
    ///Sets state for specified actuator
    void set(int actuator, State state);
    ///Gets state fro specified actuator
    State get(int actuator) const;
    ///Raw values
    std::array<State, 22> values;
};

///Transforms joint state to cartesial state
CartesialStates to_cartesial(const States &states);

///Transforms cartesial state to joint state
States to_joint(const CartesialStates &states, double position0 = std::numeric_limits<double>::quiet_NaN());

///Transforms cartesial state to joint state
States to_joint(const CartesialStates &states, const States &hint, double position0 = std::numeric_limits<double>::quiet_NaN());

}  // namespace franka_o80