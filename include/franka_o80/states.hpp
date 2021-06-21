#pragma once

#include "actuator.hpp"
#include "state.hpp"
#include <o80/states.hpp>
#include <string>

namespace franka_o80
{
///Joint-space state
typedef o80::States<franka_o80::actuator_number, franka_o80::State> States;

//Default state of the robot
States default_states();
}  // namespace franka_o80