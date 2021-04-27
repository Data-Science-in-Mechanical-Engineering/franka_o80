#pragma once

#include "constants.hpp"
#include "state.hpp"
#include <o80/state.hpp>
#include <o80/front_end.hpp>

namespace franka_o80
{
typedef o80::FrontEnd<franka_o80::queue_size, 1, o80::State<franka_o80::State, franka_o80::State>, o80::VoidExtendedState> FrontEnd;
} // namespace franka_o80