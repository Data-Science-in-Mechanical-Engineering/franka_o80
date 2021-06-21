#pragma once

#include "queue.hpp"
#include "actuator.hpp"
#include "state.hpp"
#include <o80/state.hpp>
#include <o80/front_end.hpp>

namespace franka_o80
{
typedef o80::FrontEnd<franka_o80::queue_size, franka_o80::actuator_number, franka_o80::State, o80::VoidExtendedState> FrontEnd;
} // namespace franka_o80