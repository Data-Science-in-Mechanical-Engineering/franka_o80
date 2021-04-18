#pragma once

namespace franka_o80
{
///Command queue size
static const int queue_size = 40000;
///Actuator number (7 on arm + 1 on hand)
static const int actuator_number = 8;
///Prefix for all segment id's
static char const * const segment_id_base = "franka_o80";
}  // namespace franka_o80
