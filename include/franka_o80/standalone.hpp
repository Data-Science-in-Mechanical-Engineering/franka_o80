#pragma once

#include <o80/memory_clearing.hpp>
#include <o80/standalone.hpp>
#include "driver.hpp"
#include "state.hpp"
#include "constants.hpp"

namespace franka_o80
{
class Standalone : public o80::Standalone<queue_size, actuator_number, Driver, State, o80::VoidExtendedState>
{
public:
    Standalone(std::shared_ptr<Driver> driver_ptr, double frequency, std::string segment_id);
    DriverInput convert(const o80::States<actuator_number, State> &joints);
    o80::States<actuator_number, State> convert(const DriverOutput &driver_out);
};

std::string get_segment_id(int id);
} // namespace franka_o80