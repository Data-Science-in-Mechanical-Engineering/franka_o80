#pragma once

#include "state.hpp"
#include "constants.hpp"
#include "indexes.hpp"
#include <o80/states.hpp>
#include <string>

namespace franka_o80
{
///Input or output of `Driver`
class DriverInputOutput : public o80::States<actuator_number, State>
{
protected:
    DriverInputOutput();
    DriverInputOutput(const o80::States<actuator_number, State> &states);
    std::string to_string(bool output) const;
};
}  // namespace franka_o80