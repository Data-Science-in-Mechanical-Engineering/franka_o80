#pragma once

#include "driver_input_output.hpp"
#include <iostream>
#include <string>

namespace franka_o80
{
///Output for `Driver`
class DriverOutput : public DriverInputOutput
{
public:
    //Creates driver output
    DriverOutput();
    //Creates driver output from state
    DriverOutput(const o80::States<actuator_number, State> &states);
    ///Prints driver output to `std::cout`
    //@param `true` to end output with `std::endl`
    void print(bool endl);
    ///Returs string representation of driver output
    std::string to_string() const;
};
}  // namespace franka_o80