#pragma once

#include "driver_input_output.hpp"
#include <iostream>
#include <string>

namespace franka_o80
{
///Input for `Driver`
class DriverInput : public DriverInputOutput
{
public:
    //Creates driver input
    DriverInput();
    //Creates driver input from state
    DriverInput(const o80::States<actuator_number, State> &states);
    ///Prints driver input to `std::cout`
    //@param `true` to end output with `std::endl`
    void print(bool endl);
    ///Returs string representation of driver input
    std::string to_string() const;
};
}  // namespace franka_o80