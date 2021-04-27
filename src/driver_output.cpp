#include "../include/franka_o80/driver_output.hpp"

franka_o80::DriverOutput::DriverOutput()
{}

franka_o80::DriverOutput::DriverOutput(const o80::States<actuator_number, State> &states) : DriverInputOutput(states)
{}

void franka_o80::DriverOutput::print(bool endl)
{
    std::cout << "Driver output: ";
    std::cout << DriverInputOutput::to_string(true);
    if (endl) std::cout << std::endl;
    else std::cout << " ";
}

std::string franka_o80::DriverOutput::to_string() const
{
    return DriverInputOutput::to_string(true);
}