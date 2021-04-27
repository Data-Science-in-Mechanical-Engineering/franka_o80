#include "../include/franka_o80/driver_input.hpp"

franka_o80::DriverInput::DriverInput()
{}

franka_o80::DriverInput::DriverInput(const o80::States<actuator_number, State> &states) : DriverInputOutput(states)
{}

void franka_o80::DriverInput::print(bool endl)
{
    std::cout << "Driver input: ";
    std::cout << DriverInputOutput::to_string(false);
    if (endl) std::cout << std::endl;
    else std::cout << " ";
}

std::string franka_o80::DriverInput::to_string() const
{
    return DriverInputOutput::to_string(false);
}