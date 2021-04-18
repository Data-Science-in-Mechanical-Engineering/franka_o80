#include "../include/franka_o80/driver_output.hpp"

franka_o80::DriverOutput::DriverOutput() : joint_positions{}
{}

void franka_o80::DriverOutput::print(bool endl)
{
    std::cout << "Driver output:";
    for (size_t i = 0; i < 7; i++) std::cout << " " << joint_positions.q[i];
    std::cout << " + " << gripper_width;
    if (endl) std::cout << std::endl;
    else std::cout << " ";
}