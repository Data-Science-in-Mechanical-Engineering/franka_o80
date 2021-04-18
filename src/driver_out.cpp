#include "../include/franka_o80/driver_out.hpp"

franka_o80::DriverOut::DriverOut()
{}

void franka_o80::DriverOut::print(bool endl)
{
    std::cout << "Driver output:";
	for (size_t i = 0; i < 7; i++) std::cout << " " << joint_positions.q[i];
	std::cout << " + " << gripper_width;
    if (endl) std::cout << std::endl;
	else std::cout << " ";
}