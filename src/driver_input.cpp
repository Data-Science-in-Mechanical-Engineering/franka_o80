#include "../include/franka_o80/driver_input.hpp"

franka_o80::DriverInput::DriverInput() : joint_positions{}
{}

void franka_o80::DriverInput::print(bool endl)
{
    std::cout << "Driver input:";
	for (size_t i = 0; i < 7; i++) std::cout << " " << joint_positions.q[i];
	std::cout << " + " << gripper_width;
    if (endl) std::cout << std::endl;
	else std::cout << " ";
}

std::string franka_o80::DriverInput::to_string() const
{
	std::string result;
	for (size_t i = 0; i < 7; i++)
	{
		result += " ";
		result += std::to_string(joint_positions.q[i]);
	}
	result += " + ";
	result += std::to_string(gripper_width);
	return result;
}