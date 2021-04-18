#include "../include/franka_o80/driver.hpp"

franka_o80::Driver::Driver(std::string ip) : ip_(ip)
{
}

void franka_o80::Driver::start()
{
    robot_ = std::unique_ptr<franka::Robot>(new franka::Robot(ip_));
	gripper_ = std::unique_ptr<franka::Gripper>(new franka::Gripper(ip_));
	thread_ = std::thread([](Driver *driver)
	{
		driver->robot_->control([driver](const franka::RobotState &s, franka::Duration t) -> franka::JointPositions
		{
			return driver->in_.joint_positions;
		});
	}, this);
}

void franka_o80::Driver::set(const DriverIn &in)
{
	in_ = in;
}

franka_o80::DriverOut franka_o80::Driver::get()
{
	return out_;
}

void franka_o80::Driver::stop()
{
	in_.joint_positions.motion_finished = true;
	thread_.join();
}