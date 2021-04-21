#include "../include/franka_o80/driver.hpp"
#include <limits>

void franka_o80::Driver::robot_control_function_(Driver *driver)
{
    driver->robot_->control([driver](const franka::RobotState &s, franka::Duration t) -> franka::JointPositions
    {
        franka::JointPositions joint_positions{};
        {
            std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
            joint_positions = driver->input_.joint_positions;
            driver->output_.joint_positions = franka::JointPositions(s.q);
        }
		for (size_t i = 0; i < 7; i++)
		{
			if (joint_positions.q[i] != joint_positions.q[i]) joint_positions.q[i] = s.q[i];
		}
        return joint_positions;
    });
}

void franka_o80::Driver::gripper_control_function_(Driver *driver)
{
    while (true)
    {
		double gripper_width = driver->gripper_->readOnce().width;
		bool finished;
        {
            std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
            driver->output_.gripper_width = gripper_width;
            gripper_width = driver->input_.gripper_width;
            finished = driver->input_.joint_positions.motion_finished;
        }
		
        if (finished) return;
		if (gripper_width == gripper_width) driver->gripper_->move(gripper_width, 0.01); //Fix magic number!
    }
}

franka_o80::Driver::Driver(std::string ip) : ip_(ip)
{
}

void franka_o80::Driver::start()
{
	for (size_t i = 0; i < 7; i++) input_.joint_positions.q[i] = std::numeric_limits<double>::quiet_NaN();
	input_.gripper_width = std::numeric_limits<double>::quiet_NaN();

    robot_ = std::unique_ptr<franka::Robot>(new franka::Robot(ip_));
	output_.joint_positions = franka::JointPositions(robot_->readOnce().q);
    robot_control_thread_ = std::thread(robot_control_function_, this);

    gripper_ = std::unique_ptr<franka::Gripper>(new franka::Gripper(ip_));
	output_.gripper_width = gripper_->readOnce().width;
    gripper_control_thread_ = std::thread(gripper_control_function_, this);
}

void franka_o80::Driver::set(const DriverInput &input)
{
    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        input_ = input;
    }
}

franka_o80::DriverOutput franka_o80::Driver::get()
{
    DriverOutput output;
    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        output = output_;
    }
    return output;
}

void franka_o80::Driver::stop()
{
    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        input_.joint_positions.motion_finished = true;
    }
    robot_control_thread_.join();
    gripper_control_thread_.join();
}