#include "../include/franka_o80/driver.hpp"

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
        return joint_positions;
    });
}

void franka_o80::Driver::gripper_control_function_(Driver *driver)
{
    double gripper_width = driver->gripper_->readOnce().width;
    bool finished = false;
    while (true)
    {
        {
            std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
            driver->output_.gripper_width = gripper_width;
            gripper_width = driver->input_.gripper_width;
            finished = driver->input_.joint_positions.motion_finished;
        }
        if (finished) return;
        driver->gripper_->move(gripper_width, 0.01); //Fix magic number!
        gripper_width = driver->gripper_->readOnce().width;
    }
}

franka_o80::Driver::Driver(std::string ip) : ip_(ip)
{
}

void franka_o80::Driver::start()
{
    robot_ = std::unique_ptr<franka::Robot>(new franka::Robot(ip_));
    robot_control_thread_ = std::thread(robot_control_function_, this);
    gripper_ = std::unique_ptr<franka::Gripper>(new franka::Gripper(ip_));
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