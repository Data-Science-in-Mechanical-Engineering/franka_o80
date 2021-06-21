#pragma once

#include "limits.hpp"
#include "actuator.hpp"
#include "error.hpp"
#include "driver_input.hpp"
#include "driver_output.hpp"
#ifndef FRANKA_O80_TEST
    #include <franka/robot.h>
    #include <franka/model.h>
    #include <franka/gripper.h>
    #include <franka/exception.h>
#else
    #include "franka_test.hpp"
#endif
#include <o80/driver.hpp>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_o80
{
///Driver, controls arm and hand. Mostly maintained by `Standalone`
class Driver : public o80::Driver<DriverInput, DriverOutput>
{
private:
    bool started_ = false;
    std::string ip_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    std::thread robot_control_thread_;
    std::unique_ptr<franka::Gripper> gripper_;
    std::thread gripper_control_thread_;
	Mode mode_ = Mode::invalid;
	DriverInput input_;
    DriverOutput output_;
    bool input_finished_ = false;
	std::mutex input_output_mutex_;
    Eigen::Matrix<double, 7, 7> joint_stiffness_, joint_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_, cartesian_damping_;

    void robot_write_output_(const franka::RobotState &robot_state);
    void robot_dummy_control_function_(const franka::RobotState &robot_state, franka::JointVelocities *velocities);
    void robot_torque_control_function_(const franka::RobotState &robot_state, franka::Torques *torques);
	void robot_position_control_function_(const franka::RobotState &robot_state, franka::JointPositions *positions);
    void robot_velocity_control_function_(const franka::RobotState &robot_state, franka::JointVelocities *velocities);
    void robot_cartesian_position_control_function_(const franka::RobotState &robot_state, franka::CartesianPose *cartesian_position);
    void robot_cartesian_velocity_control_function_(const franka::RobotState &robot_state, franka::CartesianVelocities *cartesian_velocity);

    void robot_control_function_();
    void gripper_control_function_();

public:
    ///Creates Driver for robot (arm) and hand (gripper) on specified IP address
    ///@param ip IPv4 address of robot and gripper
    Driver(std::string ip);
    ///Starts driver
    void start();
    ///Stops driver
    void stop();
    ///Sets input to be given to robot and gripper
    ///@param input Input to robot and gripper
    void set(const DriverInput& input);
    ///Gets output from robot and gripper
    DriverOutput get();
};
}  // namespace franka_o80
