#pragma once

#include "limits.hpp"
#include "actuator.hpp"
#include "states.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <stdexcept>
#include <functional>
#include <atomic>
#include <mutex>
#include <random>
#include <chrono>
#include <thread>
#include <array>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <syscall.h>

namespace franka {

class Finishable
{
public:
    bool motion_finished = false;
};

class RobotState
{
public:
    double q[7];
    double dq[7];
    double tau_J[7];
};

class Duration
{
public:
    Duration(unsigned int milliseconds);
};

class JointPositions : public Finishable
{
public:
    JointPositions(const std::array<double, 7> &positions);
    double q[7];
};

class JointVelocities : public Finishable
{
public:
    JointVelocities(const std::array<double, 7> &velocities);
    double dq[7];
};

class Torques : public Finishable
{
public:
    Torques(const std::array<double, 7> &torques);
    double tau_J[7];
};

class Robot
{
private:
    const std::string planning_group = "panda_arm";

    double positions_[7];
    double previous_positions_[7];
    std::atomic<bool> control_ = false;
    std::uniform_int_distribution<unsigned int> time_distribution_;
    std::default_random_engine random_engine_;
    
    std::vector<double> previous_sent_positions_;
    std::mutex send_mutex_;
    std::thread send_thread_;
    std::unique_ptr<ros::NodeHandle> node_handle_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    void create_timer_();
    void wait_timer_();
    void delete_timer_();
    RobotState state_() const;

public:
    Robot(std::string ip);
    RobotState readOnce();
    void setCollisionBehavior(
      		const std::array<double, 7> &lower_torque_thresholds_acceleration,
		const std::array<double, 7> &upper_torque_thresholds_acceleration,
		const std::array<double, 7> &lower_torque_thresholds_nominal,
		const std::array<double, 7> &upper_torque_thresholds_nominal,
		const std::array<double, 6> &lower_force_thresholds_acceleration,
		const std::array<double, 6> &upper_force_thresholds_acceleration,
		const std::array<double, 6> &lower_force_thresholds_nominal,
		const std::array<double, 6> &upper_force_thresholds_nominal);
    void setJointImpedance(const std::array<double, 7> &impedance);
    void setCartesianImpedance(const std::array<double, 6> &impedance);
    void control(std::function<JointPositions(const RobotState &state, Duration time)> positions_control);
    void control(std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control);
    void control(std::function<Torques(const RobotState &state, Duration time)> torques_control);
    void control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointPositions(const RobotState &state, Duration time)> positions_control);
    void control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control);
    void automaticErrorRecovery();
    ~Robot();
};

class GripperState
{
public:
    double width;
    double temperature;
};

class Gripper
{
private:
	double width_;
    std::atomic<bool> reading_ = false;
    std::uniform_int_distribution<unsigned int> time_distribution_;
    std::default_random_engine random_engine_;

public:
    Gripper(std::string ip);
    GripperState readOnce();
    void move(double width, double speed);
};

class CommandException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class ControlException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class InvalidOperationException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class NetworkException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class RealtimeException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

} //namespace franka
