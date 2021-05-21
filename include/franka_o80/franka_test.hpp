#pragma once

#include "constants.hpp"
#include <stdexcept>
#include <functional>
#include <atomic>
#include <mutex>
#include <random>
#include <chrono>
#include <thread>
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
    double q[7];
};

class JointVelocities : public Finishable
{
public:
    double dq[7];
};

class Torques : public Finishable
{
public:
    double tau_J[7];
};

class Robot
{
private:
	double positions_[7];
    double previous_positions_[7];
    std::atomic<bool> control_ = false;
    std::uniform_int_distribution<unsigned int> time_distribution_;
    std::default_random_engine random_engine_;
    
    #ifndef FRANKA_O80_DEBUG
        timer_t timer_;
        static void signal_handler_(int sig);
    #endif

    void create_timer_();
    void wait_timer_();
    void delete_timer_();
    RobotState state_() const;

public:
    Robot(std::string ip);
    RobotState readOnce();
    void control(std::function<JointPositions(const RobotState &state, Duration time)> positions_control);
    void control(std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control);
    void control(std::function<Torques(const RobotState &state, Duration time)> torques_control);
    void control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointPositions(const RobotState &state, Duration time)> positions_control);
    void control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control);
    void automaticErrorRecovery();
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