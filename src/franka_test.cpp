#include "../include/franka_o80/franka_test.hpp"

franka::Duration::Duration(unsigned int milliseconds)
{
}

void franka::Robot::signal_handler_(int sig)
{
}

void franka::Robot::create_timer_()
{
    if (control_.exchange(true) == true) throw franka::InvalidOperationException("franka_test: control already running");

    struct sigaction signal_action;
    signal_action.sa_flags = 0;
    signal_action.sa_handler = signal_handler_;
    sigemptyset(&signal_action.sa_mask);
    if (sigaction(SIGRTMIN, &signal_action, NULL) < 0) throw franka::RealtimeException("franka_test: sigaction failed");

    struct sigevent signal_event;
    signal_event.sigev_notify = SIGEV_SIGNAL;
    signal_event.sigev_signo = SIGRTMIN;
    signal_event.sigev_value.sival_ptr = &timer_;
    if (timer_create(CLOCK_REALTIME, &signal_event, &timer_) < 0) throw franka::RealtimeException("franka_test: timer_create failed");

    struct itimerspec timer_specification;
    timer_specification.it_value.tv_sec = 0;
    timer_specification.it_value.tv_nsec = 1000 * 1000;
    timer_specification.it_interval.tv_sec = 0;
    timer_specification.it_interval.tv_nsec = 0;
    if (timer_settime(timer_, 0, &timer_specification, nullptr) < 0) throw franka::RealtimeException("franka_test: timer_create failed");
}

void franka::Robot::delete_timer_()
{
    if (timer_delete(timer_) < 0) throw franka::RealtimeException("franka_test: timer_delete failed");
    control_ = false;
}

franka::Robot::Robot(std::string ip)
{
    for (size_t i = 0; i < 7; i++)
    {
        positions_[i] = 0.0;
        previous_positions_[i] = 0.0;
    }
}

franka::RobotState franka::Robot::readOnce()
{ 
	franka::RobotState state;
    for (size_t i = 0; i < 7; i++)
    {
        state.q[i] = positions_[i];
        state.dq[i] = (positions_[i] - previous_positions_[i]) * 1000.0;
    }
    return state;
}

void franka::Robot::control(std::function<JointPositions(const RobotState &state, Duration time)> positions_control)
{
    create_timer_();
    while (true)
    {
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }
        JointPositions positions = positions_control(readOnce(), 1);
        if (positions.motion_finished) break;
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] = positions.q[i];
        }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control)
{
    create_timer_();
    while (true)
    {
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }
        JointVelocities velocities = velocities_control(readOnce(), 1);
        if (velocities.motion_finished) break;
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] += 0.001 * velocities.dq[i];
        }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control)
{
    create_timer_();
    while (true)
    {
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }
        Torques torques = torques_control(readOnce(), 1);
        if (torques.motion_finished) break;
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
        }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointPositions(const RobotState &state, Duration time)> positions_control)
{
    create_timer_();
    while (true)
    {
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }
        RobotState state = readOnce();
        torques_control(state, 1);
        JointPositions positions = positions_control(state, 1);
        if (positions.motion_finished) break;
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] = positions.q[i];
        }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control)
{
    create_timer_();
    while (true)
    {
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }
        RobotState state = readOnce();
        torques_control(state, 1);
        JointVelocities velocities = velocities_control(state, 1);
        if (velocities.motion_finished) break;
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] += 0.001 * velocities.dq[i];
        }
    }
    delete_timer_();
}

franka::Gripper::Gripper(std::string ip) : time_distribution_(500, 2000)
{
    random_engine_.seed((unsigned int)time(nullptr));
}

franka::GripperState franka::Gripper::readOnce()
{
    if (reading_.exchange(true) == true) franka::InvalidOperationException("franka_test: control already running");
    usleep(time_distribution_(random_engine_));
    franka::GripperState state;
    state.width = width_;
    state.temperature = 20.0;
    reading_ = false;
    return state;
}

void franka::Gripper::move(double width, double speed)
{
    usleep(time_distribution_(random_engine_));
    width_ = width;
}