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

franka::RobotState franka::Robot::state_() const
{
    franka::RobotState state;
    for (size_t i = 0; i < 7; i++)
    {
        state.q[i] = positions_[i];
        state.dq[i] = (positions_[i] - previous_positions_[i]) * 1000.0;
    }
    return state;
}

franka::Robot::Robot(std::string ip) : time_distribution_(500, 2000)
{
    for (size_t i = 0; i < 7; i++)
    {
        positions_[i] = 0.0;
        previous_positions_[i] = 0.0;
    }
}

franka::RobotState franka::Robot::readOnce()
{
    if (control_.exchange(true) == true) franka::InvalidOperationException("franka_test: control already running");
    std::this_thread::sleep_for(std::chrono::microseconds(time_distribution_(random_engine_)));
    RobotState state = state_();
    control_ = false;
    return state;
}

void franka::Robot::control(std::function<JointPositions(const RobotState &state, Duration time)> positions_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for signal
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }

        //Calling callback
        RobotState state = state_();
        JointPositions positions = positions_control(state, 1);

        //Checcking input
        for (size_t i = 0; i < 7; i++)
        {
            if (positions.q[i] != positions.q[i] || positions.q[i] < franka_o80::robot_positions_min[i] || positions.q[i] > franka_o80::robot_positions_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint position"); }
            if (abs(positions_[i] - positions.q[i]) > 0.001 * franka_o80::robot_velocities_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint velocity"); }
        }

        //Updating state
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] = positions.q[i];
        }

        //Exiting
        if (positions.motion_finished) break;
    }
    delete_timer_();
}

void franka::Robot::control(std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for signal
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }

        //Calling callback
        RobotState state = state_();
        JointVelocities velocities = velocities_control(state, 1);

        //Checcking input
        for (size_t i = 0; i < 7; i++)
        {
            if (velocities.dq[i] != velocities.dq[i] || abs(velocities.dq[i]) > franka_o80::robot_velocities_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint velocity"); }
            if (positions_[i] + 0.001 * velocities.dq[i] > franka_o80::robot_positions_max[i] || positions_[i] + 0.001 * velocities.dq[i] < franka_o80::robot_positions_min[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint position"); }
        }

        //Updating state
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] += velocities.dq[i] * 0.001;
        }

        //Exiting
        if (velocities.motion_finished) break;
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for signal
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }

        //Calling callback
        RobotState state = state_();
        Torques torques = torques_control(state, 1);

        //Checcking input
        for (size_t i = 0; i < 7; i++)
        {
            if (torques.tau_J[i] != torques.tau_J[i] || abs(torques.tau_J[i]) > franka_o80::robot_torques_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint torque"); }
        }

        //Updating state
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
        }

        //Exiting
        if (torques.motion_finished) break;
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointPositions(const RobotState &state, Duration time)> positions_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for signal
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }

        //Calling callback
        RobotState state = state_();
        Torques torques = torques_control(state, 1);
        JointPositions positions = positions_control(state, 1);

        //Checcking input
        for (size_t i = 0; i < 7; i++)
        {
            if (positions.q[i] != positions.q[i] || positions.q[i] < franka_o80::robot_positions_min[i] || positions.q[i] > franka_o80::robot_positions_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint position"); }
            if (abs(positions_[i] - positions.q[i]) > 0.001 * franka_o80::robot_velocities_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint velocity"); }
            if (torques.tau_J[i] != torques.tau_J[i] || abs(torques.tau_J[i]) > franka_o80::robot_torques_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint torque"); }
        }

        //Updating state
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] = positions.q[i];
        }

        //Exiting
        if (torques.motion_finished || positions.motion_finished) break;
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for signal
        if (pause() < 0) { delete_timer_(); throw franka::RealtimeException("franka_test: pause failed"); }

        //Calling callback
        RobotState state = state_();
        Torques torques = torques_control(state, 1);
        JointVelocities velocities = velocities_control(state, 1);

        //Checcking input
        for (size_t i = 0; i < 7; i++)
        {
            if (velocities.dq[i] != velocities.dq[i] || abs(velocities.dq[i]) > franka_o80::robot_velocities_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint velocity"); }
            if (positions_[i] + 0.001 * velocities.dq[i] > franka_o80::robot_positions_max[i] || positions_[i] + 0.001 * velocities.dq[i] < franka_o80::robot_positions_min[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint position"); }
            if (torques.tau_J[i] != torques.tau_J[i] || abs(torques.tau_J[i]) > franka_o80::robot_torques_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint torque"); }
        }

        //Updating state
        for (size_t i = 0; i < 7; i++)
        {
            previous_positions_[i] = positions_[i];
            positions_[i] += velocities.dq[i] * 0.001;
        }

        //Exiting
        if (torques.motion_finished || velocities.motion_finished) break;
    }
    delete_timer_();
}

void franka::Robot::automaticErrorRecovery()
{
}

franka::Gripper::Gripper(std::string ip) : time_distribution_(500, 2000)
{
    random_engine_.seed((unsigned int)time(nullptr));
}

franka::GripperState franka::Gripper::readOnce()
{
    if (reading_.exchange(true) == true) franka::InvalidOperationException("franka_test: control already running");
    std::this_thread::sleep_for(std::chrono::microseconds(time_distribution_(random_engine_)));
    franka::GripperState state;
    state.width = width_;
    state.temperature = 20.0;
    reading_ = false;
    return state;
}

void franka::Gripper::move(double width, double speed)
{
    std::this_thread::sleep_for(std::chrono::microseconds(time_distribution_(random_engine_)));
    width_ = width;
}