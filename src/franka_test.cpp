#include "../include/franka_o80/franka_test.hpp"

franka::Duration::Duration(unsigned int milliseconds)
{
}

franka::JointPositions::JointPositions(const std::array<double, 7> &positions)
{
    for (size_t i = 0; i < 7; i++) q[i] = positions[i];
}

franka::JointVelocities::JointVelocities(const std::array<double, 7> &velocities)
{
    for (size_t i = 0; i < 7; i++) dq[i] = velocities[i];
}

franka::Torques::Torques(const std::array<double, 7> &torques)
{
    for (size_t i = 0; i < 7; i++) tau_J[i] = torques[i];
}


void franka::Robot::create_timer_()
{
    if (control_.exchange(true) == true) throw franka::InvalidOperationException("franka_test: control already running");
}

void franka::Robot::wait_timer_()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void franka::Robot::delete_timer_()
{
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
    {
        franka_o80::States dstates  = franka_o80::default_states();
        for (size_t i = 0; i < 7; i++) previous_positions_[i] = positions_[i] = dstates.values[franka_o80::robot_positions[i]];
    }
    
    char dummy[32] = "franka_o80_test";
    char *pdummy = &dummy[0];
    char **ppdummy = &pdummy;
    int cdummy = 1;
    ros::init(cdummy, ppdummy, "franka_o80_test");
    
    previous_sent_positions_.resize(7);
    for (size_t i = 0; i < 7; i++) previous_sent_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    node_handle_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    spinner_ = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner_->start();
    move_group_interface_ = std::unique_ptr<moveit::planning_interface::MoveGroupInterface>(new moveit::planning_interface::MoveGroupInterface(planning_group));
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setMaxAccelerationScalingFactor(1.0);
    
    Robot *robot = this;
    send_thread_ = std::thread([robot]
    {
        while (true)
        {
            try
            {
                bool update = false;
                {
                    std::lock_guard guard(robot->send_mutex_);
                    for (size_t i = 0; i < 7; i++)
                    {
                        if (robot->previous_sent_positions_[i] != robot->positions_[i]) update = true;
                        robot->previous_sent_positions_[i] = robot->positions_[i];
                    }
                }
                if (update)
                {
                    robot->move_group_interface_->setJointValueTarget(robot->previous_sent_positions_);
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    robot->move_group_interface_->plan(plan);
                    robot->move_group_interface_->execute(plan);
                }
                else std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            catch(...)
            {
            }
        }
    });
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
        //Waiting for timer
        wait_timer_();

        //Calling callback
        RobotState state = state_();
        JointPositions positions = positions_control(state, 1);

        //Checcking input
        printf("Position control: ");
        for (size_t i = 0; i < 7; i++)
        {
            printf("%lf ", positions.q[i]);
            if (positions.q[i] != positions.q[i] || positions.q[i] < franka_o80::robot_positions_min[i] || positions.q[i] > franka_o80::robot_positions_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint position"); }
            if (abs(positions_[i] - positions.q[i]) > 0.001 * franka_o80::robot_velocities_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint velocity"); }
        }
        printf("\n");

        //Updating state
        {
            std::lock_guard guard(send_mutex_);
            for (size_t i = 0; i < 7; i++)
            {
                previous_positions_[i] = positions_[i];
                positions_[i] = positions.q[i];
            }
        }

        //Exiting
        if (positions.motion_finished) { delete_timer_(); return; }
    }
}

void franka::Robot::control(std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for timer
        wait_timer_();

        //Calling callback
        RobotState state = state_();
        JointVelocities velocities = velocities_control(state, 1);

        //Checcking input
        //printf("Velocity control: ");
        for (size_t i = 0; i < 7; i++)
        {
            //printf("%lf ", velocities.dq[i]);
            if (velocities.dq[i] != velocities.dq[i] || abs(velocities.dq[i]) > franka_o80::robot_velocities_max[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint velocity"); }
            if (positions_[i] + 0.001 * velocities.dq[i] > franka_o80::robot_positions_max[i] || positions_[i] + 0.001 * velocities.dq[i] < franka_o80::robot_positions_min[i])
                { delete_timer_(); throw franka::ControlException("franka_test: invalid joint position"); }
        }
        //printf("\n");

        //Updating state
        {
            std::lock_guard guard(send_mutex_);
            for (size_t i = 0; i < 7; i++)
            {
                previous_positions_[i] = positions_[i];
                positions_[i] += velocities.dq[i] * 0.001;
            }
        }

        //Exiting
        if (velocities.motion_finished) { delete_timer_(); break; }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for timer
        wait_timer_();

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
        {
            std::lock_guard guard(send_mutex_);
            for (size_t i = 0; i < 7; i++)
            {
                previous_positions_[i] = positions_[i];
            }
        }

        //Exiting
        if (torques.motion_finished) { delete_timer_(); break; }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointPositions(const RobotState &state, Duration time)> positions_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for timer
        wait_timer_();

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
        {
            std::lock_guard guard(send_mutex_);
            for (size_t i = 0; i < 7; i++)
            {
                previous_positions_[i] = positions_[i];
                positions_[i] = positions.q[i];
            }
        }

        //Exiting
        if (torques.motion_finished || positions.motion_finished) { delete_timer_(); break; }
    }
    delete_timer_();
}

void franka::Robot::control(std::function<Torques(const RobotState &state, Duration time)> torques_control, std::function<JointVelocities(const RobotState &state, Duration time)> velocities_control)
{
    create_timer_();
    while (true)
    {
        //Waiting for timer
        wait_timer_();

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
        {
            std::lock_guard guard(send_mutex_);
            for (size_t i = 0; i < 7; i++)
            {
                previous_positions_[i] = positions_[i];
                positions_[i] += velocities.dq[i] * 0.001;
            }
        }

        //Exiting
        if (torques.motion_finished || velocities.motion_finished) { delete_timer_(); break; }
    }
}

void franka::Robot::automaticErrorRecovery()
{
}

franka::Robot::~Robot()
{
    ros::shutdown();
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
