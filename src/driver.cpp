#include "../include/franka_o80/driver.hpp"

void franka_o80::Driver::robot_write_output_(const franka::RobotState &robot_state)
{
    //Joints
    for (size_t i = 0; i < 7; i++)
    {
        output_.set(joint_position[i], robot_state.q[i]);
        output_.set(joint_velocity[i], robot_state.dq[i]);
        output_.set(joint_torque[i], robot_state.tau_J[i]);
    }

    //Cartesian
    Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position = transform.translation();
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Map(model_->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    Eigen::Matrix<double, 6, 1> velocity = jacobian * joint_velocities;
    for (size_t i = 0; i < 3; i++)
    {
        output_.set(cartesian_position[i], position(i));
        output_.set(cartesian_orientation, orientation);
        output_.set(cartesian_velocity[i], velocity(i));
        output_.set(cartesian_rotation[i], velocity(i + 3));
    }
}

void franka_o80::Driver::robot_dummy_control_function_(const franka::RobotState &robot_state, franka::JointVelocities *velocities)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);

    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    Mode input_mode = input_.get(control_mode).get_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();

    //Exit
    if (input_finished || (input_error == Error::ok && !input_reset && input_mode != Mode::invalid))
    {
        output_.set(control_mode, Mode::invalid);
        mode_ = input_mode;
        velocities->motion_finished = true;
    }
    //Normal
    else
    {
        output_.set(control_mode, Mode::invalid);
        mode_ = Mode::invalid;
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_torque_control_function_(const franka::RobotState &robot_state, franka::Torques *torques)
{
    std::lock_guard<std::mutex> guard(input_output_mutex_);

    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    Mode input_mode = input_.get(control_mode).get_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    
    bool switching_mode = input_mode != mode_ && !
    ((mode_ == Mode::torque || mode_ == Mode::intelligent_position || mode_ == Mode::intelligent_cartesian_position) &&
    (input_mode == Mode::torque || input_mode == Mode::intelligent_position || input_mode == Mode::intelligent_cartesian_position));

    //Exit
    if (switching_mode || input_finished || input_error != Error::ok || input_reset)
    {
        mode_ = input_mode;
        output_.set(control_mode, Mode::invalid);

        //Calculating damping torque
        Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(model_->coriolis(robot_state).data());
        Eigen::Matrix<double, 7, 1> joint_torques = -joint_damping_ * joint_velocities + coriolis;
        Eigen::Matrix<double, 7, 1>::Map(&torques->tau_J[0]) = joint_torques;
        torques->motion_finished = true;
    }
    //Intelligent position
    else if (input_mode == Mode::intelligent_position)
    {
        //std::cout << "J";

        mode_ = Mode::intelligent_position;
        output_.set(control_mode, Mode::intelligent_position);

        //Reading input
        Eigen::Matrix<double, 7, 1> input_target_joint_positions;
        for (size_t i = 0; i < 7; i++) input_target_joint_positions(i) = input_.get(joint_position[i]).get_real();

        //Calculating torque
        Eigen::Matrix<double, 7, 1> joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
        Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(model_->coriolis(robot_state).data());
        Eigen::Matrix<double, 7, 1> joint_torques = -joint_stiffness_ * (joint_positions - input_target_joint_positions) - joint_damping_ * joint_velocities + coriolis;
        Eigen::Matrix<double, 7, 1>::Map(&torques->tau_J[0]) = joint_torques;
    }
    //Intelligent cartesian position
    else if (input_mode == Mode::intelligent_cartesian_position)
    {
        //std::cout << "C";

        mode_ = Mode::intelligent_cartesian_position;
        output_.set(control_mode, Mode::intelligent_cartesian_position);

        //Reading input
        Eigen::Matrix<double, 3, 1> input_target_position;
        for (size_t i = 0; i < 3; i++) input_target_position(i) = input_.get(cartesian_position[i]).get_real();
        Eigen::Quaterniond input_target_orientation = input_.get(cartesian_orientation).get_quaternion();

        //Calculating torque
        Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position = transform.translation();
        Eigen::Quaterniond orientation; orientation = transform.linear();
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(model_->coriolis(robot_state).data());
        Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Map(model_->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - input_target_position;
        if (input_target_orientation.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
        Eigen::Quaterniond orientation_error = orientation.inverse() * input_target_orientation;
        error.tail(3) << orientation_error.x(), orientation_error.y(), orientation_error.z();
        error.tail(3) << -transform.linear() * error.tail(3);
        Eigen::Matrix<double, 7, 1> joint_torques = jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * joint_velocities)) + coriolis;
        Eigen::Matrix<double, 7, 1>::Map(&torques->tau_J[0]) = joint_torques;
    }
    //Simple torque
    else
    {
        mode_ = input_mode;
        output_.set(control_mode, input_mode);

        //Reading input and calculating torques
        for (size_t i = 0; i < 7; i++) torques->tau_J[i] = input_.get(joint_torque[i]).get_real();
    }
    
    //Writing output
    robot_write_output_(robot_state);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_position_control_function_(const franka::RobotState &robot_state, franka::JointPositions *positions)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
    
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    Mode input_mode = input_.get(control_mode).get_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();

    //Exit
    if (input_mode != mode_ || input_finished || input_error != Error::ok || input_reset)
    {
        mode_ = input_mode;
        output_.set(control_mode, Mode::invalid);
        for (size_t i = 0; i < 7; i++) positions->q[i] = robot_state.q[i];
        positions->motion_finished = true;
    }
    //Normal
    else
    {
        mode_ = input_mode;
        output_.set(control_mode, mode_);
        for (size_t i = 0; i < 7; i++) positions->q[i] = input_.get(joint_position[i]).get_real();
    }

   //Writing output
    robot_write_output_(robot_state);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_velocity_control_function_(const franka::RobotState &robot_state, franka::JointVelocities *velocities)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
    
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    Mode input_mode = input_.get(control_mode).get_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();

    //Exit
    if (input_mode != mode_ || input_finished || input_error != Error::ok || input_reset)
    {
        mode_ = input_mode;
        output_.set(control_mode, Mode::invalid);
        for (size_t i = 0; i < 7; i++) velocities->dq[i] = 0.0;
        velocities->motion_finished = true;
    }
    //Normal
    else
    {
        mode_ = input_mode;
        output_.set(control_mode, mode_);
        for (size_t i = 0; i < 7; i++) velocities->dq[i] = input_.get(joint_velocity[i]).get_real();
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_cartesian_position_control_function_(const franka::RobotState &robot_state, franka::CartesianPose *positions)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
    
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    Mode input_mode = input_.get(control_mode).get_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();

    //Exit
    if (input_mode != mode_ || input_finished || input_error != Error::ok || input_reset)
    {
        mode_ = input_mode;
        output_.set(control_mode, Mode::invalid);
        for (size_t i = 0; i < 16; i++) positions->O_T_EE[i] = robot_state.O_T_EE[i];
        for (size_t i = 0; i < 2; i++) positions->elbow[i] = robot_state.elbow[i];
        positions->motion_finished = true;
    }
    //Normal
    else
    {
        mode_ = input_mode;
        output_.set(control_mode, mode_);
        Eigen::Affine3d transform;
        transform.translation() = Eigen::Vector3d(input_.get(cartesian_position[0]).get_real(), input_.get(cartesian_position[1]).get_real(), input_.get(cartesian_position[2]).get_real());
        transform.linear() = input_.get(cartesian_orientation).get_quaternion().toRotationMatrix();
        Eigen::Matrix<double, 4, 4>::Map(&positions->O_T_EE[0]) = transform.matrix();
        positions->elbow[0] = input_.get(joint_position[2]).get_real();
        positions->elbow[1] = input_.get(joint_position[3]).get_real() > 0.0 ? 1.0 : -1.0;
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_cartesian_velocity_control_function_(const franka::RobotState &robot_state, franka::CartesianVelocities *velocities)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
        
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    Mode input_mode = input_.get(control_mode).get_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();

    //Exit
    if (input_mode != mode_ || input_finished || input_error != Error::ok || input_reset)
    {
        mode_ = input_mode;
        output_.set(control_mode, Mode::invalid);
        for (size_t i = 0; i < 6; i++) velocities->O_dP_EE[i] = 0.0;
        velocities->motion_finished = true;
    }
    //Normal
    else
    {
        mode_ = input_mode;
        output_.set(control_mode, mode_);
        for (size_t i = 0; i < 3; i++)
        {
            velocities->O_dP_EE[i] = input_.get(cartesian_velocity[i]).get_real();
            velocities->O_dP_EE[i + 3] = input_.get(cartesian_rotation[i]).get_real();
        }
        velocities->elbow[0] = input_.get(joint_position[2]).get_real();
        velocities->elbow[1] = input_.get(joint_position[3]).get_real() > 0.0 ? 1.0 : -1.0;
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_control_function_()
{
    Error output_error = Error::ok;
	while (true)
	{
        bool input_reset;
        Mode input_mode;
		bool input_finished;
        Error input_error;
		{
			//Locking
			std::lock_guard<std::mutex> guard(input_output_mutex_);
			
            //Reading input
			input_reset = input_.get(control_reset).get_real() > 0.0;
			input_mode = input_.get(control_mode).get_mode();
			input_finished = input_finished_;
            input_error = output_.get(control_error).get_error();
            
            //Writing output
            if (input_reset) output_.set(control_error, Error::ok);
            else if (input_error == Error::ok && output_error != Error::ok) output_.set(control_error, output_error);
		}

        //Entering control loop
        try
        {
            if (input_finished) return;
            else if (input_reset) robot_->automaticErrorRecovery();

            Driver *driver = this;
            if (input_reset || input_error != Error::ok || output_error != Error::ok || input_mode == Mode::invalid) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointVelocities
                {
                    franka::JointVelocities velocities(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_dummy_control_function_(robot_state, &velocities);
                    return velocities;
                });
            else if (input_mode == Mode::torque || input_mode == Mode::intelligent_position || input_mode == Mode::intelligent_cartesian_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                });
            else if (input_mode == Mode::torque_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointPositions
                {
                    franka::JointPositions positions(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_position_control_function_(robot_state, &positions);
                    return positions;
                });
            else if (input_mode == Mode::torque_velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointVelocities
                {
                    franka::JointVelocities velocities(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_velocity_control_function_(robot_state, &velocities);
                    return velocities;
                });
            else if (input_mode == Mode::torque_cartesian_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianPose
                {
                    franka::CartesianPose cartesian_position(std::array<double, 16>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_position_control_function_(robot_state, &cartesian_position);
                    return cartesian_position;
                });
            else if (input_mode == Mode::torque_cartesian_velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianVelocities
                {
                    franka::CartesianVelocities cartesian_velocity(std::array<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_velocity_control_function_(robot_state, &cartesian_velocity);
                    return cartesian_velocity;
                });
            else if (input_mode == Mode::position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointPositions
                {
                    franka::JointPositions positions(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_position_control_function_(robot_state, &positions);
                    return positions;
                });
            else if (input_mode == Mode::velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointVelocities
                {
                    franka::JointVelocities velocities(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_velocity_control_function_(robot_state, &velocities);
                    return velocities;
                });
            else if (input_mode == Mode::cartesian_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianPose
                {
                    franka::CartesianPose cartesian_position(std::array<double, 16>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_position_control_function_(robot_state, &cartesian_position);
                    return cartesian_position;
                });
            else if (input_mode == Mode::cartesian_velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianVelocities
                {
                    franka::CartesianVelocities cartesian_velocity(std::array<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_velocity_control_function_(robot_state, &cartesian_velocity);
                    return cartesian_velocity;
                });

            output_error = Error::ok;
		}
        catch (franka::CommandException &e)
        {
            output_error = Error::robot_command_exception;
            std::cout << e.what() << std::endl;
        }
		catch (franka::ControlException &e)
		{
			output_error = Error::robot_control_exception;
            std::cout << e.what() << std::endl;
		}
		catch (franka::InvalidOperationException &e)
        {
            output_error = Error::robot_invalid_operation_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::NetworkException &e)
        {
            output_error = Error::robot_network_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::RealtimeException &e)
        {
            output_error = Error::robot_realtime_exception;
            std::cout << e.what() << std::endl;
        }
        catch (std::invalid_argument &e)
        {
            output_error = Error::robot_invalid_argument_exception;
            std::cout << e.what() << std::endl;
        }
        catch (...)
        {
            output_error = Error::robot_other_exception;
        }

        //Waiting if failed
        if (output_error != Error::ok) std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void franka_o80::Driver::gripper_control_function_()
{
    Error output_error = Error::ok;
    while (true)
    {
        //Measuring state
        double width;
        double temperature;
        try
        {
		    franka::GripperState state = gripper_->readOnce(); 
            width = state.width;
            temperature = state.temperature;
            output_error = Error::ok;
        }
        catch (franka::NetworkException &e)
        {
            output_error = Error::gripper_network_exception;
        }
        catch (franka::InvalidOperationException)
        {
            output_error = Error::gripper_invalid_operation_exception;
        }
        catch (...)
        {
            output_error = Error::gripper_invalid_operation_exception;
        }

        bool input_reset;
        Mode input_mode;
        bool input_finished;
        Error input_error;
        double input_width;
        {
            //Locking
            std::lock_guard<std::mutex> guard(input_output_mutex_);

            //Reading input
            input_reset = input_.get(control_reset).get_real() > 0.0;
            input_mode = input_.get(control_mode).get_mode();
            input_finished = input_finished_;
            input_error = output_.get(control_error).get_error();
            input_width = input_.get(gripper_width).get_real();

            //Writing output
            output_.set(gripper_width, width);
            output_.set(gripper_temperature, temperature);
            output_.set(control_mode, mode_);
			output_.set(control_reset, input_reset ? 1.0 : 0.0);
			if (input_reset) output_.set(control_error, Error::ok);
            else if (input_error == Error::ok && output_error != Error::ok) output_.set(control_error, output_error);
        }
		
        //Acting
        if (input_finished) return;
        else if (!input_reset && input_error == Error::ok && input_mode != Mode::invalid) try
        {
		    if (input_width == input_width) gripper_->move(input_width, 0.1); //Fix magic number
            output_error = Error::ok;
        }
        catch (franka::NetworkException &e)
        {
            output_error = Error::gripper_network_exception;
        }
        catch (franka::InvalidOperationException)
        {
            output_error = Error::gripper_invalid_operation_exception;
        }
        catch (...)
        {
            output_error = Error::gripper_invalid_operation_exception;
        }
    }
}

franka_o80::Driver::Driver(std::string ip) : ip_(ip)
{
}

void franka_o80::Driver::start()
{
    if (started_) return;
    started_ = true;

    //Initialize constants
    const double translational_stiffness = 150.0;
    const double rotational_stiffness = 10.0;
    cartesian_stiffness_.setZero();
    cartesian_stiffness_.topLeftCorner(3, 3) << translational_stiffness * Eigen::Matrix<double, 3, 3>::Identity();
    cartesian_stiffness_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::Matrix<double, 3, 3>::Identity();
    cartesian_damping_.setZero();
    cartesian_damping_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix<double, 3, 3>::Identity();
    cartesian_damping_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix<double, 3, 3>::Identity();
    joint_stiffness_.setZero();
    joint_stiffness_.diagonal() << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    joint_damping_.setZero();
    joint_damping_.diagonal() << 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0;

    //Starting robot
    robot_ = std::unique_ptr<franka::Robot>(new franka::Robot(ip_));
    model_ = std::unique_ptr<franka::Model>(new franka::Model(robot_->loadModel()));
    robot_->automaticErrorRecovery();
    robot_->setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    franka::RobotState robot_state = robot_->readOnce();
    robot_write_output_(robot_state);
    robot_control_thread_ = std::thread([](Driver *driver) -> void { driver->robot_control_function_(); }, this);

    //Starting gripper
    gripper_ = std::unique_ptr<franka::Gripper>(new franka::Gripper(ip_));
	franka::GripperState gripper_state = gripper_->readOnce();
    output_.set(gripper_width, gripper_state.width);
    output_.set(gripper_temperature, gripper_state.temperature);
    gripper_control_thread_ = std::thread([](Driver *driver) -> void { driver->gripper_control_function_(); }, this);
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
    if (!started_) return;
    started_ = false;

    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        input_finished_ = true;
    }
    if (robot_control_thread_.joinable()) robot_control_thread_.join();
    if (gripper_control_thread_.joinable()) gripper_control_thread_.join();
}