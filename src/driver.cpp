#include "../include/franka_o80/driver.hpp"

franka_o80::Driver::Mode franka_o80::Driver::get_mode(double positions, double velocities, double torques)
{
	if (positions == 1.0 && velocities == 0.0 && torques == 0.0) return Mode::positions;
	else if (positions == 0.0 && velocities == 1.0 && torques == 0.0) return Mode::velocities;
	else if (positions == 0.0 && velocities == 0.0 && torques == 1.0) return Mode::torques;
	else if (positions == 1.0 && velocities == 0.0 && torques == 1.0) return Mode::positions_torques;
	else if (positions == 0.0 && velocities == 1.0 && torques == 1.0) return Mode::velocities_torques;
	else return Mode::invalid;
}

double franka_o80::Driver::get_control_position(Mode mode)
{
    if (mode == Mode::positions || mode == Mode::positions_torques) return 1.0;
    else return 0.0;
}

double franka_o80::Driver::get_control_velocity(Mode mode)
{
    if (mode == Mode::velocities || mode == Mode::velocities_torques) return 1.0;
    else return 0.0;
}

double franka_o80::Driver::get_control_torque(Mode mode)
{
    if (mode == Mode::torques || mode == Mode::positions_torques || mode == Mode::velocities_torques) return 1.0;
    else return 0.0;
}

void franka_o80::Driver::robot_control_function_(Driver *driver)
{
	//Zero velocity control function
	auto dummy_control = [driver](const franka::RobotState &state, franka::Duration time) -> franka::JointVelocities
	{
		bool input_reset;
		Mode input_mode;
		bool input_finished;
        double input_error;
        franka::JointVelocities input_velocities{};
		{
			//Locking
			std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
			
			//Reading input
			input_reset = driver->input_.get(control_reset) > 0.0;
            input_mode = get_mode(driver->input_.get(control_positions), driver->input_.get(control_velocities), driver->input_.get(control_torques));
			input_finished = driver->finished_;
            input_error = driver->input_.get(control_error);

			//Writing output
			for (size_t i = 0; i < 7; i++)
			{
				driver->output_.set(robot_positions[i], state.q[i]);
				driver->output_.set(robot_velocities[i], state.dq[i]);
				driver->output_.set(robot_torques[i], state.tau_J[i]);
			}
			driver->output_.set(control_positions, get_control_position(driver->mode_));
			driver->output_.set(control_velocities, get_control_velocity(driver->mode_));
			driver->output_.set(control_torques, get_control_torque(driver->mode_));
            driver->output_.set(control_reset, input_reset ? 1.0 : 0.0);
			if (input_reset) driver->output_.set(control_error, error_ok);
            
            //Switching mode
            if (input_mode != Mode::invalid) driver->mode_ = input_mode;
		}
		
		//If everything is ok or requested to finish
        if (input_finished || (input_error == error_ok && !input_reset && input_mode != Mode::invalid)) input_velocities.motion_finished = true;
		
		for (size_t i = 0; i < 7; i++) input_velocities.dq[i] = 0.0;
		return input_velocities;
	};
    
	//Position control function
	auto positions_control = [driver](const franka::RobotState &state, franka::Duration time) -> franka::JointPositions
	{
		bool input_reset;
		Mode input_mode;
        bool input_mode_switch;
		bool input_finished;
        double input_error;
        franka::JointPositions input_positions{};
		{
			//Locking
			std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
			
			//Reading input
            for (size_t i = 0; i < 7; i++) input_positions.q[i] = driver->input_.get(robot_positions[i]);
			input_reset = driver->input_.get(control_reset) > 0.0;
            input_mode = get_mode(driver->input_.get(control_positions), driver->input_.get(control_velocities), driver->input_.get(control_torques));
			input_finished = driver->finished_;
            input_error = driver->input_.get(control_error);

			//Writing output
			for (size_t i = 0; i < 7; i++)
			{
				driver->output_.set(robot_positions[i], state.q[i]);
				driver->output_.set(robot_velocities[i], state.dq[i]);
				driver->output_.set(robot_torques[i], state.tau_J[i]);
			}
			driver->output_.set(control_positions, get_control_position(driver->mode_));
			driver->output_.set(control_velocities, get_control_velocity(driver->mode_));
			driver->output_.set(control_torques, get_control_torque(driver->mode_));
            driver->output_.set(control_reset, input_reset ? 1.0 : 0.0);
			if (input_reset) driver->output_.set(control_error, error_ok);
            else if (input_error == error_ok && input_mode == Mode::invalid) driver->output_.set(control_error, error_robot_invalid_control);
            
            //Switching mode
            if (driver->mode_ != input_mode)
            {
                input_mode_switch = true;
                driver->mode_ = input_mode;
            }
            else input_mode_switch = false;
		}
		
		//If something is not ok or requested to finish
        if (input_finished || input_error != error_ok || input_reset || input_mode_switch) input_positions.motion_finished = true;
		
        for (size_t i = 0; i < 7; i++) { if (input_positions.q[i] != input_positions.q[i]) input_positions.q[i] = state.q[i]; }
		return input_positions;
	};
	
	//Velocity control function
	auto velocities_control = [driver](const franka::RobotState &state, franka::Duration time) -> franka::JointVelocities
	{
		bool input_reset;
		Mode input_mode;
        bool input_mode_switch;
		bool input_finished;
        double input_error;
        franka::JointVelocities input_velocities{};
		{
			//Locking
			std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
			
			//Reading input
            for (size_t i = 0; i < 7; i++) input_velocities.dq[i] = driver->input_.get(robot_velocities[i]);
			input_reset = driver->input_.get(control_reset) > 0.0;
            input_mode = get_mode(driver->input_.get(control_positions), driver->input_.get(control_velocities), driver->input_.get(control_torques));
			input_finished = driver->finished_;
            input_error = driver->input_.get(control_error);

			//Writing output
			for (size_t i = 0; i < 7; i++)
			{
				driver->output_.set(robot_positions[i], state.q[i]);
				driver->output_.set(robot_velocities[i], state.dq[i]);
				driver->output_.set(robot_torques[i], state.tau_J[i]);
			}
			driver->output_.set(control_positions, get_control_position(driver->mode_));
			driver->output_.set(control_velocities, get_control_velocity(driver->mode_));
			driver->output_.set(control_torques, get_control_torque(driver->mode_));
            driver->output_.set(control_reset, input_reset ? 1.0 : 0.0);
			if (input_reset) driver->output_.set(control_error, error_ok);
            else if (input_error == error_ok && input_mode == Mode::invalid) driver->output_.set(control_error, error_robot_invalid_control);
            
            //Switching mode
            if (driver->mode_ != input_mode)
            {
                input_mode_switch = true;
                driver->mode_ = input_mode;
            }
            else input_mode_switch = false;
		}
		
		//If something is not ok or requested to finish
        if (input_finished || input_error != error_ok || input_reset || input_mode_switch) input_velocities.motion_finished = true;
		
        for (size_t i = 0; i < 7; i++) { if (input_velocities.dq[i] != input_velocities.dq[i]) input_velocities.dq[i] = 0.0; }
		return input_velocities;
	};
	
	//Torque control function
	auto torques_control = [driver](const franka::RobotState &state, franka::Duration time) -> franka::Torques
	{
		bool input_reset;
		Mode input_mode;
        bool input_mode_switch;
		bool input_finished;
        double input_error;
        franka::Torques input_torques{};
		{
			//Locking
			std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
			
			//Reading input
            for (size_t i = 0; i < 7; i++) input_torques.tau_J[i] = driver->input_.get(robot_torques[i]);
			input_reset = driver->input_.get(control_reset) > 0.0;
            input_mode = get_mode(driver->input_.get(control_positions), driver->input_.get(control_velocities), driver->input_.get(control_torques));
			input_finished = driver->finished_;
            input_error = driver->input_.get(control_error);

			//Writing output
			for (size_t i = 0; i < 7; i++)
			{
				driver->output_.set(robot_positions[i], state.q[i]);
				driver->output_.set(robot_velocities[i], state.dq[i]);
				driver->output_.set(robot_torques[i], state.tau_J[i]);
			}
			driver->output_.set(control_positions, get_control_position(driver->mode_));
			driver->output_.set(control_velocities, get_control_velocity(driver->mode_));
			driver->output_.set(control_torques, get_control_torque(driver->mode_));
            driver->output_.set(control_reset, input_reset ? 1.0 : 0.0);
			if (input_reset) driver->output_.set(control_error, error_ok);
            else if (input_error == error_ok && input_mode == Mode::invalid) driver->output_.set(control_error, error_robot_invalid_control);
            
            //Switching mode
            if (driver->mode_ != input_mode)
            {
                input_mode_switch = true;
                driver->mode_ = input_mode;
            }
            else input_mode_switch = false;
		}
		
		//If something is not ok or requested to finish
        if (input_finished || input_error != error_ok || input_reset || input_mode_switch) input_torques.motion_finished = true;
		
        for (size_t i = 0; i < 7; i++) { if (input_torques.tau_J[i] != input_torques.tau_J[i]) input_torques.tau_J[i] = state.tau_J[i]; }
		return input_torques;
	};
	
    double output_error = error_ok;
	while (true)
	{
        bool input_reset;
        Mode input_mode;
		bool input_finished;
        double input_error;
		{
			//Locking
			std::lock_guard<std::mutex> guard(driver->input_output_mutex_);
			
            //Reading input
			input_reset = driver->input_.get(control_reset).get() > 0.0;
			input_mode = get_mode(driver->input_.get(control_positions), driver->input_.get(control_velocities), driver->input_.get(control_torques));
			input_finished = driver->finished_;
            input_error = driver->output_.get(control_error);
            
            //Writing output
            if (input_reset) driver->output_.set(control_error, error_ok);
            else if (input_error == error_ok && input_mode == Mode::invalid) driver->output_.set(control_error, error_robot_invalid_control);
            else if (input_error == error_ok && output_error != error_ok) driver->output_.set(control_error, output_error);
		}

        //Entering control loop
        try
        {
            if (input_finished) return;
            else if (input_reset) driver->robot_->automaticErrorRecovery();
            else if (input_reset || input_error != error_ok || input_mode == Mode::invalid) driver->robot_->control(dummy_control);
            else if (input_mode == Mode::velocities) driver->robot_->control(velocities_control);
            else if (input_mode == Mode::torques) driver->robot_->control(torques_control);
            else if (input_mode == Mode::positions) driver->robot_->control(torques_control, positions_control);
            else if (input_mode == Mode::positions) driver->robot_->control(torques_control, velocities_control);
            else driver->robot_->control(positions_control);
            output_error = error_ok;
		}
        catch (franka::CommandException &e)
        {
            output_error = error_robot_command_exception;
        }
		catch (franka::ControlException &e)
		{
			output_error = error_robot_control_exception;
		}
		catch (franka::InvalidOperationException &e)
        {
            output_error = error_robot_invalid_operation_exception;
        }
        catch (franka::NetworkException &e)
        {
            output_error = error_robot_network_exception;
        }
        catch (franka::RealtimeException &e)
        {
            output_error = error_robot_realtime_exception;
        }
        catch (std::invalid_argument &e)
        {
            output_error = error_robot_invalid_argument_exception;
        }
        catch (...)
        {
            output_error = error_robot_other_exception;
        }

        //Waiting if failed
        if (output_error != error_ok) std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void franka_o80::Driver::gripper_control_function_(Driver *driver)
{
    double output_error = error_ok;
    while (true)
    {
        //Measuring state
        double width;
        double temperature;
        try
        {
		    franka::GripperState state = driver->gripper_->readOnce(); 
            width = state.width;
            temperature = state.temperature;
            output_error = error_ok;
        }
        catch (franka::NetworkException &e)
        {
            output_error = error_gripper_network_exception;
        }
        catch (franka::InvalidOperationException)
        {
            output_error = error_gripper_invalid_operation_exception;
        }
        catch (...)
        {
            output_error = error_gripper_invalid_operation_exception;
        }

        bool input_reset;
        Mode input_mode;
        bool input_finished;
        double input_error;
        double input_width;
        {
            //Locking
            std::lock_guard<std::mutex> guard(driver->input_output_mutex_);

            //Reading input
            input_reset = driver->input_.get(control_reset) > 0.0;
            input_mode = get_mode(driver->input_.get(control_positions), driver->input_.get(control_velocities), driver->input_.get(control_torques));
            input_finished = driver->finished_;
            input_error = driver->output_.get(control_error);
            input_width = driver->input_.get(gripper_width);

            //Writing output
            driver->output_.set(gripper_width, width);
            driver->output_.set(gripper_temperature, temperature);
            driver->output_.set(control_positions, get_control_position(driver->mode_));
			driver->output_.set(control_velocities, get_control_velocity(driver->mode_));
			driver->output_.set(control_torques, get_control_torque(driver->mode_));
            driver->output_.set(control_reset, input_reset ? 1.0 : 0.0);
			if (input_reset) driver->output_.set(control_error, error_ok);
            else if (input_error == error_ok && input_mode == Mode::invalid) driver->output_.set(control_error, error_robot_invalid_control);
            else if (input_error == error_ok && output_error != error_ok) driver->output_.set(control_error, output_error);
        }
		
        //Acting
        if (input_finished) return;
        else if (!input_reset && input_error == error_ok && input_mode != Mode::invalid) try
        {
		    if (input_width == input_width) driver->gripper_->move(input_width, 0.01); //Fix magic number!
            output_error = error_ok;
        }
        catch (franka::NetworkException &e)
        {
            output_error = error_gripper_network_exception;
        }
        catch (franka::InvalidOperationException)
        {
            output_error = error_gripper_invalid_operation_exception;
        }
        catch (...)
        {
            output_error = error_gripper_invalid_operation_exception;
        }
    }
}

franka_o80::Driver::Driver(std::string ip) : ip_(ip)
{
}

void franka_o80::Driver::start()
{
    robot_ = std::unique_ptr<franka::Robot>(new franka::Robot(ip_));
    franka::RobotState robot_state = robot_->readOnce();
    for (size_t i = 0; i < 7; i++)
    {
        output_.set(robot_positions[i], robot_state.q[i]);
        output_.set(robot_velocities[i], robot_state.dq[i]);
        output_.set(robot_torques[i], robot_state.tau_J[i]);
    }
    robot_control_thread_ = std::thread(robot_control_function_, this);

    gripper_ = std::unique_ptr<franka::Gripper>(new franka::Gripper(ip_));
	franka::GripperState gripper_state = gripper_->readOnce();
    output_.set(gripper_width, gripper_state.width);
    output_.set(gripper_temperature, gripper_state.temperature);
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
        finished_ = true;
    }
    robot_control_thread_.join();
    gripper_control_thread_.join();
}