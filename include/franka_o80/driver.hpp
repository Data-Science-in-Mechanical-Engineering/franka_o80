#pragma once

#include "constants.hpp"
#include "indexes.hpp"
#include "errors.hpp"
#include "driver_input.hpp"
#include "driver_output.hpp"
#ifndef FRANKA_O80_TEST
    #include <franka/robot.h>
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

namespace franka_o80
{
///Driver, controls arm and hand. Mostly maintained by `Standalone`
class Driver : public o80::Driver<DriverInput, DriverOutput>
{
private:
	enum class Mode
	{
		invalid,
		positions,
		velocities,
		torques,
		positions_torques,
		velocities_torques
	};
	
    std::string ip_;
    std::unique_ptr<franka::Robot> robot_;
    std::thread robot_control_thread_;
    std::unique_ptr<franka::Gripper> gripper_;
    std::thread gripper_control_thread_;
	Mode mode_ = Mode::invalid;
	DriverInput input_;
    DriverOutput output_;
    bool finished_ = false;
	std::mutex input_output_mutex_;

	static Mode get_mode(double positions, double velocities, double torques);
	static double get_control_position(Mode mode);
	static double get_control_velocity(Mode mode);
	static double get_control_torque(Mode mode);
    static void robot_control_function_(Driver *driver);
    static void gripper_control_function_(Driver *driver);

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