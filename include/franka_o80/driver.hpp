#pragma once

#include "driver_in.hpp"
#include "driver_out.hpp"
#include <franka/robot.h>
#include <franka/gripper.h>
#include <o80/driver.hpp>
#include <string>
#include <memory>
#include <thread>

namespace franka_o80
{
class Driver : public o80::Driver<DriverIn, DriverOut>
{
private:
	std::string ip_;
	std::unique_ptr<franka::Robot> robot_;
	std::unique_ptr<franka::Gripper> gripper_;
	std::thread thread_;
	DriverIn in_;
	DriverOut out_;

public:
    Driver(std::string ip);
    void start();
    void stop();
    void set(const DriverIn& in);
    DriverOut get();
};
}  // namespace franka_o80