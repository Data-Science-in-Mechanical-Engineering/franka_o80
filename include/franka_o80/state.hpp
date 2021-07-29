#pragma once

#include "mode.hpp"
#include "error.hpp"
#include <stdexcept>
#include <string>
#include <limits>
#include <array>
#include <o80/command_types.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_o80
{
///Actuator state. Because of `o80` restrictions many other values, like control modes or errors, are also represented as actuators. Dynamic typazation is done here
class State
{
friend bool operator==(const State &a, const State &b);
friend bool operator!=(const State &a, const State &b);

public:
    ///Type of state
    enum class Type
    {
        real,
        quaternion,
        mode,
        error
    };

private:
    Type typ_;
    union
    {
        double real;
        std::array<double, 4> quaternion;
        Mode mode;
        Error error;
    } value_;

public:
    ///Creates state with zero real value
	State();
	///Creates state with given real value
	State(double value);
    ///Creates state with given quaternion value
    State(const Eigen::Quaterniond &value);
    ///Creates state with given quaternion value given as wxyz
    State(const Eigen::Matrix<double, 4, 1> &value);
    ///Creates state with given quaternion value given as Euler angles
    State(const Eigen::Matrix<double, 3, 1> &value);
    ///Creates state with given mode value
	State(Mode value);
    ///Creates state with given error value
	State(Error value);
	///Copies state
	State(const State &state);
	
    ///Sets state's value to real value
	void set_real(double value);
    ///Sets state's value to quaternion value
	void set_quaternion(const Eigen::Quaterniond &value);
    ///Sets state's value to quaternion value given as wxyz
	void set_wxyz(const Eigen::Matrix<double, 4, 1> &value);
    ///Sets state's value to quaternion value given as Euler angles
	void set_euler(const Eigen::Matrix<double, 3, 1> &value);
    ///Sets state's value to mode value
	void set_mode(Mode value);
    ///Sets state's value to error value
	void set_error(Error value);
    
    ///Returns state's type
    Type get_type() const;
	///Returns real value of state
	double get_real() const;
    ///Returns quaternion value
    Eigen::Quaterniond get_quaternion() const;
    ///Returns quaternion value given as wxyz
    Eigen::Matrix<double, 4, 1> get_wxyz() const;
    ///Returns quaternion value given as Euler angles
    Eigen::Matrix<double, 3, 1> get_euler() const;
    ///Returns mode value of state
	Mode get_mode() const;
    ///Returns value of state
	Error get_error() const;

    ///Returns string representation of state
	std::string to_string() const;

	///Returns if target state is reached
	static bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const State &start_state,
                  const State &current_state,
                  const State &previous_desired_state,
                  const State &target_state,
                  const o80::Speed &speed);
	///Returns intermediate state between two states
	static State intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Speed &speed);
	///Returns intermediate state between two states
    static State intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Duration_us &duration);
	///Returns intermediate state between two states
    static State intermediate_state(long int start_iteration,
                           long int current_iteration,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Iteration &iteration);
	///Serializes state
	template <class Archive> void serialize(Archive &archive)
    {
        archive(typ_);
        archive(value_.quaternion);
        //Most probably o80 initializes shared memory with size at first serialization. Because states are uninitialized, this state is smaller than will be in future
        //So we write largest value. Not safe and not perfect
    }
};

bool operator==(const State &a, const State &b);  ///<Compares states
bool operator!=(const State &a, const State &b);  ///<Compares states

}  // namespace franka_o80