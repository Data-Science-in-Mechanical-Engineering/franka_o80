#pragma once

#include "constants.hpp"
#include <stdexcept>
#include <string>
#include <limits>
#include <o80/interpolation.hpp>

namespace franka_o80
{
///Actuator state. Because of `o80` restrictions, many other values, like control signals or errors, are also represented as actuators
class State
{
public:
    ///Value of state
    double value;
    ///Creates state with zero value
	State();
	///Creates state with given value
	State(double value);
	///Copies state
	State(const State &state);
	///Sets state's value to given value
	void set(double value);
	///Returns value of state
	double get() const;
	///Returns string representation of state
	std::string to_string() const;
	///Transforms state to `double`
	operator double() const;

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
		archive(value);
    }
};


bool operator==(State a, State b);  ///<Compares states
bool operator==(double a, State b); ///<Compares states
bool operator==(State a, double b); ///<Compares states
bool operator!=(State a, State b);  ///<Compares states
bool operator!=(double a, State b); ///<Compares states
bool operator!=(State a, double b); ///<Compares states
bool operator<(State a, State b);   ///<Compares states
bool operator<(double a, State b);  ///<Compares states
bool operator<(State a, double b);  ///<Compares states
bool operator>(State a, State b);   ///<Compares states
bool operator>(double a, State b);  ///<Compares states
bool operator>(State a, double b);  ///<Compares states
bool operator<=(State a, State b);  ///<Compares states
bool operator<=(double a, State b); ///<Compares states
bool operator<=(State a, double b); ///<Compares states
bool operator>=(State a, State b);  ///<Compares states
bool operator>=(double a, State b); ///<Compares states
bool operator>=(State a, double b); ///<Compares states
State operator+(State a, State b);  ///<Adds states
State operator+(double a, State b); ///<Adds states
State operator+(State a, double b); ///<Adds states
State operator-(State a, State b);  ///<Subtracts one state from another
State operator-(double a, State b); ///<Subtracts one state from another
State operator-(State a, double b); ///<Subtracts one state from another
State operator*(State a, State b);  ///<Multipicates states
State operator*(double a, State b); ///<Multipicates states
State operator*(State a, double b); ///<Multipicates states
State operator/(State a, State b);  ///<Subtracts one state by another
State operator/(double a, State b); ///<Subtracts one state by another
State operator/(State a, double b); ///<Subtracts one state by another

}  // namespace franka_o80