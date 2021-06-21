#pragma once

#include "mode.hpp"
#include "error.hpp"
#include <stdexcept>
#include <string>
#include <limits>
#include <o80/interpolation.hpp>

namespace franka_o80
{
///Actuator state. Because of `o80` restrictions many other values, like control modes or errors, are also represented as actuators. Dynamic typazation is done here
class State
{
public:
    ///Type of state
    enum class Type
    {
        real,
        mode,
        error
    };

private:
    Type typ_;

public:
    ///Value of state
    double value;
    
    ///Creates state with zero real value
	State();
	///Creates state with given real value
	State(double value);
    ///Creates state with given mode value
	State(Mode value);
    ///Creates state with given error value
	State(Error value);
	///Copies state
	State(const State &state);
	
    ///Sets state's value to real value
	void set(double value);
    ///Sets state's value to mode value
	void set(Mode value);
    ///Sets state's value to mode value
	void set(Error value);
    
    ///Returns state's type
    Type get_type();
	///Returns real value of state
	double get() const;
    ///Returns mode value of state
	Mode get_mode() const;
    ///Returns value of state
	Error get_error() const;
	///Transforms state to `double`
	operator double() const;
    ///Transforms state to `Mode`
	operator Mode() const;
    ///Transforms state to `Error`
	operator Error() const;
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
		archive(value);
    }
};

bool operator==(State a, State b);  ///<Compares states
bool operator==(double a, State b); ///<Compares state and real
bool operator==(State a, double b); ///<Compares state and real
bool operator==(Mode a, State b);   ///<Compares state and mode
bool operator==(State a, Mode b);   ///<Compares state and mode
bool operator==(Error a, State b);  ///<Compares state and error
bool operator==(State a, Error b);  ///<Compares state and error
bool operator!=(State a, State b);  ///<Compares states
bool operator!=(double a, State b); ///<Compares state and real
bool operator!=(State a, double b); ///<Compares state and real
bool operator!=(Mode a, State b);   ///<Compares state and mode
bool operator!=(State a, Mode b);   ///<Compares state and mode
bool operator!=(Error a, State b);  ///<Compares state and error
bool operator!=(State a, Error b);  ///<Compares state and error
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