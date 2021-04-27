#include "../include/franka_o80/state.hpp"

franka_o80::State::State() : value(0.0)
{
}

franka_o80::State::State(double value) : value(value)
{
}


franka_o80::State::State(const State &state) : value(state.value)
{
}

void franka_o80::State::set(double value)
{
    this->value = value;
}

double franka_o80::State::get() const
{
    return value;
}

std::string franka_o80::State::to_string() const
{
    return std::to_string(value);
}

franka_o80::State::operator double() const
{
    return value;
}

bool franka_o80::State::finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const State &start_state,
                  const State &current_state,
                  const State &previous_desired_state,
                  const State &target_state,
                  const o80::Speed &speed)
{
    return o80::finished(start, now, start_state.value, current_state.value, target_state.value, speed);
}

franka_o80::State franka_o80::State::intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Speed &speed)
{
    return State(o80::intermediate_state(start, now, start_state.value, current_state.value, target_state.value, speed));
}

franka_o80::State franka_o80::State::intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Duration_us &duration)
{
    return State(o80::intermediate_state(start, now, start_state.value, current_state.value, target_state.value, duration));
}

franka_o80::State franka_o80::State::intermediate_state(long int start_iteration,
                           long int current_iteration,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Iteration &iteration)
{
    return State(o80::intermediate_state(start_iteration, current_iteration, start_state.value, current_state.value, target_state.value, iteration));
}

bool franka_o80::operator==(State a, State b)  { return a.value == b.value; }
bool franka_o80::operator==(double a, State b) { return a == b.value; }
bool franka_o80::operator==(State a, double b) { return a.value == b; }
bool franka_o80::operator!=(State a, State b)  { return a.value != b.value; }
bool franka_o80::operator!=(double a, State b) { return a != b.value; }
bool franka_o80::operator!=(State a, double b) { return a.value != b; }
bool franka_o80::operator<(State a, State b)   { return a.value < b.value; }
bool franka_o80::operator<(double a, State b)  { return a < b.value; }
bool franka_o80::operator<(State a, double b)  { return a.value < b; }
bool franka_o80::operator>(State a, State b)   { return a.value > b.value; }
bool franka_o80::operator>(double a, State b)  { return a > b.value; }
bool franka_o80::operator>(State a, double b)  { return a.value > b; }
bool franka_o80::operator<=(State a, State b)  { return a.value <= b.value; }
bool franka_o80::operator<=(double a, State b) { return a <= b.value; }
bool franka_o80::operator<=(State a, double b) { return a.value <= b; }
bool franka_o80::operator>=(State a, State b)  { return a.value >= b.value; }
bool franka_o80::operator>=(double a, State b) { return a >= b.value; }
bool franka_o80::operator>=(State a, double b) { return a.value >= b; }
franka_o80::State franka_o80::operator+(State a, State b)  { return a.value + b.value; }
franka_o80::State franka_o80::operator+(double a, State b) { return a + b.value; }
franka_o80::State franka_o80::operator+(State a, double b) { return a.value + b; }
franka_o80::State franka_o80::operator-(State a, State b)  { return a.value - b.value; }
franka_o80::State franka_o80::operator-(double a, State b) { return a - b.value; }
franka_o80::State franka_o80::operator-(State a, double b) { return a.value - b; }
franka_o80::State franka_o80::operator*(State a, State b)  { return a.value * b.value; }
franka_o80::State franka_o80::operator*(double a, State b) { return a * b.value; }
franka_o80::State franka_o80::operator*(State a, double b) { return a.value * b; }
franka_o80::State franka_o80::operator/(State a, State b)  { return a.value / b.value; }
franka_o80::State franka_o80::operator/(double a, State b) { return a / b.value; }
franka_o80::State franka_o80::operator/(State a, double b) { return a.value / b; }