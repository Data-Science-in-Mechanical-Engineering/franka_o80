#include "../include/franka_o80/state.hpp"

franka_o80::State::State() : typ_(Type::real), value(0.0)
{
}

franka_o80::State::State(double value) : typ_(Type::real), value(value)
{
}

franka_o80::State::State(Mode mode) : typ_(Type::mode), value(static_cast<double>(mode))
{
}

franka_o80::State::State(Error error) : typ_(Type::error), value(static_cast<double>(error))
{
}

franka_o80::State::State(const State &state) : value(state.value)
{
}

//===============================================================

void franka_o80::State::set(double value)
{
    this->typ_ = Type::real;
    this->value = value;
}

void franka_o80::State::set(Mode mode)
{
    this->typ_ = Type::mode;
    this->value = static_cast<double>(mode);
}

void franka_o80::State::set(Error error)
{
    this->typ_ = Type::error;
    this->value = static_cast<double>(error);
}

//===============================================================

franka_o80::State::Type franka_o80::State::get_type()
{
    return typ_;
}

double franka_o80::State::get() const
{
    if (typ_ != Type::real) throw std::runtime_error("franka_o80: State type error");
    return value;
}

franka_o80::Mode franka_o80::State::get_mode() const
{
    if (typ_ != Type::mode) throw std::runtime_error("franka_o80: State type error");
    return static_cast<Mode>(value);
}

franka_o80::Error franka_o80::State::get_error() const
{
    if (typ_ != Type::error) throw std::runtime_error("franka_o80: State type error");
    return static_cast<Error>(value);
}

franka_o80::State::operator double() const
{
    return get();
}

franka_o80::State::operator Mode() const
{
    return get_mode();
}

franka_o80::State::operator Error() const
{
    return get_error();
}

std::string franka_o80::State::to_string() const
{
    if (typ_ == Type::real) return std::to_string(value);
    else if (typ_ == Type::mode)
    {
        switch (get_mode())
        {
        case Mode::torque:
            return "Mode::torque";
            break;
        case Mode::torque_position:
            return "Mode::torque_position";
            break;
        case Mode::torque_velocity:
            return "Mode::torque_velocity";
            break;
        case Mode::torque_cartesian_position:
            return "Mode::torque_cartesian_position";
            break;
        case Mode::torque_cartesian_velocity:
            return "Mode::torque_cartesian_velocity";
            break;
        case Mode::position:
            return "Mode::position";
            break;
        case Mode::velocity:
            return "Mode::velocity";
            break;
        case Mode::cartesian_position:
            return "Mode::cartesian_position";
            break;
        case Mode::cartesian_velocity:
            return "Mode::cartesian_velocity";
            break;
        case Mode::intelligent_position:
            return "Mode::intelligent_position";
            break;
        case Mode::intelligent_cartesian_position:
            return "Mode::intelligent_cartesian_position";
            break;
        default:
            return "Mode::invalid";
        }
    }
    else
    {
        switch (get_error())
        {
        case Error::robot_command_exception:
            return "Error::robot_command_exception";
            break;
        case Error::robot_control_exception:
            return "Error::robot_control_exception";
            break;
        case Error::robot_invalid_operation_exception:
            return "Error::robot_invalid_operation_exception";
            break;
        case Error::robot_network_exception:
            return "Error::robot_network_exception";
            break;
        case Error::robot_realtime_exception:
            return "Error::robot_realtime_exception";
            break;
        case Error::robot_invalid_argument_exception:
            return "Error::robot_invalid_argument_exception";
            break;
        case Error::robot_other_exception:
            return "Error::robot_other_exception";
            break;
        case Error::gripper_command_exception:
            return "Error::gripper_command_exception";
            break;
        case Error::gripper_network_exception:
            return "Error::gripper_network_exception";
            break;
        case Error::gripper_invalid_operation_exception:
            return "Error::gripper_invalid_operation_exception";
            break;
        case Error::gripper_other_exception:
            return "Error::gripper_other_exception";
            break;
        default:
            return "Error::ok";
        }
    }
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
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State type error");
    else if (start_state.typ_ == Type::error)
    {
        if (start_state.value == target_state.value) return start_state;
        else return Error::robot_other_exception;
    }
    else if (start_state.typ_ == Type::mode)
    {
        if (start_state.value == target_state.value) return start_state;
        else return Mode::invalid;
    }
    else return State(o80::intermediate_state(start, now, start_state.value, current_state.value, target_state.value, speed));
}

franka_o80::State franka_o80::State::intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Duration_us &duration)
{
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State type error");
    else if (start_state.typ_ == Type::error)
    {
        if (start_state.value == target_state.value) return start_state;
        else return Error::robot_other_exception;
    }
    else if (start_state.typ_ == Type::mode)
    {
        if (start_state.value == target_state.value) return start_state;
        else return Mode::invalid;
    }
    else return State(o80::intermediate_state(start, now, start_state.value, current_state.value, target_state.value, duration));
}

franka_o80::State franka_o80::State::intermediate_state(long int start_iteration,
                           long int current_iteration,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Iteration &iteration)
{
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State type error");
    else if (start_state.typ_ == Type::error)
    {
        if (start_state.value == target_state.value) return start_state;
        else return Error::robot_other_exception;
    }
    else if (start_state.typ_ == Type::mode)
    {
        if (start_state.value == target_state.value) return start_state;
        else return Mode::invalid;
    }
    else return State(o80::intermediate_state(start_iteration, current_iteration, start_state.value, current_state.value, target_state.value, iteration));
}

bool franka_o80::operator==(State a, State b)  { if (a.get_type() != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a.value == b.value; }
bool franka_o80::operator==(double a, State b) { if (State::Type::real != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a == b.value; }
bool franka_o80::operator==(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value == b; }
bool franka_o80::operator==(Mode a, State b)   { if (State::Type::mode != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a == static_cast<Mode>(b.value); }
bool franka_o80::operator==(State a, Mode b)   { if (a.get_type() != State::Type::mode) throw std::runtime_error("franka_o80: State type error"); return static_cast<Mode>(a.value) == b; }
bool franka_o80::operator==(Error a, State b)  { if (State::Type::error != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a == static_cast<Error>(b.value); }
bool franka_o80::operator==(State a, Error b)  { if (a.get_type() != State::Type::error) throw std::runtime_error("franka_o80: State type error"); return static_cast<Error>(a.value) == b; }
bool franka_o80::operator!=(State a, State b)  { if (a.get_type() != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a.value != b.value; }
bool franka_o80::operator!=(double a, State b) { if (State::Type::real != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a != b.value; }
bool franka_o80::operator!=(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value != b; }
bool franka_o80::operator!=(Mode a, State b)   { if (State::Type::mode != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a != static_cast<Mode>(b.value); }
bool franka_o80::operator!=(State a, Mode b)   { if (a.get_type() != State::Type::mode) throw std::runtime_error("franka_o80: State type error"); return static_cast<Mode>(a.value) != b; }
bool franka_o80::operator!=(Error a, State b)  { if (State::Type::error != b.get_type()) throw std::runtime_error("franka_o80: State type error"); return a != static_cast<Error>(b.value); }
bool franka_o80::operator!=(State a, Error b)  { if (a.get_type() != State::Type::error) throw std::runtime_error("franka_o80: State type error"); return static_cast<Error>(a.value) != b; }
bool franka_o80::operator<(State a, State b)   { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value < b.value; }
bool franka_o80::operator<(double a, State b)  { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a < b.value; }
bool franka_o80::operator<(State a, double b)  { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value < b; }
bool franka_o80::operator>(State a, State b)   { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value > b.value; }
bool franka_o80::operator>(double a, State b)  { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a > b.value; }
bool franka_o80::operator>(State a, double b)  { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value > b; }
bool franka_o80::operator<=(State a, State b)  { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value <= b.value; }
bool franka_o80::operator<=(double a, State b) { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a <= b.value; }
bool franka_o80::operator<=(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value <= b; }
bool franka_o80::operator>=(State a, State b)  { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value >= b.value; }
bool franka_o80::operator>=(double a, State b) { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a >= b.value; }
bool franka_o80::operator>=(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value >= b; }
franka_o80::State franka_o80::operator+(State a, State b)  { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value + b.value; }
franka_o80::State franka_o80::operator+(double a, State b) { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a + b.value; }
franka_o80::State franka_o80::operator+(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value + b; }
franka_o80::State franka_o80::operator-(State a, State b)  { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value - b.value; }
franka_o80::State franka_o80::operator-(double a, State b) { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a - b.value; }
franka_o80::State franka_o80::operator-(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value - b; }
franka_o80::State franka_o80::operator*(State a, State b)  { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value * b.value; }
franka_o80::State franka_o80::operator*(double a, State b) { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a * b.value; }
franka_o80::State franka_o80::operator*(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value * b; }
franka_o80::State franka_o80::operator/(State a, State b)  { if (a.get_type() != State::Type::real || b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value / b.value; }
franka_o80::State franka_o80::operator/(double a, State b) { if (b.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a / b.value; }
franka_o80::State franka_o80::operator/(State a, double b) { if (a.get_type() != State::Type::real) throw std::runtime_error("franka_o80: State type error"); return a.value / b; }