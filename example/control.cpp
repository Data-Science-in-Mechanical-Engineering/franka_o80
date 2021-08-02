#include "../include/franka_o80/kinematics.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/limits.hpp"
#include <memory>
#include <string>
#include <thread>
#include <set>
#include <string.h>

class Control
{
private:
    std::unique_ptr<franka_o80::FrontEnd> front_;
    franka_o80::States target_;
    std::set<char> commands_;
    double impedances_[3] = { 1.0, 1.0, 1.0 };
    double execution_time_  = 5.0;
    bool cartesian_mode_    = false; //If true, cartesian positions are valid. If false, joint positions are valid
    bool command_added_     = false;
    bool finish_            = false;

public:
    static void help();
    Control(std::string id);
    bool commands_count(const std::string commands) const;
    void commands_insert(const std::string commands);
    void command_echo();
    void command_default();
    void command_joint_position(char command, char sign, double value);
    void command_cartesian_position(char command, char sign, double value);
    void command_cartesian_orientation(char command, char sign, const double values[4]);
    void command_gripper_width(char command, char sign, double value);
    void command_impedance(char command, char sign, const double values[3]);
    void execute(std::string input);
    void loop();
};

void Control::help()
{
    std::cout << "Welcome to franka_o80 control!"                       << std::endl;
	std::cout << "The program is created to control franka_o80 backend" << std::endl;
	std::cout << std::endl;

    std::cout << "Usage:"                         << std::endl;
    std::cout << "./control ID"                   << std::endl;
    std::cout << "Where ID is backend identifier" << std::endl;
    std::cout << std::endl;

    std::cout << "Possible commands:" << std::endl;
    std::cout << "1..7  - Joints 1 to 7.            Syntax: 1..7  +/-/= degree"                 << std::endl;
    std::cout << "x/y/z - X, Y and Z coordinates.   Syntax: x/y/z +/-/= meter"                  << std::endl;
    std::cout << "q     - Rotation in quaterions.   Syntax: q     +/-/= w      x      y      z" << std::endl;
    std::cout << "r     - Rotation in Euler angles. Syntax: r     +/-/= degree degree degree"   << std::endl;
    std::cout << "g     - Gripper width.            Syntax: g     +/-/= meter"                  << std::endl;
    std::cout << "t     - Execution time.           Syntax: t     +/-/= second"                 << std::endl;
    std::cout << "i     - Impedances.               Syntax: i     +/-/= joint  trans  rot"      << std::endl;
    std::cout << "p     - Pass.                     Syntax: p"                                  << std::endl;
    std::cout << "d     - Default position.         Syntax: d"                                  << std::endl;
    std::cout << "e     - Echo.                     Syntax: e"                                  << std::endl;
    std::cout << "f     - Finish.                   Syntax: f"                                  << std::endl;
    std::cout << "#     - Comment."                                                             << std::endl;
}

Control::Control(std::string id)
{
	front_ = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(id));
    front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Mode::QUEUE);
    front_->reset_next_index();
    target_ = front_->wait_for_next().get_observed_states();
}

bool Control::commands_count(const std::string commands) const
{
    for (size_t i = 0; i < commands.size(); i++)
    {
        if (commands[i] != ' ' && commands_.count(commands[i]) > 0) return true;
    }
    return false;
}

void Control::commands_insert(const std::string commands)
{
    for (size_t i = 0; i < commands.size(); i++)
    {
        if (commands[i] != ' ') commands_.insert(commands[i]);
    }
}

void Control::command_echo()
{
    front_->reset_next_index();
    franka_o80::States states = front_->wait_for_next().get_observed_states();

    //General    
    std::cout << "control_mode         : " << states.get(franka_o80::control_mode).to_string() << std::endl;
    std::cout << "control_error        : " << states.get(franka_o80::control_error).to_string() << std::endl;
    std::cout << "control_reset        : " << states.get(franka_o80::control_reset).get_real() << std::endl;
    std::cout << "execution time       : " << execution_time_ << std::endl;

    //Gripper
    std::cout << "gripper_width        : " << states.get(franka_o80::gripper_width).get_real() << std::endl;
    std::cout << "gripper_temperature  : " << states.get(franka_o80::gripper_temperature).get_real() << std::endl;

    //Robot joints
    std::cout << "joint_position       :"; for (size_t i = 0; i < 7; i++) std::cout << " " << 180.0 * states.get(franka_o80::joint_position[i]).get_real() / M_PI; std::cout << std::endl;
    std::cout << "joint_torque         :"; for (size_t i = 0; i < 7; i++) std::cout << " " << states.get(franka_o80::joint_torque[i]).get_real(); std::cout << std::endl;

    //Robot cartesian
    std::cout << "cartesian_position   :"; for (size_t i = 0; i < 3; i++) std::cout << " " << states.get(franka_o80::cartesian_position[i]).get_real(); std::cout << std::endl;
    std::cout << "cartesian_orientation:"; for (size_t i = 0; i < 4; i++) std::cout << " " << states.get(franka_o80::cartesian_orientation).get_wxyz()[i];
    std::cout << " ("; for (size_t i = 0; i < 3; i++) std::cout << " " << 180.0 * states.get(franka_o80::cartesian_orientation).get_euler()[i] / M_PI; std::cout << " )" << std::endl;    
}

void Control::command_default()
{
    //Check contradictions
    if (commands_count("1234567 xyz q"))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Switch to cartesian mode
    if (!cartesian_mode_)
    {
        franka_o80::joint_to_cartesian(target_);
        for (size_t i = 0; i < 3; i++)
        {
            front_->add_command(franka_o80::cartesian_position[i], target_.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
        }
        front_->add_command(franka_o80::cartesian_orientation, target_.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
        cartesian_mode_ = true;
        front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
    }

    commands_insert("xyz q");
    command_added_ = true;
    for (size_t i = 0; i < 3; i++)
    {
        target_.set(franka_o80::cartesian_position[i], franka_o80::default_states().get(franka_o80::cartesian_position[i]));
        front_->add_command(franka_o80::cartesian_position[i], franka_o80::default_states().get(franka_o80::cartesian_position[i]), o80::Duration_us::seconds(execution_time_),  o80::Mode::QUEUE);
    }
    target_.set(franka_o80::cartesian_orientation, franka_o80::default_states().get(franka_o80::cartesian_orientation));
    front_->add_command(franka_o80::cartesian_orientation, franka_o80::default_states().get(franka_o80::cartesian_orientation), o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
}

void Control::command_joint_position(char command, char sign, double value)
{
    //Check contradictions
    if (commands_count("xyz q") || commands_count(std::string(1, command)))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Switch to joint mode
    if (cartesian_mode_)
    {
        front_->reset_next_index();
        franka_o80::States states = front_->wait_for_next().get_observed_states();
        for (size_t i = 0; i < 7; i++)
        {
            target_.set(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]));
            front_->add_command(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]), o80::Mode::QUEUE);
        }
        cartesian_mode_ = false;
        front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Mode::QUEUE);
    }

    //Create actuator state
    size_t actuator = franka_o80::joint_position[command - '1'];
    franka_o80::State state;
    if (sign == '+') state.set_real(target_.get(actuator).get_real() + M_PI * value / 180.0);
    else if (sign == '-') state.set_real(target_.get(actuator).get_real() - M_PI * value / 180.0);
    else state.set_real(M_PI * value / 180.0);

    //Check
    if (state.get_real() > franka_o80::joint_position_max[command - '1'] || state.get_real() < franka_o80::joint_position_min[command - '1'])
    {
        std::cout << "Invalid joint position" << std::endl;
        return;
    }

    //Add command
    commands_insert(std::string(1, command));
    command_added_ = true;
    target_.set(actuator, state);
    front_->add_command(actuator, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
}

void Control::command_cartesian_position(char command, char sign, double value)
{
    //Check contradictions
    if (commands_count("1234567") || commands_count(std::string(1, command)))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Switch to cartesian mode
    if (!cartesian_mode_)
    {
        franka_o80::joint_to_cartesian(target_);
        for (size_t i = 0; i < 3; i++)
        {
            front_->add_command(franka_o80::cartesian_position[i], target_.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
        }
        front_->add_command(franka_o80::cartesian_orientation, target_.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
        cartesian_mode_ = true;
        front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
    }

    //Create state
    size_t actuator = franka_o80::joint_position[command - 'x'];
    franka_o80::State state;
    if (sign == '+') state.set_real(target_.get(actuator).get_real() + M_PI * value / 180.0);
    else if (sign == '-') state.set_real(target_.get(actuator).get_real() - M_PI * value / 180.0);
    else state.set_real(M_PI * value / 180.0);

    //Check
    try
    {
        franka_o80::States states = target_;
        states.set(actuator, state);
        franka_o80::cartesian_to_joint(states);
    }
    catch (...)
    {
        std::cout << "Invalid cartesian position" << std::endl;
        return;
    }

    //Add command
    commands_insert(std::string(1, command));
    command_added_ = true;
    target_.set(actuator, state);
    front_->add_command(actuator, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
}

void Control::command_cartesian_orientation(char command, char sign, const double values[4])
{
    //Check contradictions
    if (commands_count("1234567") || commands_count(std::string(1, 'q')))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Switch to cartesian mode
    if (!cartesian_mode_)
    {
        franka_o80::joint_to_cartesian(target_);
        for (size_t i = 0; i < 3; i++)
        {
            front_->add_command(franka_o80::cartesian_position[i], target_.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
        }
        front_->add_command(franka_o80::cartesian_orientation, target_.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
        cartesian_mode_ = true;
        front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
    }

    //Create state
    franka_o80::State state;
    if (command == 'q') state.set_wxyz(Eigen::Matrix<double, 4, 1>(values[0], values[1], values[2], values[3]));
    else state.set_euler(Eigen::Matrix<double, 3, 1>(M_PI * values[0] / 180, M_PI * values[1] / 180, M_PI * values[2] / 180));
    if (sign == '+') state.set_quaternion(state.get_quaternion() * target_.get(franka_o80::cartesian_orientation).get_quaternion());
    else if (sign == '-') state.set_quaternion(state.get_quaternion().inverse() * target_.get(franka_o80::cartesian_orientation).get_quaternion());

    //Check
    try
    {
        franka_o80::States states = target_;
        states.set(franka_o80::cartesian_orientation, state);
        franka_o80::cartesian_to_joint(states);
    }
    catch (...)
    {
        std::cout << "Invalid cartesian position" << std::endl;
        return;
    }
    
    //Add command
    commands_insert(std::string(1, 'q'));
    command_added_ = true;
    target_.set(franka_o80::cartesian_orientation, state);
    front_->add_command(franka_o80::cartesian_orientation, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
}

void Control::command_gripper_width(char command, char sign, double value)
{
    //Check contradictions
    if (commands_count(std::string(1, 'g')))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Create state
    franka_o80::State state;
    if (sign == '+') state.set_real(target_.get(franka_o80::gripper_width).get_real() + value);
    else if (sign == '-') state.set_real(target_.get(franka_o80::gripper_width).get_real() - value);
    else state.set_real(value);

    //Add command
    commands_insert(std::string(1, 'g'));
    command_added_ = true;
    target_.set(franka_o80::gripper_width, state);
    front_->add_command(franka_o80::gripper_width, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
}

void Control::command_impedance(char command, char sign, const double values[3])
{
    if (sign == '+')
    {
        for (size_t i = 0; i < 3; i++) impedances_[i] = values[i];
    }
    else if (sign == '-')
    {
        for (size_t i = 0; i < 3; i++) impedances_[i] += values[i];
    }
    else
    {
        for (size_t i = 0; i < 3; i++) impedances_[i] -= values[i];
    }
    for (size_t i = 0; i < 3; i++)
    {
        if (impedances_[i] < 1.0) impedances_[i] = 1.0;
    }

    for (size_t i = 0; i < 7; i++)
    {
        front_->add_command(franka_o80::joint_stiffness[i], target_.get(franka_o80::joint_stiffness[i]).get_real() * impedances_[0], o80::Mode::QUEUE);
        front_->add_command(franka_o80::joint_damping[i], target_.get(franka_o80::joint_damping[i]).get_real() * sqrt(impedances_[0]), o80::Mode::QUEUE);
    }
    for (size_t i = 0; i < 3; i++)
    {
        front_->add_command(franka_o80::cartesian_stiffness[i], target_.get(franka_o80::cartesian_stiffness[i]).get_real() * impedances_[1], o80::Mode::QUEUE);
        front_->add_command(franka_o80::cartesian_damping[i], target_.get(franka_o80::cartesian_damping[i]).get_real() * sqrt(impedances_[1]), o80::Mode::QUEUE);
        front_->add_command(franka_o80::cartesian_stiffness[i + 3], target_.get(franka_o80::cartesian_stiffness[i + 3]).get_real() * impedances_[2], o80::Mode::QUEUE);
        front_->add_command(franka_o80::cartesian_damping[i + 3], target_.get(franka_o80::cartesian_damping[i + 3]).get_real() * sqrt(impedances_[2]), o80::Mode::QUEUE);
    }
}

void Control::execute(std::string input)
{
    enum class Parser
    {
        wait_command,
        wait_sign,
        wait_value1,
        wait_value2,
        wait_value3,
        wait_value4
    };

    if (input.empty()) return;
    char *p = &input[0];
    char command = '\0';
    char sign = '\0';
    double values[4];
    Parser parser = Parser::wait_command;

    while (true) switch (parser)
    {
    case Parser::wait_command:
    {
        if (*p == '\0' || *p == '#') return;
        if (*p == '\t' || *p == ' ') { p++; }
        if (*p == 'd')               { p++; command_default(); }
        if (*p == 'e')               { p++; command_echo(); }
        if (*p == 'f')               { finish_ = true;  return; }
        if (std::strchr("1234567 xyz qr gti", *p) == nullptr) { help(); return; }
        else { command = *p++; parser = Parser::wait_sign; }
        break;
    }

    case Parser::wait_sign:
    {
        if (*p == ' ' || *p == '\t') p++;
        else if (std::strchr("+-=", *p) == nullptr) { help(); return; }
        else { sign = *p++; parser = Parser::wait_value1; }
        break;
    }

    case Parser::wait_value1:
    {
        if (*p == ' ' || *p == '\t') { p++; break; }
        char *np;
        values[0] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        if (std::strchr("qri", command)) { parser = Parser::wait_value2; break; }
        if (std::strchr("1234567", command) != nullptr) command_joint_position(command, sign, values[0]);
        else if (std::strchr("xyz", command) != nullptr) command_cartesian_position(command, sign, values[0]);
        else if (command == 'g') command_gripper_width(command, sign, values[0]);
        else
        {
            if (sign == '+') execution_time_ += values[0];
            else if (sign == '-') execution_time_ -= values[0];
            else execution_time_ = values[0];
            if (execution_time_ < 1.0) execution_time_ = 1.0;
        }
        parser = Parser::wait_command;
        break;
    }

    case Parser::wait_value2:
    {
        if (*p == ' ' || *p == '\t') { p++; break; }
        char *np;
        values[1] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        parser = Parser::wait_value3;
        break;
    }

    case Parser::wait_value3:
    {
        if (*p == ' ' || *p == '\t') { p++; break;; }
        char *np;
        values[2] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        if (command == 'q') { parser = Parser::wait_value4; break; }
        else if (command == 'r') command_cartesian_orientation(command, sign, values);
        else command_impedance(command, sign, values);
        parser = Parser::wait_command;
        break;
    }

    default:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        char *np;
        values[3] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        command_cartesian_orientation(command, sign, values);
        parser = Parser::wait_command;        
        break;
    }
    }
}

void Control::loop()
{
    while (!finish_)
    {
        std::string input;
        std::getline(std::cin, input);
        execute(input);
        commands_.clear();
        if (command_added_) front_->pulse_and_wait();
        command_added_ = false;
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) { Control::help(); return 1; }

    Control control(argv[1]);
	try
	{
        control.loop();
        return 0;
	}
	catch (std::exception &e)
	{
		std::cout << "Exception occured: " << e.what() << std::endl;
	}
	return 1;
}