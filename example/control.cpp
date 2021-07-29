#include "../include/franka_o80/kinematics.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/actuator.hpp"
#include <memory>
#include <string>
#include <thread>
#include <string.h>

class Control
{
private:
    enum class Command
    {
        no_previous_joint,
        no_previous_cartesian,
        joint,
        cartesian
    };

    std::unique_ptr<franka_o80::FrontEnd> front_;
    double execution_time_  = 5.0;
    bool finish_            = false;
    Command command_        = Command::no_previous_joint;
    bool gripper_command_   = false;

public:
    static void help();
    Control(std::string id);
    void echo();
    void defaultt();
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
    std::cout << "p     - Pass.                     Syntax: p"                                  << std::endl;
    std::cout << "d     - Default position.         Syntax: d"                                  << std::endl;
    std::cout << "e     - Echo.                     Syntax: e"                                  << std::endl;
    std::cout << "f     - Finish.                   Syntax: f"                                  << std::endl;
    std::cout << "#     - Comment."                                                             << std::endl;
}

Control::Control(std::string id)
{
	front_ = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(id));
    front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Duration_us::seconds(1.0), o80::Mode::QUEUE);
    front_->pulse_and_wait();
}

void Control::echo()
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

void Control::defaultt()
{
    //Read states if need it
    franka_o80::States states;
    if (command_ == Command::no_previous_joint) { front_->reset_next_index(); states = front_->wait_for_next().get_observed_states(); }

    //Remind cartesian target
    if (command_ == Command::no_previous_joint)
    {
        for (size_t i = 0; i < 3; i++) front_->add_command(franka_o80::cartesian_position[i], states.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
        front_->add_command(franka_o80::cartesian_orientation, states.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
    }

    //Add command
    front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
    for (size_t i = 0; i < 3; i++) front_->add_command(franka_o80::cartesian_position[i], franka_o80::default_states().get(franka_o80::cartesian_position[i]), o80::Duration_us::seconds(execution_time_),  o80::Mode::QUEUE);
    front_->add_command(franka_o80::cartesian_orientation, franka_o80::default_states().get(franka_o80::cartesian_orientation), o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
    command_ = Command::cartesian;
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
        if (*p == '\t' || *p == ' ') { p++; continue; }
        if (*p == 'd')               { p++; defaultt(); continue; }
        if (*p == 'e')               { p++; echo();     continue; }
        if (*p == 'f')               { finish_ = true;  return;   }
        if (std::strchr("1234567 xyz qr gt", *p) == nullptr) { help(); return; }
        else { command = *p++; parser = Parser::wait_sign; continue; }
    }

    case Parser::wait_sign:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        else if (std::strchr("+-=", *p) == nullptr) { help(); return; }
        else { sign = *p++; parser = Parser::wait_value1; continue; }
    }

    case Parser::wait_value1:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        char *np;
        values[0] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        if (std::strchr("qr", command)) { parser = Parser::wait_value2; continue; }
        if (std::strchr("1234567", command)  != nullptr)
        {
            //Read states if need it
            if (command_ == Command::cartesian) { help(); return; }
            franka_o80::States states;
            if (sign != '=' || command_ == Command::no_previous_cartesian) { front_->reset_next_index(); states = front_->wait_for_next().get_observed_states(); }

            //Remind joint target
            if (command_ == Command::no_previous_cartesian)
                for (size_t i = 0; i < 7; i++) front_->add_command(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]), o80::Mode::QUEUE);
            
            //Set target
            size_t actuator = franka_o80::joint_position[command - '1'];
            franka_o80::State state;
            if (sign == '+') state.set_real(states.get(actuator).get_real() + M_PI * values[0] / 180.0);
            else if (sign == '-') state.set_real(states.get(actuator).get_real() - M_PI * values[0] / 180.0);
            else state.set_real(M_PI * values[0] / 180.0);

            //Add command
            front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Mode::QUEUE);
            front_->add_command(actuator, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            command_ = Command::joint;
        }
        else if (std::strchr("xyz", command)  != nullptr)
        {
            //Read states if need it
            if (command_ == Command::joint) { help(); return; }
            franka_o80::States states;
            if (sign != '=' || command_ == Command::no_previous_joint) { front_->reset_next_index(); states = front_->wait_for_next().get_observed_states(); }

            //Remind cartesian target
            if (command_ == Command::no_previous_joint)
            {
                for (size_t i = 0; i < 3; i++) front_->add_command(franka_o80::cartesian_position[i], states.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
                front_->add_command(franka_o80::cartesian_orientation, states.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
            }

            //Set target
            size_t actuator = franka_o80::cartesian_position[command - 'x'];
            franka_o80::State state;
            if (sign == '+') state.set_real(states.get(actuator).get_real() + values[0]);
            else if (sign == '-') state.set_real(states.get(actuator).get_real() - values[0]);
            else state.set_real(values[0]);

            //Add command
            front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
            front_->add_command(actuator, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            command_ = Command::cartesian;
        }
        else if (command == 'g')
        {
            //Read states if need it
            franka_o80::States states;
            if (sign != '=') { front_->reset_next_index(); states = front_->wait_for_next().get_observed_states(); }

            //Set target
            franka_o80::State state;
            if (sign == '+') state.set_real(states.get(franka_o80::gripper_width).get_real() + values[0]);
            else if (sign == '-') state.set_real(states.get(franka_o80::gripper_width).get_real() - values[0]);
            else state.set_real(values[0]);

            //Add command
            front_->add_command(franka_o80::gripper_width, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            gripper_command_ = true;
        }
        else
        {
            if (sign == '+') execution_time_ += values[0];
            else if (sign == '-') execution_time_ -= values[0];
            else execution_time_ = values[0];
            if (execution_time_ < 1.0) execution_time_ = 1.0;
        }
        parser = Parser::wait_command;
        continue;
    }

    case Parser::wait_value2:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        char *np;
        values[1] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        parser = Parser::wait_value3;
        continue;
    }

    case Parser::wait_value3:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        char *np;
        values[2] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        if (command == 'q') { parser = Parser::wait_value4; continue; }

        //Read states if need it
        franka_o80::States states;
        if (sign != '=' || command_ == Command::no_previous_joint) { front_->reset_next_index(); states = front_->wait_for_next().get_observed_states(); }

        //Remind cartesian target
        if (command_ == Command::no_previous_joint)
        {
            for (size_t i = 0; i < 3; i++) front_->add_command(franka_o80::cartesian_position[i], states.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
            front_->add_command(franka_o80::cartesian_orientation, states.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
        }

        //Set target
        franka_o80::State state(Eigen::Matrix<double, 3, 1>(M_PI * values[0] / 180, M_PI * values[1] / 180, M_PI * values[2] / 180));
        if (sign == '+') state.set_quaternion(state.get_quaternion() * states.get(franka_o80::cartesian_orientation).get_quaternion());
        else if (sign == '-') state.set_quaternion(state.get_quaternion().inverse() * states.get(franka_o80::cartesian_orientation).get_quaternion());
        
        //Add command
        front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
        front_->add_command(franka_o80::cartesian_orientation, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
        command_ = Command::cartesian;
        parser = Parser::wait_command;
        continue;
    }

    default:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        char *np;
        values[3] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;

        //Read states if need it
        franka_o80::States states;
        if (sign != '=' || command_ == Command::no_previous_joint) { front_->reset_next_index(); states = front_->wait_for_next().get_observed_states(); }

        //Remind cartesian target
        if (command_ == Command::no_previous_joint)
        {
            for (size_t i = 0; i < 3; i++) front_->add_command(franka_o80::cartesian_position[i], states.get(franka_o80::cartesian_position[i]), o80::Mode::QUEUE);
            front_->add_command(franka_o80::cartesian_orientation, states.get(franka_o80::cartesian_orientation), o80::Mode::QUEUE);
        }

        //Set target
        franka_o80::State state(Eigen::Matrix<double, 4, 1>(values[0], values[1], values[2], values[3]));
        if (sign == '+') state.set_quaternion(state.get_quaternion() * states.get(franka_o80::cartesian_orientation).get_quaternion());
        else if (sign == '-') state.set_quaternion(state.get_quaternion().inverse() * states.get(franka_o80::cartesian_orientation).get_quaternion());

        //Add command
        front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_cartesian_position, o80::Mode::QUEUE);
        front_->add_command(franka_o80::cartesian_orientation, state, o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
        command_ = Command::cartesian;
        parser = Parser::wait_command;
        continue;
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
        if (command_ == Command::joint || command_ == Command::cartesian || gripper_command_) front_->pulse_and_wait();
        if (command_ == Command::joint) command_ = Command::no_previous_joint;
        if (command_ == Command::cartesian) command_ = Command::no_previous_cartesian;
        gripper_command_ = false;
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