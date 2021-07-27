#include "../include/franka_o80/kinematics.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/actuator.hpp"
#include <memory>
#include <string>
#include <string.h>

class Control
{
private:
    std::unique_ptr<franka_o80::FrontEnd> front_;
    double execution_time_;
    bool quit_request_;

public:
    static void help();
    Control(std::string id);
    void echo();
    void execute(std::string input);
    void loop();
};

void Control::help()
{
    std::cout << "Welcome to franka_o80 control!"                                     << std::endl;
	std::cout << "The program is created to control backends"                         << std::endl;
	std::cout << std::endl;
    std::cout << "Usage:"                                                             << std::endl;
    std::cout << "./control ID       to start control loop with given backend"        << std::endl;
    std::cout << "Where:"                                                             << std::endl;
    std::cout << "ID is backend identifier"                                           << std::endl;
    std::cout << std::endl;
    std::cout << "Control loop syntax: [command [+/-/= value]]+ [# comment]"          << std::endl;
    std::cout << "Possible commands: 1..7     - joints 1..7"                          << std::endl;
    std::cout << "                   x, y, z  - X, Y and Z coordinates"               << std::endl;
    std::cout << "                   u, v, w  - Roll, pitch, yaw Euler angles"        << std::endl;
    std::cout << "                   g        - gripper width"                        << std::endl;
    std::cout << "                   t        - execution time"                       << std::endl;
    std::cout << "                   e        - echo"                                 << std::endl;
    std::cout << "                   q        - quit"                                 << std::endl;
}

Control::Control(std::string id)
{
	front_ = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(id));
    front_->add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Duration_us::seconds(1.0), o80::Mode::QUEUE);
    front_->pulse_and_wait();
    execution_time_ = 5.0;
    quit_request_ = false;
}

void Control::echo()
{
    front_->reset_next_index();
    franka_o80::States states = front_->wait_for_next().get_observed_states();
    
    std::cout << "control_mode         : " << states.get(franka_o80::control_mode).to_string() << std::endl;
    std::cout << "control_error        : " << states.get(franka_o80::control_error).to_string();
    std::cout << "control_reset        : " << states.get(franka_o80::control_reset).get();

    //Gripper
    std::cout << "gripper_width        : " << states.get(franka_o80::gripper_width).get();
    std::cout << "gripper_temperature  : " << states.get(franka_o80::gripper_temperature).get();

    //Robot joints
    std::cout << "joint_position       :"; for (size_t i = 0; i < 7; i++) std::cout << " " << states.get(franka_o80::joint_position[i]).get(); std::cout << std::endl;
    //std::cout << "joint_velocity:"; for (size_t i = 0; i < 7; i++) std::cout << " " << states.get(franka_o80::joint_velocity[i]).get(); std::cout << std::endl;
    std::cout << "joint_torque         :"; for (size_t i = 0; i < 7; i++) std::cout << " " << states.get(franka_o80::joint_torque[i]).get(); std::cout << std::endl;

    //Robot cartesian
    std::cout << "cartesian_position   :"; for (size_t i = 0; i < 3; i++) std::cout << " " << states.get(franka_o80::cartesian_position[i]).get(); std::cout << std::endl;
    std::cout << "cartesian_orientation:"; for (size_t i = 0; i < 3; i++) std::cout << " " << 180.0 * states.get(franka_o80::cartesian_orientation[i]).get() / M_PI; std::cout << std::endl;
    //std::cout << "cartesian_velocitiy:"; for (size_t i = 0; i < 3; i++) std::cout << " " << states.get(franka_o80::cartesian_velocitiy[i]).get(); std::cout << std::endl;
    //std::cout << "cartesian_rotation:"; for (size_t i = 0; i < 3; i++) std::cout << " " << states.get(franka_o80::cartesian_rotation[i]).get(); std::cout << std::endl;

    std::cout << "time                 : " << execution_time_ << std::endl;
}

void Control::execute(std::string input)
{
    enum class State
    {
        wait_command,
        wait_sign,
        wait_value
    };

    if (input.empty()) return;
    char *p = &input[0];
    char command = '\0';
    char sign = '\0';
    State state = State::wait_command;

    while (true)
    {
        switch (state)
        {
            case State::wait_command:
            if (*p == '\0') return;
            else if (*p == '#') return;
            else if (*p == ' ' || *p == '\t') p++;
            else if (*p == 'e') { echo(); p++; }
            else if (*p == 'q') { quit_request_ = true; return; }
            else if (std::strchr("1234567 xyz uvw g t", *p)  != nullptr) { state = State::wait_sign; command = *p; p++; }
            else { help(); return; }
            break;

            case State::wait_sign:
            if (*p == ' ' || *p == '\t') p++;
            else if (std::strchr("+-=", *p)  != nullptr) { state = State::wait_value; sign = *p; p++; }
            else { help(); return; }
            break;

            default: //case State::wait_value:
            if (*p == ' ' || *p == '\t') { p++; break; }
            char *np;
            double value = strtod(p, &np);
            if (np == p) { help(); return; }
            state = State::wait_command;
            p = np;
            front_->reset_next_index();
            franka_o80::States states = front_->wait_for_next().get_observed_states();    
            if (std::strchr("1234567", *p)  != nullptr)
            {
                value = M_PI * value / 180.0;
                size_t actuator = franka_o80::joint_position[command - '1'];
                if (sign == '+') states.values[actuator].value += value;
                else if (sign == '-') states.values[actuator].value -= value;
                else states.values[actuator].value = value;
                front_->add_command(actuator, states.values[actuator], o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            }
            else if (std::strchr("xyz", *p)  != nullptr)
            {
                size_t actuator = franka_o80::cartesian_position[command - 'x'];
                if (sign == '+') states.values[actuator].value += value;
                else if (sign == '-') states.values[actuator].value -= value;
                else states.values[actuator].value = value;
                franka_o80::cartesian_to_joint(states);
                for (size_t i = 0; i < 7; i++) front_->add_command(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]), o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            }
            else if (std::strchr("uvw", *p)  != nullptr)
            {
                value = M_PI * value / 180.0;
                size_t actuator = franka_o80::cartesian_position[command - 'u'];
                if (sign == '+') states.values[actuator].value += value;
                else if (sign == '-') states.values[actuator].value -= value;
                else states.values[actuator].value = value;
                franka_o80::cartesian_to_joint(states);
                for (size_t i = 0; i < 7; i++) front_->add_command(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]), o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            }
            else if (std::strchr("g", *p)  != nullptr)
            {
                size_t actuator = franka_o80::gripper_width;
                if (sign == '+') states.values[actuator].value += value;
                else if (sign == '-') states.values[actuator].value -= value;
                else states.values[actuator].value = value;
                front_->add_command(actuator, states.values[actuator], o80::Duration_us::seconds(execution_time_), o80::Mode::QUEUE);
            }
            else
            {
                if (sign == '+') execution_time_ += value;
                else if (sign == '-') execution_time_ -= value;
                else execution_time_ = value;
                if (execution_time_ < 1.0) execution_time_ = 1.0;
            }
        }
    }
}

void Control::loop()
{
    while (!quit_request_)
    {
        std::string input;
        std::getline(std::cin, input);
        execute(input);
        front_->pulse_and_wait();
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