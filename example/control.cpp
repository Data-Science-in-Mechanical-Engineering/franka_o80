#include "../include/franka_o80/kinematics.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/actuator.hpp"
#include <string.h>

double execution_time = 1.0;

void help()
{
    std::cout << "Welcome to franka_o80 control!"                                     << std::endl;
	std::cout << "The program is created to control backends"                         << std::endl;
	std::cout << std::endl;
    std::cout << "Usage:"                                                             << std::endl;
    std::cout << "./control ID       to start control loop with given backend"        << std::endl;
    std::cout << "Where:"                                                             << std::endl;
    std::cout << "ID is backend identifier"                                           << std::endl;
    std::cout << std::endl;
    std::cout << "Control loop syntax: command / command sign v / command x y z"      << std::endl;
    std::cout << "Possible signs:    + - ="                                           << std::endl;
    std::cout << "Possible commands: 1..7     - joints 1..7"                          << std::endl;
    std::cout << "                   x, y, z  - X, Y and Z coordinates"               << std::endl;
    std::cout << "                   u, v, w  - Roll, pitch, yaw Euler angles"        << std::endl;
    std::cout << "                   g        - gripper width"                        << std::endl;
    std::cout << "                   t        - execution time"                       << std::endl;
    std::cout << "                   e        - echo"                                 << std::endl;
    std::cout << "                   q        - quit"                                 << std::endl;
}

void echo(franka_o80::FrontEnd *front)
{
    front->reset_next_index();
    franka_o80::States states = front->wait_for_next().get_observed_states();
    
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

    std::cout << "time                 : " << execution_time << std::endl;
}

void set1(franka_o80::FrontEnd *front, std::string input)
{
    //Command
    char *p = &input[0];
    while (*p != '\0' && *p == ' ') p++;
    char command = *p;

    //Sign
    p++;
    while (*p != '\0' && *p == ' ') p++;
    char sign = *p;
    if (*p != '+' && *p != '-' && *p != '=') { help(); return; }

    //Number
    p++;
    while (*p != '\0' && *p == ' ') p++;
    double number = strtod(p, &p);

    front->reset_next_index();
    franka_o80::States states = front->wait_for_next().get_observed_states();
    if (std::strchr("1234567", command)  != nullptr)
    {
        //Joints
        number = M_PI * number / 180.0;
        size_t actuator = franka_o80::joint_position[command - '1'];
        if (sign == '+') states.values[actuator].value += number;
        else if (sign == '-') states.values[actuator].value -= number;
        else states.values[actuator].value = number;
        front->add_command(actuator, states.values[actuator], o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
        front->pulse_and_wait();
    }
    else if (std::strchr("xyz", command) != nullptr)
    {
        //Cartesian
        size_t actuator = franka_o80::cartesian_position[command - 'x'];
        if (sign == '+') states.values[actuator].value += number;
        else if (sign == '-') states.values[actuator].value -= number;
        else states.values[actuator].value = number;
        franka_o80::cartesian_to_joint(states);
        for (size_t i = 0; i < 7; i++) front->add_command(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]), o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
        front->pulse_and_wait();
    }
    else if (std::strchr("uvw", command) != nullptr)
    {
        //Roll-patch-yaw
        number = M_PI * number / 180.0;
        size_t actuator = franka_o80::cartesian_position[command - 'u'];
        if (sign == '+') states.values[actuator].value += number;
        else if (sign == '-') states.values[actuator].value -= number;
        else states.values[actuator].value = number;
        franka_o80::cartesian_to_joint(states);
        for (size_t i = 0; i < 7; i++) front->add_command(franka_o80::joint_position[i], states.get(franka_o80::joint_position[i]), o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
        front->pulse_and_wait();
    }
    else if (command == 'g')
    {
        //Gripper
        size_t actuator = franka_o80::gripper_width;
        if (sign == '+') states.values[actuator].value += number;
        else if (sign == '-') states.values[actuator].value -= number;
        else states.values[actuator].value = number;
        front->add_command(actuator, states.values[actuator], o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
        front->pulse_and_wait();
    }
    else if (command == 't')
    {
        //Execution time
        if (sign == '+') execution_time += number;
        else if (sign == '-') execution_time -= number;
        else execution_time = number;
        if (execution_time < 1.0) execution_time = 1.0;
    }
}

int run(int argc, char **argv)
{
    if (argc != 2) { help(); return 1; }

    //Reading ID
    const char *id = argv[1];
    
    //Creating frontend
	franka_o80::FrontEnd frontend(id);
    frontend.add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
    frontend.pulse_and_wait();
	
    while (true)
    {
        std::cout << "Command: ";
        std::string input;
        std::getline(std::cin, input);
        char *p = &input[0];
        while (*p != '\0' && *p == ' ') p++;
        char command = *p;

        if (std::strchr("1234567gxyzuvwt", command) != nullptr) set1(&frontend, input);
        else if (command == 'e') echo(&frontend);
        else if (command == 'q') break;
        else help();
    }

	return 0;
}

int main(int argc, char **argv)
{
	try
	{
		return run(argc, argv);
	}
	catch (std::exception &e)
	{
		std::cout << "Exception occured: " << e.what() << std::endl;
		std::cout << "Terminating..." << std::endl;
	}
	return 1;
}