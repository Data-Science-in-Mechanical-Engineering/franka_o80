#include "../include/franka_o80/cartesial_states.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/indexes.hpp"
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
    std::cout << "                   f, r     - forward and right effector vectors"   << std::endl;
    std::cout << "                   g        - gripper width"                        << std::endl;
    std::cout << "                   t        - execution time"                       << std::endl;
    std::cout << "                   e        - echo"                                 << std::endl;
    std::cout << "                   q        - quit"                                 << std::endl;
}

void echo(franka_o80::FrontEnd *front)
{
    front->reset_next_index();
    franka_o80::States states = front->wait_for_next().get_observed_states();
    for (size_t i = 0; i < 7; i++) std::cout << "Joint " << i + 1 << " : " << 180.0 * states.get(franka_o80::robot_positions[i]).get() / M_PI << std::endl;
    std::cout << std::endl;
    
    franka_o80::CartesialStates cartesial = franka_o80::to_cartesial(states);
    std::cout << "Position:"; for (size_t i = 0; i < 3; i++) std::cout << " " << cartesial.get(franka_o80::cartesial_positions[i]).get(); std::cout << std::endl;
    std::cout << "Forward :"; for (size_t i = 0; i < 3; i++) std::cout << " " << cartesial.get(franka_o80::cartesial_forward[i]).get();   std::cout << std::endl;
    std::cout << "Right   :"; for (size_t i = 0; i < 3; i++) std::cout << " " << cartesial.get(franka_o80::cartesial_right[i]).get();     std::cout << std::endl;
    std::cout << std::endl;

    std::cout << "Gripper width      : " << states.get(franka_o80::gripper_width) << std::endl;
    std::cout << "Gripper temperature: " << states.get(franka_o80::gripper_temperature) << std::endl;
    std::cout << std::endl;

    std::cout << "Execution time     : " << execution_time << std::endl;
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
        size_t index = franka_o80::robot_positions[command - '1'];
        if (sign == '+') states.values[index].value += number;
        else if (sign == '-') states.values[index].value -= number;
        else states.values[index].value = number;
        front->add_command(index, states.values[index], o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
        front->pulse_and_wait();
    }
    else if (std::strchr("xyz", command) != nullptr)
    {
        //Cartesial
        size_t index = franka_o80::cartesial_positions[command - 'x'];
        franka_o80::CartesialStates cartesial = franka_o80::to_cartesial(states);
        if (sign == '+') cartesial.values[index].value += number;
        else if (sign == '-') cartesial.values[index].value -= number;
        else cartesial.values[index].value = number;
        states = franka_o80::to_joint(cartesial);
        for (size_t i = 0; i < 7; i++) front->add_command(franka_o80::robot_positions[i], states.get(franka_o80::robot_positions[i]), o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
        front->pulse_and_wait();
    }
    else if (command == 'g')
    {
        //Gripper
        size_t index = franka_o80::gripper_width;
        if (sign == '+') states.values[index].value += number;
        else if (sign == '-') states.values[index].value -= number;
        else states.values[index].value = number;
        front->add_command(index, states.values[index], o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
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

void set3(franka_o80::FrontEnd *front, std::string input)
{
    //Command
    char *p = &input[0];
    while (*p != '\0' && *p == ' ') p++;
    char command = *p;

    //Numbers
    double numbers[3];
    p++;
    while (*p != '\0' && *p == ' ') p++;
    numbers[0] = strtod(p, &p);
    p++;
    while (*p != '\0' && *p == ' ') p++;
    numbers[1] = strtod(p, &p);
    p++;
    while (*p != '\0' && *p == ' ') p++;
    numbers[2] = strtod(p, &p);

    front->reset_next_index();
    franka_o80::States states = front->wait_for_next().get_observed_states();
    franka_o80::CartesialStates cartesial = franka_o80::to_cartesial(states);
    if (command == 'f')
    {
        //Forward
        for (size_t i = 0; i < 3; i++) cartesial.values[franka_o80::cartesial_forward[i]] = numbers[i];
    }
    else
    {
        //Right
        for (size_t i = 0; i < 3; i++) cartesial.values[franka_o80::cartesial_right[i]] = numbers[i];
    }
    states = franka_o80::to_joint(cartesial);
    for (size_t i = 0; i < 7; i++) front->add_command(franka_o80::robot_positions[i], states.get(franka_o80::robot_positions[i]), o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
    front->pulse_and_wait();
}

int run(int argc, char **argv)
{
    if (argc != 2) { help(); return 1; }

    //Reading ID
    const char *id = argv[1];
    
    //Creating frontend
	franka_o80::FrontEnd frontend(id);
    frontend.add_command(franka_o80::control_positions, 1.0, o80::Duration_us::seconds(execution_time), o80::Mode::QUEUE);
    frontend.pulse_and_wait();
	
    while (true)
    {
        std::cout << "Command: ";
        std::string input;
        std::getline(std::cin, input);
        char *p = &input[0];
        while (*p != '\0' && *p == ' ') p++;
        char command = *p;

        if (std::strchr("1234567gxyzt", command) != nullptr) set1(&frontend, input);
        else if (std::strchr("fr", command) != nullptr) set3(&frontend, input);
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