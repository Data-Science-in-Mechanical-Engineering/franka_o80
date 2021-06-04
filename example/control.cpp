#include "../include/franka_o80/cartesial_states.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/indexes.hpp"
#include <string.h>

void help()
{
	std::cout << "Enters control loop"              << std::endl;
    std::cout << "ID is segment identifier"         << std::endl;
    std::cout << "IP is IP address of new backend"  << std::endl;
	std::cout << "Usage: ./set ID [IP]"             << std::endl;
}

void echo(franka_o80::FrontEnd *front)
{
    franka_o80::States states = front->wait_for_next().get_observed_states();
    for (size_t i = 0; i < 7; i++)
    {
        std::cout << "Joint " << i + 1 << ": " << states.get(franka_o80::robot_positions[i]).get() << std::endl;
    }
    
    franka_o80::CartesialStates cartesial = franka_o80::to_cartesial(states);
    std::cout << "Position:";
    for (size_t i = 0; i < 3; i++) std::cout << " " << cartesial.get(franka_o80::cartesial_positions[i]).get();
    std::cout << std::endl;
    std::cout << "Forward:";
    for (size_t i = 0; i < 3; i++) std::cout << " " << cartesial.get(franka_o80::cartesial_forward[i]).get();
    std::cout << std::endl;
    std::cout << "Right:";
    for (size_t i = 0; i < 3; i++) std::cout << " " << cartesial.get(franka_o80::cartesial_right[i]).get();
    std::cout << std::endl;
    
    std::cout << "Gripper width: " << states.get(franka_o80::gripper_width) << std::endl;
    std::cout << "Gripper temperature: " << states.get(franka_o80::gripper_temperature) << std::endl;
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

    franka_o80::States states = front->wait_for_next().get_observed_states();
    if (std::strchr("1234567", command)  != nullptr)
    {
        //Joints
        size_t index = franka_o80::robot_positions[command - '1'];
        if (sign == '+') states.values[index].value += number;
        else if (sign == '-') states.values[index].value -= number;
        else states.values[index].value = number;
        front->add_command(index, states.values[index], o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    }
    else if (std::strchr("g", command) != nullptr)
    {
        //Gripper
        size_t index = franka_o80::gripper_width;
        if (sign == '+') states.values[index].value += number;
        else if (sign == '-') states.values[index].value -= number;
        else states.values[index].value = number;
        front->add_command(index, states.values[index], o80::Duration_us::seconds(1), o80::Mode::QUEUE);
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
        for (size_t i = 0; i < 7; i++) front->add_command(franka_o80::robot_positions[i], states.get(franka_o80::robot_positions[i]), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
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
    for (size_t i = 0; i < 7; i++) front->add_command(franka_o80::robot_positions[i], states.get(franka_o80::robot_positions[i]), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
}

int run(int argc, char **argv)
{
    if (argc != 2 && argc != 3) { help(); return 1; }

    //Reading ID
    const char *id = argv[1];

    //Reading IP
    if (argc == 3)
    {
        const char *ip = argv[2];
        franka_o80::start_standalone(id, ip);
    }
    
    //Creating frontend
	franka_o80::FrontEnd frontend(id);
    frontend.add_command(franka_o80::control_positions, 1.0, o80::Duration_us::seconds(1), o80::Mode::QUEUE);
	
    while (true)
    {
        std::cout << "Command: ";
        std::string input;
        std::getline(std::cin, input);
        char *p = &input[0];
        while (*p != '\0' && *p == ' ') p++;
        char command = *p;

        if (std::strchr("1234567gxyz", command) != nullptr) set1(&frontend, input);
        else if (std::strchr("fr", command) != nullptr) set3(&frontend, input);
        else if (std::strchr("e", command) != nullptr) echo(&frontend);
        else if (command == 'q') break;
        else help();
    }

	//Waiting
	frontend.pulse_and_wait();

    //Stop if was started
    if (argc == 3) franka_o80::stop_standalone(id);

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