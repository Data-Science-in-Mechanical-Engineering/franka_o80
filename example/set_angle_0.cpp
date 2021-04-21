#include <o80/front_end.hpp>
#include "../include/franka_o80/constants.hpp"
#include "../include/franka_o80/state.hpp"
#include "../include/franka_o80/segment_id.hpp"

void help()
{
	std::cout << "Simpliest example of usage of franka_arm_api" << std::endl;
	std::cout << "Usage: ./set_angle_0 N a" << std::endl;
}

int run(int argc, char **argv)
{
	//Reading N
	if (argc != 2) { help(); return 1; }
	char *end;
	int n = strtol(argv[1], &end, 10);
	if (*end != '\0' || n < 0) { help(); return 1; }

	//Reading a
	double a = strtod(argv[2], &end);
	if (*end != '\0') { help(); return 1; }

	//Creating frontend
	o80::FrontEnd<franka_o80::queue_size, franka_o80::actuator_number, franka_o80::State, o80::VoidExtendedState>frontend(franka_o80::get_segment_id(n));
	
	//Giving command
	frontend.add_command(0, franka_o80::State(a), o80::Duration_us::seconds(1), o80::Mode::QUEUE);

	//Waiting
	frontend.pulse_and_wait();

	return 0;
}

int main(int argc, char **argv)
{
	return run(argc, argv);
}