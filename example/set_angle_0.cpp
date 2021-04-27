#include <o80/front_end.hpp>
#include "../include/franka_o80/front_end.hpp"
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
	franka_o80::FrontEnd frontend(franka_o80::get_segment_id(n));
	
	//Giving command
	franka_o80::State state(a);
	frontend.add_command(0, state, o80::Duration_us::seconds(1), o80::Mode::QUEUE);

	//Waiting
	frontend.pulse_and_wait();

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