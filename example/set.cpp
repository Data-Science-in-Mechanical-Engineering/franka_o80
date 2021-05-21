#include <o80/front_end.hpp>
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/segment_id.hpp"
#include "../include/franka_o80/indexes.hpp"

void help()
{
	std::cout << "Simpliest example of usage of franka_arm_api" << std::endl;
    std::cout << "S is segment number"                          << std::endl;
    std::cout << "N is actuator number"                         << std::endl;
    std::cout << "A is angle"                                   << std::endl;
    std::cout << "T is time in seconds"                         << std::endl;
	std::cout << "Usage: ./set_angle_0 S N A T"                 << std::endl;
}

int run(int argc, char **argv)
{
    //Reading S
	if (argc != 5) { help(); return 1; }
	char *end;
	int s = strtol(argv[1], &end, 10);
	if (*end != '\0' || s < 0) { help(); return 1; }

	//Reading N
	int n = strtol(argv[2], &end, 10);
	if (*end != '\0' || n < 0 || n > 7) { help(); return 1; }

	//Reading A
	double a = strtod(argv[3], &end);
	if (*end != '\0' || a != a || a < franka_o80::robot_positions_min[n] || a > franka_o80::robot_positions_max[n]) { help(); return 1; }

    //Reading T
	int t = strtol(argv[4], &end, 10);
	if (*end != '\0' || t < 1) { help(); return 1; }

	//Creating frontend
	franka_o80::FrontEnd frontend(franka_o80::get_segment_id(n));
	
	//Giving command
	frontend.add_command(franka_o80::robot_positions[n], franka_o80::State(a), o80::Duration_us::seconds(t), o80::Mode::QUEUE);

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