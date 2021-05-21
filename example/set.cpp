#include <o80/front_end.hpp>
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/segment_id.hpp"
#include "../include/franka_o80/indexes.hpp"

void help()
{
	std::cout << "Sets joint angle"         << std::endl;
    std::cout << "S is segment number"      << std::endl;
    std::cout << "N is actuator number"     << std::endl;
    std::cout << "A is angle"               << std::endl;
    std::cout << "T is time in seconds"     << std::endl;
	std::cout << "Usage: ./set S N A T"     << std::endl;
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
    frontend.add_command(franka_o80::control_positions, franka_o80::State(1.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    frontend.add_command(franka_o80::control_velocities, franka_o80::State(0.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    frontend.add_command(franka_o80::control_torques, franka_o80::State(0.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    for (size_t i = 0; i < 2000; i++) frontend.wait_for_next();
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