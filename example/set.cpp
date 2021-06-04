#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/indexes.hpp"
#include <thread>

void help()
{
	std::cout << "Sets joint angle"          << std::endl;
    std::cout << "ID is segment identifier"  << std::endl;
    std::cout << "N  is actuator number"     << std::endl;
    std::cout << "A  is angle"               << std::endl;
    std::cout << "T  is time in seconds"     << std::endl;
	std::cout << "Usage: ./set ID N A T"      << std::endl;
}

int run(int argc, char **argv)
{
    if (argc != 5) { help(); return 1; }

    //Reading ID
    const char *id = argv[1];

    //Reading N
	char *end;
	int n = strtol(argv[2], &end, 10);
	if (*end != '\0' || n < 0 || n > 7) { help(); return 1; }

	//Reading A
	double a = strtod(argv[3], &end);
	if (*end != '\0' || a != a || a < franka_o80::robot_positions_min[n] || a > franka_o80::robot_positions_max[n]) { help(); return 1; }

    //Reading T
	int t = strtol(argv[4], &end, 10);
	if (*end != '\0' || t < 1) { help(); return 1; }

	//Creating frontend
	franka_o80::FrontEnd frontend(id);
	
	//Giving command
    frontend.add_command(franka_o80::control_positions, franka_o80::State(1.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    frontend.add_command(franka_o80::control_velocities, franka_o80::State(0.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    frontend.add_command(franka_o80::control_torques, franka_o80::State(0.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE);
    frontend.pulse_and_wait();
	
    frontend.add_command(franka_o80::robot_positions[n], franka_o80::State(a), o80::Duration_us::seconds(t), o80::Mode::QUEUE);
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