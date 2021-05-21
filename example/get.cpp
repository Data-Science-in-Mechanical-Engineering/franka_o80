#include <o80/front_end.hpp>
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/segment_id.hpp"
#include "../include/franka_o80/indexes.hpp"

void help()
{
	std::cout << "Gets current joint angle" << std::endl;
    std::cout << "S is segment number"      << std::endl;
    std::cout << "N is actuator number"     << std::endl;
	std::cout << "Usage: ./get S N"         << std::endl;
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

	//Creating frontend
	franka_o80::FrontEnd frontend(franka_o80::get_segment_id(n));

    //Reading states
    auto states = frontend.wait_for_next().get_observed_states();
    std::cout << states.get(franka_o80::robot_positions[n]).get() << std::endl;

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