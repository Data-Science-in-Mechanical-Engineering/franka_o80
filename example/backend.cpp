#include "../include/franka_o80/driver.hpp"
#include "../include/franka_o80/standalone.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace franka_o80
{
	void help();
	int start(int argc, char **argv);
	int stop(int argc, char **argv);
	int status(int argc, char **argv);
	int run(int argc, char **argv);
}

void franka_o80::help()
{
	std::cout << "Welcome to franka_o80_backend!" << std::endl;
	std::cout << "The program is created to control o80::Backend's" << std::endl;
	std::cout << "N  is number of backend" << std::endl;
	std::cout << "F  is frequency" << std::endl;
	std::cout << "IP is Franka Arm's IPv4 address" << std::endl;
	std::cout << "Usage:" << std::endl;
	std::cout << "Starts backend daemon:             ./franka_o80_backend daemon N start F IP" << std::endl;
	std::cout << "Stops  backend daemon:             ./franka_o80_backend daemon N stop" << std::endl;
	std::cout << "Checks backend daemon:             ./franka_o80_backend daemon N status" << std::endl;
	std::cout << "Starts backend, stops with Ctrl+C: ./franka_o80_backend N F IP" << std::endl;
}

int franka_o80::start(int argc, char **argv)
{
	//Reading N and F
	char *end;
	int n = strtol(argv[1], &end, 10);
	if (*end != '\0' || n < 0) { help(); return 1; }
	double f = strtod(argv[4], &end);
	if (*end != '\0' || f < 0.0f) { help(); return 1; }

	//Creating daemon
	pid_t pid = fork();
	if (pid < 0) return 1;
	if (pid == 0)
	{
		setsid();
		signal(SIGCHLD, SIG_IGN);
		signal(SIGHUP, SIG_IGN);
		umask(0);
		chdir("/");
		std::cout << "Starting backend" << std::endl;
		for (int x = sysconf(_SC_OPEN_MAX); x >= 0; x--) close (x);
		o80::start_standalone<franka_o80::Driver, franka_o80::Standalone, std::string>(franka_o80::get_segment_id(n), f, false, argv[5]);
	}
	return 0;
}

int franka_o80::stop(int argc, char **argv)
{
	//Reading N
	char *end;
	int n = strtol(argv[1], &end, 10);
	if (*end != '\0' || n < 0) { help(); return 1; }

	//Stopping standalone
	std::cout << "Stopping backend" << std::endl;
	o80::stop_standalone(franka_o80::get_segment_id(n));
	return 0;
}

int franka_o80::status(int argc, char **argv)
{
	//Reading N
	char *end;
	int n = strtol(argv[1], &end, 10);
	if (*end != '\0' || n < 0) { help(); return 1; }
	
	//Reading standalone status
	if (o80::standalone_is_running(franka_o80::get_segment_id(n))) std::cout << "1" <<std::endl;
	else std::cout << "0" <<std::endl;

	return 0;
}

int franka_o80::run(int argc, char **argv)
{
	//Reading N and F
	char *end;
	static int n = strtol(argv[1], &end, 10);
	if (*end != '\0' || n < 0) { help(); return 1; }
	double f = strtod(argv[2], &end);
	if (*end != '\0' || f < 0.0f) { help(); return 1; }
	
	//Starting standalone
	std::cout << "Starting backend" << std::endl;
	o80::start_standalone<franka_o80::Driver, franka_o80::Standalone, std::string>(franka_o80::get_segment_id(n), f, false, argv[3]);
	static bool finish = false;
	signal(SIGINT, [](int sig)
	{
		if (sig == SIGINT)
		{
			std::cout << "Stopping backend" << std::endl;
			o80::stop_standalone(franka_o80::get_segment_id(n));
			finish = true;
		}
	});

	//Main loop
	while (!finish) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	return 0;
}

int main(int argc, char **argv)
{
	// ./franka_o80_backend daemon N start F IP
	if (argc == 6 && std::string(argv[1]) == "daemon" && std::string(argv[3]) == "start") return franka_o80::start(argc, argv);
	// ./franka_o80_backend daemon N stop
	else if (argc == 4 && std::string(argv[1]) == "daemon" && std::string(argv[3]) == "stop") return franka_o80::stop(argc, argv);
	// ./franka_o80_backend daemon N status
	else if (argc == 4 && std::string(argv[1]) == "daemon" && std::string(argv[3]) == "status")	return franka_o80::status(argc, argv);
	// ./franka_o80_backend N F IP
	else if (argc == 4) return franka_o80::run(argc, argv);
	
	franka_o80::help();
	return 1;
}