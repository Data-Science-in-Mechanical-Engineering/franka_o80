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
	std::cout << "ID is identifier of backend" << std::endl;
	std::cout << "IP is Franka Arm's IPv4 address" << std::endl;
	std::cout << "Usage:" << std::endl;
	std::cout << "Starts backend daemon:             ./franka_o80_backend daemon ID start IP" << std::endl;
	std::cout << "Stops  backend daemon:             ./franka_o80_backend daemon ID stop" << std::endl;
	std::cout << "Checks backend daemon:             ./franka_o80_backend daemon ID status" << std::endl;
	std::cout << "Starts backend, stops with Ctrl+C: ./franka_o80_backend ID IP" << std::endl;
}

int franka_o80::start(int argc, char **argv)
{
	//Reading ID
	const char *id = argv[2];

    //Reading IP
    const char *ip = argv[4];

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
		franka_o80::start_standalone(id, ip);
	}
	return 0;
}

int franka_o80::stop(int argc, char **argv)
{
	//Reading ID
	const char *id = argv[1];

	//Stopping standalone
	std::cout << "Stopping backend" << std::endl;
	franka_o80::stop_standalone(id);
	return 0;
}

int franka_o80::status(int argc, char **argv)
{
	//Reading ID
	const char *id = argv[1];
	
	//Reading standalone status
	if (franka_o80::standalone_is_running(id)) std::cout << "1" <<std::endl;
	else std::cout << "0" <<std::endl;

	return 0;
}

int franka_o80::run(int argc, char **argv)
{
	//Reading ID
	static const char *id = argv[1];

    //Reading IP
    static const char *ip = argv[2];
	
	//Starting standalone
	std::cout << "Starting backend" << std::endl;
	franka_o80::start_standalone(id, ip);
	static bool finish = false;
	signal(SIGINT, [](int sig)
	{
		if (sig == SIGINT)
		{
			std::cout << "Stopping backend" << std::endl;
			franka_o80::stop_standalone(id);
			finish = true;
		}
	});

	//Main loop
	while (!finish) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	return 0;
}

int main(int argc, char **argv)
{
	try
	{
		// ./franka_o80_backend daemon ID start IP
		if (argc == 5 && std::string(argv[1]) == "daemon" && std::string(argv[3]) == "start") return franka_o80::start(argc, argv);
		// ./franka_o80_backend daemon ID stop
		else if (argc == 4 && std::string(argv[1]) == "daemon" && std::string(argv[3]) == "stop") return franka_o80::stop(argc, argv);
		// ./franka_o80_backend daemon ID status
		else if (argc == 4 && std::string(argv[1]) == "daemon" && std::string(argv[3]) == "status")	return franka_o80::status(argc, argv);
		// ./franka_o80_backend ID IP
		else if (argc == 3) return franka_o80::run(argc, argv);
		// ./franka_o80_backend ...
		else franka_o80::help();
	}
	catch (std::exception &e)
	{
		std::cout << "Exception occured: " << e.what() << std::endl;
		std::cout << "Terminating..." << std::endl;
	}
	return 1;
}