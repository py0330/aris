#include <iostream>
#include <aris.h>

#include "test_control_server.h"

int main(int argc, char *argv[])
{
	test_control_server();

	std::cout << "test_control_server finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}