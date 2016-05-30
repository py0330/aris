#include <iostream>
#include <aris.h>

#include "test_control_server.h"



int main(int argc, char *argv[])
{
	test_control_server();

	std::cout << "test control server finished" << std::endl;

	char aaaaa;
	std::cin >> aaaaa;

	return 0;
}