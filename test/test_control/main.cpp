#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"



int main(int argc, char *argv[])
{
	test_control_ethercat();

	std::cout << "finished" << std::endl;

	char aaaaa;
	std::cin >> aaaaa;

	return 0;
}