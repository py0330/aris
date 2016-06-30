#include <iostream>
#include <aris.h>

#include "test_core_xml.h"
#include "test_core_command.h"


int main(int argc, char *argv[])
{
	test_command();

	std::cout << "test_core finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}