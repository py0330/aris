#include <iostream>
#include <aris.h>

#include "test_core_xml.h"
#include "test_core_command.h"


int main(int argc, char *argv[])
{
	test_command();

	std::cout << "finished" << std::endl;

	char aaaaa;
	std::cin >> aaaaa;

	return 0;
}