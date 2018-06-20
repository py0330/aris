#include <iostream>
#include <aris_core.h>
#include <aris_plan.h>

#include "test_plan_command.h"
#include "test_plan_function.h"

int main(int argc, char *argv[])
{
	test_command();
	test_function();

	std::cout << "test_core finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}