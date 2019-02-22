#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>

#include "test_plan_function.h"

int main(int argc, char *argv[])
{
	test_function();



	std::cout << "test_core finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}