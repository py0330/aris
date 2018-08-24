#include <iostream>
#include "test_robot_rokae.h"


#include <aris_robot.h>


int main(int argc, char *argv[])
{
	test_robot_rokae();
	

	std::cout << "test_robot finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}