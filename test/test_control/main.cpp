#include <iostream>
#include <aris_control.h>

#include "test_control_master_slave.h"
#include "test_control_ethercat.h"
#include "test_control_motion.h"


int main(int argc, char *argv[])
{
	test_control_master_slave();
	//test_control_ethercat();
	//test_control_motion();

    std::cout << "test_control finished, press any key to continue" << std::endl;
    std::cin.get();

    return 0;
}
