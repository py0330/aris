#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"
#include "test_control_motion.h"


int main(int argc, char *argv[])
{
    if (argc !=2)throw std::runtime_error("please input the cmd name 'ethercat' or 'motion' for testing the function");

    std::string cmd{argv[1]};
    if(cmd == "ethercat"){
        test_control_ethercat();
    }
    else if(cmd == "motion"){
        test_control_motion();
    }
    else
        throw std::runtime_error("please input the cmd name 'ethercat' or 'motion' for testing the function");

    std::cout << "test_control finished, press any key to continue" << std::endl;
    std::cin.get();

    return 0;
}
