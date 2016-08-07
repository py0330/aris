#include <iostream>
#include <aris.h>

#include "test_core_xml.h"


using namespace aris::core;

const double error = 1e-10;

void test_core_xml()
{
	std::cout << aris::core::logExeName() << std::endl;
	std::cout << aris::core::logFileName() << std::endl;
	std::cout << aris::core::logDirPath() << std::endl;
	std::cout << aris::core::logFileTimeFormat(std::chrono::system_clock::now())<<std::endl;

}