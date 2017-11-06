#include <iostream>
#include <aris_control.h>
#include "test_control_master_slave.h"

using namespace aris::control;

void test_construct()
{
	aris::control::Master m;
	
	m.setControlStrategy([]() 
	{
		static int count{0};
		std::cout << "count:" << ++count << std::endl;
	});
	m.start();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	m.stop();


	std::cout << m.xmlString() << std::endl;
}

void test_control_master_slave()
{
	test_construct();
}
