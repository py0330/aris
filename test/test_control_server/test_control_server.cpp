#include <iostream>
#include <aris.h>

#include "test_control_server.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

void test_control_server()
{
	auto &cs = aris::server::ControlServer::instance();
	try 
	{
		cs.createModel<aris::dynamic::Model>();
		cs.createController<aris::control::Controller>();
		cs.createSensorRoot<aris::sensor::SensorRoot>();
		
		cs.sensorRoot().registerChildType<aris::sensor::Imu,false,false,false,false>();

		cs.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
	}
	catch (std::exception &e)
	{
		std::cout << e.what()<<std::endl;
	}
}