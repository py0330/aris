#include <iostream>
#include <iomanip>

#include "aris_core.h"
#include "aris_sensor.h"
#include "aris_dynamic.h"

int main()
{
	try
	{
		aris::sensor::SensorRoot sensor_root;

		sensor_root.start();

		auto &sensor = sensor_root.sensorPool().front();
		for (int i = 0; i < 1000; ++i)
		{
			aris::core::msSleep(100);
			
			auto data_protector = sensor_root.sensorPool().front().dataProtector();
		}

		sensor_root.stop();
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
	
	char aaa;
	std::cin>>aaa;
	return 0;
}

