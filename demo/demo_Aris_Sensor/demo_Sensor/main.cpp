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
		sensor_root.registerChildType<aris::sensor::Imu, false, false, false, false>();

#ifdef WIN32
		sensor_root.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
		sensor_root.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

		sensor_root.start();

		auto &sensor = sensor_root.sensorPool().front();
		for (int i = 0; i < 1000; ++i)
		{
			aris::core::msSleep(100);
			
			auto data_protector = sensor_root.sensorPool().front().dataProtector();

			double eul[3];

			auto imu = static_cast<const aris::sensor::ImuData &>(data_protector.data());

			aris::dynamic::dsp(imu.acc, 1, 3);

			//std::cout << i << std::endl;

			
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

