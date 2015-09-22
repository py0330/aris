#include <iostream>
#include <iomanip>

#include "Aris_Core.h"
#include "Aris_IMU.h"

int main()
{
	Aris::Sensor::IMU imu;

	imu.Start();

	while (true)
	{
		auto data = imu.GetSensorData();

		std::cout << data.Get().a << "    " << data.Get().b <<"    "<< data.Get().c << std::endl;

		Aris::Core::Sleep(1);
	}








	char aaa;
	std::cin>>aaa;
	return 0;
}

