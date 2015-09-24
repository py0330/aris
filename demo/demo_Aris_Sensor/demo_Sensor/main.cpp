#include <iostream>
#include <iomanip>

#include "Aris_Core.h"
#include "Aris_IMU.h"


class SENSOR :public Aris::Sensor::SENSOR_BASE<double>
{
	virtual void UpdateData(double &data)
	{
		static double sensorData = 0;
		data = sensorData;
		sensorData++;
		Aris::Core::Sleep(10);
	}
};


int main()
{
	//Aris::Sensor::IMU imu;

	//imu.Start();

	//while (true)
	//{
	//	auto data = imu.GetSensorData();

	//	std::cout << data.Get().a << "    " << data.Get().b <<"    "<< data.Get().c << std::endl;

	//	Aris::Core::Sleep(1);
	//}

	SENSOR sensor;

	sensor.Start();

	for (int i = 0; i < 200;++i)
	{
		
		
		auto data = sensor.GetSensorData();

		//std::cout << data.Get()<< std::endl;

		Aris::Core::Sleep(1);
	}

	{
		auto data = sensor.GetSensorData();

		std::cout << data.Get() << std::endl;
	}
	sensor.Stop();






	char aaa;
	std::cin>>aaa;
	return 0;
}

