#include "Platform.h"

#include <iostream>
#include <iomanip>

#include "Aris_Core.h"
#include "Aris_IMU.h"
#include "Aris_DynKer.h"

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
	Aris::Core::DOCUMENT doc;
#ifdef PLATFORM_IS_WINDOWS
	doc.LoadFile("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	doc.LoadFile("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif

	//auto p = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU");
	
	Aris::Sensor::IMU imu;

	imu.Start();
	
	for (int i = 0; i < 1000;++i)
	{
		auto data = imu.GetSensorData();

		double eul[3];
		//data.Get().ToBodyEul(eul);
		//Aris::DynKer::dsp(eul, 1, 3);
		
		data.Get().ToEulBody2Ground(eul, PI);
		Aris::DynKer::dsp(eul, 1, 3);

		//double pm[16];
		//data.Get().ToBodyPm(pm, 0.0);
		//Aris::DynKer::dsp(pm, 4, 4);

		//Aris::DynKer::dsp(data.Get().eul321, 1, 3);

		Aris::Core::Sleep(1);
	}

	imu.Stop();


	//SENSOR sensor;

	//sensor.Start();

	//for (int i = 0; i < 200;++i)
	//{
	//	
	//	
	//	auto data = sensor.GetSensorData();

	//	//std::cout << data.Get()<< std::endl;

	//	Aris::Core::Sleep(1);
	//}

	//{
	//	auto data = sensor.GetSensorData();

	//	std::cout << data.Get() << std::endl;
	//}
	//sensor.Stop();






	char aaa;
	std::cin>>aaa;
	return 0;
}

