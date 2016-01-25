#include <iostream>
#include <iomanip>

#include "aris_core.h"
#include "aris_sensor.h"
#include "aris_dynamic.h"

class SENSOR :public Aris::Sensor::SensorBase<double>
{
	virtual void updateData(double &data)
	{
		static double sensorData = 0;
		data = sensorData;
		sensorData++;
		Aris::Core::msSleep(10);
	}
};


#ifdef UNIX
Aris::Sensor::KINECT kinect;
#endif

int main()
{
	Aris::Core::XmlDocument doc;
#ifdef WIN32
	doc.LoadFile("C:\\Robots\\resource\\Robot_Type_I\\Robot_III.xml");
#endif
#ifdef UNIX
	doc.LoadFile("/usr/Robots/resource/Robot_Type_I/Robot_III.xml");
#endif

/*
#ifdef UNIX
	kinect.start();

	for (int i = 0; i < 1000; ++i)
	{
		auto data = kinect.getSensorData();
		std::cout<<"data:"<<data.get().gridMap[100][100]<<std::endl;
		Aris::Core::msSleep(100);
	}
	
	kinect.stop();
#endif
*/
	/*
	auto p = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU");
	
	Aris::Sensor::IMU imu(p);

	imu.start();
	
	for (int i = 0; i < 1000;++i)
	{
		auto data = imu.getSensorData();

		double eul[3];
		//data.get().ToBodyEul(eul);
		//Aris::Dynamic::dsp(eul, 1, 3);
		
		data.get().toEulBody2Ground(eul, PI, "321");
		Aris::Dynamic::dsp(eul, 1, 3);

		//double pm[16];
		//data.get().ToBodyPm(pm, 0.0);
		//Aris::Dynamic::dsp(pm, 4, 4);

		//Aris::Dynamic::dsp(data.get().eul321, 1, 3);

		Aris::Core::msSleep(1);
	}

	imu.stop();
	*/

	//SENSOR sensor;

	//sensor.start();

	//for (int i = 0; i < 200;++i)
	//{
	//	
	//	
	//	auto data = sensor.getSensorData();

	//	//std::cout << data.get()<< std::endl;

	//	Aris::Core::msSleep(1);
	//}

	//{
	//	auto data = sensor.getSensorData();

	//	std::cout << data.get() << std::endl;
	//}
	//sensor.stop();



	Aris::Core::MsgRT::instance[0].copy("123");


	char aaa;
	std::cin>>aaa;
	return 0;
}

