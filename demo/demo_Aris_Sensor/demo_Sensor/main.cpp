#include <iostream>
#include <iomanip>

#include "aris_core.h"
#include "aris_sensor.h"
#include "aris_dynamic.h"

class SENSOR :public aris::sensor::SensorBase<double>
{
	virtual void updateData(double &data)
	{
		static double sensorData = 0;
		data = sensorData;
		sensorData++;
		aris::core::msSleep(10);
	}
};


#ifdef UNIX
aris::sensor::KINECT kinect;
#endif

int main()
{
	aris::core::XmlDocument doc;
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
		aris::core::msSleep(100);
	}
	
	kinect.stop();
#endif
*/
	/*
	auto p = doc.RootElement()->FirstChildElement("server")->FirstChildElement("Sensors")->FirstChildElement("IMU");
	
	aris::sensor::IMU imu(p);

	imu.start();
	
	for (int i = 0; i < 1000;++i)
	{
		auto data = imu.getSensorData();

		double eul[3];
		//data.get().ToBodyEul(eul);
		//aris::dynamic::dsp(eul, 1, 3);
		
		data.get().toEulBody2Ground(eul, PI, "321");
		aris::dynamic::dsp(eul, 1, 3);

		//double pm[16];
		//data.get().ToBodyPm(pm, 0.0);
		//aris::dynamic::dsp(pm, 4, 4);

		//aris::dynamic::dsp(data.get().eul321, 1, 3);

		aris::core::msSleep(1);
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

	//	aris::core::msSleep(1);
	//}

	//{
	//	auto data = sensor.getSensorData();

	//	std::cout << data.get() << std::endl;
	//}
	//sensor.stop();



	aris::core::MsgRT::instance[0].copy("123");


	char aaa;
	std::cin>>aaa;
	return 0;
}

