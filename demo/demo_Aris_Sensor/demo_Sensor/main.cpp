

#include <iostream>
#include <iomanip>

#include "aris_core.h"
#include "aris_imu.h"
#include "aris_dyn_kernel.h"
#ifdef UNIX
#include "aris_vision.h"
#endif

class SENSOR :public Aris::Sensor::SensorBase<double>
{
	virtual void UpdateData(double &data)
	{
		static double sensorData = 0;
		data = sensorData;
		sensorData++;
		Aris::Core::Sleep(10);
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
	kinect.Start();

	for (int i = 0; i < 1000; ++i)
	{
		auto data = kinect.GetSensorData();
		std::cout<<"data:"<<data.Get().gridMap[100][100]<<std::endl;
		Aris::Core::Sleep(100);
	}
	
	kinect.Stop();
#endif
*/
	/*
	auto p = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU");
	
	Aris::Sensor::IMU imu(p);

	imu.Start();
	
	for (int i = 0; i < 1000;++i)
	{
		auto data = imu.GetSensorData();

		double eul[3];
		//data.Get().ToBodyEul(eul);
		//Aris::DynKer::dsp(eul, 1, 3);
		
		data.Get().ToEulBody2Ground(eul, PI, "321");
		Aris::DynKer::dsp(eul, 1, 3);

		//double pm[16];
		//data.Get().ToBodyPm(pm, 0.0);
		//Aris::DynKer::dsp(pm, 4, 4);

		//Aris::DynKer::dsp(data.Get().eul321, 1, 3);

		Aris::Core::Sleep(1);
	}

	imu.Stop();
	*/

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



	Aris::Core::MsgRT::instance[0].Copy("123");


	char aaa;
	std::cin>>aaa;
	return 0;
}

