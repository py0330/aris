#include "Platform.h"
#include <Aris_Motion.h>
#include <iostream>

#ifdef PLATFORM_IS_LINUX
#include "ecrt.h"
#endif

int main()
{
	Aris::Core::DOCUMENT doc;
#ifdef PLATFORM_IS_WINDOWS
	doc.LoadFile("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	doc.LoadFile("/usr/Robots/resource/Robot_Type_I/Robot_VIII.xml");
#endif

#ifdef PLATFORM_IS_LINUX


	auto ele = doc.RootElement()->FirstChildElement("Server")
		->FirstChildElement("Control")->FirstChildElement("EtherCat");

	auto pMas = Aris::Control::ETHERCAT_MASTER::CreateMaster<Aris::Control::CONTROLLER>();
	std::cout<<"1"<<std::endl;	
	pMas->LoadXml(ele);
	std::cout<<"2"<<std::endl;
	pMas->Run();
	std::cout<<"3"<<std::endl;
	
	while (true)
	{
		Aris::Core::MSG msg;
		pMas->MsgPipe().RecvInNRT(msg);
		std::cout << "NRT msg length:" << msg.GetLength()<<" pos:" << *reinterpret_cast<std::int32_t*>(msg.GetDataAddress())<<std::endl;
		//msg.SetLength(10);
		msg.Copy("congratulations\n");
		pMas->MsgPipe().SendToRT(msg);
	}
	
	
#endif
	
	char a;
	std::cin>>a;

	return 0;
}
