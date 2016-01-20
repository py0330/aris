﻿
#include <aris_motion.h>
#include <iostream>

#ifdef UNIX
#include "ecrt.h"
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

#ifdef UNIX


	auto ele = doc.RootElement()->FirstChildElement("Server")
		->FirstChildElement("Control")->FirstChildElement("EtherCat");

	auto pMas = Aris::Control::EthercatMaster::CreateMaster<Aris::Control::EthercatController>();
	std::cout<<"1"<<std::endl;	
	pMas->LoadXml(ele);
	std::cout<<"2"<<std::endl;
	pMas->Start();
	std::cout<<"3"<<std::endl;
	
	while (true)
	{
		Aris::Core::Msg msg;
		pMas->MsgPipe().RecvInNRT(msg);
		std::cout << "NRT msg length:" << msg.GetLength()<<" pos:" << *reinterpret_cast<std::int32_t*>(msg.GetDataAddress())<<std::endl;
		//msg.SetLength(10);
		msg.Copy("congratulations\n");
		//pMas->MsgPipe().SendToRT(msg);
	}
	
	
#endif
	
	char a;
	std::cin>>a;

	return 0;
}