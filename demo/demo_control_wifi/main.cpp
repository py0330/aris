///// \example demo_command_parser_xml/main.cpp
///// 本例子展示基于xml解析命令字符串的过程，xml文件位于：安装目录/resource/demo_command_parser_xml/command.xml，内容如下：
///// \include demo_command_parser_xml/resource/command.xml
///// 以下为C++源码
/////

#include <iostream>
#include "aris.h"

int main()
{
	//<InverseResult Name = "robot" Modify = "20170813 17:00" Create = "20170813 17:00">
	//<MotionList>
	//	<PathStep motion0 = "-1.60700461670704E-17" motion1 = "0" motion2 = "-3.26542169715915E-16" / >
	aris::core::XmlDocument doc;
	doc.LoadFile("C:\\Users\\py033\\Desktop\\res.xml");

	auto mot_list = doc.RootElement()->FirstChildElement("MotionList");

	std::ofstream f1, f2, f3;
	f1.open("C:\\Users\\py033\\Desktop\\m1.txt");
	f2.open("C:\\Users\\py033\\Desktop\\m2.txt");
	f3.open("C:\\Users\\py033\\Desktop\\m3.txt");

	int num = 0;
	for (auto ele = mot_list->FirstChildElement("PathStep"); ele; ele = ele->NextSiblingElement())
	{
		f1 << ele->Attribute("motion0") << ',';
		f2 << ele->Attribute("motion1") << ',';
		f3 << ele->Attribute("motion2") << ',';
		num++;
	}
	std::cout << "total num:" << num << std::endl;
	
	
	
	
	
	
	//aris::core::Socket socket("socket");

	//socket.setOnReceivedConnection([](aris::core::Socket* sock, const char* remote_ip, int port)->int
	//{
	//	std::cout << "received connection from ip:" << remote_ip << "  port:" << port << std::endl;
	//	return 0;
	//});
	//socket.setOnReceivedMsg([](aris::core::Socket* sock, aris::core::Msg &msg)->int
	//{
	//	std::cout << "received size:" << msg.size() << "  type:" << msg.type() << "  id:" << msg.msgID() << std::endl;
	//	return 0;
	//});
	//socket.startServer("8080");


	//for (;;)
	//{

	//}


	//aris::core::Msg msg;
	//msg.setMsgID(0);
	//msg.resize(0);
	//socket.sendMsg(msg);
	
	std::cout << "demo_command_parser_xml finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

