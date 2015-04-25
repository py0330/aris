#include "Client.h"
#include <iostream>
#include <cstring>

#ifdef PLATFORM_IS_WINDOWS
#include "windows.h"
#endif
#ifdef PLATFORM_IS_LINUX
#include   <unistd.h>
#endif

using namespace std;
using namespace Aris::Core;

CONN *pVisualSystem, *pControlSystem;

/*客户端不存在等待连接的问题，因此没有接到连接的消息和回调函数*/


/*当视觉客户端和服务器失去连接时，执行本函数，主要是跳出消息循环，退出程序*/
int OnVisualSystemLost(Aris::Core::MSG &msg)
{
	cout << "Vision system connection lost" << endl;

	Aris::Core::StopMsgLoop();

	return 0;
}

/*当视觉客户端收到数据时，执行本函数*/
int OnVisualSystemDataNeeded(Aris::Core::MSG &msg)
{
	/*只要收到数据，就认为服务器在索取地图数据，于是生成地图并发送过去*/
	
	static int i=0;
	
	i++;

	cout << "Visual data needed" << endl;
	double map[9] = { (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i };

	Aris::Core::MSG data;
	data.Copy(map, sizeof(map));

	pVisualSystem->SendData(data);

	return 0;
}

/*当控制客户端和服务器失去连接时，执行本函数，主要是跳出消息循环，退出程序*/
int OnControlSystemLost(Aris::Core::MSG &msg)
{
	cout << "Control system connection lost" << endl;

	return 0;
}

/*当控制客户端收到数据时，执行本函数*/
int OnControlCommandReceived(Aris::Core::MSG &msg)
{
	/*只要收到数据，就认为服务器让机器人行动，于是sleep 2秒之后发送回去数据，告诉机器人已经走到位*/
	
	cout << "begin walking" << endl;
#ifdef PLATFORM_IS_WINDOWS
	Sleep(2000);
#endif
#ifdef PLATFORM_IS_LINUX
	usleep(2000000);
#endif
	
	

	Aris::Core::MSG data;
	pControlSystem->SendData(data);
	cout << "end walking" << endl;
	
	return 0;

}

/*分析收到的数据，首先判断是从哪个客户端发来的消息，之后提取数据并发消息给主线程*/
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
	if (pConn == pVisualSystem)
	{
		Aris::Core::PostMsg(Aris::Core::MSG(VisualSystemDataNeeded));
	}
	else if (pConn == pControlSystem)
	{
		Aris::Core::PostMsg(Aris::Core::MSG(ControlCommandReceived));
	}

	return 0;
}

/*分析失去的连接，首先判断是从哪个客户端失去的连接，之后给主线程发送消息*/
int OnConnectionLost(Aris::Core::CONN *pConn)
{
	if (pConn == pVisualSystem)
		PostMsg(Aris::Core::MSG(VisualSystemLost));
	else if (pConn == pControlSystem)
		PostMsg(Aris::Core::MSG(ControlSystemLost));

	return 0;
}


