#include "Server.h"
#include "iostream"
#include <cstring>
using namespace std;

using namespace Aris::Core;

Aris::Core::CONN VisualSystem, ControlSystem;


/*当和视觉客户端建立连接时，执行本函数*/
int OnVisualSystemConnected(Aris::Core::MSG &msg)
{
	cout << "Received connection from vision system:" << endl;
	cout << "    Remote IP is: " << msg.GetDataAddress()+4 << endl;
	cout << "    Port is     : " << *((int*)msg.GetDataAddress()) << endl << endl;

	/*像视觉客户端发送数据，索要地图*/
	Aris::Core::MSG data(0,0);
	VisualSystem.SendData(data);

	return 0;
}

/*当从视觉客户端收到地图时，执行本函数*/
int OnVisualSystemDataReceived(Aris::Core::MSG &msg)
{
	cout << "Received data from vision system:" << endl;

	double map[9];
	msg.Paste(map, sizeof(map));

	for (int i = 0; i < 9; ++i)
	{
		cout <<"    "<< map[i] << endl;
	}


	/*询问用户该地图是否满足需求*/
	cout << "Is map OK?" << endl;
	
	char answer[16];
	
	while (1)
	{
		cin >> answer;

		
		if (strcmp(answer, "yes") == 0)
		{
			Aris::Core::MSG data;
			ControlSystem.SendData(data);
			cout << "command robot to walk" << endl;
			break;
		}
		else if (strcmp(answer, "no") == 0)
		{
			Aris::Core::MSG data;
			VisualSystem.SendData(data);
			break;
		}

	}

	
	



	return 0;
}

/*当从视觉客户端失去连接时，执行本函数，主要是跳出消息循环，退出程序*/
int OnVisualSystemLost(Aris::Core::MSG &msg)
{
	cout << "Vision system connection lost" << endl;

	int res=VisualSystem.StartServer("5688");
	cout<<"err num is:"<<res<<endl;

	return 0;
}

/*当和控制客户端建立连接时，执行本函数*/
int OnControlSystemConnected(Aris::Core::MSG &msg)
{
	cout << "Received connection from vision system:" << endl;
	cout << "    Remote IP is: " << msg.GetDataAddress()+4 << endl;
	cout << "    Port is     : " << *((int*)msg.GetDataAddress()) << endl << endl;

	return 0;
}

/*当从控制客户端收到数据时，执行本函数*/
int OnControlTrajectoryFinished(Aris::Core::MSG &msg)
{
	/*只要收到数据就认为控制客户端已经走完对应的轨迹*/
	cout << "Robot walk finished, Now ask for map" << endl;
	Aris::Core::MSG data;
	VisualSystem.SendData(data);


	return 0;
}

///*当和控制客户端失去连接时，执行本函数*/
int OnControlSystemLost(Aris::Core::MSG &msg)
{
	cout << "Control system connection lost" << endl;

	int res=ControlSystem.StartServer("5689");
	cout<<"err num is:"<<res<<endl;

	return 0;
}







/*分析收到的数据，首先判断是从哪个客户端发来的消息，之后提取数据并发消息给主线程*/
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
{
	if (pConn == &VisualSystem)
	{
		msg.SetMsgID(VisualSystemDataReceived);
		Aris::Core::PostMsg(msg);
	}
	else if (pConn == &ControlSystem)
	{
		msg.SetMsgID(ControlTrajectoryFinished);
		Aris::Core::PostMsg(msg);
	}
	
	
	return 0;
}
/*分析收到的连接，首先判断是从哪个客户端建立的连接，之后给主线程发送消息*/
int OnConnectionReceived(Aris::Core::CONN *pConn, const char *addr, int port)
{
	Aris::Core::MSG msg;
	
	if (pConn == &VisualSystem)
	{
		msg.SetMsgID( VisualSystemConnected);
		msg.Copy(&port,sizeof(int));
		msg.CopyMore(addr, strlen(addr)+1);
		
		PostMsg(msg);
	}
	else if (pConn == &ControlSystem)
	{
		msg.SetMsgID(ControlSystemConnected);
		msg.Copy(&port, sizeof(int));
		msg.CopyMore(addr, strlen(addr)+1);

		PostMsg(msg);
	}

	return 0;
}
/*分析失去的连接，首先判断是从哪个客户端失去的连接，之后给主线程发送消息*/
int OnConnectionLost(Aris::Core::CONN *pConn)
{
	if (pConn == &VisualSystem)
		PostMsg(Aris::Core::MSG(VisualSystemLost));
	else if (pConn == &ControlSystem)
		PostMsg(Aris::Core::MSG(ControlSystemLost));

	return 0;
}
