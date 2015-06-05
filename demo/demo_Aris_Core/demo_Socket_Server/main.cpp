#include <iostream>
#include <iomanip> 
#include <cstring>

#include <Platform.h>
#include <Aris_Socket.h>
#include <Aris_Message.h>

enum ClientMessage
{
	VisualSystemConnected,
	VisualSystemDataReceived,
	VisualSystemLost,
	ControlSystemConnected,
	ControlTrajectoryFinished,
	ControlSystemLost,
};

using namespace std;


#ifdef PLATFORM_IS_LINUX
#include <unistd.h>
#endif

/*以下用于VS内存泄露检测*/
#ifdef PLATFORM_IS_WINDOWS  
#define CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>  
_CrtMemState s1, s2, s3;
#endif
/*内存检测泄露完毕*/


Aris::Core::CONN VisualSystem, ControlSystem;

int main()
{
	
	/*以下用于VS内存泄露检测*/
#ifdef PLATFORM_IS_WINDOWS  
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtMemCheckpoint(&s1);
	//_CrtSetBreakAlloc(203);
#endif

	
	/*内存检测泄露完毕*/
	{	
		/*注册所有的消息函数*/
		Aris::Core::RegisterMsgCallback(VisualSystemConnected, [](Aris::Core::MSG &msg)
		{
			cout << "Received connection from vision system:" << endl;
			cout << "    Remote IP is: " << msg.GetDataAddress() + 4 << endl;
			cout << "    Port is     : " << *((int*)msg.GetDataAddress()) << endl << endl;

			/*像视觉客户端发送数据，索要地图*/
			Aris::Core::MSG data(0, 0);
			VisualSystem.SendData(data);

			return 0;
		});
		Aris::Core::RegisterMsgCallback(VisualSystemDataReceived, [](Aris::Core::MSG &msg)
		{
			cout << "Received data from vision system:" << endl;

			double map[9];
			msg.Paste(map, sizeof(map));

			for (int i = 0; i < 9; ++i)
			{
				cout << "    " << map[i] << endl;
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
		});
		Aris::Core::RegisterMsgCallback(ControlSystemConnected, [](Aris::Core::MSG &msg)
		{
			cout << "Received connection from control system:" << endl;
			cout << "    Remote IP is: " << msg.GetDataAddress() + 4 << endl;
			cout << "    Port is     : " << *((int*)msg.GetDataAddress()) << endl << endl;

			return 0;
		});
		Aris::Core::RegisterMsgCallback(ControlTrajectoryFinished, [](Aris::Core::MSG &msg)
		{
			/*只要收到数据就认为控制客户端已经走完对应的轨迹*/
			cout << "Robot walk finished, Now ask for map" << endl;
			Aris::Core::MSG data;
			VisualSystem.SendData(data);


			return 0;
		});

		/*设置所有CONN类型的回调函数*/
		VisualSystem.SetCallBackOnReceivedConnection([](Aris::Core::CONN *pConn, const char *addr, int port)
		{
			Aris::Core::MSG msg;

			msg.SetMsgID(VisualSystemConnected);
			msg.Copy(&port, sizeof(int));
			msg.CopyMore(addr, strlen(addr) + 1);

			PostMsg(msg);

			return 0;
		});
		VisualSystem.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
		{
			msg.SetMsgID(VisualSystemDataReceived);
			Aris::Core::PostMsg(msg);
			return 0;
		});
		VisualSystem.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
		{
			cout << "Vision system connection lost" << endl;
			return pConn->StartServer("5688");
		});
		ControlSystem.SetCallBackOnReceivedConnection([](Aris::Core::CONN *pConn, const char *addr, int port)
		{
			Aris::Core::MSG msg;
			
			msg.SetMsgID(ControlSystemConnected);
			msg.Copy(&port, sizeof(int));
			msg.CopyMore(addr, strlen(addr) + 1);

			PostMsg(msg);

			return 0;
		});
		ControlSystem.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
		{
			msg.SetMsgID(ControlTrajectoryFinished);
			Aris::Core::PostMsg(msg);
			return 0;
		});
		ControlSystem.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
		{
			cout << "Control system connection lost" << endl;
			return pConn->StartServer("5689");
		});
		
		/*打开客户端*/
		VisualSystem.StartServer("5688");
		ControlSystem.StartServer("5689");

		/*开始消息循环*/
		Aris::Core::RunMsgLoop();
	}

#ifdef PLATFORM_IS_WINDOWS  
	_CrtMemCheckpoint(&s2);
	if (_CrtMemDifference(&s3, &s1, &s2))
		_CrtMemDumpStatistics(&s3);
	//_CrtDumpMemoryLeaks();
#endif
	
	
}