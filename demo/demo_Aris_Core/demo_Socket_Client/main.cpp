#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <thread>

#include <Platform.h>
#include <Aris_Socket.h>
#include <Aris_Message.h>

#ifdef PLATFORM_IS_LINUX
#include <unistd.h>
#endif

#ifdef PLATFORM_IS_WINDOWS
#include <Windows.h>
#endif

using namespace std;
using namespace Aris::Core;

enum ClientMessage
{
	VisualSystemDataNeeded,
	VisualSystemLost,
	ControlSystemLost,
	ControlCommandReceived,
};

/*以下用于VS内存泄露检测*/
#ifdef PLATFORM_IS_WINDOWS  
#define CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>  
_CrtMemState s1, s2, s3;
#endif
/*内存检测泄露完毕*/


int main()
{
	
	/*以下用于VS内存泄露检测*/
#ifdef PLATFORM_IS_WINDOWS  
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtMemCheckpoint(&s1);
	//_CrtSetBreakAlloc(52);
#endif
	/*内存检测泄露完毕*/

	
	/*设置完毕*/
	{
		/*以下在不同的操作系统下设置不同的服务器IP*/
#ifdef PLATFORM_IS_WINDOWS  
		char RemoteIp[] = "127.0.0.1";
#endif
#ifdef PLATFORM_IS_LINUX  
		char RemoteIp[] = "127.0.0.1";
#endif

		CONN VisualSystem, ControlSystem;

		/*注册所有的消息函数*/
		Aris::Core::RegisterMsgCallback(VisualSystemDataNeeded, [&VisualSystem](Aris::Core::MSG &msg)
		{
			/*只要收到数据，就认为服务器在索取地图数据，于是生成地图并发送过去*/
			static int i = 0;

			i++;

			cout << "Visual data needed" << endl;
			double map[9] = { (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i };

			Aris::Core::MSG data;
			data.Copy(map, sizeof(map));

			VisualSystem.SendData(data);

			return 0;
		});
		Aris::Core::RegisterMsgCallback(VisualSystemLost, [](Aris::Core::MSG &msg)
		{
			cout << "Vision system connection lost" << endl;

			Aris::Core::StopMsgLoop();

			return 0;
		});
		Aris::Core::RegisterMsgCallback(ControlCommandReceived, [&ControlSystem](Aris::Core::MSG &msg)
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
			ControlSystem.SendData(data);
			cout << "end walking" << endl;

			return 0;

		});
		Aris::Core::RegisterMsgCallback(ControlSystemLost, [](Aris::Core::MSG &msg)
		{
			cout << "Control system connection lost" << endl;

			return 0;
		});

		/*设置所有CONN类型的回调函数*/
		VisualSystem.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &data)
		{
			Aris::Core::PostMsg(Aris::Core::MSG(VisualSystemDataNeeded));

			return 0;
		});
		VisualSystem.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
		{
			PostMsg(Aris::Core::MSG(VisualSystemLost));

			return 0;
		});
		ControlSystem.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &data)
		{
			Aris::Core::PostMsg(Aris::Core::MSG(ControlCommandReceived));

			return 0;
		});
		ControlSystem.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
		{
			PostMsg(Aris::Core::MSG(ControlSystemLost));

			return 0;
		});

		/*以下使用lambda函数做回调*/
		VisualSystem.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &data)
		{
			Aris::Core::PostMsg(Aris::Core::MSG(VisualSystemDataNeeded));
			return 0;
		});

		/*连接服务器*/
		VisualSystem.Connect(RemoteIp, "5688");
		ControlSystem.Connect(RemoteIp, "5689");


		/*开始消息循环*/
		Aris::Core::RunMsgLoop();
	}

#ifdef PLATFORM_IS_WINDOWS  
	_CrtMemCheckpoint(&s2);
	if (_CrtMemDifference(&s3, &s1, &s2))
		_CrtMemDumpStatistics(&s3);
	//_CrtDumpMemoryLeaks();
#endif
	
	int a;
	cin >> a;

}
