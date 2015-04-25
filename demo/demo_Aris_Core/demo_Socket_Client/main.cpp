#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>

#include "Platform.h"
#include "Aris_Socket.h"
#include "Client.h"

using namespace std;
using namespace Aris::Core;


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
		pVisualSystem = &VisualSystem;
		pControlSystem = &ControlSystem;


		/*注册所有的消息函数*/
		Aris::Core::RegisterMsgCallback(VisualSystemDataNeeded, OnVisualSystemDataNeeded);
		Aris::Core::RegisterMsgCallback(VisualSystemLost, OnVisualSystemLost);
		Aris::Core::RegisterMsgCallback(ControlCommandReceived, OnControlCommandReceived);
		Aris::Core::RegisterMsgCallback(ControlSystemLost, OnControlSystemLost);

		/*设置所有CONN类型的回调函数*/
		VisualSystem.SetCallBackOnReceivedData(OnConnDataReceived);
		VisualSystem.SetCallBackOnLoseConnection(OnConnectionLost);
		ControlSystem.SetCallBackOnReceivedData(OnConnDataReceived);
		ControlSystem.SetCallBackOnLoseConnection(OnConnectionLost);

		/*以下使用lambda函数做回调*/
		VisualSystem.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &data)
		{
			cout << "using new lambda callback" << endl;
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
