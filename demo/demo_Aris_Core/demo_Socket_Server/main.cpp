#include <iostream>
#include <iomanip> 
#include <cstring>

#include "Server.h"

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
		Aris::Core::RegisterMsgCallback(VisualSystemConnected, OnVisualSystemConnected);
		Aris::Core::RegisterMsgCallback(VisualSystemDataReceived, OnVisualSystemDataReceived);
		Aris::Core::RegisterMsgCallback(VisualSystemLost, OnVisualSystemLost);
		Aris::Core::RegisterMsgCallback(ControlSystemConnected, OnControlSystemConnected);
		Aris::Core::RegisterMsgCallback(ControlTrajectoryFinished, OnControlTrajectoryFinished);
		Aris::Core::RegisterMsgCallback(ControlSystemLost, OnControlSystemLost);

		/*设置所有CONN类型的回调函数*/
		VisualSystem.SetCallBackOnReceivedConnection(OnConnectionReceived);
		VisualSystem.SetCallBackOnReceivedData(OnConnDataReceived);
		VisualSystem.SetCallBackOnLoseConnection(OnConnectionLost);
		ControlSystem.SetCallBackOnReceivedConnection(OnConnectionReceived);
		ControlSystem.SetCallBackOnReceivedData(OnConnDataReceived);
		ControlSystem.SetCallBackOnLoseConnection(OnConnectionLost);
		
		cout << "vision system address:" << (void*)&VisualSystem << endl;
		cout << "control system address:" << (void*)&ControlSystem << endl;


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