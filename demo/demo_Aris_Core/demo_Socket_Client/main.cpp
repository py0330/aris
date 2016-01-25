#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <thread>


#include "aris_core_socket.h"
#include "aris_core_msg_loop.h"

#ifdef UNIX
#include <unistd.h>
#endif

#ifdef WIN32
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
#ifdef WIN32  
#define CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>  
_CrtMemState s1, s2, s3;
#endif
/*内存检测泄露完毕*/


int main()
{
	
	/*以下用于VS内存泄露检测*/
#ifdef WIN32  
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtMemCheckpoint(&s1);
	//_CrtSetBreakAlloc(52);
#endif
	/*内存检测泄露完毕*/

	
	/*设置完毕*/
	{
		/*以下在不同的操作系统下设置不同的服务器IP*/
#ifdef WIN32  
		char RemoteIp[] = "127.0.0.1";
#endif
#ifdef UNIX  
		char RemoteIp[] = "127.0.0.1";
#endif

		Socket VisualSystem, ControlSystem;

		/*注册所有的消息函数*/
		Aris::Core::registerMsgCallback(VisualSystemDataNeeded, [&VisualSystem](Aris::Core::Msg &msg)
		{
			/*只要收到数据，就认为服务器在索取地图数据，于是生成地图并发送过去*/
			static int i = 0;

			i++;

			cout << "Visual data needed" << endl;
			double map[9] = { (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i, (double)i };

			Aris::Core::Msg data;
			data.copy(map, sizeof(map));

			VisualSystem.sendMsg(data);

			return 0;
		});
		Aris::Core::registerMsgCallback(VisualSystemLost, [](Aris::Core::Msg &msg)
		{
			cout << "Vision system connection lost" << endl;

			Aris::Core::stopMsgLoop();

			return 0;
		});
		Aris::Core::registerMsgCallback(ControlCommandReceived, [&ControlSystem](Aris::Core::Msg &msg)
		{
			/*只要收到数据，就认为服务器让机器人行动，于是sleep 2秒之后发送回去数据，告诉机器人已经走到位*/

			cout << "begin walking" << endl;
#ifdef WIN32
			msSleep(2000);
#endif
#ifdef UNIX
			usleep(2000000);
#endif



			Aris::Core::Msg data;
			ControlSystem.sendMsg(data);
			cout << "end walking" << endl;

			return 0;

		});
		Aris::Core::registerMsgCallback(ControlSystemLost, [](Aris::Core::Msg &msg)
		{
			cout << "Control system connection lost" << endl;

			return 0;
		});

		/*设置所有Socket类型的回调函数*/
		VisualSystem.setOnReceivedMsg([](Aris::Core::Socket *pConn, Aris::Core::Msg &data)
		{
			Aris::Core::postMsg(Aris::Core::Msg(VisualSystemDataNeeded));

			return 0;
		});
		VisualSystem.setOnLoseConnection([](Aris::Core::Socket *pConn)
		{
			postMsg(Aris::Core::Msg(VisualSystemLost));

			return 0;
		});
		ControlSystem.setOnReceivedMsg([](Aris::Core::Socket *pConn, Aris::Core::Msg &data)
		{
			Aris::Core::postMsg(Aris::Core::Msg(ControlCommandReceived));

			return 0;
		});
		ControlSystem.setOnLoseConnection([](Aris::Core::Socket *pConn)
		{
			postMsg(Aris::Core::Msg(ControlSystemLost));

			return 0;
		});

		/*以下使用lambda函数做回调*/
		VisualSystem.setOnReceivedMsg([](Aris::Core::Socket *pConn, Aris::Core::Msg &data)
		{
			Aris::Core::postMsg(Aris::Core::Msg(VisualSystemDataNeeded));
			return 0;
		});

		/*连接服务器*/
		
		Aris::Core::log("before connect");
		try
		{
			VisualSystem.connect(RemoteIp, "5688");
			ControlSystem.connect(RemoteIp, "5689");
		}
		catch (std::logic_error &e)
		{
			cout << e.what();
			exit(0);
		}
		Aris::Core::log("after connect");


		Aris::Core::Msg ret = ControlSystem.sendRequest(Aris::Core::Msg());
		
		cout << (char*)ret.data() << endl;

		


		/*开始消息循环*/
		Aris::Core::runMsgLoop();
	}

#ifdef WIN32  
	_CrtMemCheckpoint(&s2);
	if (_CrtMemDifference(&s3, &s1, &s2))
		_CrtMemDumpStatistics(&s3);
	//_CrtDumpMemoryLeaks();
#endif
	
}
