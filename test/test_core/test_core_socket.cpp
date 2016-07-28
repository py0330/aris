#include <iostream>
#include <future>
#include <aris.h>
#include "test_core_socket.h"

using namespace aris::core;
const char xml_data[] =
"<root>"
"    <server type=\"Socket\" port=\"5866\"/>"
"    <client type=\"Socket\" ip=\"127.0.0.1\" port=\"5866\"/>"
"</root>";


void test_socket()
{
	try
	{
		std::future<void> ft[8];
		
		Socket server, client;

		server.startServer("5866");
		server.setOnReceivedMsg([](Socket *, Msg &msg) 
		{
			std::cout << msg.data();

			return 0;
		});

		client.connect("127.0.0.1", "5866");

		for (auto i = 0; i < 8; ++i)
		{
			ft[i] = std::async(std::launch::async, [&client, i]() 
			{
				for (auto j = 0; j<1000; ++j)
					client.sendMsg(Msg("thread " + std::to_string(i) + " count: " + std::to_string(j) + "\n"));
			});
		}


		for (auto i = 0; i < 8; ++i)
		{
			ft[i].wait();
		}

		msSleep(5000);

		
	
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
}

