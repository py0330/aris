#include <iostream>
#include <future>
#include <aris.h>
#include "test_core_socket.h"

using namespace aris::core;
const char xml_data[] =
"<root>"
"    <server type=\"Socket\" port=\"5866\"/>"
"    <client type=\"Socket\" remote_ip=\"127.0.0.1\" port=\"5866\"/>"
"</root>";


void test_core_socket()
{
	try
	{
		Root root;
		root.registerChildType<Socket>();
		auto& server = root.add<Socket>("server");
		auto& client = root.add<Socket>("client");

		server.setOnReceivedMsg([](Socket *, Msg &msg)
		{
			std::cout << msg.data() << std::endl;

			return 0;
		});
		server.setOnReceivedRequest([](Socket *, Msg &msg) 
		{
			std::cout << msg.data() << std::endl;
			return Msg(std::string(msg.data()) + " was received");
		});
		
		server.setOnLoseConnection( [](Socket*)
		{
			std::cout << "connection lost" << std::endl;
			return 0;
		});
		client.setRemoteIP("127.0.0.1");
		client.setPort("5866");
		
		server.startServer("5866");
		client.connect();

		enum { THREAD_NUM = 8 };
		std::future<void> ft[THREAD_NUM];
		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft[i] = std::async(std::launch::async, [&client, i]() 
			{
				for (auto j = 0; j < 100; ++j)
				{
					auto msg = client.sendRequest(Msg("thread " + std::to_string(i) + " count: " + std::to_string(j)));
					std::cout << msg.data() << std::endl;
				}
					
			});
		}

		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft[i].wait();
		}

		client.stop();

		std::string s;
		std::getline(std::cin, s);
		
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}

	try
	{
		Root root;
		root.registerChildType<Socket>();

		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_data);
		root.loadXml(xml_doc);

		auto& server = static_cast<Socket&>(*root.findByName("server"));
		auto& client = static_cast<Socket&>(*root.findByName("client"));

		server.setOnReceivedMsg([](Socket *, Msg &msg)
		{
			std::cout << msg.data();

			return 0;
		});
		server.setOnLoseConnection([](Socket*)
		{
			std::cout << "connection lost" << std::endl;
			return 0;
		});

		server.startServer();
		client.connect();


		enum { THREAD_NUM = 8 };
		std::future<void> ft[THREAD_NUM];
		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft[i] = std::async(std::launch::async, [&client, i]()
			{
				for (auto j = 0; j<100; ++j)
					client.sendMsg(Msg("thread " + std::to_string(i) + " count: " + std::to_string(j) + "\n"));
			});
		}

		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft[i].wait();
		}

		client.stop();

		char s;
		std::cin >> s;

		server.startServer();
		client.connect();
		client.sendMsg(Msg("123456\n"));


		std::cin >> s;

	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}


	std::cout << "test socket finished" << std::endl;
}

