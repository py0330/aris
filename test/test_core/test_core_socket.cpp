#include <iostream>
#include <future>
#include <aris_core.h>
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
		Object root;
		root.registerType<Socket>();

		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_data);
		root.loadXmlDoc(xml_doc);

		auto& server = static_cast<Socket&>(*root.findByName("server"));
		auto& client = static_cast<Socket&>(*root.findByName("client"));

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
					client.sendMsg(Msg("thread " + std::to_string(i) + " count: " + std::to_string(j)));
			});
		}

		for (auto i = 0; i < THREAD_NUM; ++i) ft[i].wait();
		client.stop();

		std::string s;
		std::getline(std::cin, s);

		server.startServer();
		client.connect();
		client.sendMsg(Msg("123456\n"));
		client.stop();

		std::getline(std::cin, s);

	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}


	std::cout << "test socket finished" << std::endl;
}

