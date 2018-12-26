#include <iostream>
#include <future>
#include <aris_core.h>
#include "test_core_socket.h"

using namespace aris::core;

void test_socket_xml()
{
	const char xml_data[] =
		"<root>"
		"    <server type=\"Socket\" port=\"5866\"/>"
		"    <client type=\"Socket\" remote_ip=\"127.0.0.1\" port=\"5866\"/>"
		"</root>";
	
	auto root = aris::core::Object();
	root.registerType<Socket>();

	root.loadXmlStr(xml_data);
	auto str1 = root.xmlString();
	auto &server = root.children().front();
	auto &client = root.children().back();
	
	std::string xml_str;
	server.saveXmlStr(xml_str);
	if (server.xmlString() != "<server type=\"Socket\" port=\"5866\"/>\n") std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
	if (client.xmlString() != "<client type=\"Socket\" remote_ip=\"127.0.0.1\" port=\"5866\"/>\n") std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
}
void test_socket_multi_thread()
{
	auto test_func = [](aris::core::Socket::TYPE type)->void
	{
		try
		{
			Socket server("server", "", "5866"), client("client", "127.0.0.1", "5866");

			enum { THREAD_NUM = 1 };
			int message_round[THREAD_NUM]{ 0 };
			int request_round[THREAD_NUM]{ 0 };
			int request_answer[THREAD_NUM]{ 0 };
			std::atomic_bool lose_executed{ false }, connect_executed{ false };
			server.setOnReceivedConnection([&](Socket*, const char*, int)
			{
				connect_executed = true;
				return 0;
			});
			server.setOnReceivedMsg([&](Socket *, Msg &msg)
			{
				std::string str(msg.data(), msg.size());
				std::stringstream ss(str);
				std::string word;
				int thread_id, num;

				ss >> word;
				if (word != "message")std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
				ss >> thread_id;
				if (thread_id > 7 || thread_id < -1)std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
				ss >> word;
				if (word != "count")std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
				ss >> num;
				if (num != message_round[thread_id])std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;

				message_round[thread_id] = num + 4;

				std::cout << "rece msg:" << thread_id << "   " << word << std::endl;

				return 0;
			});
			server.setOnLoseConnection([&](Socket*)
			{
				lose_executed = true;
				return 0;
			});

			server.startServer("", type);
			client.connect("", "", type);
			
			client.setConnectType(type);

			std::future<void> ft_message[THREAD_NUM];
			for (auto i = 0; i < THREAD_NUM; ++i)
			{
				ft_message[i] = std::async(std::launch::async, [&client, i]()
				{
					for (auto j = 0; j < 400; j += 4)
						client.sendMsg(Msg("message " + std::to_string(i) + " count " + std::to_string(j)));
				});
			}

			for (auto i = 0; i < THREAD_NUM; ++i) ft_message[i].wait();

			std::cout <<"server state"<< server.state()<<std::endl;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			client.stop();

			for (auto i = 0; i < THREAD_NUM; ++i)
			{
				if (message_round[i] != 400)std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
			}

			

			if (!connect_executed)std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
			if (!lose_executed)std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
		}
		catch (std::exception &e)
		{
			std::cout << e.what();
		}
	};

	//test_func(aris::core::Socket::TCP);
	
	std::cout << "test udp" << std::endl;
	test_func(aris::core::Socket::UDP);
	
}

void test_core_socket()
{
	std::cout << std::endl << "-----------------test socket---------------------" << std::endl;
	//test_socket_xml();
	test_socket_multi_thread();
	std::cout << "-----------------test socket finished------------" << std::endl << std::endl;
}