#include <iostream>
#include <future>
#include <sstream>
#include <aris/core/core.hpp>
#include "test_core_socket.h"

using namespace aris::core;

void test_socket_multi_thread(){
	auto test_func = [](aris::core::Socket::Type type)->void{
		try{
			Socket server("server", "", "5866", type), client("client", "127.0.0.1", "5866", type);

			enum { THREAD_NUM = 8 };
			int message_round[THREAD_NUM]{ 0 };
			int request_round[THREAD_NUM]{ 0 };
			int request_answer[THREAD_NUM]{ 0 };
			std::atomic_bool lose_executed{ false }, connect_executed{ false };
			server.setOnReceivedConnection([&](Socket*, const char*, int){
				connect_executed = true;
				return 0;
			});
			server.setOnReceivedMsg([&](Socket *, Msg &msg){
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

				return 0;
			});
			server.setOnReceivedRawData([&](Socket *, const char *data, int size){
				std::string str(data, size);
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
				if (num != message_round[thread_id])std::cout << __FILE__ << __LINE__ << "test_socket failed"<< std::endl;

				message_round[thread_id] = num + 4;

				return 0;
			});
			server.setOnLoseConnection([&](Socket*){
				lose_executed = true;
				return 0;
			});

			server.startServer();
			client.connect();

			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			std::future<void> ft_message[THREAD_NUM];
			for (auto i = 0; i < THREAD_NUM; ++i){
				ft_message[i] = std::async(std::launch::async, [&client, i](){
					for (auto j = 0; j < 400; j += 4){
						Msg msg("message " + std::to_string(i) + " count " + std::to_string(j));
						
						try{
							client.sendMsg(msg);
						}
						catch (std::exception &)
						{}
						
						try{
							client.sendRawData(msg.data(), msg.size());
						}
						catch (std::exception &)
						{}
					}
				});
			}
			for (auto i = 0; i < THREAD_NUM; ++i) ft_message[i].wait();
			client.stop();
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			for (auto i = 0; i < THREAD_NUM; ++i){
				if (message_round[i] != 400)std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
			}

			if (!connect_executed && (type != aris::core::Socket::Type::UDP && type != aris::core::Socket::Type::UDP_RAW) )std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
			if (!lose_executed && (type != aris::core::Socket::Type::UDP && type != aris::core::Socket::Type::UDP_RAW))std::cout << __FILE__ << __LINE__ << "test_socket failed" << std::endl;
		}
		catch (std::exception &e){
			std::cout << e.what();
		}
	};

	std::cout << "test tcp" << std::endl;
	test_func(aris::core::Socket::Type::TCP);
	std::cout << "test tcp raw" << std::endl;
	test_func(aris::core::Socket::Type::TCP_RAW);
	std::cout << "test udp" << std::endl;
	test_func(aris::core::Socket::Type::UDP);
	std::cout << "test udp raw" << std::endl;
	test_func(aris::core::Socket::Type::UDP_RAW);
	std::cout << "test web" << std::endl;
	test_func(aris::core::Socket::Type::WEB);
	std::cout << "test web raw" << std::endl;
	test_func(aris::core::Socket::Type::WEB_RAW);
}
void test_socket_connect_time_out() {
	auto test_func = [](aris::core::Socket::Type type)->void {
		Socket server("server", "", "5866", type), client("client", "127.0.0.1", "5866", type);

		server.setOnLoseConnection([](aris::core::Socket* s)->int {
			std::cout << "server lose connection" << std::endl;
			return 0;
			});

		client.setOnLoseConnection([](aris::core::Socket* s)->int {
			std::cout << "client lose connection" << std::endl;
			return 0;
			});


		client.setConnectTimeoutMs(1000);
		try {
			client.connect();
			std::cout << "failed to connect with timeout" << std::endl;
		}
		catch (std::runtime_error& e) {
		}

		try {
			std::thread t([&]() {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				server.startServer();
				});
			t.detach();
			client.connect();
			cout() << "connected" << std::endl;
		}
		catch (std::runtime_error& e) {
			cout() << "failed to connect with timeout" << std::endl;
		}
		
		std::this_thread::sleep_for(std::chrono::seconds(2));
		std::cout << "stop client" << std::endl;
		client.stop();

		

		{
			std::thread t;
			
			try {
				t = std::thread([&]() {
					std::this_thread::sleep_for(std::chrono::milliseconds(1500));
					server.startServer();
					});
				
				client.connect();
				t.join();
			}
			catch (std::runtime_error& e) {
				t.join();
			}
		}
	};
	std::cout << "test tcp connect time out" << std::endl;
	test_func(aris::core::Socket::Type::TCP);
}
void test_socket()
{
	std::cout << std::endl << "-----------------test socket---------------------" << std::endl;
	//test_socket_multi_thread();
	test_socket_connect_time_out();
	std::cout << "-----------------test socket finished------------" << std::endl;
}