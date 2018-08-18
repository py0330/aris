#include <iostream>
#include <thread>
#include <future>
#include <aris_core.h>
#include "test_core_pipe.h"


using namespace aris::core;

long mod(long a, long b)
{
	return (a%b + b) % b;
}

void test_pipe_multi_thread()
{
	aris::core::Pipe pipe;

	auto fu = std::async(std::launch::async, [&pipe]() 
	{
		for (int i = 0; i < 1000; ++i)
		{
			if (!pipe.sendMsg(aris::core::Msg("from " + std::to_string(i) + " num " + std::to_string(i * 4))))
			{
				std::cout << __FILE__ << __LINE__ << "test_pipe failed" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(1));
		}
	});

	aris::core::Msg msg;
	
	for(;;)
	{
		static int failed_count{ 0 };
		if (pipe.recvMsg(msg)) 
		{
			std::string str(msg.data(), msg.size());
			std::stringstream ss(str);
			std::string word;
			static int round{ 0 }; 
			int id, num;

			ss >> word;
			if (word != "from")std::cout << __FILE__ << __LINE__ << "test_pipe failed" << std::endl;
			ss >> id;
			if (id != round)std::cout << __FILE__ << __LINE__ << "test_pipe failed" << std::endl;
			ss >> word;
			if (word != "num")std::cout << __FILE__ << __LINE__ << "test_pipe failed" << std::endl;
			ss >> num;
			if (num != round * 4)std::cout << __FILE__ << __LINE__ << "test_pipe failed" << std::endl;

			round++;
			
			failed_count = 0;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			failed_count++;
		}
		
		if (failed_count > 100)break; // 超过100ms没有数据，那么退出

	}

	fu.wait();
}

void test_core_pipe()
{
	std::cout << std::endl << "-----------------test pipe---------------------" << std::endl;
	test_pipe_multi_thread();
	std::cout << "-----------------test pipe finished------------" << std::endl << std::endl;
}