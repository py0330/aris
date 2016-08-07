#include <iostream>
#include <thread>
#include <future>

#include <aris.h>

#include "test_core_pipe.h"


using namespace aris::core;

long mod(long a, long b)
{
	return (a%b + b) % b;
}

void test_core_pipe()
{
	aris::core::Pipe pipe;

	auto fu = std::async(std::launch::async, [&pipe]() 
	{
		for (int i = 0; i < 1000; ++i)
		{
			if (!pipe.sendMsg(aris::core::Msg("from pipe:" + std::to_string(i))))
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				std::cout << "pool is full" << std::endl;
			}
				
		}
		
		
	});

	aris::core::Msg msg;
	
	for(;;)
	{
		static int failed_count{ 0 };
		if (pipe.recvMsg(msg)) 
		{
			failed_count = 0;
			std::cout << msg.data() << std::endl;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			failed_count++;
		}
		
		if (failed_count > 10)break;

	}
	

	//

	fu.wait();

	std::cout << "finished" << std::endl;
}