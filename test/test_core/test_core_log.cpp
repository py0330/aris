#include <sstream>
#include <future>
#include <aris/core/core.hpp>
#include "test_core_msg.h"

using namespace aris::core;

void test_log_multi_thread()
{
	try
	{
		enum { THREAD_NUM = 4 };
		std::future<void> ft_debug[THREAD_NUM], ft_info[THREAD_NUM], ft_error[THREAD_NUM], ft_fatal[THREAD_NUM];
		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft_debug[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_DEBUG << "debug thread " << i << " count " << j << std::endl;
				}
			});

			ft_info[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_INFO << "info thread " << i << " count " << j << std::endl;
				}
			});

			ft_error[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_ERROR << "error thread " << i << " count " << j << std::endl;
				}
			});

			ft_fatal[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_FATAL << "fatal thread " << i << " count " << j << std::endl;
				}
			});
		}
		for (auto i = 0; i < THREAD_NUM; ++i) 
		{
			ft_debug[i].wait();
			ft_info[i].wait();
			ft_error[i].wait();
			ft_fatal[i].wait();
		}
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
}
void test_log_every_multi_thread()
{
	try
	{
		enum { THREAD_NUM = 4 };
		std::future<void> ft_debug[THREAD_NUM], ft_info[THREAD_NUM], ft_error[THREAD_NUM], ft_fatal[THREAD_NUM];
		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft_debug[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_DEBUG_EVERY_N(10) << "debug thread " << i << " count " << j << std::endl;
				}
			});

			ft_info[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_INFO_EVERY_N(10) << "info thread " << i << " count " << j << std::endl;
				}
			});

			ft_error[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_ERROR_EVERY_N(10) << "error thread " << i << " count " << j << std::endl;
				}
			});

			ft_fatal[i] = std::async(std::launch::async, [i]()
			{
				for (auto j = 0; j < 100; ++j)
				{
					LOG_FATAL_EVERY_N(10) << "fatal thread " << i << " count " << j << std::endl;
				}
			});
		}
		for (auto i = 0; i < THREAD_NUM; ++i)
		{
			ft_debug[i].wait();
			ft_info[i].wait();
			ft_error[i].wait();
			ft_fatal[i].wait();
		}
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
}

void test_core_log()
{
	std::cout << std::endl << "-----------------test log---------------------" << std::endl;
	
	test_log_multi_thread();

	logFile("test_log_every.txt");
	test_log_every_multi_thread();

	std::cout << "-----------------test log finished------------" << std::endl << std::endl;
}