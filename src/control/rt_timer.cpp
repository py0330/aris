#include "aris/control/rt_timer.hpp"
#include "aris/core/log.hpp"

#if defined(ARIS_USE_XENOMAI3)
extern "C"
{
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <sys/mman.h>
}
namespace aris::control
{
	thread_local std::int64_t last_time_;

	auto aris_mlockall()->void { if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) THROW_FILE_LINE("lock failed"); }
	auto aris_rt_task_create()->std::any
	{
		std::any rt_task = RT_TASK();

		// 为了使用返回值优化，这里必须判断，不能用三目运算符 //
		if (auto ret = rt_task_create(&std::any_cast<RT_TASK&>(rt_task), "realtime core", 0, 99, T_JOINABLE)) rt_task = std::any();

		return rt_task;
	}
	auto aris_rt_task_start(std::any& rt_task, void(*task_func)(void*), void*param)->int { return rt_task_start(&std::any_cast<RT_TASK&>(rt_task), task_func, param); }
	auto aris_rt_task_join(std::any& rt_task)->int { return rt_task_join(&std::any_cast<RT_TASK&>(rt_task)); }
	auto aris_rt_task_set_periodic(int nanoseconds)->int
	{
		last_time_ = aris_rt_timer_read();
		return rt_task_set_periodic(NULL, TM_NOW, nanoseconds);
	}
	auto aris_rt_task_wait_period()->int
	{
		unsigned long overruns_r;
		auto ret = rt_task_wait_period(&overruns_r);
		last_time_ += 1000000 * (ret == ETIMEDOUT ? overruns_r : 1);
		return ret;
	}
	auto aris_rt_timer_read()->std::int64_t { return rt_timer_read(); }

	auto aris_rt_time_since_last_time()->std::int64_t { return aris_rt_timer_read() - last_time_; }
}

#elif defined(ARIS_USE_RT_PREEMT)

extern "C"
{
#include <errno.h>
#include <mqueue.h>
#include <signal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
}
#include <atomic>

#define NSEC_PER_SEC 1000000000
namespace aris::control
{
	struct RT_TASK
	{
		pthread_t cyclic_thread;
		pthread_attr_t thattr;
	};
	// should not have global variables
	thread_local int nanoseconds{ 1000000 };
	thread_local struct timespec last_time_, begin_time_;
	//
	auto aris_mlockall()->void { if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) THROW_FILE_LINE("lock failed"); }
	auto aris_rt_task_create()->std::any
	{
		auto task = std::make_shared<RT_TASK>();
		pthread_attr_init(&task->thattr);
		pthread_attr_setdetachstate(&task->thattr, PTHREAD_CREATE_JOINABLE);
		return task;
	}
	auto aris_rt_task_start(std::any& handle, void(*task_func)(void*), void*param)->int
	{
		// map from void(void*) to void*(void*)
		std::atomic_bool is = false;
		void* package[3]{ reinterpret_cast<void*>(task_func), param, &is };
		auto linux_task_func = [](void* package)->void*
		{
			auto real = reinterpret_cast<void**>(package);
			auto func = reinterpret_cast<void(*)(void*)>(real[0]);
			auto param = reinterpret_cast<void*>(real[1]);
			auto is = reinterpret_cast<std::atomic_bool*>(real[2]);

			is->store(true);
			func(param);
			return nullptr;
		};

		auto &task = std::any_cast<std::shared_ptr<RT_TASK>&>(handle);
		auto ret = pthread_create(&task->cyclic_thread, &task->thattr, linux_task_func, package);
		if (ret) {
			THROW_FILE_LINE("create rt_thread failed");
			return -1;
		}
		while (!is); // protect memory "package"
		return 0;
	}
	auto aris_rt_task_join(std::any& handle)->int
	{
		auto &task = std::any_cast<std::shared_ptr<RT_TASK>&>(handle);
		pthread_join(task->cyclic_thread, NULL);
		return 0;
	}
	auto aris_rt_task_set_periodic(int nanoseconds)->int
	{
		control::nanoseconds = nanoseconds;
		clock_gettime(CLOCK_MONOTONIC, &begin_time_);
		last_time_ = begin_time_;
		return 0;
	};
	auto aris_rt_task_wait_period()->int
	{
		last_time_.tv_nsec += nanoseconds;
		while (last_time_.tv_nsec >= NSEC_PER_SEC) {
			last_time_.tv_nsec -= NSEC_PER_SEC;
			last_time_.tv_sec++;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &last_time_, NULL);
		return 0;
	};
	auto aris_rt_timer_read()->std::int64_t
	{
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		return (now.tv_sec - begin_time_.tv_sec)*NSEC_PER_SEC + now.tv_nsec - begin_time_.tv_nsec;
	}

	// in nano seconds
	auto aris_rt_time_since_last_time()->std::int64_t
	{
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		return (now.tv_sec - last_time_.tv_sec)*NSEC_PER_SEC + now.tv_nsec - last_time_.tv_nsec;
	}
}
#else
#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <iostream>

namespace aris::control
{
	// should not have global variables
	thread_local int nanoseconds{ 1000000 };
	thread_local std::chrono::time_point<std::chrono::high_resolution_clock> last_time_, begin_time_;
	//

	auto aris_mlockall()->void {}
	auto aris_rt_task_create()->std::any { return std::make_shared<std::thread>(); }
	auto aris_rt_task_start(std::any& handle, void(*task_func)(void*), void*param)->int
	{
		*std::any_cast<std::shared_ptr<std::thread>&>(handle) = std::thread(task_func, param);
		return 0;
	}
	auto aris_rt_task_join(std::any& handle)->int
	{
		std::any_cast<std::shared_ptr<std::thread>&>(handle)->join();
		return 0;
	}
	auto aris_rt_task_set_periodic(int nanoseconds)->int
	{
		control::nanoseconds = nanoseconds;
		last_time_ = begin_time_ = std::chrono::high_resolution_clock::now();
		return 0;
	};
	auto aris_rt_task_wait_period()->int
	{
		last_time_ += std::chrono::nanoseconds(nanoseconds);
		std::this_thread::sleep_until(last_time_);
		return 0;
	};
	auto aris_rt_timer_read()->std::int64_t
	{
		auto now = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::nanoseconds>(now - begin_time_).count();
	}

	// in nano seconds
	auto aris_rt_time_since_last_time()->std::int64_t
	{
		return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - last_time_).count();
	}
}
#endif
