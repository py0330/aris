#ifdef ARIS_USE_XENOMAI
extern "C"
{
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <sys/mman.h>
}
#endif

#include <chrono>
#include <thread>
#include <memory>
#include <vector>

#include "aris/control/ethercat_kernel.hpp"

namespace aris::control
{
#ifdef ARIS_USE_XENOMAI
	thread_local std::int64_t last_time_;


	auto aris_mlockall()->void { if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) throw std::runtime_error("lock failed"); }
	auto aris_rt_task_create()->std::any
	{
		std::any rt_task = RT_TASK();

		// 为了使用返回值优化，这里必须判断，不能用三目运算符 //
		if(rt_task_create(&std::any_cast<RT_TASK&>(rt_task), "realtime core", 0, 99, T_JOINABLE))
			rt_task = std::any();

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
		last_time_ += 1000000;
		return rt_task_wait_period(NULL); 
	}
	auto aris_rt_timer_read()->std::int64_t { return rt_timer_read(); }

	auto aris_rt_time_since_last_time()->std::int64_t { return aris_rt_timer_read() - last_time_; }
#else
	// should not have global variables
	thread_local int nanoseconds{ 1000 };
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
#endif
}
