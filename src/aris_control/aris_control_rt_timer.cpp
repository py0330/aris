#ifdef USE_XENOMAI
extern "C"
{
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
}
#endif

#include <chrono>
#include <thread>
#include <memory>
#include <vector>

#include "aris_control_ethercat_kernel.h"

namespace aris::control
{
#ifndef USE_XENOMAI
	// should not have global variables
	int nanoseconds{ 1000 };
	std::chrono::time_point<std::chrono::high_resolution_clock> last_time, begin_time;
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
		last_time = begin_time = std::chrono::high_resolution_clock::now();
		return 0;
	};
	auto aris_rt_task_wait_period()->int
	{
		last_time = last_time + std::chrono::nanoseconds(nanoseconds);
		std::this_thread::sleep_until(last_time);
		return 0;
	};
	auto aris_rt_timer_read()->std::int64_t
	{
		auto now = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::nanoseconds>(now - begin_time).count();
	}
#endif

#ifdef USE_XENOMAI
	auto aris_mlockall()->void { if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) throw std::runtime_error("lock failed"); }
	auto aris_rt_task_create()->std::any
	{
		std::any rt_task = RT_TASK();

		// 为了使用返回值优化，这里必须判断，不能用三目运算符 //
		if(rt_task_create(&std::any_cast<RT_TASK&>(rt_task), "realtime core", 0, 99, T_FPU | T_JOINABLE))
			rt_task = std::any();

		return rt_task;
	}
	auto aris_rt_task_start(std::any& rt_task, void(*task_func)(void*), void*param)->int { return rt_task_start(&std::any_cast<RT_TASK&>(rt_task), task_func, param); }
	auto aris_rt_task_join(std::any& rt_task)->int { return rt_task_join(&std::any_cast<RT_TASK&>(rt_task)); }
	auto aris_rt_task_set_periodic(int nanoseconds)->int { return rt_task_set_periodic(NULL, TM_NOW, nanoseconds); }
	auto aris_rt_task_wait_period()->int { return rt_task_wait_period(NULL); }
	auto aris_rt_timer_read()->std::int64_t { return rt_timer_read(); }
#endif
}
