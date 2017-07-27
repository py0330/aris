#ifdef UNIX
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
#endif

#include <chrono>
#include <thread>
#include <memory>
#include <vector>

#include "aris_control_ethercat_kernel.h"

namespace aris
{
	namespace control
	{
#ifdef WIN32
        // should not have global variables
		int nanoseconds{1000};
		std::chrono::time_point<std::chrono::high_resolution_clock> last_time, begin_time;
		//

		struct RtTaskHandle :public Handle { std::thread task; };

		auto aris_mlockall()->void {}
		auto aris_rt_task_create()->Handle*
		{
			std::unique_ptr<Handle> handle(new RtTaskHandle);
			return handle.release();
		}
		auto aris_rt_task_start(Handle* handle, void(*task_func)(void*), void*param)->int
		{
			static_cast<RtTaskHandle*>(handle)->task = std::thread(task_func, param);
			return 0;
		}
		auto aris_rt_task_join(Handle* handle)->int
		{
			static_cast<RtTaskHandle*>(handle)->task.join();
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

#ifdef UNIX
		struct RtTaskHandle :public Handle { RT_TASK task; };

		auto aris_mlockall()->void { if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) throw std::runtime_error("lock failed"); }
		auto aris_rt_task_create()->Handle*
		{
			std::unique_ptr<Handle> handle(new RtTaskHandle);
			return rt_task_create(&static_cast<RtTaskHandle*>(handle.get())->task, "realtime core", 0, 99, T_FPU | T_JOINABLE) ? nullptr : handle.release();
		}
		auto aris_rt_task_start(Handle* handle, void(*task_func)(void*), void*param)->int{return rt_task_start(&static_cast<RtTaskHandle*>(handle)->task, task_func, param);}
		auto aris_rt_task_join(Handle* handle)->int { return rt_task_join(&static_cast<RtTaskHandle*>(handle)->task); }
		auto aris_rt_task_set_periodic(int nanoseconds)->int { return rt_task_set_periodic(NULL, TM_NOW, nanoseconds); }
		auto aris_rt_task_wait_period()->int { return rt_task_wait_period(NULL); }
		auto aris_rt_timer_read()->std::int64_t { return rt_timer_read(); }
#endif
	}
}
