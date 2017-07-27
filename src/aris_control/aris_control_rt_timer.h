#ifndef ARIS_CONTROL_RT_TIMER_H
#define ARIS_CONTROL_RT_TIMER_H

namespace aris
{
	namespace control
	{	
		struct Handle { virtual ~Handle() = default; };

		auto aris_mlockall()->void;

		auto aris_rt_task_create()->Handle*;
		auto aris_rt_task_start(Handle* handle, void(*task_func)(void*), void*param)->int;
		auto aris_rt_task_join(Handle* handle)->int;
		auto aris_rt_task_set_periodic(int nanoseconds)->int;
		auto aris_rt_task_wait_period()->int;
		auto aris_rt_timer_read()->std::int64_t;
	}
}

#endif
