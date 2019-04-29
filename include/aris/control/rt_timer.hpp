#ifndef ARIS_CONTROL_RT_TIMER_H_
#define ARIS_CONTROL_RT_TIMER_H_

#include <cstdint>
#include <any>

namespace aris::control
{
	auto aris_mlockall()->void;

	auto aris_rt_task_create()->std::any;
	auto aris_rt_task_start(std::any& handle, void(*task_func)(void*), void*param)->int;
	auto aris_rt_task_join(std::any& handle)->int;
	auto aris_rt_task_set_periodic(int nanoseconds)->int;
	auto aris_rt_task_wait_period()->int;
	auto aris_rt_timer_read()->std::int64_t;

	// in nano seconds
	auto aris_rt_time_since_last_time()->std::int64_t;
}

#endif
