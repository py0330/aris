#ifndef ARIS_PLAN_FUNCTION_H_
#define ARIS_PLAN_FUNCTION_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>

namespace aris
{
	namespace dynamic { class Model; }
	namespace control { class Master; }
	
	
	/// \brief 轨迹规划命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace plan 
	{
		struct PlanParam
		{
			std::uint32_t count_;
			aris::dynamic::Model* model_;
			aris::control::Master* master_;
			void *param_;
			std::uint32_t param_size_;
		};
		using PlanFunction = std::function<int(const PlanParam &plan_param)>;
		using ParseFunction = std::function<int(const PlanParam &plan_param)>;





	}
}


#endif