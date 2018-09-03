#ifndef ARIS_ROBOT_UR_H_
#define ARIS_ROBOT_UR_H_

#include <memory>

#include <aris_dynamic.h>
#include <aris_control.h>
#include <aris_plan.h>

/// \brief 机器人命名空间
/// \ingroup aris
/// 
///
///
namespace aris::robot
{
	auto createModelUr5(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerUr5()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootUr5()->std::unique_ptr<aris::plan::PlanRoot>;
}


#endif