#ifndef ARIS_ROBOT_STEWART_H_
#define ARIS_ROBOT_STEWART_H_

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
	auto createModelStewart(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerStewart()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootStewart()->std::unique_ptr<aris::plan::PlanRoot>;
}


#endif