#ifndef ARIS_ROBOT_ROKAE_H_
#define ARIS_ROBOT_ROKAE_H_

#include <memory>

#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/plan/plan.hpp>

/// \brief 机器人命名空间
/// \ingroup aris
/// 
///
///
namespace aris::robot
{
	auto ARIS_API createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto ARIS_API createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto ARIS_API createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
	auto ARIS_API createRokaeXB4Interface()->std::string;
}


#endif