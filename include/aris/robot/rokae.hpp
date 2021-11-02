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
namespace aris::robot::rokae::xb4{
	auto ARIS_API createModel(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto ARIS_API createMaster()->std::unique_ptr<aris::control::Master>;
	auto ARIS_API createController()->std::unique_ptr<aris::control::Controller>;
	auto ARIS_API createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif