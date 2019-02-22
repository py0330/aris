#ifndef ARIS_ROBOT_SERVO_PRESS_H_
#define ARIS_ROBOT_SERVO_PRESS_H_

#include <memory>

#include <aris/dynamic/dynamic.hpp>
#include <aris/control/control.hpp>
#include <aris/plan/plan.hpp>

/// \brief 机器人命名空间
/// \ingroup aris
/// 
///
///
namespace aris::robot
{
	auto createModelServoPress(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerServoPress()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootServoPress()->std::unique_ptr<aris::plan::PlanRoot>;
}


#endif