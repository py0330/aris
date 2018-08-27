#ifndef ARIS_ROBOT_ROKAE_H_
#define ARIS_ROBOT_ROKAE_H_

#include <memory>

#include <aris_dynamic.h>
#include <aris_control.h>

/// \brief 机器人命名空间
/// \ingroup aris
/// 
///
///
namespace aris::robot
{
	auto createRokaeModel()->std::unique_ptr<aris::dynamic::Model>;
	auto createRokaeController()->std::unique_ptr<aris::control::Controller>;
}


#endif