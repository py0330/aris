#ifndef ARIS_ROBOT_UR_H_
#define ARIS_ROBOT_UR_H_

#include <memory>

#include <aris_dynamic.h>
#include <aris_control.h>

namespace aris
{
	/// \brief 机器人命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace robot
	{
		
		auto createUr5Model()->std::unique_ptr<aris::dynamic::Model>;
		auto createUr5Controller()->std::unique_ptr<aris::control::Controller>;
	}
}


#endif