#ifndef ARIS_ROBOT_UR_H_
#define ARIS_ROBOT_UR_H_

#include <memory>

#include <aris_dynamic.h>

namespace aris
{
	/// \brief 机器人命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace robot
	{
		auto create_ur5()->std::unique_ptr<aris::dynamic::Model>;
	}
}


#endif