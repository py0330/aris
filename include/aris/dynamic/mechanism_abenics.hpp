#ifndef ARIS_DYNAMIC_MECHANISM_ABENICS_H_
#define ARIS_DYNAMIC_MECHANISM_ABENICS_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	//---------------------------------------------------------------------------------------------
	// 蓝思球形机构
	//---------------------------------------------------------------------------------------------
	
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct ARIS_API AbenicsParam{

	};

	auto ARIS_API createModelAbenics(const AbenicsParam&param)->std::unique_ptr<aris::dynamic::Model>;

	///
	/// @}
}

#endif
