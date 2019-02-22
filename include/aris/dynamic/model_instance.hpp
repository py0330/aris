#ifndef ARIS_DYNAMIC_MODEL_INSTANCE_
#define ARIS_DYNAMIC_MODEL_INSTANCE_

#include <aris/dynamic/model.hpp>

namespace aris::dynamic
{
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createModelStewart()->std::unique_ptr<aris::dynamic::Model>;
}

#endif
