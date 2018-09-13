#ifndef ARIS_DYNAMIC_MODEL_INSTANCE_
#define ARIS_DYNAMIC_MODEL_INSTANCE_

#include <aris_dynamic_model.h>

namespace aris::dynamic
{
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
}

#endif
