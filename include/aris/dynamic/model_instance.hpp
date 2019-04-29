#ifndef ARIS_DYNAMIC_MODEL_INSTANCE_H_
#define ARIS_DYNAMIC_MODEL_INSTANCE_H_

#include <aris/dynamic/model.hpp>

namespace aris::dynamic
{
	auto createModelStewart()->std::unique_ptr<aris::dynamic::Model>;
}

#endif
