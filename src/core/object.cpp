#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>

#include "aris/core/log.hpp"
#include "aris/core/object.hpp"
#include "aris/core/reflection.hpp"

namespace aris::core
{
	ARIS_REGISTRATION
	{
		class_<NamedObject>("NamedObject")
			.prop("name", &NamedObject::setName, &NamedObject::name);
	}
}