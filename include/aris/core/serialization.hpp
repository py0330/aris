#ifndef ARIS_CORE_SERIALIZATION_H_
#define ARIS_CORE_SERIALIZATION_H_

#include <string>
#include <filesystem>

#include <aris/core/reflection.hpp>

namespace aris::core
{
	auto toXmlString(aris::core::Instance ins)->std::string;
	auto fromXmlString(aris::core::Instance ins, std::string_view xml_str)->void;
}

#endif
