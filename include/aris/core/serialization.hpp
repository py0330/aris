#ifndef ARIS_CORE_SERIALIZATION_H_
#define ARIS_CORE_SERIALIZATION_H_

#include <string>
#include <filesystem>

#include <aris_lib_export.h>
#include <aris/core/reflection.hpp>

namespace aris::core
{
	auto ARIS_API toXmlString(aris::core::Instance ins)->std::string;
	auto ARIS_API fromXmlString(aris::core::Instance ins, std::string_view xml_str)->void;
}

#endif
