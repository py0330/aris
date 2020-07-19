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

	auto ARIS_API toXmlFile(aris::core::Instance ins, const std::filesystem::path &file)->void;
	auto ARIS_API fromXmlFile(aris::core::Instance ins, const std::filesystem::path &file)->void;

	auto ARIS_API toJsonString(aris::core::Instance ins)->std::string;
	auto ARIS_API fromJsonString(aris::core::Instance ins, std::string_view xml_str)->void;
}

#endif
