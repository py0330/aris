#ifndef ARIS_CORE_SERIALIZATION_H_
#define ARIS_CORE_SERIALIZATION_H_

#include <string>
#include <filesystem>

#include <aris/core/reflection.hpp>

namespace aris::core
{
	
	
	
	template<typename T>
	auto toXmlString(T && obj)->std::string;
	template<typename T>
	auto toXmlFile(T && obj, std::filesystem::path file)->void;
}

#endif
