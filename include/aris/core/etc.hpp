#ifndef ARIS_CORE_ETC_H_
#define ARIS_CORE_ETC_H_

#include <string>
#include <vector>

namespace aris::core{
	auto trimLR(std::string_view input) -> std::string;
	auto split(std::string_view s, char delimiter) -> std::vector<std::string>;
}

#endif
