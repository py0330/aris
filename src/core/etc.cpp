#include "aris/core/etc.hpp"

#include <string>
#include <sstream>

namespace aris::core {
	auto trimLR(std::string_view input) -> std::string {
		std::string ret(input);
		ret.erase(0, ret.find_first_not_of(" \t\n\r\f\v"));// trim l
		ret.erase(ret.find_last_not_of(" \t\n\r\f\v") + 1);// trim r
		return ret;
	}
	auto split(std::string_view s, char delimiter) -> std::vector<std::string> {
		std::vector<std::string> tokens;
		std::string token, str(s);
		std::istringstream tokenStream(str);
		while (std::getline(tokenStream, token, delimiter)) {
			tokens.push_back(token);
		}
		return tokens;
	}
}



