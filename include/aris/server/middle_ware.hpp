#ifndef ARIS_SERVER_MIDDLE_WARE_HPP_
#define ARIS_SERVER_MIDDLE_WARE_HPP_

#include <string_view>
#include <functional>

#include <aris_lib_export.h>

#include "aris/server/interface.hpp"

namespace aris::server {
	class ARIS_API MiddleWare {
	public:
		MiddleWare() {}
		virtual ~MiddleWare() {}

		auto virtual init()->void {}
		auto virtual executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface *interface)->int;     // 默认实现仅转发指令到控制器
		auto virtual executeCmd(std::string_view cmd_str, Interface *interface)->void;
	};

}   // namespace aris::server

#endif  // ARIS_SERVER_MIDDLE_WARE_HPP_