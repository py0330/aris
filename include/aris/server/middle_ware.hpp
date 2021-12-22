#ifndef ARIS_SERVER_MIDDLE_WARE_HPP_
#define ARIS_SERVER_MIDDLE_WARE_HPP_

#include <string_view>
#include <functional>

#include <aris_lib_export.h>

#include "aris/server/interface.hpp"

namespace aris::server {

	class InternalInterface : public Interface {
	public:
		auto virtual open()->void override { is_open_ = true; }
		auto virtual close()->void override { is_open_ = false; }
		auto virtual isConnected() const->bool override { return is_open_; }

	private:
		bool is_open_{true};

	public:
		InternalInterface(const std::string &name = "Internal") : Interface(name) {}
	};

	class ARIS_API MiddleWare {
	public:
		MiddleWare() {}
		virtual ~MiddleWare() {}

		auto virtual init()->void {}
		auto virtual executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface *interface)->int;     // 默认实现仅转发指令到控制器
		auto virtual executeCmd(std::string_view cmd_str, Interface *interface)->void;
		auto virtual executeCmd(std::string_view cmd_str)->void;

	protected:
		static std::uint64_t cmd_id_;
	};

}   // namespace aris::server

#endif  // ARIS_SERVER_MIDDLE_WARE_HPP_