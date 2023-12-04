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
		auto virtual executeCmd(std::string_view cmd_str)->void;

	protected:
		static std::uint64_t cmd_id_;
	};

	class ARIS_API ProgramMiddleware : public MiddleWare {
	public:
		auto isAutoMode() -> bool;
		auto isAutoRunning() -> bool;
		auto isAutoPaused() -> bool;
		auto isAutoStopped() -> bool;
		auto lastError() -> std::string;
		auto lastErrorCode() -> int;
		auto lastErrorLine() -> int;
		auto currentFileLine() -> std::tuple<std::string, int>;
		auto executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface* interface) -> int override;
		~ProgramMiddleware();

		ProgramMiddleware();
		ProgramMiddleware(ProgramMiddleware&& other);
		ProgramMiddleware& operator=(ProgramMiddleware&& other);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

}   // namespace aris::server

#endif  // ARIS_SERVER_MIDDLE_WARE_HPP_