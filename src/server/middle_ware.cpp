#include "aris/server/middle_ware.hpp"

#include "aris/core/core.hpp"
#include "aris/server/control_server.hpp"

#include "json.hpp"

namespace aris::server {

auto MiddleWare::executeCmd(std::string_view str, std::function<void(std::string)> send_ret)->int {
	aris::server::ControlServer::instance().executeCmdInCmdLine(std::string(str), [send_ret](aris::plan::Plan &plan)->void
		{
			// only copy if it is a str
			if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
			{
				js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
				js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
				send_ret(aris::server::parse_ret_value(*js));
			}
			else
			{
				std::vector<std::pair<std::string, std::any>> ret_js;
				ret_js.push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
				ret_js.push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
				send_ret(aris::server::parse_ret_value(ret_js));
			}
		});
	
	return 0;
}

ARIS_REGISTRATION{
		
		aris::core::class_<MiddleWare>("MiddleWare")
			;
	}

}   // namespace aris::server