#include "aris/server/middle_ware.hpp"

#include "aris/core/core.hpp"
#include "aris/server/control_server.hpp"

#include "json.hpp"

namespace aris::server {

auto MiddleWare::executeCmd(std::string_view str, std::function<void(std::string)> send_ret)->int {
	aris::server::ControlServer::instance().executeCmdInCmdLine(std::string(str), [send_ret](aris::plan::Plan &plan)->void
		{
			ARIS_COUT_PLAN((&plan)) << "return code :" << plan.retCode() << "\n";
			ARIS_COUT_PLAN((&plan)) << "return msg  :" << plan.retMsg() << std::endl;

			LOG_INFO << "cmd " << plan.cmdId() << " return code   :" << plan.retCode() << "\n" << std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << "return message:" << plan.retMsg() << std::endl;

			// only copy if it is a str
			if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
			{
				js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
				js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
				send_ret(aris::server::parse_ret_value(*js));
				std::cout << aris::server::parse_ret_value(*js) << std::endl;
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

auto MiddleWare::executeCmd(std::string cmd_str)->void {
	executeCmd(cmd_str, [](std::string){});
}

ARIS_REGISTRATION{
		
		aris::core::class_<MiddleWare>("MiddleWare")
			;
	}

}   // namespace aris::server