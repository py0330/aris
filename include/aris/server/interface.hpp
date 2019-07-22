#ifndef ARIS_SERVER_INTERFACE_H_
#define ARIS_SERVER_INTERFACE_H_

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <future>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>
#include <aris/sensor/sensor.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/plan/plan.hpp>

namespace aris::server
{
	class Interface :public aris::core::Object
	{
	public:
		auto virtual open()->void = 0;
		auto virtual close()->void = 0;
		auto virtual onCmdFinished(std::shared_ptr<aris::plan::PlanTarget> &target)->void = 0;
		auto executeCmd(const aris::core::Msg &cmd_string)->std::shared_ptr<aris::plan::PlanTarget>;

		Interface() = default;
		ARIS_REGISTER_TYPE(Interface);
		ARIS_DEFINE_BIG_FOUR(Interface);
	};

	class WebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual onCmdFinished(std::shared_ptr<aris::plan::PlanTarget> &target)->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;

		WebInterface(const std::string &port);
		ARIS_REGISTER_TYPE(WebInterface);
		ARIS_DEFINE_BIG_FOUR(WebInterface);

	private:
		aris::core::Socket sock_;
	};

	class CmdLineInterface :public Interface
	{
	public:
		auto virtual open()->void = 0;
		auto virtual close()->void = 0;
		auto virtual sendRet(const std::string &ret)->void = 0;

		ARIS_REGISTER_TYPE(CmdLineInterface);
		ARIS_DEFINE_BIG_FOUR(CmdLineInterface);
	};


}

#endif

