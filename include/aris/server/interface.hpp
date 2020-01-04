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
	class InterfaceRoot : public aris::core::Object
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;

		ARIS_REGISTER_TYPE(InterfaceRoot);

	private:
		aris::core::XmlDocument doc_;
	};
	class Interface :public aris::core::Object
	{
	public:
		auto virtual open()->void = 0;
		auto virtual close()->void = 0;

		Interface(const std::string &name = "interface");
		ARIS_REGISTER_TYPE(Interface);
		ARIS_DEFINE_BIG_FOUR(Interface);
	};

	class ProgramWebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto isAutoRunning()->bool;
		auto isAutoMode()->bool;
		auto currentLine()->int;

		ProgramWebInterface(const std::string &name = "pro_interface", const std::string &port = "5866", aris::core::Socket::TYPE type = aris::core::Socket::WEB);
		ProgramWebInterface(ProgramWebInterface && other);
		ProgramWebInterface& operator=(ProgramWebInterface&& other);
		ARIS_REGISTER_TYPE(ProgramWebInterface);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
		aris::core::Socket *sock_;
	};

	auto parse_ret_value(std::vector<std::pair<std::string, std::any>> &ret)->std::string;
	class WebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;

		WebInterface(const std::string &name = "websock_interface", const std::string &port = "5866", aris::core::Socket::TYPE type = aris::core::Socket::WEB);
		ARIS_REGISTER_TYPE(WebInterface);
		ARIS_DEFINE_BIG_FOUR(WebInterface);

	private:
		aris::core::Socket *sock_;
	};
	class HttpInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;

		virtual ~HttpInterface();
		HttpInterface(const std::string &name = "http_interface", const std::string &port = "8000", const std::string &document_root = "./");
		HttpInterface(const HttpInterface & other) = delete;
		HttpInterface(HttpInterface && other);
		HttpInterface &operator=(const HttpInterface& other) = delete;
		HttpInterface &operator=(HttpInterface&& other);
		ARIS_REGISTER_TYPE(HttpInterface);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class GetInfo :public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		GetInfo() {	this->command().setName("get_i");}
		ARIS_REGISTER_TYPE(GetInfo);
	};
}

#endif

