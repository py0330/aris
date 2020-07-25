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
	class ARIS_API InterfaceRoot
	{
	};
	class ARIS_API Interface
	{
	public:
		auto virtual open()->void = 0;
		auto virtual close()->void = 0;

		Interface(const std::string &name = "interface");
		ARIS_DEFINE_BIG_FOUR(Interface);
	};
	class ARIS_API ProgramWebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto resetSocket(aris::core::Socket *sock)->void;
		auto socket()->aris::core::Socket&;
		auto isAutoMode()->bool;
		auto isAutoRunning()->bool;
		auto isAutoPaused()->bool;
		auto isAutoStopped()->bool;
		auto lastError()->std::string;
		auto lastErrorCode()->int;
		auto lastErrorLine()->int;
		auto currentFileLine()->std::tuple<std::string, int>;
		
		~ProgramWebInterface();
		ProgramWebInterface(const std::string &name = "pro_interface", const std::string &port = "5866", aris::core::Socket::TYPE type = aris::core::Socket::WEB);
		ProgramWebInterface(ProgramWebInterface && other);
		ProgramWebInterface& operator=(ProgramWebInterface&& other);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	auto ARIS_API parse_ret_value(std::vector<std::pair<std::string, std::any>> &ret)->std::string;
	class ARIS_API WebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto resetSocket(aris::core::Socket *sock)->void;
		auto socket()->aris::core::Socket&;

		~WebInterface();
		WebInterface(const std::string &name = "websock_interface", const std::string &port = "5866", aris::core::Socket::TYPE type = aris::core::Socket::WEB);
		ARIS_DECLARE_BIG_FOUR(WebInterface);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	class ARIS_API HttpInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto documentRoot()->std::string;
		auto setDocumentRoot(const std::string &root)->void;
		auto port()->std::string;
		auto setPort(const std::string &root)->void;

		virtual ~HttpInterface();
		HttpInterface(const std::string &name = "http_interface", const std::string &port = "8000", const std::string &document_root = "./");
		HttpInterface(const HttpInterface & other) = delete;
		HttpInterface(HttpInterface && other);
		HttpInterface &operator=(const HttpInterface& other) = delete;
		HttpInterface &operator=(HttpInterface&& other);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif