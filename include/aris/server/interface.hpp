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

namespace aris::server{

	class ARIS_API Interface : public aris::core::NamedObject{
	public:
		auto virtual open()->void = 0;
		auto virtual close()->void = 0;
		auto virtual isConnected() const->bool = 0;
		auto resetOnConnected(unsigned priority, const std::function<void(const Interface *interface)> &cbk=nullptr) noexcept->void;
		auto removeAllOnConnected() noexcept->void;
		auto resetOnDisconnected(unsigned priority, const std::function<void(const Interface *interface)> &cbk=nullptr) noexcept->void;
		auto removeAllOnDisconnected() noexcept->void;

	protected:
		struct Imp;
		std::unique_ptr<Imp> imp_;

	public:
		Interface(const std::string &name = "interface");
		virtual ~Interface();
		ARIS_DELETE_BIG_FOUR(Interface)
	};
	class ARIS_API ProgramWebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual isConnected() const->bool override;
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
		ProgramWebInterface(const std::string &name = "pro_interface", const std::string &port = "5866", aris::core::Socket::Type type = aris::core::Socket::Type::WEB);
		// ProgramWebInterface(ProgramWebInterface && other);
		// ProgramWebInterface& operator=(ProgramWebInterface&& other);
		ARIS_DELETE_BIG_FOUR(ProgramWebInterface)

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	class ARIS_API InternalInterface : public Interface {
	public:
		auto virtual open()->void override { is_open_ = true; }
		auto virtual close()->void override { is_open_ = false; }
		auto virtual isConnected() const->bool override { return is_open_; }

	private:
		bool is_open_{true};

	public:
		InternalInterface(const std::string& name = "Internal") : Interface(name) {}
	};
	class ARIS_API TerminalInterface : public Interface {
	public:
		auto virtual open()->void override { is_open_ = true; }
		auto virtual close()->void override { is_open_ = false; }
		auto virtual isConnected() const->bool override { return is_open_; }

	private:
		bool is_open_{true};

	public:
		TerminalInterface(const std::string& name = "Terminal") : Interface(name) {}
	};


	auto ARIS_API parse_ret_value(std::vector<std::pair<std::string, std::any>> &ret)->std::string;
	class ARIS_API WebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual isConnected() const->bool override;
		auto resetSocket(aris::core::Socket *sock)->void;
		auto socket()->aris::core::Socket&;

		~WebInterface();
		WebInterface(const std::string &name = "tcp_interface", const std::string &port = "5866", aris::core::Socket::Type type = aris::core::Socket::Type::WEB);
		ARIS_DELETE_BIG_FOUR(WebInterface);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	class ARIS_API HttpInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual isConnected() const->bool override;
		auto documentRoot()->std::string;
		auto setDocumentRoot(const std::string &root)->void;
		auto port()->std::string;
		auto setPort(const std::string &root)->void;

		virtual ~HttpInterface();
		HttpInterface(const std::string &name = "http_interface", const std::string &port = "8000", const std::string &document_root = "./");
		// HttpInterface(const HttpInterface & other) = delete;
		// HttpInterface(HttpInterface && other);
		// HttpInterface &operator=(const HttpInterface& other) = delete;
		// HttpInterface &operator=(HttpInterface&& other);
		ARIS_DELETE_BIG_FOUR(HttpInterface)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif