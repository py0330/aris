#ifndef ARIS_CORE_WEBSOCKET_H_
#define ARIS_CORE_WEBSOCKET_H_

#include <functional>
#include <memory>
#include <stdexcept>

#include <aris_core_object.h>
#include <aris_core_msg.h>

namespace aris::core
{
	class WebSocket : public Object
	{
	public:
		enum State
		{
			IDLE = 0,
			WAITING_FOR_CONNECTION,
			WORKING,
			WAITING_FOR_REPLY
		};
		class StartServerError :public std::runtime_error
		{
		public:
			WebSocket * socket_;
			int id_;
		private:
			StartServerError(const char* what, WebSocket *socket_in, int id) : runtime_error(what), socket_(socket_in), id_(id) {};
			friend class WebSocket;
		};
		class ConnectError :public std::runtime_error
		{
		public:
			WebSocket * socket_;
			int id_;
		private:
			ConnectError(const char* what, WebSocket *socket, int id) : runtime_error(what), socket_(socket), id_(id) {}
			friend class WebSocket;
		};
		class SendDataError :public std::runtime_error
		{
		public:
			WebSocket * socket_;
			int id_;
		private:
			SendDataError(const char* what, WebSocket *socket, int id) : runtime_error(what), socket_(socket), id_(id) {}
			friend class WebSocket;
		};
		class SendRequestError :public std::runtime_error
		{
		public:
			WebSocket * socket_;
			int id_;
		private:
			SendRequestError(const char* what, WebSocket *socket, int id) : runtime_error(what), socket_(socket), id_(id) {}
			friend class WebSocket;
		};

	public:
		static auto Type()->const std::string & { static const std::string type("WebSocket"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto isConnected()->bool;
		auto state()->State;
		auto startServer(const std::string &port = std::string())->void;
		auto connect(const std::string &remote_ip = std::string(), const std::string &port = std::string())->void;
		auto stop()->void;
		auto sendMsg(const aris::core::MsgBase &data)->void;
		auto remoteIP()const->const std::string &;
		auto port()const->const std::string &;
		auto setRemoteIP(const std::string &remote_ip)->void;
		auto setPort(const std::string &port)->void;
		auto setOnReceivedMsg(std::function<int(WebSocket*, aris::core::Msg &)> = nullptr)->void;
		auto setOnReceivedConnection(std::function<int(WebSocket*, const char* remote_ip, int remote_port)> = nullptr)->void;
		auto setOnLoseConnection(std::function<int(WebSocket*)> = nullptr)->void;
		auto setOnReceivedRequest(std::function<aris::core::Msg(WebSocket*, aris::core::Msg &)> = nullptr)->void;
		auto setOnAcceptError(std::function<void(WebSocket*)> = nullptr)->void;
		auto setOnReceiveError(std::function<void(WebSocket*)> = nullptr)->void;

		virtual ~WebSocket();
		WebSocket(const std::string &name = "socket", const std::string& remote_ip = "", const std::string& port = "");
		WebSocket(const WebSocket & other) = delete;
		WebSocket(WebSocket && other) = default;
		WebSocket &operator=(const WebSocket& other) = delete;
		WebSocket &operator=(WebSocket&& other) = default;

	private:
		struct Imp;
		const std::unique_ptr<Imp> imp_;
	};
}


#endif
