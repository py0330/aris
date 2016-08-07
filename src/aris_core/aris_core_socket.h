#ifndef ARIS_CORE_SOCKET_H_
#define ARIS_CORE_SOCKET_H_

#include <functional>
#include <memory>
#include <stdexcept>

#include <aris_core_msg.h>
#include <aris_core_xml.h>

namespace aris
{
	namespace core
	{
		class Socket : public Object
		{
		public:
			enum State
			{
				IDLE = 0,/*!< \brief 空闲状态 */
				WAITING_FOR_CONNECTION,/*!< \brief 服务器已经打开端口，等待客户端连接 */
				WORKING,/*!< \brief Socket已经连接好，可以传输数据 */
				WAITING_FOR_REPLY
			};
			class StartServerError :public std::runtime_error
			{
			public:
				Socket *socket_;
				int id_;
			private:
				StartServerError(const char* what, Socket *socket_in, int id) : runtime_error(what), socket_(socket_in), id_(id) {};
				friend class Socket;
			};
			class ConnectError :public std::runtime_error
			{
			public:
				Socket *socket_;
				int id_;
			private:
				ConnectError(const char* what, Socket *socket, int id) : runtime_error(what), socket_(socket), id_(id) {}
				friend class Socket;
			};
			class SendDataError :public std::runtime_error
			{
			public:
				Socket *socket_;
				int id_;
			private:
				SendDataError(const char* what, Socket *socket, int id) : runtime_error(what), socket_(socket), id_(id) {}
				friend class Socket;
			};
			class SendRequestError :public std::runtime_error
			{
			public:
				Socket *socket_;
				int id_;
			private:
				SendRequestError(const char* what, Socket *socket, int id) : runtime_error(what), socket_(socket), id_(id) {}
				friend class Socket;
			};
		
		public:
			static auto Type()->const std::string &{ static const std::string type("Socket"); return std::ref(type); }
			virtual auto type() const->const std::string& override{ return Type(); }
			auto isConnected()->bool;
			auto state()->State;
			auto startServer(const std::string &port = std::string())->void;
			auto connect(const std::string &remote_ip = std::string(), const std::string &port = std::string())->void;
			auto stop()->void;
			auto sendMsg(const aris::core::Msg &data)->void;
			auto sendRequest(const aris::core::Msg &request)->Msg;
			auto remoteIP()const->const std::string &;
			auto port()const->const std::string &;
			auto setRemoteIP(const std::string &remote_ip)->void;
			auto setPort(const std::string &port)->void;
			auto setOnReceivedMsg(std::function<int(Socket*, aris::core::Msg &)> = nullptr)->void;
			auto setOnReceivedConnection(std::function<int(Socket*, const char* remote_ip, int remote_port)> = nullptr)->void;
			auto setOnLoseConnection(std::function<int(Socket*)> = nullptr)->void;
			auto setOnReceivedRequest(std::function<aris::core::Msg(Socket*, aris::core::Msg &)> = nullptr)->void;
			auto setOnAcceptError(std::function<void(Socket*)> = nullptr)->void;
			auto setOnReceiveError(std::function<void(Socket*)> = nullptr)->void;

			virtual ~Socket();
			Socket(const std::string &name, const std::string& remote_ip = std::string(), const std::string& port = std::string());
			Socket(Object &father, const aris::core::XmlElement &xml_ele);
			Socket(const Socket & other) = delete;
			Socket(Socket && other) = delete;
			Socket &operator=(const Socket& other) = delete;
			Socket &operator=(Socket&& other) = delete;
		
		private:
			struct Imp;
			const std::unique_ptr<Imp> imp_;
		};
	}
}


#endif
