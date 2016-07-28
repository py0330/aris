#ifndef ARIS_CORE_SOCKET_H_
#define ARIS_CORE_SOCKET_H_

#include <functional>
#include <memory>
#include <stdexcept>

#include <aris_core_msg.h>

namespace aris
{
	namespace core
	{
		/// \brief Socket connection class
		///
		///
		class Socket final
		{
		public:
			auto isConnected()->bool;
			auto startServer(const char *port)->void;
			
			auto connect(const char *address, const char *port)->void;
			auto stop()->void;
			auto sendMsg(const aris::core::Msg &data)->void;
			auto sendRequest(const aris::core::Msg &request)->Msg;
			auto setOnReceivedMsg(std::function<int(Socket*, aris::core::Msg &)> = nullptr)->void;
			auto setOnReceivedConnection(std::function<int(Socket*, const char* remote_ip, int remote_port)> = nullptr)->void;
			auto setOnLoseConnection(std::function<int(Socket*)> = nullptr)->void;
			auto setOnReceivedRequest(std::function<aris::core::Msg(Socket*, aris::core::Msg &)> = nullptr)->void;
			auto setOnAcceptError(std::function<void(Socket*)> = nullptr)->void;
			auto setOnReceiveError(std::function<void(Socket*)> = nullptr)->void;
			
			virtual ~Socket();
			Socket();
			Socket(const Socket & other) = delete;
			Socket(Socket && other) = delete;
			Socket &operator=(const Socket& other) = delete;
			Socket &operator=(Socket&& other) = delete;
		public:
			enum State
			{
				IDLE,/*!< \brief 空闲状态 */
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

		private:
			struct Imp;
			const std::unique_ptr<Imp> imp_;
		};
	}
}


#endif
