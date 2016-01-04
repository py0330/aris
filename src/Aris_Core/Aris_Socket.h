#ifndef ARIS_SOCKET_H_
#define ARIS_SOCKET_H_

#include <functional>
#include <memory>
#include <stdexcept>

#include <Aris_Core.h>

namespace Aris
{
	namespace Core
	{
		/** \brief Socket connection class
		*
		*/
		class Socket final
		{
		public:
			/** \brief 构造函数
			*
			*
			*/
			Socket();
			/** \brief 析构函数
			*
			*
			*/
			~Socket();
			/** \brief 查看Socket是否处于连接状态
			*
			*
			*/
			bool IsConnected();
			/** \brief 本Socket作为服务器来使用，并打开相应端口
			*
			*\param port 为服务器打开的端口号，例如"1234"
			*/
			void StartServer(const char *port);
			/** \brief 当Socket作为服务器端使用时，关闭服务器端
			*
			*/
			void Connect(const char *address, const char *port);
			/** \brief 关闭客户端
			*
			*/
			void Close();
			/** \brief 使用Socket发送数据
			*
			* \param data 待发送的数据。
			*/
			void SendData(const Aris::Core::Msg &data);
			/** \brief 使用Socket发送问讯，此后函数阻塞，直到对面应答
			*
			* \param data 待发送的数据。
			*/
			Aris::Core::Msg SendRequest(const Aris::Core::Msg &request);
			/** \brief 设置收到数据时，Socket所需要执行的函数
			*
			* \param OnReceivedData 为形如int(Socket*, Aris::Core::Msg &)的函数。每当Socket收到数据后在Socket自己的内部线程中执行。
			*/
			void SetOnReceivedData(std::function<int(Socket*, Aris::Core::Msg &)> = nullptr);
			/** \brief 设置服务器端收到连接后所执行的函数
			*
			* \param OnReceivedConnection 为形如int(Socket*, const char* pRemoteIP, int remotePort)的函数。每当Socket收到连接后在Socket自己的内部线程中执行。
			*/
			void SetOnReceivedConnection(std::function<int(Socket*, const char* pRemoteIP, int remotePort)> = nullptr);
			/** \brief 设置服务器端收到连接后所执行的函数
			*
			* \param OnLoseConnection 为形如int(Socket*)的函数。每当Socket失去连接后在Socket自己的内部线程中执行。
			*/
			void SetOnLoseConnection(std::function<int(Socket*)> = nullptr);
			/** \brief 设置收到讯问时，Socket所需要回答的函数
			*
			* \param 为形如Aris::Core::Msg(Socket*, Aris::Core::Msg &)的函数。每当Socket收到问讯后在Socket自己的内部线程中执行。
			*/
			void SetOnReceiveRequest(std::function<Aris::Core::Msg(Socket*, Aris::Core::Msg &)> = nullptr);
			/** \brief 设置出现监听错误时，Socket所需要执行的函数
			*
			* \param 为形如void(Socket*)的函数。在Socket自己的内部线程中执行。
			*/
			void SetOnAcceptError(std::function<void(Socket*)> = nullptr);
			/** \brief 设置出现接收数据错误时，Socket所需要执行的函数
			*
			* \param 为形如void(Socket*)的函数。在Socket自己的内部线程中执行。
			*/
			void SetOnReceiveError(std::function<void(Socket*)> = nullptr);

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
				Socket *pConn;
				int id;

			private:
				StartServerError(const char* what, Socket *pConn, int id) : runtime_error(what), pConn(pConn), id(id) {};
				friend class Socket;
			};
			class ConnectError :public std::runtime_error
			{
			public:
				Socket *pConn;
				int id;

			private:
				ConnectError(const char* what, Socket *pConn, int id) : runtime_error(what), pConn(pConn), id(id) {};
				friend class Socket;
			};
			class SendDataError :public std::runtime_error
			{
			public:
				Socket *pConn;
				int id;

			private:
				SendDataError(const char* what, Socket *pConn, int id) : runtime_error(what), pConn(pConn), id(id) {};
				friend class Socket;
			};
			class SendRequestError :public std::runtime_error
			{
			public:
				Socket *pConn;
				int id;

			private:
				SendRequestError(const char* what, Socket *pConn, int id) : runtime_error(what), pConn(pConn), id(id) {};
				friend class Socket;
			};

		private:
			Socket(const Socket & other) = delete;
			Socket(Socket && other) = delete;
			Socket &operator=(const Socket& other) = delete;
			Socket &operator=(Socket&& other) = delete;

		private:
			struct Imp;
			const std::unique_ptr<Imp> pConnStruct;
		};
	}
}


#endif
