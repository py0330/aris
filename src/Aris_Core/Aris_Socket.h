#ifndef ARIS_SOCKET_H_
#define ARIS_SOCKET_H_

#include <functional>
#include <memory>
#include <stdexcept>

#include <Platform.h>
#include <Aris_Core.h>

namespace Aris
{
	namespace Core
	{
		/** \brief Socket connection class
		*
		*/
		class CONN final
		{
		public:
			/** \brief 构造函数
			*
			*
			*/
			CONN();
			/** \brief 析构函数
			*
			*
			*/
			~CONN();
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
			/** \brief 使用CONN发送数据
			*
			* \param data 待发送的数据。
			*/
			void SendData(const Aris::Core::MSG &data);
			/** \brief 使用CONN发送问讯，此后函数阻塞，直到对面应答
			*
			* \param data 待发送的数据。
			*/
			Aris::Core::MSG SendRequest(const Aris::Core::MSG &request);
			/** \brief 设置收到数据时，CONN所需要执行的函数
			*
			* \param OnReceivedData 为形如int(CONN*, Aris::Core::MSG &)的函数。每当CONN收到数据后在CONN自己的内部线程中执行。
			*/
			void SetOnReceivedData(std::function<int(CONN*, Aris::Core::MSG &)> = nullptr);
			/** \brief 设置服务器端收到连接后所执行的函数
			*
			* \param OnReceivedConnection 为形如int(CONN*, const char* pRemoteIP, int remotePort)的函数。每当CONN收到连接后在CONN自己的内部线程中执行。
			*/
			void SetOnReceivedConnection(std::function<int(CONN*, const char* pRemoteIP, int remotePort)> = nullptr);
			/** \brief 设置服务器端收到连接后所执行的函数
			*
			* \param OnLoseConnection 为形如int(CONN*)的函数。每当CONN失去连接后在CONN自己的内部线程中执行。
			*/
			void SetOnLoseConnection(std::function<int(CONN*)> = nullptr);
			/** \brief 设置收到讯问时，CONN所需要回答的函数
			*
			* \param 为形如Aris::Core::MSG(CONN*, Aris::Core::MSG &)的函数。每当CONN收到问讯后在CONN自己的内部线程中执行。
			*/
			void SetOnReceiveRequest(std::function<Aris::Core::MSG(CONN*, Aris::Core::MSG &)> = nullptr);
			/** \brief 设置出现监听错误时，CONN所需要执行的函数
			*
			* \param 为形如void(CONN*)的函数。在CONN自己的内部线程中执行。
			*/
			void SetOnAcceptError(std::function<void(CONN*)> = nullptr);
			/** \brief 设置出现接收数据错误时，CONN所需要执行的函数
			*
			* \param 为形如void(CONN*)的函数。在CONN自己的内部线程中执行。
			*/
			void SetOnReceiveError(std::function<void(CONN*)> = nullptr);

		public:
			enum STATE
			{
				IDLE,/*!< \brief 空闲状态 */
				WAITING_FOR_CONNECTION,/*!< \brief 服务器已经打开端口，等待客户端连接 */
				WORKING,/*!< \brief Socket已经连接好，可以传输数据 */
				WAITING_FOR_REPLY
			};

			class START_SERVER_ERROR :public std::runtime_error
			{
			public:
				CONN *pConn;
				int id;

			private:
				START_SERVER_ERROR(const char* what, CONN *pConn, int id)
					: runtime_error(what)
					, pConn(pConn)
					, id(id)
				{

				}

				friend class CONN;
			};
			class CONNECT_ERROR :public std::runtime_error
			{
				friend class CONN;
			public:
				CONN *pConn;
				int id;

			private:
				CONNECT_ERROR(const char* what, CONN *pConn, int id)
					: runtime_error(what)
					, pConn(pConn)
					, id(id)
				{

				}
			};
			class SEND_DATA_ERROR :public std::runtime_error
			{
				friend class CONN;
			public:
				CONN *pConn;
				int id;

			private:
				SEND_DATA_ERROR(const char* what, CONN *pConn, int id)
					: runtime_error(what)
					, pConn(pConn)
					, id(id)
				{

				}
			};
			class SEND_REQUEST_ERROR :public std::runtime_error
			{
			public:
				CONN *pConn;
				int id;

				friend class CONN;

			private:
				SEND_REQUEST_ERROR(const char* what, CONN *pConn, int id)
					: runtime_error(what)
					, pConn(pConn)
					, id(id)
				{
				}
			};

		private:
			CONN(const CONN & other) = delete;
			CONN(CONN && other) = delete;
			CONN &operator=(const CONN& other) = delete;
			CONN &operator=(CONN&& other) = delete;

		private:
			struct CONN_STRUCT;
			const std::unique_ptr<CONN_STRUCT> pConnStruct;
			static void _ReceiveThread(CONN::CONN_STRUCT* pCONN_STRUCT);
			static void _AcceptThread(CONN::CONN_STRUCT* pCONN_STRUCT);
		};
	}
}


#endif
