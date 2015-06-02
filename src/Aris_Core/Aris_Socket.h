#ifndef ARIS_SOCKET_H_
#define ARIS_SOCKET_H_

#include <functional>

#include <Platform.h>
#include <Aris_Thread.h>
#include <Aris_Core.h>

namespace Aris
{
	namespace Core
	{
		/** \brief Socket类型
		*
		*/
		class CONN
		{
		private:
			struct CONN_STRUCT;
			CONN_STRUCT *pConnStruct;
			static void _ReceiveThread(CONN::CONN_STRUCT* pCONN_STRUCT);
			static void _AcceptThread(CONN::CONN_STRUCT* pCONN_STRUCT);

#ifdef PLATFORM_IS_WINDOWS 
			char _pData[650];
#endif
#ifdef PLATFORM_IS_LINUX 
			char _pData[400];
#endif
			

		public:
			enum STATE
			{
				IDLE,/*!< \brief 空闲状态 */
				WAITING_FOR_CONNECTION,/*!< \brief 服务器已经打开端口，等待客户端连接 */
				WORKING/*!< \brief Socket已经连接好，可以传输数据 */
			};
			
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

			CONN(const CONN & other) = delete;
			CONN &operator=(const CONN& other) = delete;
			CONN(CONN && other) = delete;
			CONN &operator=(CONN&& other) = delete;

		public:
			/** \brief 查看Socket是否处于连接状态
			*
			*
			*/
			bool IsConnected();
			/** \brief 本Socket作为服务器来使用，并打开相应端口
			*
			*\param port 为服务器打开的端口号，例如"1234"
			*/
			int StartServer(const char *port);
			/** \brief 当Socket作为服务器端使用时，关闭服务器端
			*
			*/
			int Connect(const char *address, const char *port);
			/** \brief 关闭客户端
			*
			*/
			void Close();
			/** \brief 设置收到数据时，CONN所需要执行的函数
			*
			* \param OnReceivedData 为形如int(CONN*, Aris::Core::MSG &)的函数。每当CONN收到数据后在CONN自己的内部线程中执行。
			*/
			int SetCallBackOnReceivedData(std::function<int(CONN*, Aris::Core::MSG &)> = nullptr);
			/** \brief 设置服务器端收到连接后所执行的函数
			*
			* \param OnReceivedConnection 为形如int(CONN*, const char* pRemoteIP, int remotePort)的函数。每当CONN收到连接后在CONN自己的内部线程中执行。
			*/
			int SetCallBackOnReceivedConnection(std::function<int(CONN*, const char* pRemoteIP, int remotePort)> = nullptr);
			/** \brief 设置服务器端收到连接后所执行的函数
			*
			* \param OnLoseConnection 为形如int(CONN*)的函数。每当CONN失去连接后在CONN自己的内部线程中执行。
			*/
			int SetCallBackOnLoseConnection(std::function<int(CONN*)> = nullptr);
			/** \brief 使用CONN发送数据
			*
			* \param data 待发送的数据。
			*/
			int SendData(const Aris::Core::MSG &data);
		};
	}
}


#endif
