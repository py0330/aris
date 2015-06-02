#include "Aris_Socket.h"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <stdint.h>
#include <new>

#ifdef PLATFORM_IS_WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <ws2tcpip.h>
#endif

#ifdef PLATFORM_IS_LINUX
#include<pthread.h>
#include<semaphore.h>
#include<netdb.h>
#include<unistd.h>
#include<arpa/inet.h>
#endif

namespace Aris
{
	namespace Core
	{
		struct CONN::CONN_STRUCT
		{
			CONN* pConn;
			
			int _iRes, _Err;
			int _LisnSocket, _ConnSocket;   //也可以用SOCKET类型
			struct sockaddr_in _ServerAddr, _ClientAddr;
			socklen_t _SinSize;
			CONN::STATE _ConnState;

			union
			{
				unsigned int _ReceivedDataLength;
				char _ReceivedDataHeader[MSG_HEADER_LENGTH];
			};

			std::function<int(CONN *, Aris::Core::MSG &)> _OnReceivedData;
			std::function<int(CONN *, const char *, int)> _OnReceivedConnection;
			std::function<int(CONN *)> _OnLoseConnection;

			/*线程同步变量*/
			std::thread _recvDataThread, _recvConnThread;
			std::recursive_mutex _re_mutex;
			std::mutex _close_mutex,_cv_mutex;
			std::condition_variable _cv;

			Aris::Core::MSG _ReceivedData;//收到的数据，为临时变量，外界无法访问
			
			/* 连接的socket */
#ifdef PLATFORM_IS_WINDOWS
			WSADATA _WsaData;              //windows下才用，linux下无该项
#endif
			CONN_STRUCT()
				: _LisnSocket(0)
				, _ConnSocket(0)
				, _OnReceivedData(nullptr)
				, _OnReceivedConnection(nullptr)
				, _OnLoseConnection(nullptr)
				, _SinSize(sizeof(struct sockaddr_in))
				, _ConnState(CONN::IDLE)
			{
			};

			~CONN_STRUCT() = default;
		};
		
		void CONN::_AcceptThread(CONN::CONN_STRUCT* pConnS)
		{
			int lisnSock,connSock;
			struct sockaddr_in clientAddr;
			socklen_t sinSize; 

			/*以下从对象中copy内容，此时Start_Server在阻塞，因此CONN内部数据安全，
			拷贝好后告诉Start_Server函数已经拷贝好*/
			lisnSock = pConnS->_LisnSocket;
			clientAddr = pConnS->_ClientAddr;
			sinSize = pConnS->_SinSize;

			pConnS->_ConnState = WAITING_FOR_CONNECTION;
			
			/*通知主线程，accept线程已经拷贝完毕，准备监听*/
			std::unique_lock<std::mutex> cv_lck(pConnS->_cv_mutex);
			pConnS->_cv.notify_one();
			cv_lck.unlock();
			cv_lck.release();

			/* 服务器阻塞,直到客户程序建立连接 */
			connSock = accept(lisnSock, (struct sockaddr *)(&clientAddr), &sinSize);

			/*检查是否正在Close，如果不能锁住，则证明正在close，于是结束线程释放资源*/
			std::unique_lock<std::mutex> cls_lck(pConnS->_close_mutex, std::defer_lock);
			if (!cls_lck.try_lock())
			{
				return;
			}

			std::unique_lock<std::recursive_mutex> lck(pConnS->_re_mutex);
			cls_lck.unlock();
			cls_lck.release();


			/*否则，开始开启数据线程*/
			if (connSock == -1)
			{
				pConnS->pConn->Close();
				return;
			}

			/* 创建线程 */
			pConnS->_ConnState = CONN::WORKING;
			pConnS->_ConnSocket = connSock;
			pConnS->_ClientAddr = clientAddr;

			cv_lck = std::unique_lock<std::mutex>(pConnS->_cv_mutex);
			pConnS->_recvDataThread = std::thread(_ReceiveThread, pConnS);
			pConnS->_cv.wait(cv_lck);


			if (pConnS->_OnReceivedConnection != nullptr)
			{
				pConnS->_OnReceivedConnection(pConnS->pConn, inet_ntoa(pConnS->_ClientAddr.sin_addr), ntohs(pConnS->_ClientAddr.sin_port));
			}

			return;
		}
		void CONN::_ReceiveThread(CONN::CONN_STRUCT* pConnS)
		{
			char Header[MSG_HEADER_LENGTH];
			int connSocket = pConnS->_ConnSocket;

			/*通知accept线程已经准备好，下一步开始收发数据*/
			std::unique_lock<std::mutex> cv_lck(pConnS->_cv_mutex);
			pConnS->_cv.notify_one();
			cv_lck.unlock();
			cv_lck.release();

			/*开启接受数据的循环*/
			while (1)
			{
				int res = recv(connSocket, Header, MSG_HEADER_LENGTH, 0);

				/*检查是否正在Close，如果不能锁住，则证明正在close，于是结束线程释放资源，
				若能锁住，则开始获取CONN_STRUCT所有权*/
				std::unique_lock<std::mutex> cls_lck(pConnS->_close_mutex, std::defer_lock);
				if (!cls_lck.try_lock())
				{
					return;
				}

				std::unique_lock<std::recursive_mutex> lck(pConnS->_re_mutex);
				cls_lck.unlock();
				cls_lck.release();
				
				/*证明没有在close，于是正常接收数据*/
				pConnS->_iRes = res;
				memcpy(pConnS->_ReceivedDataHeader, Header, MSG_HEADER_LENGTH);


				if (pConnS->_iRes <= 0)
				{
					pConnS->pConn->Close();

					if (pConnS->_OnLoseConnection != 0)
						pConnS->_OnLoseConnection(pConnS->pConn);

					return;
				}

				/*读取数据头*/
				pConnS->_ReceivedData.SetLength(pConnS->_ReceivedDataLength);
				memcpy(pConnS->_ReceivedData._pData, pConnS->_ReceivedDataHeader, MSG_HEADER_LENGTH);

				/*读取数据*/
				if (pConnS->_ReceivedData.GetLength()>0)
					pConnS->_iRes = recv(pConnS->_ConnSocket, pConnS->_ReceivedData.GetDataAddress(), pConnS->_ReceivedData.GetLength(), 0);

				if (pConnS->_iRes <= 0)
				{
					pConnS->pConn->Close();

					if (pConnS->_OnLoseConnection != 0)
						pConnS->_OnLoseConnection(pConnS->pConn);

					return;
				}
				else
				{
					pConnS->_OnReceivedData(pConnS->pConn, pConnS->_ReceivedData);
				}
			}		
		}

		CONN::CONN()
			:pConnStruct(reinterpret_cast<CONN_STRUCT*>(_pData))
		{
			static_assert(sizeof(CONN_STRUCT) <= sizeof(CONN::_pData), "Aris::Core::CONN need more memory");

			new(pConnStruct)CONN_STRUCT;
			pConnStruct->pConn = this;
		}
		CONN::~CONN()
		{
			Close();
			pConnStruct->~CONN_STRUCT();
		}

		void CONN::Close()
		{
			std::lock(pConnStruct->_re_mutex, pConnStruct->_close_mutex);
			std::unique_lock<std::recursive_mutex> lck1(pConnStruct->_re_mutex, std::adopt_lock);
			std::unique_lock<std::mutex> lck2(pConnStruct->_close_mutex, std::adopt_lock);
			
			if (pConnStruct->_ConnState == CONN::WAITING_FOR_CONNECTION)
			{
#ifdef PLATFORM_IS_WINDOWS
				shutdown(pConnStruct->_LisnSocket, 2);
				closesocket(pConnStruct->_LisnSocket);
				WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
				shutdown(pConnStruct->_LisnSocket, 2);
				close(pConnStruct->_LisnSocket);
#endif
			}
			else if (pConnStruct->_ConnState == CONN::WORKING)
			{
#ifdef PLATFORM_IS_WINDOWS
				shutdown(pConnStruct->_ConnSocket, 2);
				shutdown(pConnStruct->_LisnSocket, 2);
				closesocket(pConnStruct->_ConnSocket);
				closesocket(pConnStruct->_LisnSocket);
				WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
				shutdown(pConnStruct->_ConnSocket, 2);
				shutdown(pConnStruct->_LisnSocket, 2);
				close(pConnStruct->_ConnSocket);
				close(pConnStruct->_LisnSocket);
#endif
			}
			
			if (std::this_thread::get_id() == pConnStruct->_recvDataThread.get_id())
			{
				pConnStruct->_recvDataThread.detach();
			}
			else if(pConnStruct->_recvDataThread.joinable())
			{
				pConnStruct->_recvDataThread.join();
			}
				

			if (std::this_thread::get_id() == pConnStruct->_recvConnThread.get_id())
			{
				pConnStruct->_recvConnThread.detach();
			}
			else if(pConnStruct->_recvConnThread.joinable())
			{
				pConnStruct->_recvConnThread.join();
			}

			pConnStruct->_ConnState = CONN::IDLE;
		}
		bool CONN::IsConnected()
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);

			if (pConnStruct->_ConnState == WORKING)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		int CONN::StartServer(const char *port)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);
			
			if (pConnStruct->_ConnState != IDLE)
			{
				return -1;
			}

			/* 启动服务器 */
#ifdef PLATFORM_IS_WINDOWS 
			if (WSAStartup(0x0101, &pConnStruct->_WsaData) != 0)
			{
				return -2;
			}
#endif

			/* 服务器端开始建立socket描述符 */
			if ((pConnStruct->_LisnSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
			{
				return -3;
			}

			/* 服务器端填充_ServerAddr结构 */
			memset(&pConnStruct->_ServerAddr, 0, sizeof(struct sockaddr_in));
			pConnStruct->_ServerAddr.sin_family = AF_INET;
			pConnStruct->_ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
			pConnStruct->_ServerAddr.sin_port = htons(atoi(port));

			/* 捆绑_LisnSocket描述符 */
			if (::bind(pConnStruct->_LisnSocket, (struct sockaddr *)(&pConnStruct->_ServerAddr), sizeof(struct sockaddr)) == -1)
			{
#ifdef PLATFORM_IS_WINDOWS
				pConnStruct->_Err = WSAGetLastError();
#endif
				return -4;
			}

			/* 监听_LisnSocket描述符 */
			if (listen(pConnStruct->_LisnSocket, 5) == -1)
			{
				return -5;
			}

			/* 启动等待连接的线程 */
			std::unique_lock<std::mutex> cv_lck(pConnStruct->_cv_mutex);
			pConnStruct->_recvConnThread = std::thread(CONN::_AcceptThread, this->pConnStruct);
			pConnStruct->_cv.wait(cv_lck);
			
			return 0;
		}
		int CONN::Connect(const char *address, const char *port)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);
			
			if (pConnStruct->_ConnState != IDLE)
			{
				return false;
			}

			/* 启动服务器 */
#ifdef PLATFORM_IS_WINDOWS
			if (WSAStartup(0x0101, &pConnStruct->_WsaData) != 0)
			{
				return false;
			}
#endif
			/* 服务器端开始建立socket描述符 */
			if ((pConnStruct->_ConnSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
			{
				return false;
			}

			/* 客户端填充_ServerAddr结构 */
			memset(&pConnStruct->_ServerAddr, 0, sizeof(pConnStruct->_ServerAddr));
			pConnStruct->_ServerAddr.sin_family = AF_INET;
			pConnStruct->_ServerAddr.sin_addr.s_addr = inet_addr(address); //与linux不同
			pConnStruct->_ServerAddr.sin_port = htons(atoi(port));

			/* 连接 */
			if (connect(pConnStruct->_ConnSocket, (const struct sockaddr *)&pConnStruct->_ServerAddr, sizeof(pConnStruct->_ServerAddr)) == -1)
			{
				return false;
			}

			/* Start Thread */
			pConnStruct->_recvDataThread = std::thread(_ReceiveThread, this->pConnStruct);
			
			pConnStruct->_ConnState = WORKING;

			return 0;
		}
		int CONN::SendData(const Aris::Core::MSG &data)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);
			
			int ret = 0;

			if (pConnStruct->_ConnState != WORKING)
			{
				ret = -1;
			}
			else
			{
				pConnStruct->_Err = send(pConnStruct->_ConnSocket, data._pData, data.GetLength() + MSG_HEADER_LENGTH, 0);
				if (pConnStruct->_Err == -1)
					ret = -1;
			}

			return ret;
		}
		int CONN::SetCallBackOnReceivedData(std::function<int(CONN*, Aris::Core::MSG &)> OnReceivedData)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);
			
			pConnStruct->_OnReceivedData = OnReceivedData;
			return 0;
		}
		int CONN::SetCallBackOnReceivedConnection(std::function<int(CONN*, const char*, int)> OnReceivedConnection)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);
			
			pConnStruct->_OnReceivedConnection = OnReceivedConnection;
			return 0;
		}
		int CONN::SetCallBackOnLoseConnection(std::function<int(CONN*)> OnLoseConnection)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_re_mutex);
			
			pConnStruct->_OnLoseConnection = OnLoseConnection;
			return 0;
		}
	}
}

