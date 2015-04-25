#include "Aris_Socket.h"

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

using namespace std;

#define pCONN_STRUCT ((CONN_STRUCT*)(((CONN *)pConn)->_pData))

namespace Aris
{
	namespace Core
	{
		struct CONN_STRUCT
		{
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

			int(*_OnReceivedData)(CONN *, Aris::Core::MSG &);
			int(*_OnReceivedConnection)(CONN *, const char *, int);
			int(*_OnLoseConnection)(CONN *);

			Aris::Core::THREAD _ReceiveConnectionThread, _ReceiveDataThread;
			Aris::Core::MUTEX _DataSection; //保护所有数据
			Aris::Core::MSG _ReceivedData;//收到的数据，为临时变量，外界无法访问
			

			/* 连接的socket */
#ifdef PLATFORM_IS_WINDOWS
			WSADATA _WsaData;              //windows下才用，linux下无该项
#endif
		};
		
		void* _ConnReceiveConnectionThreadFunc(void* pConn)
		{
			/* 服务器阻塞,直到客户程序建立连接 */
			pCONN_STRUCT->_ConnSocket = accept(pCONN_STRUCT->_LisnSocket, (struct sockaddr *)(&pCONN_STRUCT->_ClientAddr), &pCONN_STRUCT->_SinSize);

			pCONN_STRUCT->_DataSection.Lock();

			if (pCONN_STRUCT->_ConnSocket == -1)
			{
				pCONN_STRUCT->_DataSection.Unlock();
				return 0;
			}

			/* 创建线程 */

			pCONN_STRUCT->_ReceiveDataThread.Start(pCONN_STRUCT);
			pCONN_STRUCT->_ConnState = CONN::WORKING;

			if (pCONN_STRUCT->_OnReceivedConnection != 0)
			{
				pCONN_STRUCT->_OnReceivedConnection((CONN*)pConn, inet_ntoa(pCONN_STRUCT->_ClientAddr.sin_addr), ntohs(pCONN_STRUCT->_ClientAddr.sin_port));
			}
				

			pCONN_STRUCT->_DataSection.Unlock();

			return 0;
		}
		void* _ConnReceiveDataThreadFunc(void* pConn)
		{
			while (1)
			{
				pCONN_STRUCT->_iRes = recv(pCONN_STRUCT->_ConnSocket, pCONN_STRUCT->_ReceivedDataHeader, MSG_HEADER_LENGTH, 0);
				pCONN_STRUCT->_DataSection.Lock();

				if (pCONN_STRUCT->_iRes <= 0)
				{
#ifdef PLATFORM_IS_WINDOWS
					closesocket(pCONN_STRUCT->_ConnSocket);
					closesocket(pCONN_STRUCT->_LisnSocket);
					WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
					close(pCONN_STRUCT->_ConnSocket);
					close(pCONN_STRUCT->_LisnSocket);
#endif
					pCONN_STRUCT->_ConnState = CONN::IDLE;


					pCONN_STRUCT->_DataSection.Unlock();

					if (pCONN_STRUCT->_OnLoseConnection != 0)
						pCONN_STRUCT->_OnLoseConnection((CONN*)pConn);

					return 0;
				}

				/*读取数据头*/
				pCONN_STRUCT->_ReceivedData.SetLength(pCONN_STRUCT->_ReceivedDataLength);
				memcpy(pCONN_STRUCT->_ReceivedData._pData, pCONN_STRUCT->_ReceivedDataHeader, MSG_HEADER_LENGTH);

				/*读取数据*/
				if (pCONN_STRUCT->_ReceivedData.GetLength()>0)
					pCONN_STRUCT->_iRes = recv(pCONN_STRUCT->_ConnSocket, pCONN_STRUCT->_ReceivedData.GetDataAddress(), pCONN_STRUCT->_ReceivedData.GetLength(), 0);


				if (pCONN_STRUCT->_iRes <= 0)
				{
#ifdef PLATFORM_IS_WINDOWS
					closesocket(pCONN_STRUCT->_ConnSocket);
					closesocket(pCONN_STRUCT->_LisnSocket);
					WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
					close(pCONN_STRUCT->_ConnSocket);
					close(pCONN_STRUCT->_LisnSocket);
#endif
					pCONN_STRUCT->_ConnState = CONN::IDLE;

					pCONN_STRUCT->_DataSection.Unlock();

					if (pCONN_STRUCT->_OnLoseConnection != 0)
						pCONN_STRUCT->_OnLoseConnection((CONN*)pConn);

					return 0;
				}
				else
				{
					pCONN_STRUCT->_DataSection.Unlock();
					pCONN_STRUCT->_OnReceivedData((CONN*)pConn, pCONN_STRUCT->_ReceivedData);
				}
			}
		}

		CONN::CONN()
		{
			static_assert(sizeof(CONN_STRUCT) <= CONN::_SIZE, "Aris::Core::CONN need more memory");

			((CONN_STRUCT*)_pData)->_LisnSocket = 0;
			((CONN_STRUCT*)_pData)->_ConnSocket = 0;
			((CONN_STRUCT*)_pData)->_OnReceivedData = 0;
			((CONN_STRUCT*)_pData)->_OnReceivedConnection = 0;
			((CONN_STRUCT*)_pData)->_OnLoseConnection = 0;
			((CONN_STRUCT*)_pData)->_SinSize = sizeof(struct sockaddr_in);
			((CONN_STRUCT*)_pData)->_ConnState = IDLE;

			new(&((CONN_STRUCT*)_pData)->_ReceiveConnectionThread)Aris::Core::THREAD(_ConnReceiveConnectionThreadFunc);
			new(&((CONN_STRUCT*)_pData)->_ReceiveDataThread)Aris::Core::THREAD(_ConnReceiveDataThreadFunc);
			new(&((CONN_STRUCT*)_pData)->_DataSection)Aris::Core::MUTEX;
			new(&((CONN_STRUCT*)_pData)->_ReceivedData)Aris::Core::MSG;
			
		}
		CONN::~CONN()
		{
			_TerminateConnThread();

			(&((CONN_STRUCT*)_pData)->_ReceiveConnectionThread)->~THREAD();
			(&((CONN_STRUCT*)_pData)->_ReceiveDataThread)->~THREAD();
			(&((CONN_STRUCT*)_pData)->_DataSection)->~MUTEX();
			(&((CONN_STRUCT*)_pData)->_ReceivedData)->~MSG();
		}

		bool CONN::_TerminateConnThread()
		{
			((CONN_STRUCT*)_pData)->_DataSection.Lock();

			if (((CONN_STRUCT*)_pData)->_ConnState == WAITING_FOR_CONNECTION)
			{
				((CONN_STRUCT*)_pData)->_ReceiveConnectionThread.Terminate();

#ifdef PLATFORM_IS_WINDOWS
				closesocket(((CONN_STRUCT*)_pData)->_ConnSocket);
				closesocket(((CONN_STRUCT*)_pData)->_LisnSocket);
				WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
				close(((CONN_STRUCT*)_pData)->_ConnSocket);
				close(((CONN_STRUCT*)_pData)->_LisnSocket);
#endif

				((CONN_STRUCT*)_pData)->_ConnState = IDLE;
			}
			else if (((CONN_STRUCT*)_pData)->_ConnState == WORKING)
			{
				((CONN_STRUCT*)_pData)->_ReceiveDataThread.Terminate();

#ifdef PLATFORM_IS_WINDOWS
				closesocket(((CONN_STRUCT*)_pData)->_ConnSocket);
				closesocket(((CONN_STRUCT*)_pData)->_LisnSocket);
				WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
				close(((CONN_STRUCT*)_pData)->_ConnSocket);
				close(((CONN_STRUCT*)_pData)->_LisnSocket);
#endif
				((CONN_STRUCT*)_pData)->_ConnState = IDLE;
			}

			((CONN_STRUCT*)_pData)->_DataSection.Unlock();
			return true;
		}

		bool CONN::IsConnected()
		{
			if (((CONN_STRUCT*)_pData)->_ConnState == WORKING)
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
			((CONN_STRUCT*)_pData)->_DataSection.Lock();

			if (((CONN_STRUCT*)_pData)->_ConnState != IDLE)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return -1;
			}

			/* 启动服务器 */
#ifdef PLATFORM_IS_WINDOWS 
			if (WSAStartup(0x0101, &((CONN_STRUCT*)_pData)->_WsaData) != 0)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return -2;
			}
#endif

			/* 服务器端开始建立socket描述符 */
			if ((((CONN_STRUCT*)_pData)->_LisnSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return -3;
			}

			/* 服务器端填充_ServerAddr结构 */
			memset(&((CONN_STRUCT*)_pData)->_ServerAddr, 0, sizeof(struct sockaddr_in));
			((CONN_STRUCT*)_pData)->_ServerAddr.sin_family = AF_INET;
			((CONN_STRUCT*)_pData)->_ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
			((CONN_STRUCT*)_pData)->_ServerAddr.sin_port = htons(atoi(port));

			/* 捆绑_LisnSocket描述符 */
			if (bind(((CONN_STRUCT*)_pData)->_LisnSocket, (struct sockaddr *)(&((CONN_STRUCT*)_pData)->_ServerAddr), sizeof(struct sockaddr)) == -1)
			{
#ifdef PLATFORM_IS_WINDOWS
				((CONN_STRUCT*)_pData)->_Err = WSAGetLastError();
#endif
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return -4;
			}

			/* 监听_LisnSocket描述符 */
			if (listen(((CONN_STRUCT*)_pData)->_LisnSocket, 5) == -1)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return -5;
			}

			/* 启动等待连接的线程 */
			if (((CONN_STRUCT*)_pData)->_ReceiveConnectionThread.IsRunning())
			{
				((CONN_STRUCT*)_pData)->_ReceiveConnectionThread.Join();
			}

			if (((CONN_STRUCT*)_pData)->_ReceiveConnectionThread.Start(this) != 0)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return -6;
			}
			
			((CONN_STRUCT*)_pData)->_ConnState = WAITING_FOR_CONNECTION;

			((CONN_STRUCT*)_pData)->_DataSection.Unlock();
			return 0;
		}
		int CONN::CloseServer()
		{
			_TerminateConnThread();

			return 0;
		}
		int CONN::Connect(const char *address, const char *port)
		{
			((CONN_STRUCT*)_pData)->_DataSection.Lock();

			if (((CONN_STRUCT*)_pData)->_ConnState != IDLE)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return false;
			}

			/* 启动服务器 */
#ifdef PLATFORM_IS_WINDOWS
			if (WSAStartup(0x0101, &((CONN_STRUCT*)_pData)->_WsaData) != 0)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return false;
			}
#endif
			/* 服务器端开始建立socket描述符 */
			if ((((CONN_STRUCT*)_pData)->_ConnSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return false;
			}

			/* 客户端填充_ServerAddr结构 */
			memset(&((CONN_STRUCT*)_pData)->_ServerAddr, 0, sizeof(((CONN_STRUCT*)_pData)->_ServerAddr));
			((CONN_STRUCT*)_pData)->_ServerAddr.sin_family = AF_INET;
			((CONN_STRUCT*)_pData)->_ServerAddr.sin_addr.s_addr = inet_addr(address); //与linux不同
			((CONN_STRUCT*)_pData)->_ServerAddr.sin_port = htons(atoi(port));

			/* 连接 */

			if (connect(((CONN_STRUCT*)_pData)->_ConnSocket, (const struct sockaddr *)&((CONN_STRUCT*)_pData)->_ServerAddr, sizeof(((CONN_STRUCT*)_pData)->_ServerAddr)) == -1)
			{
				((CONN_STRUCT*)_pData)->_DataSection.Unlock();
				return false;
			}

			((CONN_STRUCT*)_pData)->_DataSection.Unlock();

			/* Start Thread */
			((CONN_STRUCT*)_pData)->_ReceiveDataThread.Start(this);
			((CONN_STRUCT*)_pData)->_ConnState = WORKING;

			return 0;
		}
		int CONN::DisConnect()
		{
			_TerminateConnThread();

			return 0;
		}
		int CONN::SendData(const Aris::Core::MSG &data)
		{
			int ret = 0;
			
			((CONN_STRUCT*)_pData)->_DataSection.Lock();

			if (((CONN_STRUCT*)_pData)->_ConnState != WORKING)
			{
				ret = -1;
			}
			else
			{
				((CONN_STRUCT*)_pData)->_Err = send(((CONN_STRUCT*)_pData)->_ConnSocket, data._pData, data.GetLength() + MSG_HEADER_LENGTH, 0);
				if (((CONN_STRUCT*)_pData)->_Err == -1)
					ret = -1;
			}

			((CONN_STRUCT*)_pData)->_DataSection.Unlock();
			return ret;
		}
		int CONN::SetCallBackOnReceivedData(int(*ArgOnReceivedData)(CONN*,Aris::Core::MSG &))
		{
			((CONN_STRUCT*)_pData)->_OnReceivedData = ArgOnReceivedData;
			return 0;
		}
		int CONN::SetCallBackOnReceivedConnection(int(*OnReceivedConnection)(CONN *, const char *,int))
		{
			((CONN_STRUCT*)_pData)->_OnReceivedConnection = OnReceivedConnection;
			return 0;
		}
		int CONN::SetCallBackOnLoseConnection(int(*OnLoseConnection)(CONN*))
		{
			((CONN_STRUCT*)_pData)->_OnLoseConnection = OnLoseConnection;
			return 0;
		}
	}
}

