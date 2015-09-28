#include "Platform.h"
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
			
			int _LisnSocket, _ConnSocket;   //也可以用SOCKET类型
			struct sockaddr_in _ServerAddr, _ClientAddr;
			socklen_t _SinSize;
			CONN::STATE _ConnState;
			

			std::function<Aris::Core::MSG(CONN *, Aris::Core::MSG &)> onReceivedRequest;
			std::function<int(CONN *, Aris::Core::MSG &)> onReceivedData;
			std::function<int(CONN *, const char *, int)> onReceivedConnection;
			std::function<int(CONN *)> onLoseConnection;

			std::function<void(CONN *)> onAcceptError;
			std::function<void(CONN *)> onReceiveError;
			

			/*线程同步变量*/
			std::recursive_mutex _state_mutex;

			std::thread _recvDataThread, _recvConnThread;
			std::mutex _close_mutex,_cv_mutex;
			std::condition_variable _cv;

			std::condition_variable_any _cv_reply_data_received;
			Aris::Core::MSG _replyData;
			
			/* 连接的socket */
#ifdef PLATFORM_IS_WINDOWS
			WSADATA _WsaData;              //windows下才用，linux下无该项
#endif
			CONN_STRUCT()
				: _LisnSocket(0)
				, _ConnSocket(0)
				, _SinSize(sizeof(struct sockaddr_in))
				, _ConnState(CONN::IDLE)
				, onReceivedRequest(nullptr)
				, onReceivedData(nullptr)
				, onReceivedConnection(nullptr)
				, onLoseConnection(nullptr)
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

			std::unique_lock<std::recursive_mutex> lck(pConnS->_state_mutex);
			cls_lck.unlock();
			cls_lck.release();

			/*否则，开始开启数据线程*/
			if (connSock == -1)
			{
				pConnS->pConn->Close();

				if (pConnS->onAcceptError != nullptr)
					pConnS->onAcceptError(pConnS->pConn);

				return;
			}
			

			/* 创建线程 */
			pConnS->_ConnState = CONN::WORKING;
			pConnS->_ConnSocket = connSock;
			pConnS->_ClientAddr = clientAddr;

			cv_lck = std::unique_lock<std::mutex>(pConnS->_cv_mutex);
			pConnS->_recvDataThread = std::thread(_ReceiveThread, pConnS);
			pConnS->_cv.wait(cv_lck);


			if (pConnS->onReceivedConnection != nullptr)
			{
				pConnS->onReceivedConnection(pConnS->pConn, inet_ntoa(pConnS->_ClientAddr.sin_addr), ntohs(pConnS->_ClientAddr.sin_port));
			}

			return;
		}
		void CONN::_ReceiveThread(CONN::CONN_STRUCT* pConnS)
		{
			union HEAD
			{
				MSG_HEADER msgHeader;
				char header[sizeof(MSG_HEADER)];
			} head;
			Aris::Core::MSG receivedData;
			
			int connSocket = pConnS->_ConnSocket;

			/*通知accept线程已经准备好，下一步开始收发数据*/
			std::unique_lock<std::mutex> cv_lck(pConnS->_cv_mutex);
			pConnS->_cv.notify_one();
			cv_lck.unlock();
			cv_lck.release();

			/*开启接受数据的循环*/
			while (1)
			{
				int res = recv(connSocket, head.header, sizeof(MSG_HEADER), 0);

				/*检查是否正在Close，如果不能锁住，则证明正在close，于是结束线程释放资源，
				若能锁住，则开始获取CONN_STRUCT所有权*/
				std::unique_lock<std::mutex> close_lck(pConnS->_close_mutex, std::defer_lock);
				if (!close_lck.try_lock())
				{
					return;
				}

				/*证明没有在close，于是正常接收消息头*/
				std::unique_lock<std::recursive_mutex> state_lck(pConnS->_state_mutex);
				close_lck.unlock();
				close_lck.release();
				
				if (res <= 0)
				{
					pConnS->pConn->Close();

					if (pConnS->onLoseConnection != 0)
						pConnS->onLoseConnection(pConnS->pConn);

					return;
				}

				/*接收消息本体*/
				receivedData.SetLength(head.msgHeader.msgLength);
				memcpy(receivedData._pData, head.header, sizeof(MSG_HEADER));

				if (receivedData.GetLength()>0)
					res = recv(connSocket, receivedData.GetDataAddress(), receivedData.GetLength(), 0);

				if (res <= 0)
				{
					pConnS->pConn->Close();

					if (pConnS->onLoseConnection != nullptr)
						pConnS->onLoseConnection(pConnS->pConn);

					return;
				}

				/*根据消息type来确定消息类型*/
				switch (head.msgHeader.msgType)
				{
				case SOCKET_GENERAL_DATA:
				{
					if (pConnS->onReceivedData != nullptr)
						pConnS->onReceivedData(pConnS->pConn, receivedData);
					break;
				}
				case SOCKET_REQUEST:
				{
					Aris::Core::MSG m;
					if (pConnS->onReceivedRequest != nullptr)
					{
						m = pConnS->onReceivedRequest(pConnS->pConn, receivedData);
					}
					m.SetType(SOCKET_REPLY);

					if (send(pConnS->_ConnSocket, m._pData, m.GetLength() + sizeof(MSG_HEADER), 0) == -1)
					{
						pConnS->pConn->Close();
						if (pConnS->onLoseConnection != nullptr)
							pConnS->onLoseConnection(pConnS->pConn);
						return;
					}
					break;
				}
				case SOCKET_REPLY:
				{
					if (pConnS->_ConnState != WAITING_FOR_REPLY)
					{
						if (pConnS->onReceiveError != nullptr)
							pConnS->onReceiveError(pConnS->pConn);

						return;
					}
					else
					{
						pConnS->_replyData.Swap(receivedData);
						pConnS->_cv_reply_data_received.notify_one();
					}

					break;
				}
				}
			}		
		}

		CONN::CONN()
			:pConnStruct(new CONN_STRUCT)
		{
			pConnStruct->pConn = this;
		}
		CONN::~CONN()
		{
			Close();
		}

		void CONN::Close()
		{
			std::lock(pConnStruct->_state_mutex, pConnStruct->_close_mutex);
			std::unique_lock<std::recursive_mutex> lck1(pConnStruct->_state_mutex, std::adopt_lock);
			std::unique_lock<std::mutex> lck2(pConnStruct->_close_mutex, std::adopt_lock);
			
			switch (pConnStruct->_ConnState)
			{
			case IDLE:
				return;
			case WAITING_FOR_CONNECTION:
#ifdef PLATFORM_IS_WINDOWS
				shutdown(pConnStruct->_LisnSocket, 2);
				closesocket(pConnStruct->_LisnSocket);
				WSACleanup();
#endif
#ifdef PLATFORM_IS_LINUX
				shutdown(pConnStruct->_LisnSocket, 2);
				close(pConnStruct->_LisnSocket);
#endif
				break;
			case WORKING:
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
				break;
			case WAITING_FOR_REPLY:
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
				pConnStruct->_cv_reply_data_received.notify_one();
				break;
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
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);

			switch (pConnStruct->_ConnState)
			{
			case WORKING:
			case WAITING_FOR_REPLY:
				return true;
			default:
				return false;
			}
		}
		void CONN::StartServer(const char *port)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);
			
			switch (pConnStruct->_ConnState)
			{
			case IDLE:
				break;
			default:
				throw(START_SERVER_ERROR("CONN can't start as server, because it is not at idle state\n",this,0));
			}

			/* 启动服务器 */
#ifdef PLATFORM_IS_WINDOWS 
			if (WSAStartup(0x0101, &pConnStruct->_WsaData) != 0)
			{
				throw(START_SERVER_ERROR("CONN can't start as server, because it can't WSAStartup\n", this, 0));
			}
#endif

			/* 服务器端开始建立socket描述符 */
			if ((pConnStruct->_LisnSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
			{
				throw(START_SERVER_ERROR("CONN can't start as server, because it can't socket\n", this, 0));
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
				int err = WSAGetLastError();
#endif
				throw(START_SERVER_ERROR("CONN can't start as server, because it can't bind\n", this, 0));
			}

			/* 监听_LisnSocket描述符 */
			if (listen(pConnStruct->_LisnSocket, 5) == -1)
			{
				throw(START_SERVER_ERROR("CONN can't start as server, because it can't listen\n", this, 0));
			}

			/* 启动等待连接的线程 */
			std::unique_lock<std::mutex> cv_lck(pConnStruct->_cv_mutex);
			pConnStruct->_recvConnThread = std::thread(CONN::_AcceptThread, this->pConnStruct.get());
			pConnStruct->_cv.wait(cv_lck);
			
			return;
		}
		void CONN::Connect(const char *address, const char *port)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);
			
			switch (pConnStruct->_ConnState)
			{
			case IDLE:
				break;
			default:
				throw CONNECT_ERROR("CONN can't connect, because it is busy now, please close it\n", this, 0);
			}

			/* 启动服务器 */
#ifdef PLATFORM_IS_WINDOWS
			if (WSAStartup(0x0101, &pConnStruct->_WsaData) != 0)
			{
				throw CONNECT_ERROR("CONN can't connect, because can't WSAStartup\n", this, 0);
			}
#endif
			/* 服务器端开始建立socket描述符 */
			if ((pConnStruct->_ConnSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
			{
				throw CONNECT_ERROR("CONN can't connect, because can't socket\n", this, 0);
			}

			/* 客户端填充_ServerAddr结构 */
			memset(&pConnStruct->_ServerAddr, 0, sizeof(pConnStruct->_ServerAddr));
			pConnStruct->_ServerAddr.sin_family = AF_INET;
			pConnStruct->_ServerAddr.sin_addr.s_addr = inet_addr(address); //与linux不同
			pConnStruct->_ServerAddr.sin_port = htons(atoi(port));

			/* 连接 */
			if (connect(pConnStruct->_ConnSocket, (const struct sockaddr *)&pConnStruct->_ServerAddr, sizeof(pConnStruct->_ServerAddr)) == -1)
			{
				throw CONNECT_ERROR("CONN can't connect, because can't connect\n", this, 0);
			}

			/* Start Thread */
			pConnStruct->_recvDataThread = std::thread(_ReceiveThread, this->pConnStruct.get());
			
			pConnStruct->_ConnState = WORKING;

			return;
		}
		void CONN::SendData(const Aris::Core::MSG &data)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);

			switch (pConnStruct->_ConnState)
			{
			case WORKING:
			case WAITING_FOR_REPLY:
				if (send(pConnStruct->_ConnSocket, data._pData, data.GetLength() + sizeof(MSG_HEADER), 0) == -1)
					throw SEND_DATA_ERROR("CONN failed sending data, because network failed\n", this, 0);
				else
					return;
			default:
				throw SEND_DATA_ERROR("CONN failed sending data, because CONN is not at right state\n", this, 0);
			}
		}
		Aris::Core::MSG  CONN::SendRequest(const Aris::Core::MSG &request)
		{
			std::unique_lock<std::recursive_mutex> state_lck(pConnStruct->_state_mutex);

			switch (pConnStruct->_ConnState)
			{
			case WORKING:
				pConnStruct->_ConnState = WAITING_FOR_REPLY;
				break;
			default:
				throw SEND_REQUEST_ERROR("CONN failed sending request, because CONN is not at right state\n", this, 0);
			}

			Aris::Core::MSG _request = request;
			_request.SetType(SOCKET_REQUEST);
			
			try
			{
				SendData(_request);
			}
			catch (SEND_DATA_ERROR &error)
			{
				throw SEND_REQUEST_ERROR(error.what(), this, 0);
			}
			

			pConnStruct->_cv_reply_data_received.wait(state_lck);
			if (pConnStruct->_ConnState != WAITING_FOR_REPLY)
			{
				throw SEND_REQUEST_ERROR("CONN failed sending request, because CONN is closed before it receive a reply\n", this, 0);
			}
			else
			{
				MSG reply;
				reply.Swap(pConnStruct->_replyData);
				return reply;
			}
		}
		void CONN::SetOnReceivedData(std::function<int(CONN*, Aris::Core::MSG &)> OnReceivedData)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);
			pConnStruct->onReceivedData = OnReceivedData;
		}
		void CONN::SetOnReceiveRequest(std::function<Aris::Core::MSG(CONN*, Aris::Core::MSG &)> OnReceivedRequest)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);
			pConnStruct->onReceivedRequest = OnReceivedRequest;
		}
		void CONN::SetOnReceivedConnection(std::function<int(CONN*, const char*, int)> OnReceivedConnection)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);
			pConnStruct->onReceivedConnection = OnReceivedConnection;
		}
		void CONN::SetOnLoseConnection(std::function<int(CONN*)> OnLoseConnection)
		{
			std::unique_lock<std::recursive_mutex> lck(pConnStruct->_state_mutex);
			pConnStruct->onLoseConnection = OnLoseConnection;
		}
		void CONN::SetOnAcceptError(std::function<void(CONN*)> onAcceptError)
		{
			pConnStruct->onAcceptError = onAcceptError;
		}
		void CONN::SetOnReceiveError(std::function<void(CONN*)> onReceiveError)
		{
			pConnStruct->onReceiveError = onReceiveError;
		}
	}
}

