#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <stdint.h>
#include <new>

#ifdef WIN32
#include <ws2tcpip.h>
#endif

#ifdef UNIX
#include<pthread.h>
#include<semaphore.h>
#include<netdb.h>
#include<unistd.h>
#include<arpa/inet.h>
#endif

#include "aris_core_socket.h"

namespace Aris
{
	namespace Core
	{
		struct Socket::Imp
		{
			Socket* pConn;
			
			int _LisnSocket, _ConnSocket;   //也可以用SOCKET类型
			struct sockaddr_in _ServerAddr, _ClientAddr;
			socklen_t _SinSize;
			Socket::State _ConnState;

			std::function<Aris::Core::Msg(Socket *, Aris::Core::Msg &)> onReceivedRequest;
			std::function<int(Socket *, Aris::Core::Msg &)> onReceivedData;
			std::function<int(Socket *, const char *, int)> onReceivedConnection;
			std::function<int(Socket *)> onLoseConnection;

			std::function<void(Socket *)> onAcceptError;
			std::function<void(Socket *)> onReceiveError;
			

			/*线程同步变量*/
			std::recursive_mutex _state_mutex;

			std::thread _recvDataThread, _recvConnThread;
			std::mutex _close_mutex,_cv_mutex;
			std::condition_variable _cv;

			std::condition_variable_any _cv_reply_data_received;
			Aris::Core::Msg _replyData;
			
			/* 连接的socket */
#ifdef WIN32
			WSADATA _WsaData;              //windows下才用，linux下无该项
#endif
			Imp() : _LisnSocket(0), _ConnSocket(0), _SinSize(sizeof(struct sockaddr_in)), _ConnState(Socket::IDLE)
				, onReceivedRequest(nullptr), onReceivedData(nullptr), onReceivedConnection(nullptr), onLoseConnection(nullptr) {};

			~Imp() = default;

			enum MsgType
			{
				SOCKET_GENERAL_DATA,
				SOCKET_REQUEST,
				SOCKET_REPLY
			};

			static void receiveThread(Socket::Imp* pCONN_STRUCT);
			static void acceptThread(Socket::Imp* pCONN_STRUCT);
		};
		
		auto Socket::Imp::acceptThread(Socket::Imp* pConnS)->void
		{
			int lisnSock,connSock;
			struct sockaddr_in clientAddr;
			socklen_t sinSize; 

			/*以下从对象中copy内容，此时start_Server在阻塞，因此Socket内部数据安全，
			拷贝好后告诉start_Server函数已经拷贝好*/
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
				pConnS->pConn->stop();

				if (pConnS->onAcceptError != nullptr)
					pConnS->onAcceptError(pConnS->pConn);

				return;
			}
			

			/* 创建线程 */
			pConnS->_ConnState = Socket::WORKING;
			pConnS->_ConnSocket = connSock;
			pConnS->_ClientAddr = clientAddr;

			cv_lck = std::unique_lock<std::mutex>(pConnS->_cv_mutex);
			pConnS->_recvDataThread = std::thread(receiveThread, pConnS);
			pConnS->_cv.wait(cv_lck);


			if (pConnS->onReceivedConnection != nullptr)
			{
				pConnS->onReceivedConnection(pConnS->pConn, inet_ntoa(pConnS->_ClientAddr.sin_addr), ntohs(pConnS->_ClientAddr.sin_port));
			}

			return;
		}
		auto Socket::Imp::receiveThread(Socket::Imp* pConnS)->void
		{
			union HEAD
			{
				MsgHeader msgHeader;
				char header[sizeof(MsgHeader)];
			} head;
			Aris::Core::Msg receivedData;
			
			int connSocket = pConnS->_ConnSocket;

			/*通知accept线程已经准备好，下一步开始收发数据*/
			std::unique_lock<std::mutex> cv_lck(pConnS->_cv_mutex);
			pConnS->_cv.notify_one();
			cv_lck.unlock();
			cv_lck.release();

			/*开启接受数据的循环*/
			for (;;)
			{
				int res = recv(connSocket, head.header, sizeof(MsgHeader), 0);

				/*检查是否正在Close，如果不能锁住，则证明正在close，于是结束线程释放资源，
				若能锁住，则开始获取Imp所有权*/
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
					pConnS->pConn->stop();

					if (pConnS->onLoseConnection != 0)
						pConnS->onLoseConnection(pConnS->pConn);

					return;
				}

				/*接收消息本体*/
				receivedData.resize(head.msgHeader.msg_size);
				memcpy(receivedData.data_, head.header, sizeof(MsgHeader));

				if (receivedData.size()>0)
					res = recv(connSocket, receivedData.data(), receivedData.size(), 0);

				if (res <= 0)
				{
					pConnS->pConn->stop();

					if (pConnS->onLoseConnection != nullptr)
						pConnS->onLoseConnection(pConnS->pConn);

					return;
				}

				/*根据消息type来确定消息类型*/
				switch (head.msgHeader.msg_type)
				{
				case SOCKET_GENERAL_DATA:
					if (pConnS->onReceivedData )pConnS->onReceivedData(pConnS->pConn, receivedData);
					break;
				case SOCKET_REQUEST:
				{
					Aris::Core::Msg m;
					if (pConnS->onReceivedRequest)m = pConnS->onReceivedRequest(pConnS->pConn, receivedData);

					m.setType(SOCKET_REPLY);

					if (send(pConnS->_ConnSocket, m.data_, m.size() + sizeof(MsgHeader), 0) == -1)
					{
						pConnS->pConn->stop();
						if (pConnS->onLoseConnection != nullptr)pConnS->onLoseConnection(pConnS->pConn);
						return;
					}
					break;
				}
				case SOCKET_REPLY:
					if (pConnS->_ConnState != WAITING_FOR_REPLY)
					{
						if (pConnS->onReceiveError)pConnS->onReceiveError(pConnS->pConn);
						return;
					}
					else
					{
						pConnS->_replyData.swap(receivedData);
						pConnS->_cv_reply_data_received.notify_one();
					}

					break;
				}
			}		
		}

		Socket::Socket():pImp(new Imp)
		{
			pImp->pConn = this;
		}
		Socket::~Socket()
		{
			stop();
		}

		auto Socket::stop()->void
		{
			std::lock(pImp->_state_mutex, pImp->_close_mutex);
			std::unique_lock<std::recursive_mutex> lck1(pImp->_state_mutex, std::adopt_lock);
			std::unique_lock<std::mutex> lck2(pImp->_close_mutex, std::adopt_lock);
			
			switch (pImp->_ConnState)
			{
			case IDLE:
				return;
			case WAITING_FOR_CONNECTION:
#ifdef WIN32
				shutdown(pImp->_LisnSocket, 2);
				closesocket(pImp->_LisnSocket);
				WSACleanup();
#endif
#ifdef UNIX
				shutdown(pImp->_LisnSocket, 2);
				close(pImp->_LisnSocket);
#endif
				break;
			case WORKING:
#ifdef WIN32
				shutdown(pImp->_ConnSocket, 2);
				shutdown(pImp->_LisnSocket, 2);
				closesocket(pImp->_ConnSocket);
				closesocket(pImp->_LisnSocket);
				WSACleanup();
#endif
#ifdef UNIX
				shutdown(pImp->_ConnSocket, 2);
				shutdown(pImp->_LisnSocket, 2);
				close(pImp->_ConnSocket);
				close(pImp->_LisnSocket);
#endif
				break;
			case WAITING_FOR_REPLY:
#ifdef WIN32
				shutdown(pImp->_ConnSocket, 2);
				shutdown(pImp->_LisnSocket, 2);
				closesocket(pImp->_ConnSocket);
				closesocket(pImp->_LisnSocket);
				WSACleanup();
#endif
#ifdef UNIX
				shutdown(pImp->_ConnSocket, 2);
				shutdown(pImp->_LisnSocket, 2);
				close(pImp->_ConnSocket);
				close(pImp->_LisnSocket);
#endif
				pImp->_cv_reply_data_received.notify_one();
				break;
			}
			
			if (std::this_thread::get_id() == pImp->_recvDataThread.get_id())
			{
				pImp->_recvDataThread.detach();
			}
			else if(pImp->_recvDataThread.joinable())
			{
				pImp->_recvDataThread.join();
			}
				
			if (std::this_thread::get_id() == pImp->_recvConnThread.get_id())
			{
				pImp->_recvConnThread.detach();
			}
			else if(pImp->_recvConnThread.joinable())
			{
				pImp->_recvConnThread.join();
			}

			pImp->_ConnState = Socket::IDLE;
		}
		auto Socket::isConnected()->bool
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);

			switch (pImp->_ConnState)
			{
			case WORKING:
			case WAITING_FOR_REPLY:
				return true;
			default:
				return false;
			}
		}
		auto Socket::startServer(const char *port)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			
			switch (pImp->_ConnState)
			{
			case IDLE:
				break;
			default:
				throw(StartServerError( "Socket can't Start as server, because it is not at idle state\n",this,0 ));
			}

			/* 启动服务器 */
#ifdef WIN32 
			if (WSAStartup(0x0101, &pImp->_WsaData) != 0)
			{
				throw(StartServerError("Socket can't Start as server, because it can't WSAstartup\n", this, 0));
			}
#endif

			/* 服务器端开始建立socket描述符 */
			if ((pImp->_LisnSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
			{
				throw(StartServerError("Socket can't Start as server, because it can't socket\n", this, 0));
			}

			/* 服务器端填充_ServerAddr结构 */
			memset(&pImp->_ServerAddr, 0, sizeof(struct sockaddr_in));
			pImp->_ServerAddr.sin_family = AF_INET;
			pImp->_ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
			pImp->_ServerAddr.sin_port = htons(atoi(port));

			/* 捆绑_LisnSocket描述符 */
			if (::bind(pImp->_LisnSocket, (struct sockaddr *)(&pImp->_ServerAddr), sizeof(struct sockaddr)) == -1)
			{
#ifdef WIN32
				int err = WSAGetLastError();
#endif
				throw(StartServerError("Socket can't Start as server, because it can't bind\n", this, 0));
			}

			/* 监听_LisnSocket描述符 */
			if (listen(pImp->_LisnSocket, 5) == -1)
			{
				throw(StartServerError("Socket can't Start as server, because it can't listen\n", this, 0));
			}

			/* 启动等待连接的线程 */
			std::unique_lock<std::mutex> cv_lck(pImp->_cv_mutex);
			pImp->_recvConnThread = std::thread(Socket::Imp::acceptThread, this->pImp.get());
			pImp->_cv.wait(cv_lck);
			
			return;
		}
		auto Socket::connect(const char *address, const char *port)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			
			switch (pImp->_ConnState)
			{
			case IDLE:
				break;
			default:
				throw ConnectError("Socket can't connect, because it is busy now, please close it\n", this, 0);
			}

			/* 启动服务器 */
#ifdef WIN32
			if (WSAStartup(0x0101, &pImp->_WsaData) != 0)
			{
				throw ConnectError("Socket can't connect, because can't WSAstartup\n", this, 0);
			}
#endif
			/* 服务器端开始建立socket描述符 */
			if ((pImp->_ConnSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
			{
				throw ConnectError("Socket can't connect, because can't socket\n", this, 0);
			}

			/* 客户端填充_ServerAddr结构 */
			memset(&pImp->_ServerAddr, 0, sizeof(pImp->_ServerAddr));
			pImp->_ServerAddr.sin_family = AF_INET;
			pImp->_ServerAddr.sin_addr.s_addr = inet_addr(address); //与linux不同
			pImp->_ServerAddr.sin_port = htons(atoi(port));

			/* 连接 */
			if (::connect(pImp->_ConnSocket, (const struct sockaddr *)&pImp->_ServerAddr, sizeof(pImp->_ServerAddr)) == -1)
			{
				throw ConnectError("Socket can't connect, because can't connect\n", this, 0);
			}

			/* Start Thread */
			pImp->_recvDataThread = std::thread(Imp::receiveThread, this->pImp.get());
			
			pImp->_ConnState = WORKING;

			return;
		}
		auto Socket::sendMsg(const Aris::Core::Msg &data)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);

			switch (pImp->_ConnState)
			{
			case WORKING:
			case WAITING_FOR_REPLY:
				if (send(pImp->_ConnSocket, data.data_, data.size() + sizeof(MsgHeader), 0) == -1)
					throw SendDataError("Socket failed sending data, because network failed\n", this, 0);
				else
					return;
			default:
				throw SendDataError("Socket failed sending data, because Socket is not at right state\n", this, 0);
			}
		}
		auto Socket::sendRequest(const Aris::Core::Msg &request)->Aris::Core::Msg
		{
			std::unique_lock<std::recursive_mutex> state_lck(pImp->_state_mutex);

			switch (pImp->_ConnState)
			{
			case WORKING:
				pImp->_ConnState = WAITING_FOR_REPLY;
				break;
			default:
				throw SendRequestError("Socket failed sending request, because Socket is not at right state\n", this, 0);
			}

			Aris::Core::Msg _request = request;
			_request.setType(Socket::Imp::SOCKET_REQUEST);
			
			try
			{
				sendMsg(_request);
			}
			catch (SendDataError &error)
			{
				throw SendRequestError(error.what(), this, 0);
			}
			

			pImp->_cv_reply_data_received.wait(state_lck);
			if (pImp->_ConnState != WAITING_FOR_REPLY)
			{
				throw SendRequestError("Socket failed sending request, because Socket is closed before it receive a reply\n", this, 0);
			}
			else
			{
				Msg reply;
				reply.swap(pImp->_replyData);
				return reply;
			}
		}
		auto Socket::setOnReceivedMsg(std::function<int(Socket*, Aris::Core::Msg &)> OnReceivedData)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			pImp->onReceivedData = OnReceivedData;
		}
		auto Socket::setOnReceivedRequest(std::function<Aris::Core::Msg(Socket*, Aris::Core::Msg &)> OnReceivedRequest)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			pImp->onReceivedRequest = OnReceivedRequest;
		}
		auto Socket::setOnReceivedConnection(std::function<int(Socket*, const char*, int)> OnReceivedConnection)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			pImp->onReceivedConnection = OnReceivedConnection;
		}
		auto Socket::setOnLoseConnection(std::function<int(Socket*)> OnLoseConnection)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			pImp->onLoseConnection = OnLoseConnection;
		}
		auto Socket::setOnAcceptError(std::function<void(Socket*)> onAcceptError)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			pImp->onAcceptError = onAcceptError;
		}
		auto Socket::setOnReceiveError(std::function<void(Socket*)> onReceiveError)->void
		{
			std::unique_lock<std::recursive_mutex> lck(pImp->_state_mutex);
			pImp->onReceiveError = onReceiveError;
		}
	}
}

