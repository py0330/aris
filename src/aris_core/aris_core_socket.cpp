#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <stdint.h>
#include <new>
#include <future>

#ifdef WIN32
#include <ws2tcpip.h>
#ifdef max
#undef max
#endif
#endif

#include <errno.h>

#ifdef UNIX
#include<pthread.h>
#include<semaphore.h>
#include<netdb.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<signal.h>
#endif

#include "aris_core_socket.h"
#include "aris_core_log.h"

namespace aris::core
{
	auto safe_recv(decltype(socket(AF_INET, SOCK_STREAM, 0)) s, char *data, int size) -> int
	{
		int result{ 0 };
		for (; result < size; )
		{
			int ret = recv(s, data + result, size - result, 0);
			if (ret <= 0)
			{
#ifdef WIN32
				closesocket(s);
				WSACleanup();
#endif
#ifdef UNIX
				close(s);
#endif
				result = ret;
				break;
			}

			result += ret;
		}

		return result;
	};
	auto close_sock(decltype(socket(AF_INET, SOCK_STREAM, 0)) s)->int
	{
#ifdef WIN32
		auto ret = closesocket(s);
		WSACleanup();
#endif
#ifdef UNIX
		auto ret = close(s);
#endif
		return ret;
	}


	struct Socket::Imp
	{
		Socket* socket_;
		Socket::State state_;
		Socket::TYPE type_;

		std::function<int(Socket *, aris::core::Msg &)> onReceivedMsg;
		std::function<int(Socket*, const char *data, int size)> onReceivedData;
		std::function<int(Socket *, const char *, int)> onReceivedConnection;
		std::function<int(Socket *)> onLoseConnection;

		decltype(socket(AF_INET, SOCK_STREAM, 0)) lisn_socket_, recv_socket_;  //也可以用SOCKET类型
		struct sockaddr_in server_addr_, client_addr_;
		socklen_t sin_size_;

		std::string remote_ip_, port_;

		// 线程同步变量 //
		std::recursive_mutex state_mutex_;

		std::thread recv_thread_, accept_thread_;
		std::mutex close_mutex_;

		// 连接的socket //
#ifdef WIN32
		WSADATA wsa_data_;             //windows下才用,linux下无该项
#endif
		~Imp() = default;
		Imp(Socket* sock) :socket_(sock), lisn_socket_(0), recv_socket_(0), sin_size_(sizeof(struct sockaddr_in)), state_(Socket::IDLE)
			, onReceivedMsg(nullptr), onReceivedData(nullptr), onReceivedConnection(nullptr), onLoseConnection(nullptr) {}

		static void receiveThread(Socket::Imp* imp, std::promise<void> receive_thread_ready);
		static void acceptThread(Socket::Imp* imp, std::promise<void> accept_thread_ready);
	};
	auto Socket::Imp::acceptThread(Socket::Imp* imp, std::promise<void> accept_thread_ready)->void
	{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		// 改变状态 //
		imp->state_ = WAITING_FOR_CONNECTION;

		// 通知主线程,accept线程已经拷贝完毕,准备监听 //
		accept_thread_ready.set_value();

		// 服务器阻塞,直到客户程序建立连接 //
		imp->recv_socket_ = accept(imp->lisn_socket_, (struct sockaddr *)(&imp->client_addr_), &imp->sin_size_);

		std::cout << "lisn shutdown:" << shutdown(imp->lisn_socket_, 2) << std::endl;
		std::cout << "lisn close:" << close_sock(imp->lisn_socket_) << std::endl;
		
		
		// 否则,开始开启数据线程 //
		if (imp->recv_socket_ == -1)return;
		if (imp->onReceivedConnection)imp->onReceivedConnection(imp->socket_, inet_ntoa(imp->client_addr_.sin_addr), ntohs(imp->client_addr_.sin_port));

		std::promise<void> receive_thread_ready;
		auto fut = receive_thread_ready.get_future();
		imp->recv_thread_ = std::thread(receiveThread, imp, std::move(receive_thread_ready));
		fut.wait();

		return;
	}
	auto Socket::Imp::receiveThread(Socket::Imp* imp, std::promise<void> receive_thread_ready)->void
	{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		// 改变状态 //
		imp->state_ = Socket::WORKING;

		// 通知accept或connect线程已经准备好,下一步开始收发数据 //
		receive_thread_ready.set_value();

		aris::core::Msg recv_msg;
		recv_msg.resize(1024);
		// 开启接受数据的循环 //
		for (;;)
		{
			switch (imp->type_)
			{
			case TCP:
			{
				const auto &lose = [&]()
				{
					// 如果正在stop，那么不回调，直接返回 //
					std::unique_lock<std::mutex> close_lck(imp->close_mutex_, std::defer_lock);
					if (!close_lck.try_lock())return;

					// 自己关闭自己 //
					std::unique_lock<std::recursive_mutex> state_lck(imp->state_mutex_);
					imp->state_ = IDLE;
					if (imp->onLoseConnection)imp->onLoseConnection(imp->socket_);
					imp->recv_thread_.detach();
					return;
				};
				
				// 接收消息 //
				if (safe_recv(imp->recv_socket_, reinterpret_cast<char *>(&recv_msg.header()), sizeof(MsgHeader)) <= 0) { lose(); return; }
				recv_msg.resize(recv_msg.size());
				if (recv_msg.size() > 0 && safe_recv(imp->recv_socket_, recv_msg.data(), recv_msg.size()) <= 0) { lose(); return; }
				if (imp->onReceivedMsg)imp->onReceivedMsg(imp->socket_, recv_msg);

				break;
			}
			case WEB:
			case WEB_RAW:
			{
				// tbd //
				break;
			}
			case UDP:
			{
				int ret = recvfrom(imp->recv_socket_, reinterpret_cast<char *>(&recv_msg.header()), 1024, 0, (struct sockaddr *)(&imp->client_addr_), &imp->sin_size_);
				
				std::cout << "udp ret:" << ret << std::endl;
				
				if (ret <= 0)
				{					
					// 如果正在stop，那么不回调，直接返回 //
					std::unique_lock<std::mutex> close_lck(imp->close_mutex_, std::defer_lock);
					if (!close_lck.try_lock())return;
				}
				if (ret != sizeof(MsgHeader) + recv_msg.size())
				{
					LOG_ERROR << "UDP msg size not correct" << std::endl;
					continue;
				}

				if(imp->onReceivedMsg)imp->onReceivedMsg(imp->socket_, recv_msg);
				break;
			}
			case UDP_RAW:
			{
				// tbd //
				break;
			}
			}
		}
	}
	auto Socket::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		setRemoteIP(attributeString(xml_ele, "remote_ip", std::string()));
		setPort(attributeString(xml_ele, "port", std::string()));

		Object::loadXml(xml_ele);
	}
	auto Socket::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Object::saveXml(xml_ele);

		if (!imp_->remote_ip_.empty())xml_ele.SetAttribute("remote_ip", imp_->remote_ip_.c_str());
		if (!imp_->port_.empty())xml_ele.SetAttribute("port", imp_->port_.c_str());
	}
	auto Socket::stop()->void
	{
		std::lock(imp_->state_mutex_, imp_->close_mutex_);
		std::unique_lock<std::recursive_mutex> lck1(imp_->state_mutex_, std::adopt_lock);
		std::unique_lock<std::mutex> lck2(imp_->close_mutex_, std::adopt_lock);

		switch (imp_->state_)
		{
		case IDLE:
			break;
		case WAITING_FOR_CONNECTION:
			shutdown(imp_->lisn_socket_, 2);
			if (imp_->accept_thread_.joinable())imp_->accept_thread_.join();
			break;
		case WORKING:
			switch (connectType())
			{
			case TCP:
			case WEB:
			case WEB_RAW:
				if (shutdown(imp_->recv_socket_, 2) < 0) LOG_ERROR << "shutdown error:" << errno << std::endl;
				break;
			case UDP:
			case UDP_RAW:
				if (close_sock(imp_->recv_socket_) < 0)LOG_ERROR << "shutdown error:" << errno << std::endl;
				break;
			}

			if(imp_->recv_thread_.joinable())imp_->recv_thread_.join();
			break;
		}

		imp_->state_ = Socket::IDLE;
	}
	auto Socket::startServer(const std::string &port)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		if (!port.empty())setPort(port);
		if (this->port().empty())throw std::runtime_error("Socket can't Start as server, because it has empty port\n");

		switch (imp_->state_)
		{
		case IDLE:
			break;
		default:
			throw(std::runtime_error("Socket can't Start as server, because it is not at idle state\n"));
		}

		// 启动服务器 //
#ifdef WIN32 
		if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)throw(std::runtime_error("Socket can't Start as server, because it can't WSAstartup\n"));
#endif

		//////////////////////////////////////////////////////////////////////////////////////////////
		int sock_type;
		switch (connectType())
		{
		case TCP:
		case WEB:
		case WEB_RAW:
			sock_type = SOCK_STREAM;
			break;
		case UDP:
		case UDP_RAW:
			sock_type = SOCK_DGRAM;
			break;
		}
		///////////////////////////////////////////////////////////////////////////////////////////////

		// 服务器端开始建立socket描述符 //
		if (static_cast<int>(imp_->lisn_socket_ = socket(AF_INET, sock_type, 0)) == -1)throw(std::runtime_error("Socket can't Start as server, because it can't socket\n"));

		// 设置socketopt选项,使得地址在程序结束后立即可用 //
		int nvalue = 1;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&nvalue), sizeof(int)) < 0)throw std::runtime_error("setsockopt failed: SO_REUSEADDR \n");

#ifdef WIN32
		DWORD read_timeout = 10;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0)throw std::runtime_error("setsockopt failed: SO_RCVTIMEO \n");
#endif
#ifdef UNIX
		struct timeval read_timeout;
		read_timeout.tv_sec = 0;
		read_timeout.tv_usec = 10000;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0)throw std::runtime_error("setsockopt failed: SO_RCVTIMEO \n");
#endif

		// 服务器端填充server_addr_结构,并且bind //
		memset(&imp_->server_addr_, 0, sizeof(struct sockaddr_in));
		imp_->server_addr_.sin_family = AF_INET;
		imp_->server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
		imp_->server_addr_.sin_port = htons(std::stoi(this->port()));
		if (::bind(imp_->lisn_socket_, (struct sockaddr *)(&imp_->server_addr_), sizeof(struct sockaddr)) == -1)
		{
#ifdef WIN32
			int err = WSAGetLastError();
#endif
			throw(std::runtime_error("Socket can't Start as server, because it can't bind\n"));
		}
		
		if (connectType() == TCP || connectType() == WEB || connectType() == WEB_RAW)
		{
			// 监听lisn_socket_描述符 //
			if (listen(imp_->lisn_socket_, 5) == -1)throw(std::runtime_error("Socket can't Start as server, because it can't listen\n"));

			// 启动等待连接的线程 //
			std::promise<void> accept_thread_ready;
			auto ready = accept_thread_ready.get_future();
			imp_->accept_thread_ = std::thread(Socket::Imp::acceptThread, this->imp_.get(), std::move(accept_thread_ready));
			imp_->accept_thread_.detach();
			ready.wait();
		}
		else
		{
			imp_->recv_socket_ = imp_->lisn_socket_;
			
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(Socket::Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();
		}

		return;
	}
	auto Socket::connect(const std::string &remote_ip, const std::string &port)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		if (!remote_ip.empty())setRemoteIP(remote_ip);
		if (!port.empty())setPort(port);
		if (remoteIP().empty())throw std::runtime_error("Socket can't connect, because it empty ip address\n");
		if (this->port().empty())throw std::runtime_error("Socket can't connect, because it empty port\n");

		switch (imp_->state_)
		{
		case IDLE:
			break;
		default:
			throw std::runtime_error("Socket can't connect, because it is busy now, please close it\n");
		}

		// 启动服务器 //
#ifdef WIN32
		if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)throw std::runtime_error("Socket can't connect, because can't WSAstartup\n");
#endif

		///////////////////////////////////////////////////////////////////////////////
		int sock_type;
		switch (connectType())
		{
		case TCP:
			sock_type = SOCK_STREAM;
			break;
		case UDP:
			sock_type = SOCK_DGRAM;
			break;
		}
		//////////////////////////////////////////////////////////////////////////////

		// 服务器端开始建立socket描述符 //
		if ((imp_->recv_socket_ = socket(AF_INET, sock_type, 0)) < 0)throw std::runtime_error("Socket can't connect, because can't socket\n");

		// 客户端填充server_addr_结构 //
		memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
		imp_->server_addr_.sin_family = AF_INET;
		imp_->server_addr_.sin_addr.s_addr = inet_addr(remoteIP().c_str());
		imp_->server_addr_.sin_port = htons(std::stoi(this->port()));

		if (connectType() == TCP || connectType() == WEB || connectType() == WEB_RAW)
		{
			// 连接 //
			if (::connect(imp_->recv_socket_, (const struct sockaddr *)&imp_->server_addr_, sizeof(imp_->server_addr_)) == -1)
				throw std::runtime_error("Socket can't connect, because can't connect\n");

			imp_->state_ = Socket::WORKING;

			// Start Thread //
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();
		}
		
		imp_->state_ = Socket::WORKING;
		return;
	}
	auto Socket::sendMsg(const aris::core::MsgBase &data)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_)
		{
		case WORKING:
		{
			switch (imp_->type_)
			{
			case TCP:
			case WEB:
			case WEB_RAW:
				if (send(imp_->recv_socket_, reinterpret_cast<const char *>(&data.header()), data.size() + sizeof(MsgHeader), 0) == -1)
					throw std::runtime_error("Socket failed sending data, because network failed\n");
				else
					return;

				break;
			case UDP:
			case UDP_RAW:
			{
				memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
				imp_->server_addr_.sin_family = AF_INET;
				imp_->server_addr_.sin_addr.s_addr = inet_addr(remoteIP().c_str());
				imp_->server_addr_.sin_port = htons(std::stoi(this->port()));
				
				if (sendto(imp_->recv_socket_, reinterpret_cast<const char *>(&data.header()), data.size() + sizeof(MsgHeader), 0, (const struct sockaddr *)&imp_->server_addr_, sizeof(imp_->server_addr_)) == -1)
					throw std::runtime_error("Socket failed sending data, because network failed\n");
				else
					return;

				break;
			}
			}
		}
		default:
			throw std::runtime_error("Socket failed sending data, because Socket is not at right state\n");
		}
	}
	auto Socket::isConnected()->bool
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_)
		{
		case WORKING:
			return true;
		default:
			return false;
		}
	}
	auto Socket::state()->State
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->state_;
	};
	auto Socket::remoteIP()const->const std::string & 
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->remote_ip_;
	}
	auto Socket::port()const->const std::string & 
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->port_;
	}
	auto Socket::setRemoteIP(const std::string &remote_ip)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->remote_ip_ = remote_ip;
	}
	auto Socket::setPort(const std::string &port)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->port_ = port;
	}
	auto Socket::connectType()const->TYPE { return imp_->type_; }
	auto Socket::setConnectType(const TYPE type)->void { imp_->type_ = type; }
	auto Socket::setOnReceivedMsg(std::function<int(Socket*, aris::core::Msg &)> OnReceivedData)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedMsg = OnReceivedData;
	}
	auto Socket::setOnReceivedRawData(std::function<int(Socket*, const char *data, int size)> func)->void 
	{ 
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedData = func;
	}
	auto Socket::setOnReceivedConnection(std::function<int(Socket*, const char*, int)> OnReceivedConnection)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedConnection = OnReceivedConnection;
	}
	auto Socket::setOnLoseConnection(std::function<int(Socket*)> OnLoseConnection)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onLoseConnection = OnLoseConnection;
	}
	Socket::~Socket() 
	{ 
		std::cout << this->name()<< this->state() << std::endl;
		stop();
	}
	Socket::Socket(const std::string &name, const std::string& remote_ip, const std::string& port) :Object(name), imp_(new Imp(this))
	{
		setRemoteIP(remote_ip);
		setPort(port);
	}
}