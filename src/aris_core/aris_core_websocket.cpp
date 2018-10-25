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
#include <regex>

#ifdef WIN32
#include <ws2tcpip.h>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

#ifdef UNIX
#include<pthread.h>
#include<semaphore.h>
#include<netdb.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<signal.h>
#endif

#include "aris_core_websocket.h"
#include "aris_core_log.h"
#include "sha1.h"

namespace aris::core
{
	// please refer to 
	// https://www.cnblogs.com/chyingp/p/websocket-deep-in.html
	// sha1 hash 生成出来的是纯数字，可以把它改成2进制来保存
	// 
	std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len)
	{
		static const std::string base64_chars =
			"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			"abcdefghijklmnopqrstuvwxyz"
			"0123456789+/";

		std::string ret;
		int i = 0;
		int j = 0;
		unsigned char char_array_3[3];
		unsigned char char_array_4[4];

		while (in_len--) {
			char_array_3[i++] = *(bytes_to_encode++);
			if (i == 3) {
				char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
				char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
				char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
				char_array_4[3] = char_array_3[2] & 0x3f;

				for (i = 0; (i <4); i++)
					ret += base64_chars[char_array_4[i]];
				i = 0;
			}
		}

		if (i)
		{
			for (j = i; j < 3; j++)
				char_array_3[j] = '\0';

			char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
			char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
			char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

			for (j = 0; (j < i + 1); j++)
				ret += base64_chars[char_array_4[j]];

			while ((i++ < 3))
				ret += '=';

		}

		return ret;

	}
	
	auto pack_data(const aris::core::MsgBase &msg)->std::string
	{
		int size = msg.size() + sizeof(MsgHeader);

		std::string s;
		if (size < 126)
		{
			s.resize(size + 2);
			s[0] = char(0x82);// binary data, 0x81 is text data
			s[1] = char(size);
			std::copy_n(msg.data() - sizeof(MsgHeader), size, &s[2]);
		}
		else if (size < 0xFFFF)
		{
			s.resize(size + 4);
			s[0] = char(0x82);
			s[1] = char(126);
			s[2] = size >> 8;
			s[3] = size & 0xFF;
 			std::copy_n(msg.data() - sizeof(MsgHeader), size, &s[4]);
		}
		else
		{
			s.resize(size + 10);
			s[0] = char(0x82);
			s[1] = char(127);
			s[2] = 0;
			s[3] = 0;
			s[4] = 0;
			s[5] = 0;
			s[6] = size >> 24;
			s[7] = size >> 16;
			s[8] = size >> 8;
			s[9] = size & 0xFF;
			std::copy_n(msg.data() - sizeof(MsgHeader), size, &s[10]);
		}


		return s;
	};

	struct WebSocket::Imp
	{
		WebSocket* socket_;
		WebSocket::State state_;

		std::function<int(WebSocket *, aris::core::Msg &)> onReceivedData;
		std::function<int(WebSocket *, const char *, int)> onReceivedConnection;
		std::function<int(WebSocket *)> onLoseConnection;

		decltype(socket(AF_INET, SOCK_STREAM, 0)) lisn_socket_, recv_socket_;  //也可以用SOCKET类型
		struct sockaddr_in server_addr_, client_addr_;
		socklen_t sin_size_;

		std::string remote_ip_, port_;

		// 线程同步变量 //
		std::recursive_mutex state_mutex_;

		std::thread recv_thread_, accept_thread_;

		// 连接的socket //
#ifdef WIN32
		WSADATA wsa_data_;             //windows下才用,linux下无该项
#endif
		~Imp() = default;
		Imp(WebSocket* sock) :socket_(sock), lisn_socket_(0), recv_socket_(0), sin_size_(sizeof(struct sockaddr_in)), state_(WebSocket::IDLE)
			, onReceivedData(nullptr), onReceivedConnection(nullptr), onLoseConnection(nullptr) {}

		static void receiveThread(WebSocket::Imp* imp, std::promise<void> receive_thread_ready);
		static void acceptThread(WebSocket::Imp* imp, std::promise<void> accept_thread_ready);
	};
	auto WebSocket::Imp::acceptThread(WebSocket::Imp* imp, std::promise<void> accept_thread_ready)->void
	{
		// 改变状态 //
		imp->state_ = WAITING_FOR_CONNECTION;

		// 通知主线程,accept线程已经拷贝完毕,准备监听 //
		accept_thread_ready.set_value();

#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif


		// 开启循环去接听连接，防止不符合 websocket 的连接进来 //
		for (;;)
		{
			// 服务器阻塞,直到客户程序建立连接 //
			imp->recv_socket_ = accept(imp->lisn_socket_, (struct sockaddr *)(&imp->client_addr_), &imp->sin_size_);

			// 否则,开始开启数据线程 //
			if (imp->recv_socket_ == -1)
			{
				LOG_ERROR << "websocket failed to accept" << std::endl;
#ifdef WIN32
				shutdown(imp->lisn_socket_, 2);
				closesocket(imp->lisn_socket_);
				WSACleanup();
#endif
#ifdef UNIX
				shutdown(imp->lisn_socket_, 2);
				close(imp->lisn_socket_);
#endif
				imp->state_ = IDLE;
				imp->onLoseConnection(imp->socket_);
				return;
			}

			///////////////////////////////////////////////////
			char recv_data[1024]{ 0 };

			std::this_thread::sleep_for(std::chrono::seconds(3));

			// 接受数据 //
			int res = recv(imp->recv_socket_, recv_data, 1024, 0);
			if (res <= 0)
			{
				LOG_ERROR << "websocket shake hand failed : " << res << std::endl;
#ifdef WIN32
				shutdown(imp->recv_socket_, 2);
				closesocket(imp->recv_socket_);
#endif
#ifdef UNIX
				shutdown(imp->recv_socket_, 2);
				close(imp->recv_socket_);
#endif
				continue;
			}

			std::string handShakeText(recv_data, res);
			std::istringstream istream(handShakeText);

			// 找到Sec-WebSocket-Key //
			std::map<std::string, std::string> header_map;
			std::string header;
			while (std::getline(istream, header) && header != "\r")
			{
				if (header[header.size() - 1] != '\r')
				{
					continue; //end
				}
				else
				{
					header.erase(header.end() - 1);    //remove last char
				}

				auto end = header.find(": ", 0);
				if (end != std::string::npos)
				{
					std::string key = header.substr(0, end);
					std::string value = header.substr(end + 2);
					header_map[key] = value;
				}
			}

			std::string server_key;
			try 
			{
				server_key = header_map.at("Sec-WebSocket-Key");
			}
			catch(std::exception &e)
			{
				LOG_ERROR << "websocket shake hand failed : invalid key" << std::endl;
#ifdef WIN32
				shutdown(imp->recv_socket_, 2);
				closesocket(imp->recv_socket_);
#endif
#ifdef UNIX
				shutdown(imp->recv_socket_, 2);
				close(imp->recv_socket_);
#endif
				continue;
			}
			server_key += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

			// 找到返回的key //
			SHA1 checksum;
			checksum.update(server_key);
			std::string hash = checksum.final();

			std::uint32_t message_digest[5];
			for (int i = 0; i < 20; ++i)
			{
				char num[5] = "0x00";

				std::copy_n(hash.data() + i * 2, 2, num + 2);

				std::uint8_t n = std::stoi(num, 0, 16);

				*(reinterpret_cast<unsigned char*>(message_digest) + i) = n;
			}

			auto ret_hey = base64_encode(reinterpret_cast<const unsigned char*>(message_digest), 20);

			std::string shake_hand;
			shake_hand = "HTTP/1.1 101 Switching Protocols\r\n"
				"Upgrade: websocket\r\n"
				"Connection: Upgrade\r\n"
				"Sec-WebSocket-Accept: " + ret_hey + std::string("\r\n\r\n");

			auto ret = send(imp->recv_socket_, shake_hand.c_str(), shake_hand.size(), 0);
			if (ret == -1) 
			{
				LOG_ERROR << "websocket shake hand failed : lose connection before hand shake successful" << std::endl;
#ifdef WIN32
				shutdown(imp->recv_socket_, 2);
				closesocket(imp->recv_socket_);
#endif
#ifdef UNIX
				shutdown(imp->recv_socket_, 2);
				close(imp->recv_socket_);
#endif
				continue;
			};

			break;
		}

		// 关闭监听端口 //
#ifdef WIN32
		shutdown(imp->lisn_socket_, 2);
		closesocket(imp->lisn_socket_);
#endif
#ifdef UNIX
		shutdown(imp->lisn_socket_, 2);
		close(imp->lisn_socket_);
#endif

		if (imp->onReceivedConnection)imp->onReceivedConnection(imp->socket_, inet_ntoa(imp->client_addr_.sin_addr), ntohs(imp->client_addr_.sin_port));

		// 创建线程 //
		imp->state_ = WebSocket::WORKING;

		std::promise<void> receive_thread_ready;
		auto fut = receive_thread_ready.get_future();
		imp->recv_thread_ = std::thread(receiveThread, imp, std::move(receive_thread_ready));
		imp->recv_thread_.detach();
		fut.wait();

		return;
	}
	auto WebSocket::Imp::receiveThread(WebSocket::Imp* imp, std::promise<void> receive_thread_ready)->void
	{
		// 通知accept或connect线程已经准备好,下一步开始收发数据 //
		receive_thread_ready.set_value();

#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif

		//////////////////////////////////////////////////////
		// lose //
		const auto &lose = [&]() -> void
		{
#ifdef WIN32
			shutdown(imp->recv_socket_, 2);
			closesocket(imp->recv_socket_);
			WSACleanup();
#endif
#ifdef UNIX
			shutdown(imp->recv_socket_, 2);
			close(imp->recv_socket_);
#endif
			imp->state_ = IDLE;
			if (imp->onLoseConnection)imp->onLoseConnection(imp->socket_);
		};

		//////////////////////////////////////////////////////
		// 循环去收指定size //
		const auto &safe_recv = [&](decltype(socket(AF_INET, SOCK_STREAM, 0)) s, char *data, int size) -> int
		{
			int result{ 0 };
			for (; result < size; )
			{
				int ret = recv(s, data + result, size - result, 0);
				if (ret <= 0)
				{
					result = ret;
					break;
				}

				result += ret;
			}

			if (result <= 0)lose();

			return result;
		};

		// 开启接受数据的循环 //
		for (;;)
		{
			// 开始接受web sock 的消息 //
			std::int64_t real_length{ 0 };
			std::string payload_data;

			for (bool fin{ false }; !fin;)
			{
				// 接受头 //
				char web_head[2];
				if (safe_recv(imp->recv_socket_, web_head, 2) <= 0)return;

				// 是否最后一帧 //
				fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

				// 获取opcode //
				std::int8_t op_code = web_head[0] & 0x0f;
				if (op_code == 0x08) { lose(); return; }

				// 获取数据长度
				std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
				if (payload_len == 126)
				{
					char length_char[2];
					if (safe_recv(imp->recv_socket_, length_char, 2) <= 0)return;

					union 
					{
						std::uint16_t length;
						char reverse_char[2];
					};
					for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
					payload_len = length;
				}
				else if (payload_len == 127)
				{
					char length_char[8];
					if (safe_recv(imp->recv_socket_, length_char, 8) <= 0)return;

					char reverse_char[8];
					for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
					std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
				}

				//////////////////////////////////保护，数据不能太大///////////////////////////////
				if (payload_len > 65536 || payload_len + payload_data.size() > 0x00020000)
				{
					LOG_ERROR << "websocket receive too large object" << std::endl;
					lose();
					return;
				}

				// 获取掩码
				bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
				char masks[4];
				if (safe_recv(imp->recv_socket_, masks, 4) <= 0)return;

				// 用掩码读取出数据 //
				auto last_size = payload_data.size();
				payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
				if (safe_recv(imp->recv_socket_, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0)return;

				for (int i{ 0 }; i<payload_len; ++i)
				{
					payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
				}
			}

			// 把web sock 的东西转成 msg //
			aris::core::Msg receivedData;
			receivedData.resize(payload_data.size() - sizeof(aris::core::MsgHeader));
			std::copy(payload_data.data(), payload_data.data() + payload_data.size(), reinterpret_cast<char*>(&receivedData.header()));

			if (receivedData.size() != payload_data.size() - sizeof(aris::core::MsgHeader))
			{
				LOG_ERROR << "websocket receive wrong msg size" << std::endl;
				lose();
				return;
			}

			///////////////////////////////////////////////////////
			// 根据消息type来确定消息类型 //
			if (imp->onReceivedData)imp->onReceivedData(imp->socket_, receivedData);
		}
	}
	auto WebSocket::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		setRemoteIP(attributeString(xml_ele, "remote_ip", std::string()));
		setPort(attributeString(xml_ele, "port", std::string()));

		Object::loadXml(xml_ele);
	}
	auto WebSocket::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Object::saveXml(xml_ele);

		if (!imp_->remote_ip_.empty())xml_ele.SetAttribute("remote_ip", imp_->remote_ip_.c_str());
		if (!imp_->port_.empty())xml_ele.SetAttribute("port", imp_->port_.c_str());
	}
	auto WebSocket::stop()->void
	{
		switch (imp_->state_)
		{
		case IDLE:
			return;
		case WAITING_FOR_CONNECTION:
#ifdef WIN32
			shutdown(imp_->lisn_socket_, 2);
			closesocket(imp_->lisn_socket_);
			WSACleanup();
#endif
#ifdef UNIX
			shutdown(imp_->lisn_socket_, 2);
			close(imp_->lisn_socket_);
#endif
			break;
		case WORKING:
#ifdef WIN32
			shutdown(imp_->recv_socket_, 2);
			closesocket(imp_->recv_socket_);
			WSACleanup();
#endif
#ifdef UNIX
			shutdown(imp_->recv_socket_, 2);
			close(imp_->recv_socket_);
#endif
			break;
		}

		if (std::this_thread::get_id() == imp_->recv_thread_.get_id())
		{
			imp_->recv_thread_.detach();
		}
		else if (imp_->recv_thread_.joinable())
		{
			imp_->recv_thread_.join();
		}

		if (std::this_thread::get_id() == imp_->accept_thread_.get_id())
		{
			imp_->accept_thread_.detach();
		}
		else if (imp_->accept_thread_.joinable())
		{
			imp_->accept_thread_.join();
		}

		imp_->state_ = WebSocket::IDLE;
	}
	auto WebSocket::state()->State
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->state_;
	};
	auto WebSocket::startServer(const std::string &port)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		if (!port.empty())setPort(port);
		if (this->port().empty())throw std::runtime_error("WebSocket can't Start as server, because it has empty port\n");

		switch (imp_->state_)
		{
		case IDLE:
			break;
		default:
			throw(std::runtime_error("WebSocket can't Start as server, because it is not at idle state\n"));
		}

		// 启动服务器 //
#ifdef WIN32 
		if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)throw(std::runtime_error("WebSocket can't Start as server, because it can't WSAstartup\n"));
#endif

		// 服务器端开始建立socket描述符 //
		if (static_cast<int>(imp_->lisn_socket_ = socket(AF_INET, SOCK_STREAM, 0)) == -1)throw(std::runtime_error("WebSocket can't Start as server, because it can't socket\n"));

		// 设置socketopt选项,使得地址在程序结束后立即可用 //
		int nvalue = 1;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&nvalue), sizeof(int)) < 0)
			throw std::runtime_error("WebSocket can't set REUSEADDR option\n");

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
			throw(std::runtime_error("WebSocket can't Start as server, because it can't bind"));
		}

		// 监听lisn_socket_描述符 //
		if (listen(imp_->lisn_socket_, 5) == -1)throw(std::runtime_error("WebSocket can't Start as server, because it can't listen\n"));

		// 启动等待连接的线程 //
		std::promise<void> accept_thread_ready;
		auto ready = accept_thread_ready.get_future();
		imp_->accept_thread_ = std::thread(WebSocket::Imp::acceptThread, this->imp_.get(), std::move(accept_thread_ready));
		imp_->accept_thread_.detach();
		ready.wait();

		return;
	}
	auto WebSocket::sendMsg(const aris::core::MsgBase &data)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif

		switch (imp_->state_)
		{
		case WORKING:
		{
			auto s = pack_data(data);
			
			if (send(imp_->recv_socket_, s.data(), s.size(), 0) == -1)
				throw std::runtime_error("WebSocket failed sending data, because network failed\n");
			else
				return;
		}
		default:
			throw std::runtime_error("WebSocket failed sending data, because WebSocket is not at right state\n");
		}
	}
	auto WebSocket::remoteIP()const->const std::string & {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->remote_ip_;
	}
	auto WebSocket::port()const->const std::string & {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->port_;
	}
	auto WebSocket::setRemoteIP(const std::string &remote_ip)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->remote_ip_ = remote_ip;
	}
	auto WebSocket::setPort(const std::string &port)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->port_ = port;
	}
	auto WebSocket::setOnReceivedMsg(std::function<int(WebSocket*, aris::core::Msg &)> OnReceivedData)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedData = OnReceivedData;
	}
	auto WebSocket::setOnReceivedConnection(std::function<int(WebSocket*, const char*, int)> OnReceivedConnection)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedConnection = OnReceivedConnection;
	}
	auto WebSocket::setOnLoseConnection(std::function<int(WebSocket*)> OnLoseConnection)->void
	{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onLoseConnection = OnLoseConnection;
	}
	WebSocket::~WebSocket() { stop(); }
	WebSocket::WebSocket(const std::string &name, const std::string& remote_ip, const std::string& port) :Object(name), imp_(new Imp(this))
	{
		setRemoteIP(remote_ip);
		setPort(port);
	}
}