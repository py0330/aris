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

#include "aris/core/reflection.hpp"

#ifdef WIN32
#include <ws2tcpip.h>
#ifdef max
#undef max
#endif
#endif

#ifdef UNIX
#include <pthread.h>
#include <semaphore.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <signal.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <fcntl.h>
#endif

#include <map>
#include <sstream>

#include <errno.h>

#include "aris/core/socket.hpp"
#include "aris/core/log.hpp"
#include "aris/core/sha1.h"


#define SOCKET_FAILED_ACCEPT                         aris::core::LogLvl::kError, -3001, {"socket failed to accept : %d" \
																						 "Socket 接受链接失败：%d"		}
#define WEBSOCKET_SHAKE_HAND_FAILED                  aris::core::LogLvl::kError, -3002, {"websocket shake hand failed : %d"\
                                                                                         "Websocket 协议握手失败：%d"}
#define WEBSOCKET_SHAKE_HAND_FAILED_INVALID_KEY      aris::core::LogLvl::kError, -3003, {"websocket shake hand failed : invalid key"\
                                                                                         "Websocket 协议握手失败：非法的key值"}
#define WEBSOCKET_SHAKE_HAND_FAILED_LOOSE_CONNECTION aris::core::LogLvl::kError, -3004, {"websocket shake hand failed : lose connection before succesful"\
                                                                                         "Websocket 协议握手失败：提前失去连接"}
#define WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT           aris::core::LogLvl::kError, -3005, {"websocket receive too large or negative object, size:%ji"\
                                                                                         "Websocket 数据接收失败，过大的数据包，字节数：%ji"}
#define WEBSOCKET_RECEIVE_RAW                        aris::core::LogLvl::kError, -3006, {"websocket espect msg, but receive raw data"\
                                                                                         "Websocket 数据接受失败，数据不是消息类型（Msg类型）"}
#define WEBSOCKET_RECEIVE_WRONG_MSG_SIZE             aris::core::LogLvl::kError, -3007, {"websocket receive wrong msg size, msg size:%i payload size:%ji"\
                                                                                         "Websocket 数据接受失败，错误的消息大小，消息大小：%i，负载大小：%ji"}
#define SOCKET_UDP_WRONG_MSG_SIZE                    aris::core::LogLvl::kError, -3008, {"UDP msg size not correct"\
                                                                                         "UDP 消息大小不对"}
#define SOCKET_SHUT_DOWN_ERROR                       aris::core::LogLvl::kError, -3009, {"socket shut down error %d"\
                                                                                         "Socket Shutdown 关闭错误：%d"}
#define SOCKET_SHUT_CLOSE_ERROR                      aris::core::LogLvl::kError, -3010, {"socket close error %d"\
                                                                                         "Socket Close 关闭错误：%d"}

namespace aris::core{
	auto close_sock(decltype(socket(AF_INET, SOCK_STREAM, 0)) s)->int{
#ifdef WIN32
		auto ret = closesocket(s);
#endif
#ifdef UNIX
		auto ret = close(s);
#endif
		return ret;
	}
	auto safe_recv(decltype(socket(AF_INET, SOCK_STREAM, 0)) s, char *data, int size) -> int{
		int result{ 0 };
		for (; result < size; ){
			int ret = recv(s, data + result, size - result, 0);
			if (ret <= 0){
				close_sock(s);
				result = ret;
				break;
			}

			result += ret;
		}

		return result;
	};

	// please refer to 
	// https://www.cnblogs.com/chyingp/p/websocket-deep-in.html
	// sha1 hash 生成出来的是纯数字，可以把它改成2进制来保存
	// 
	std::string base64_encode2(unsigned char const* bytes_to_encode, unsigned int in_len){
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

		if (i){
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
	auto pack_data_server(const char *data, int size)->std::string{
		std::string s;
		if (size < 126){
			s.resize(std::size_t(size) + 2);
			s[0] = char(0x82);// binary data, 0x81 is text data
			s[1] = char(size);
			std::copy_n(data, size, &s[2]);
		}
		else if (size < 0xFFFF){
			s.resize(std::size_t(size) + 4);
			s[0] = char(0x82);
			s[1] = char(126);
			s[2] = size >> 8;
			s[3] = size & 0xFF;
			std::copy_n(data, size, &s[4]);
		}
		else{
			s.resize(std::size_t(size) + 10);
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
			std::copy_n(data, size, &s[10]);
		}

		return s;
	};
	auto pack_data_client(const char *data, int size)->std::string{
		std::string s;
		if (size < 126){
			s.resize(std::size_t(size) + 6);
			s[0] = char(0x82);// binary data, 0x81 is text data
			s[1] = char(size) | char(0x80);
			std::copy_n(data, size, &s[6]);
		}
		else if (size < 0xFFFF){
			s.resize(std::size_t(size) + 8);
			s[0] = char(0x82);
			s[1] = char(126) | char(0x80);
			s[2] = size >> 8;
			s[3] = size & 0xFF;
			std::copy_n(data, size, &s[8]);
		}
		else{
			s.resize(std::size_t(size) + 14);
			s[0] = char(0x82);
			s[1] = char(127) | char(0x80);
			s[2] = 0;
			s[3] = 0;
			s[4] = 0;
			s[5] = 0;
			s[6] = size >> 24;
			s[7] = size >> 16;
			s[8] = size >> 8;
			s[9] = size & 0xFF;
			std::copy_n(data, size, &s[14]);
		}

		std::fill_n(&s[s.size() - size - 4], 4, 0xf0);
		for (int i = 0; i < size; ++i){
			s[i + s.size() - size] = s[i + s.size() - size] ^ s[s.size() - size - 4 + i % 4];
		}

		return s;
	};
	auto make_header_map(const std::string &hand_shake_text)->std::map<std::string, std::string>{
		std::istringstream istream(hand_shake_text);

		// 找到Sec-WebSocket-Key //
		std::map<std::string, std::string> header_map;
		std::string header;
		while (std::getline(istream, header) && header != "\r"){
			if (header[header.size() - 1] != '\r'){
				continue; //end
			}
			else{
				header.erase(header.end() - 1);    //remove last char
			}

			auto end = header.find(": ", 0);
			if (end != std::string::npos){
				std::string key = header.substr(0, end);
				std::string value = header.substr(end + 2);
				header_map[key] = value;
			}
		}

		return header_map;
	}

	struct Socket::Imp{
		Socket* socket_;
		Socket::State state_{ State::IDLE };
		Socket::Type type_{ Type::TCP };
		bool is_server_{false};
		std::int64_t connect_time_out_{ -1 };

		std::function<int(Socket *, aris::core::Msg &)> onReceivedMsg;
		std::function<int(Socket *, const char *data, int size)> onReceivedData;
		std::function<int(Socket *, const char *, int)> onReceivedConnection;
		std::function<int(Socket *)> onLoseConnection;

		decltype(socket(AF_INET, SOCK_STREAM, 0)) lisn_socket_, recv_socket_;  //也可以用SOCKET类型
		struct sockaddr_in server_addr_ {}, client_addr_{};
		socklen_t sin_size_;

		std::string remote_ip_, port_;

		// 线程同步变量 //
		std::recursive_mutex state_mutex_;

		std::thread recv_thread_, accept_thread_;
		std::recursive_mutex close_mutex_;

		// 连接的socket //
#ifdef WIN32
		WSADATA wsa_data_;             //windows下才用,linux下无该项
#endif
		~Imp() = default;
		Imp(Socket* sock) :socket_(sock), lisn_socket_(0), recv_socket_(0), sin_size_(sizeof(struct sockaddr_in)), state_(Socket::State::IDLE)
			, onReceivedMsg(nullptr), onReceivedData(nullptr), onReceivedConnection(nullptr), onLoseConnection(nullptr) {}

		static void receiveThread(Socket::Imp* imp, std::promise<void> receive_thread_ready);
		static void acceptThread(Socket::Imp* imp, std::promise<void> accept_thread_ready);

		auto lose_tcp()->void{
			// 需要用close_lck 来确保本段代码不会被stop中断 //
			std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
			if (!close_lck.try_lock()){
				close_sock(recv_socket_);
			}
			else{
				shutdown(recv_socket_, 2);
				close_sock(recv_socket_);
				
				std::unique_lock<std::recursive_mutex> state_lck(state_mutex_);
				state_ = State::IDLE;
				recv_thread_.detach();
				if (onLoseConnection)onLoseConnection(socket_);
				return;
			}
		}
	};
	auto Socket::Imp::acceptThread(Socket::Imp* imp, std::promise<void> accept_thread_ready)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		// 改变状态 //
		imp->state_ = State::WAITING_FOR_CONNECTION;

		// 通知主线程,accept线程已经拷贝完毕,准备监听 //
		accept_thread_ready.set_value();

		// 开始建立连接 //
		for (;;){
			// 服务器阻塞,直到客户程序建立连接 //
			imp->recv_socket_ = accept(imp->lisn_socket_, (struct sockaddr *)(&imp->client_addr_), &imp->sin_size_);
			
			if (imp->recv_socket_ == -1){
				ARIS_LOG(SOCKET_FAILED_ACCEPT, (int)imp->recv_socket_);
				
				std::unique_lock<std::recursive_mutex> close_lck(imp->close_mutex_, std::defer_lock);
				if (!close_lck.try_lock()){
					close_sock(imp->lisn_socket_);
					imp->state_ = State::IDLE;
					imp->accept_thread_.detach();
					return;
				}
				else {
					std::cout << "some error happned when socket accept" << std::endl;
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
				
			}

			if (imp->type_ == Type::WEB || imp->type_ == Type::WEB_RAW){
				std::this_thread::sleep_for(std::chrono::seconds(3));
				
				char recv_data[1024]{ 0 };
				int res = recv(imp->recv_socket_, recv_data, 1024, 0);
				if (res <= 0){
					ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED, res);
					shutdown(imp->recv_socket_, 2);
					close_sock(imp->recv_socket_);
					continue;
				}

				auto header_map = make_header_map(recv_data);
				std::string server_key;
				try{
					server_key = header_map.at("Sec-WebSocket-Key");
				}
				catch (std::exception &){
					ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_INVALID_KEY);
					shutdown(imp->recv_socket_, 2);
					close_sock(imp->recv_socket_);
					continue;
				}
				server_key += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

				// 找到返回的key //
				SHA1 checksum;
				checksum.update(server_key);
				std::string hash = checksum.final();

				std::uint32_t message_digest[5]{};
				for (Size i = 0; i < 20; ++i){
					char num[5] = "0x00";
					std::copy_n(hash.data() + i * 2, 2, num + 2);
					std::uint8_t n = std::stoi(num, 0, 16);
					*(reinterpret_cast<unsigned char*>(message_digest) + i) = n;
				}

				auto ret_hey = base64_encode2(reinterpret_cast<const unsigned char*>(message_digest), 20);

				std::string shake_hand;
				shake_hand = "HTTP/1.1 101 Switching Protocols\r\n"
					"Upgrade: websocket\r\n"
					"Connection: Upgrade\r\n"
					"Sec-WebSocket-Accept: " + ret_hey + std::string("\r\n\r\n");

				auto ret = send(imp->recv_socket_, shake_hand.c_str(), static_cast<int>(shake_hand.size()), 0);
				
				if (ret == -1){
					ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_LOOSE_CONNECTION);
					shutdown(imp->recv_socket_, 2);
					close_sock(imp->recv_socket_);
					continue;
				};
			}

			break;
		}
		
		shutdown(imp->lisn_socket_, 2);
		close_sock(imp->lisn_socket_);

		// 否则,开始开启数据线程 //
		if (imp->onReceivedConnection)imp->onReceivedConnection(imp->socket_, inet_ntoa(imp->client_addr_.sin_addr), ntohs(imp->client_addr_.sin_port));

		std::promise<void> receive_thread_ready;
		auto fut = receive_thread_ready.get_future();
		imp->recv_thread_ = std::thread(receiveThread, imp, std::move(receive_thread_ready));
		fut.wait();

		imp->accept_thread_.detach();
		return;
	}
	auto Socket::Imp::receiveThread(Socket::Imp* imp, std::promise<void> receive_thread_ready)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		// 设置为 non-blocking 模式 //
#ifdef WIN32
		u_long block = 0;
		if (ioctlsocket(imp->recv_socket_, FIONBIO, &block) == SOCKET_ERROR) {
			imp->lose_tcp();
		}
#endif
#ifdef UNIX
		long arg;
		if ((arg = fcntl(imp->recv_socket_, F_GETFL, NULL)) < 0) {
			imp->lose_tcp();
		}
		arg &= (~O_NONBLOCK);
		if (fcntl(imp->recv_socket_, F_SETFL, arg) < 0) {
			imp->lose_tcp();
		}
#endif
		
		// 改变状态 //
		imp->state_ = Socket::State::WORKING;

		// 通知accept或connect线程已经准备好,下一步开始收发数据 //
		receive_thread_ready.set_value();

		aris::core::Msg recv_msg;
		recv_msg.resize(1024);

		// 开启接受数据的循环 //
		for (;;){
			switch (imp->type_){
			case Type::TCP:{
				MsgHeader header;
				if (safe_recv(imp->recv_socket_, reinterpret_cast<char *>(&header), sizeof(MsgHeader)) <= 0) { imp->lose_tcp(); return; }
				if (header.msg_size_ > 0x00100000 || header.msg_size_ < 0) { imp->lose_tcp(); return; }
				recv_msg.resize(header.msg_size_);
				recv_msg.header() = header;
				if (recv_msg.size() > 0 && safe_recv(imp->recv_socket_, recv_msg.data(), recv_msg.size()) <= 0) { imp->lose_tcp(); return; }
				if (imp->onReceivedMsg)imp->onReceivedMsg(imp->socket_, recv_msg);
				break;
			}
			case Type::TCP_RAW: {
				char data[1024];
				int ret = recv(imp->recv_socket_, data, 1024, 0);
				if (ret <= 0) {
					close_sock(imp->recv_socket_);
					imp->lose_tcp(); return;
				}
				if (ret > 0 && imp->onReceivedData)imp->onReceivedData(imp->socket_, data, ret);
				break;
			}
			case Type::WEB:{
				std::string payload_data;
				for (bool fin{ false }; !fin;){
					// 接受头 //
					char web_head[2];
					if (safe_recv(imp->recv_socket_, web_head, 2) <= 0) { imp->lose_tcp(); return; }

					// 是否最后一帧 //
					fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

					// 获取opcode //
					std::int8_t op_code = web_head[0] & 0x0f;
					if (op_code == 0x08) { imp->lose_tcp(); return; }

					// 获取数据长度
					std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
					if (payload_len == 126)	{
						char length_char[2];
						if (safe_recv(imp->recv_socket_, length_char, 2) <= 0) { imp->lose_tcp(); return; }

						union{
							std::uint16_t length;
							char reverse_char[2];
						};
						for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
						payload_len = length;
					}
					else if (payload_len == 127){
						char length_char[8];
						if (safe_recv(imp->recv_socket_, length_char, 8) <= 0) { imp->lose_tcp(); return; }

						char reverse_char[8];
						for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
						std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
					}

					//////////////////////////////////保护，数据不能太大///////////////////////////////
					if (payload_len < 0 || payload_len > 0x00080000 || payload_len + payload_data.size() > 0x00100000){
						ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
						imp->lose_tcp();
						return;
					}

					// 获取掩码
					bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
					char masks[4];
					if (mask_flag && safe_recv(imp->recv_socket_, masks, 4) <= 0) { imp->lose_tcp(); return; }

					// 用掩码读取出数据 //
					auto last_size = payload_data.size();
					payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
					if (safe_recv(imp->recv_socket_, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0) { imp->lose_tcp(); return; }

					if (mask_flag) {
						for (int i{ 0 }; i < payload_len; ++i) {
							payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
						}
					}
				}

				//////////////////////////////////保护，最短长度不能小于MsgHeader的数据长度///////////////////////////////
				if (payload_data.size() < sizeof(aris::core::MsgHeader)){
					ARIS_LOG(WEBSOCKET_RECEIVE_RAW);
					break;
				}

				// 把web sock 的东西转成 msg //
				recv_msg.resize(static_cast<aris::core::MsgSize>(payload_data.size() - sizeof(aris::core::MsgHeader)));
				std::copy(payload_data.data(), payload_data.data() + payload_data.size(), reinterpret_cast<char*>(&recv_msg.header()));

				if (recv_msg.size() != payload_data.size() - sizeof(aris::core::MsgHeader)){
					ARIS_LOG(WEBSOCKET_RECEIVE_WRONG_MSG_SIZE, recv_msg.size(), payload_data.size());
					break;
				}

				if (imp->onReceivedMsg)imp->onReceivedMsg(imp->socket_, recv_msg);
				break;
			}
			case Type::WEB_RAW:{
				std::int64_t real_length{ 0 };
				std::string payload_data;

				for (bool fin{ false }; !fin;){
					// 接受头 //
					char web_head[2];
					if (safe_recv(imp->recv_socket_, web_head, 2) <= 0) { imp->lose_tcp(); return; }

					// 是否最后一帧 //
					fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

					// 获取opcode //
					std::int8_t op_code = web_head[0] & 0x0f;
					if (op_code == 0x08) { imp->lose_tcp(); return; }

					// 获取数据长度
					std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
					if (payload_len == 126){
						char length_char[2];
						if (safe_recv(imp->recv_socket_, length_char, 2) <= 0) { imp->lose_tcp(); return; }

						union{
							std::uint16_t length;
							char reverse_char[2];
						};
						for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
						payload_len = length;
					}
					else if (payload_len == 127){
						char length_char[8];
						if (safe_recv(imp->recv_socket_, length_char, 8) <= 0) { imp->lose_tcp(); return; }

						char reverse_char[8];
						for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
						std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
					}

					//////////////////////////////////保护，数据不能太大///////////////////////////////
					if (payload_len > 0x00100000 || payload_len + payload_data.size() > 0x00200000){
						ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
						imp->lose_tcp();
						return;
					}

					// 获取掩码
					bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
					char masks[4];
					if (mask_flag && safe_recv(imp->recv_socket_, masks, 4) <= 0) { imp->lose_tcp(); return; }

					// 用掩码读取出数据 //
					auto last_size = payload_data.size();
					payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
					if (safe_recv(imp->recv_socket_, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0) { imp->lose_tcp(); return; }

					if (mask_flag) {
						for (int i{ 0 }; i < payload_len; ++i) {
							payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
						}
					}
				}

				if (imp->onReceivedData)imp->onReceivedData(imp->socket_, payload_data.data(), static_cast<int>(payload_data.size()));

				break;
			}
			case Type::UDP:{
				int ret = recvfrom(imp->recv_socket_, reinterpret_cast<char *>(&recv_msg.header()), 1024, 0, (struct sockaddr *)(&imp->client_addr_), &imp->sin_size_);
				
				std::unique_lock<std::recursive_mutex> close_lck(imp->close_mutex_, std::defer_lock);
				if (ret <= 0 && !close_lck.try_lock()) return;
				if (ret != sizeof(MsgHeader) + recv_msg.size()){
					ARIS_LOG(SOCKET_UDP_WRONG_MSG_SIZE);
					continue;
				}
				if(ret > 0 && imp->onReceivedMsg)imp->onReceivedMsg(imp->socket_, recv_msg);
				break;
			}
			case Type::UDP_RAW:{
				char data[1024];
				int ret = recvfrom(imp->recv_socket_, data, 1024, 0, (struct sockaddr *)(&imp->client_addr_), &imp->sin_size_);
				std::unique_lock<std::recursive_mutex> close_lck(imp->close_mutex_, std::defer_lock);
				if (ret <= 0 && !close_lck.try_lock()) return;
				if (ret > 0 && imp->onReceivedData)imp->onReceivedData(imp->socket_, data, ret);
				break;
			}}
		}
	}
	auto Socket::stop()->void{
		std::lock(imp_->state_mutex_, imp_->close_mutex_);
		std::unique_lock<std::recursive_mutex> lck1(imp_->state_mutex_, std::adopt_lock);
		std::unique_lock<std::recursive_mutex> lck2(imp_->close_mutex_, std::adopt_lock);

		switch (imp_->state_){
		case State::IDLE:
			break;
		case State::WAITING_FOR_CONNECTION:
			shutdown(imp_->lisn_socket_, 2);
			close_sock(imp_->lisn_socket_);
			while (imp_->accept_thread_.joinable())
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			break;
		case State::WORKING:
			switch (connectType()){
			case Type::TCP:
			case Type::TCP_RAW:
			case Type::WEB:
			case Type::WEB_RAW:
				if (shutdown(imp_->recv_socket_, 2) < 0) ARIS_LOG(SOCKET_SHUT_DOWN_ERROR, errno);
				break;
			case Type::UDP:
			case Type::UDP_RAW:
				if (close_sock(imp_->recv_socket_) < 0) ARIS_LOG(SOCKET_SHUT_CLOSE_ERROR, errno);
				break;
			}

			if(imp_->recv_thread_.joinable())imp_->recv_thread_.join();
			break;
		}

		imp_->state_ = Socket::State::IDLE;
	}
	auto Socket::startServer(const std::string &port)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		if (!port.empty())setPort(port);
		if (this->port().empty())THROW_FILE_LINE("Socket can't Start as server, because it has empty port\n");

		switch (imp_->state_){
		case State::IDLE:
			break;
		default:
			THROW_FILE_LINE("Socket can't Start as server, because it is not at idle state\n");
		}

		imp_->is_server_ = true;

		//////////////////////////////////////////////////////////////////////////////////////////////
		int sock_type;
		switch (connectType()){
		case Type::TCP:
		case Type::TCP_RAW:
		case Type::WEB:
		case Type::WEB_RAW:
			sock_type = SOCK_STREAM;
			break;
		case Type::UDP:
		case Type::UDP_RAW:
			sock_type = SOCK_DGRAM;
			break;
		}
		///////////////////////////////////////////////////////////////////////////////////////////////

		// 服务器端开始建立socket描述符 //
		if (static_cast<int>(imp_->lisn_socket_ = socket(AF_INET, sock_type, 0)) == -1)THROW_FILE_LINE("Socket can't Start as server, because it can't socket\n");

		// linux 下设置keep alive
#ifdef UNIX
		if (sock_type == SOCK_STREAM){
			int tcp_timeout = 10000; //10 seconds before aborting a write()
			if (setsockopt(imp_->lisn_socket_, SOL_TCP, TCP_USER_TIMEOUT, &tcp_timeout, sizeof(int)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_USER_TIMEOUT FAILED");
			}

			// Set the option active //
			int keepAlive = 1; // 开启keepalive属性
			int keepIdle = 5; // 如该连接在5秒内没有任何数据往来,则进行探测 
			int keepInterval = 1; // 探测时发包的时间间隔为5 秒
			int keepCount = 5; // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发.

			if (setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepAlive, sizeof(keepAlive)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt SO_KEEPALIVE FAILED");
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPIDLE, (void*)&keepIdle, sizeof(keepIdle)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_KEEPIDLE FAILED");
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPINTVL, (void*)&keepInterval, sizeof(keepInterval)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_KEEPINTVL FAILED");
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPCNT, (void*)&keepCount, sizeof(keepCount)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_KEEPCNT FAILED");
			}
		}
#endif


		// 设置socketopt选项,使得地址在程序结束后立即可用 //
		int nvalue = 1;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&nvalue), sizeof(int)) < 0) {
			close_sock(imp_->lisn_socket_);
			THROW_FILE_LINE("setsockopt failed: SO_REUSEADDR \n");
		}
			

		// 服务器端填充server_addr_结构,并且bind //
		memset(&imp_->server_addr_, 0, sizeof(struct sockaddr_in));
		imp_->server_addr_.sin_family = AF_INET;
		imp_->server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
		imp_->server_addr_.sin_port = htons(std::stoi(this->port()));
		if (::bind(imp_->lisn_socket_, (struct sockaddr *)(&imp_->server_addr_), sizeof(struct sockaddr)) == -1){
#ifdef WIN32
			int err = WSAGetLastError();
#endif
			close_sock(imp_->lisn_socket_);
			THROW_FILE_LINE("Socket can't Start as server, because it can't bind\n");
		}
		
		if (connectType() == Type::TCP || connectType() == Type::TCP_RAW || connectType() == Type::WEB || connectType() == Type::WEB_RAW){
			// 监听lisn_socket_描述符 //
			if (listen(imp_->lisn_socket_, 5) == -1) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("Socket can't Start as server, because it can't listen\n");
			}

			// 启动等待连接的线程 //
			std::promise<void> accept_thread_ready;
			auto ready = accept_thread_ready.get_future();
			imp_->accept_thread_ = std::thread(Socket::Imp::acceptThread, this->imp_.get(), std::move(accept_thread_ready));
			ready.wait();
		}
		else{
			// 因为UDP没法shutdown，所以用非阻塞模式 //
#ifdef WIN32
			DWORD read_timeout = 10;
			if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("setsockopt failed: SO_RCVTIMEO \n");
			}
#endif
#ifdef UNIX
			struct timeval read_timeout;
			read_timeout.tv_sec = 0;
			read_timeout.tv_usec = 10000;
			if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0) {
				close_sock(imp_->lisn_socket_);
				THROW_FILE_LINE("setsockopt failed: SO_RCVTIMEO \n");
			}
#endif
			imp_->recv_socket_ = imp_->lisn_socket_;
			
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(Socket::Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();
		}

		return;
	}
	auto Socket::connect(const std::string& remote_ip, const std::string& port)->void {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		// check ip & port //
		if (!remote_ip.empty())setRemoteIP(remote_ip);
		if (!port.empty())setPort(port);
		if (remoteIP().empty())THROW_FILE_LINE("Socket can't connect, because it empty ip address\n");
		if (this->port().empty())THROW_FILE_LINE("Socket can't connect, because it empty port\n");

		// check state //
		switch (imp_->state_) {
		case State::IDLE:
			break;
		default:
			THROW_FILE_LINE("Socket can't connect, because it is busy now, please close it\n");
		}
		imp_->is_server_ = false;

		// 根据 UDP 或 TCP 设置连接类型 //
		int sock_type;
		switch (connectType()) {
		case Type::TCP:
		case Type::TCP_RAW:
		case Type::WEB:
		case Type::WEB_RAW:
			sock_type = SOCK_STREAM;
			break;
		case Type::UDP:
		case Type::UDP_RAW:
			sock_type = SOCK_DGRAM;
			break;
		}

		// 填充 ip & port //
		memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
		imp_->server_addr_.sin_family = AF_INET;
		imp_->server_addr_.sin_addr.s_addr = inet_addr(remoteIP().c_str());
		imp_->server_addr_.sin_port = htons(std::stoi(this->port()));

		// 建立 socket 描述符 //
		if ((imp_->recv_socket_ = socket(AF_INET, sock_type, 0)) < 0)THROW_FILE_LINE("Socket can't connect, because can't socket\n");

		// 设置 time_out //
		if (imp_->connect_time_out_ >= 0) {
#ifdef WIN32
			u_long block = 1;
			if (ioctlsocket(imp_->recv_socket_, FIONBIO, &block) == SOCKET_ERROR) {
#endif
#ifdef UNIX
			if (fcntl(imp_->recv_socket_, F_SETFL, O_NONBLOCK) < 0) {
#endif
				close_sock(imp_->recv_socket_);
				THROW_FILE_LINE("Socket can't connect, because can't set time out\n");
			}
		}

		// 连接 socket //
		switch (connectType()){
		case Type::TCP:
		case Type::TCP_RAW:{
			// 连接 //
			if (::connect(imp_->recv_socket_, (const struct sockaddr*)&imp_->server_addr_, sizeof(imp_->server_addr_)) == -1) {
				fd_set setW, setE;
				FD_ZERO(&setW);
				FD_SET(imp_->recv_socket_, &setW);
				FD_ZERO(&setE);
				FD_SET(imp_->recv_socket_, &setE);

				timeval time_out = { 0 };
				time_out.tv_sec = imp_->connect_time_out_ / 1000;
				time_out.tv_usec = (imp_->connect_time_out_ % 1000) * 1000;

#ifdef WIN32
				if (WSAGetLastError() == WSAEWOULDBLOCK) {
					// connection pending
					int ret = select(0, NULL, &setW, &setE, &time_out);
					if (ret < 0){
						close_sock(imp_->recv_socket_);
						THROW_FILE_LINE("Socket can't connect, because failed to select\n");
					}
					else if (ret == 0) {
						close_sock(imp_->recv_socket_);
						WSASetLastError(WSAETIMEDOUT);
						THROW_FILE_LINE("Socket can't connect, because time out\n");
					}
					else {
					
					}

					if (FD_ISSET(imp_->recv_socket_, &setE)){
						// connection failed
						int err = 0, err_size = sizeof(int);
						getsockopt(imp_->recv_socket_, SOL_SOCKET, SO_ERROR, (char*) &err, &err_size);
						close_sock(imp_->recv_socket_);
						WSASetLastError(err);
						THROW_FILE_LINE("Socket can't connect, because failed to FD_ISSET\n");
					}
				}
				else {
					close_sock(imp_->recv_socket_);
					THROW_FILE_LINE("Socket can't connect, because can't connect\n");
				}
#endif
#ifdef UNIX
				do {
					auto ret = select(imp_->recv_socket_ + 1, NULL, &setW, &setE, &time_out);
					if (ret > 0) {
						int so_error;
						socklen_t len = sizeof(so_error);
						if (getsockopt(imp_->recv_socket_, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) {
							close_sock(imp_->recv_socket_);
							THROW_FILE_LINE("Socket can't connect, because can't getsockopt\n");
						}
						if (so_error) {
							close_sock(imp_->recv_socket_);
							if (time_out.tv_sec == 0 && time_out.tv_usec == 0)
								THROW_FILE_LINE("Socket can't connect, because getsockopt error\n");
							else
								continue;
						}
						else
							break;// 正常结束
					}
					else if (ret == 0) {
						close_sock(imp_->recv_socket_);
						THROW_FILE_LINE("Socket can't connect, because time out\n");
					}
					else {
						close_sock(imp_->recv_socket_);
						THROW_FILE_LINE("Socket can't connect, because failed to select\n");
					}
				} while (1);
#endif	
			}

			imp_->state_ = Socket::State::WORKING;

			// Start Thread //
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();

			return;
		}
		case Type::WEB:
		case Type::WEB_RAW:{
			if (::connect(imp_->recv_socket_, (const struct sockaddr *)&imp_->server_addr_, sizeof(imp_->server_addr_)) == -1) {
				close_sock(imp_->recv_socket_);
				THROW_FILE_LINE("Socket can't connect, because can't connect\n");
			}

			char handshake_text[]{
				"GET / HTTP/1.1\r\n"
				"Host: localhost:8080\r\n"
				"Origin: http://127.0.0.1:3000\r\n"
				"Connection: Upgrade\r\n"
				"Upgrade: websocket\r\n"
				"Sec-WebSocket-Version: 13\r\n"
				"Sec-WebSocket-Key: w4v7O6xFTi36lq3RNcgctw==\r\n\r\n" };

			if(send(imp_->recv_socket_, handshake_text, static_cast<int>(std::strlen(handshake_text)), 0) == -1)
				THROW_FILE_LINE("Socket can't connect, web sock error 1\n");

			char recv_data[1024]{ 0 };
			int res = recv(imp_->recv_socket_, recv_data, 1024, 0);
			if(res <= 0)THROW_FILE_LINE("Socket can't connect, web sock error 2\n");

			auto header_map = make_header_map(recv_data);

			////////////   这里应该check更多，tbd //
			if(header_map.find("Sec-WebSocket-Accept") == header_map.end())THROW_FILE_LINE("Socket can't connect, web sock error 3\n");

			imp_->state_ = Socket::State::WORKING;

			// Start Thread //
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();

			return;
		}
		case Type::UDP:
		case Type::UDP_RAW:
			imp_->state_ = Socket::State::WORKING;
			return;
		}
	}
	auto Socket::sendMsg(const aris::core::MsgBase &data)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif

		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:{
			switch (imp_->type_){
			case Type::TCP:
				if (send(imp_->recv_socket_, reinterpret_cast<const char *>(&data.header()), data.size() + sizeof(MsgHeader), 0) == -1)
					THROW_FILE_LINE("Socket failed sending data, because network failed\n");
				else
					return;
				break;
			case Type::WEB:{
				auto packed_data = imp_->is_server_ 
					? pack_data_server(reinterpret_cast<const char*>(&data.header()), data.size() + sizeof(aris::core::MsgHeader))
					: pack_data_client(reinterpret_cast<const char*>(&data.header()), data.size() + sizeof(aris::core::MsgHeader));
				if (send(imp_->recv_socket_, packed_data.data(), static_cast<int>(packed_data.size()), 0) == -1)
					THROW_FILE_LINE("Socket failed sending data, because network failed\n");
				else
					return;
				break;
			}
			case Type::UDP:{
				memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
				imp_->server_addr_.sin_family = AF_INET;
				imp_->server_addr_.sin_addr.s_addr = inet_addr(remoteIP().c_str());
				imp_->server_addr_.sin_port = htons(std::stoi(this->port()));
				
				if (sendto(imp_->recv_socket_, reinterpret_cast<const char *>(&data.header()), data.size() + sizeof(MsgHeader), 0, (const struct sockaddr *)&imp_->server_addr_, sizeof(imp_->server_addr_)) == -1)
					THROW_FILE_LINE("Socket failed sending data, because network failed\n");
				else
					return;

				break;
			}
			default:
				THROW_FILE_LINE("Socket failed send msg, because Socket is not at right MODE\n");
			}
		}
		default:
			THROW_FILE_LINE("Socket failed sending data, because Socket is not at right STATE\n");
		}
	}
	auto Socket::sendRawData(const char *data, int size)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:{
			switch (imp_->type_){
			case Type::TCP_RAW: {
				if (send(imp_->recv_socket_, data, size, 0) == -1)
					THROW_FILE_LINE("Socket failed sending data, because network failed\n");
				else
					return;
				break;
			}
			case Type::WEB_RAW:{
				auto packed_data = imp_->is_server_ ? pack_data_server(data, size) : pack_data_client(data, size);
				if (send(imp_->recv_socket_, packed_data.data(), static_cast<int>(packed_data.size()), 0) == -1)
					THROW_FILE_LINE("Socket failed sending data, because network failed\n");
				else
					return;
				break;
			}
			case Type::UDP_RAW:{
				memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
				imp_->server_addr_.sin_family = AF_INET;
				imp_->server_addr_.sin_addr.s_addr = inet_addr(remoteIP().c_str());
				imp_->server_addr_.sin_port = htons(std::stoi(this->port()));

				if (sendto(imp_->recv_socket_, data, size, 0, (const struct sockaddr *)&imp_->server_addr_, sizeof(imp_->server_addr_)) == -1)
					THROW_FILE_LINE("Socket failed sending data, because network failed\n");
				else
					return;

				break;
			}
			default:
				THROW_FILE_LINE("Socket failed send raw data, because Socket is not at right MODE\n");
			}
		}
		default:
			THROW_FILE_LINE("Socket failed send raw data, because Socket is not at right STATE\n");
		}
	}
	auto Socket::isConnected()->bool{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:
			return true;
		default:
			return false;
		}
	}
	auto Socket::state()->State{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->state_;
	};
	
	auto Socket::port()const->const std::string&{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->port_;
	}
	auto Socket::setPort(const std::string &port)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->port_ = port;
	}
	auto Socket::remoteIP()const->const std::string &{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->remote_ip_;
	}
	auto Socket::setRemoteIP(const std::string &remote_ip)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->remote_ip_ = remote_ip;
	}
	auto Socket::connectType()const->Type { return imp_->type_; }
	auto Socket::connectTimeoutMs()const->std::int64_t {
		return imp_->connect_time_out_;
	}
	auto Socket::setConnectTimeoutMs(std::int64_t time_out_ms)->void {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->connect_time_out_ = time_out_ms;
	}

	auto Socket::setConnectType(const Type type)->void { imp_->type_ = type; }
	auto Socket::setOnReceivedMsg(std::function<int(Socket*, aris::core::Msg &)> OnReceivedData)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedMsg = OnReceivedData;
	}
	auto Socket::setOnReceivedRawData(std::function<int(Socket*, const char *data, int size)> func)->void{ 
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedData = func;
	}
	auto Socket::setOnReceivedConnection(std::function<int(Socket*, const char*, int)> OnReceivedConnection)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedConnection = OnReceivedConnection;
	}
	auto Socket::setOnLoseConnection(std::function<int(Socket*)> OnLoseConnection)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onLoseConnection = OnLoseConnection;
	}
	Socket::~Socket(){ 
		if (imp_){
			stop();
#ifdef WIN32 
			WSACleanup();
#endif
		}
	}
	Socket::Socket(const std::string &name, const std::string& remote_ip, const std::string& port, Type type) :imp_(new Imp(this)){
		// 启动服务器 //
#ifdef WIN32 
		if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)THROW_FILE_LINE("Socket can't Start as server, because it can't WSAstartup\n");
#endif
		setRemoteIP(remote_ip);
		setPort(port);
		setConnectType(type);
	}
	Socket::Socket(Socket&&s)noexcept{
		imp_ = std::move(s.imp_);
		imp_->socket_ = this;
	};
	Socket& Socket::operator=(Socket&&s)noexcept {
		imp_ = std::move(s.imp_);
		imp_->socket_ = this;
		return *this;
	}

	ARIS_REGISTRATION{
		aris::core::class_<Socket::Type>("Socket::connect_type")
			.textMethod([](Socket::Type *v)->std::string{
				auto type = *reinterpret_cast<Socket::Type*>(v);
				if (type == Socket::Type::TCP)return "TCP";
				else if (type == Socket::Type::TCP_RAW)return "TCP_RAW";
				else if (type == Socket::Type::WEB)return "WEB";
				else if (type == Socket::Type::WEB_RAW)return "WEB_RAW";
				else if (type == Socket::Type::UDP)return "UDP";
				else if (type == Socket::Type::UDP_RAW)return "UDP_RAW";
				else THROW_FILE_LINE("unknown connect type");
			},[](Socket::Type *v,std::string_view str)->void{
				if (str == "TCP")*reinterpret_cast<Socket::Type*>(v) = Socket::Type::TCP;
				else if (str == "TCP_RAW")*reinterpret_cast<Socket::Type*>(v) = Socket::Type::TCP_RAW;
				else if (str == "WEB")*reinterpret_cast<Socket::Type*>(v) = Socket::Type::WEB;
				else if (str == "WEB_RAW")*reinterpret_cast<Socket::Type*>(v) = Socket::Type::WEB_RAW;
				else if (str == "UDP")*reinterpret_cast<Socket::Type*>(v) = Socket::Type::UDP;
				else if (str == "UDP_RAW")*reinterpret_cast<Socket::Type*>(v) = Socket::Type::UDP_RAW;
				else THROW_FILE_LINE("unknown connect type");
			});

		class_<Socket>("Socket")
			.prop("connect_type", &Socket::setConnectType, &Socket::connectType)
			.prop("remote_ip", &Socket::setRemoteIP, &Socket::remoteIP)
			.prop("port", &Socket::setPort, &Socket::port);
	}
}