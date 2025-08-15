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
#include <list>

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
#include <exception>

#include <errno.h>

#include "aris/core/socket_multi_io.hpp"
#include "aris/core/log.hpp"
#include "aris/core/sha1.h"


#define SOCKET_FAILED_ACCEPT                         aris::core::LogLvl::kError, -3001, {"socket failed to accept : %d" \
																						 "SocketMultiIo 接受链接失败：%d"		}
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
                                                                                         "SocketMultiIo Shutdown 关闭错误：%d"}
#define SOCKET_SHUT_CLOSE_ERROR                      aris::core::LogLvl::kError, -3010, {"socket close error %d"\
                                                                                         "SocketMultiIo Close 关闭错误：%d"}
#define SOCKET_SERVER_START_ERROR                    aris::core::LogLvl::kError, -3011, {"SocketServer startServer error: %s"\
                                                                                         "SocketServer startServer 错误：%s"}
#define SOCKET_SERVER_STOP_ERROR                     aris::core::LogLvl::kError, -3012, {"SocketServer stop error: %d"\
                                                                                         "SocketServer stop 错误：%d"}
#define SOCKET_SERVER_SEND_MSG_ERROR                 aris::core::LogLvl::kError, -3013, {"SocketServer sendMsg error: %d"\
                                                                                         "SocketServer sendMsg 错误：%d"}
#define SOCKET_SERVER_SEND_RAW_DATA_ERROR            aris::core::LogLvl::kError, -3014, {"SocketServer sendRawData error: %d"\
                                                                                         "SocketServer sendRawData 错误：%d"}

namespace aris::core{
	

	struct SocketRecvData {
		int required_length_{ sizeof(aris::core::MsgHeader)}, received_length_{0};
		std::string str_;
	};

	auto aris_send(SOCKET_T sock, const char* buf, int len, int flag)->int {
#ifdef WIN32
		return ::send(sock, buf, len, flag);
#endif
#ifdef UNIX
		return ::send(sock, buf, len, flag | MSG_NOSIGNAL);
#endif
	}
	auto aris_close(SOCKET_T s)->int{
#ifdef WIN32
		auto ret = closesocket(s);
#endif
#ifdef UNIX
		auto ret = close(s);
#endif
		return ret;
	}
	auto safe_recv333(SOCKET_T s, char *data, int size) -> int{
		int result{ 0 };
		for (; result < size; ){
			int ret = recv(s, data + result, size - result, 0);
			if (ret <= 0){
				aris_close(s);
				result = ret;
				break;
			}

			result += ret;
		}

		return result;
	};
	auto safe_recv2(SOCKET_T s, SocketRecvData &data) -> int {
		data.str_.resize(data.required_length_);
		auto ret = ::recv(s, data.str_.data() + data.received_length_, data.required_length_ - data.received_length_, 0);
		if (ret >= 0) data.received_length_ += ret;
		return ret;
	};

	// please refer to 
	// https://www.cnblogs.com/chyingp/p/websocket-deep-in.html
	// sha1 hash 生成出来的是纯数字，可以把它改成2进制来保存
	// 
	std::string base64_encode2_2(unsigned char const* bytes_to_encode, unsigned int in_len){
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
	auto pack_data_server2(const char *data, int size)->std::string{
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
	auto pack_data_client2(const char *data, int size)->std::string{
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
	auto make_header_map2(const std::string &hand_shake_text)->std::map<std::string, std::string>{
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

	struct SocketMultiIo::Imp{
		SocketMultiIo* socket_;
		SocketMultiIo::State state_{ State::IDLE };
		SocketMultiIo::Type type_{ Type::TCP };
		bool is_server_{false};
		std::int64_t connect_time_out_{ -1 };

		std::map<SOCKET_T, SocketRecvData> recv_data_;
		std::list<SOCKET_T> recv_sockets_;

		std::function<int(decltype(socket(AF_INET, SOCK_STREAM, 0)), aris::core::Msg &)> onReceivedMsg;
		std::function<int(SocketMultiIo *, const char *data, int size)> onReceivedData;
		std::function<int(SocketMultiIo *, const char *, int)> onReceivedConnection;
		std::function<int(SocketMultiIo *)> onLoseConnection;

		SOCKET_T lisn_socket_, recv_socket_;  //也可以用SOCKET类型
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
		Imp(SocketMultiIo* sock) :socket_(sock), lisn_socket_(0), recv_socket_(0), sin_size_(sizeof(struct sockaddr_in)), state_(SocketMultiIo::State::IDLE)
			, onReceivedMsg(nullptr), onReceivedData(nullptr), onReceivedConnection(nullptr), onLoseConnection(nullptr) {}

		static void receiveThread(SocketMultiIo::Imp* imp, std::promise<void> receive_thread_ready);
		static void acceptThread(SocketMultiIo::Imp* imp, std::promise<void> accept_thread_ready);

		auto lose_tcp()->void{
			// 需要用close_lck 来确保本段代码不会被stop中断 //
			std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
			if (!close_lck.try_lock()){
				aris_close(recv_socket_);
			}
			else{
				shutdown(recv_socket_, 2);
				aris_close(recv_socket_);
				
				std::unique_lock<std::recursive_mutex> state_lck(state_mutex_);
				state_ = State::IDLE;
				//recv_thread_.detach();
				if (onLoseConnection)onLoseConnection(socket_);
				return;
			}
		}
		auto lose_tcp2(SOCKET_T sock) -> void {
			// 需要用close_lck 来确保本段代码不会被stop中断 //
			std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
			if (!close_lck.try_lock()) {
				aris_close(sock);
			}
			else {
				shutdown(sock, 2);
				aris_close(sock);

				std::unique_lock<std::recursive_mutex> state_lck(state_mutex_);
				if (onLoseConnection)
					onLoseConnection(socket_);
				return;
			}
		}
		auto recv_from_sock(SOCKET_T recv_sock, SocketRecvData &recv_data) -> void {
			aris::core::Msg recv_msg;
			recv_msg.resize(1024);

			// 开启接受数据的循环 //
			for (;;) {
				switch (type_) {
				case SocketMultiIo::Type::TCP: {
					MsgHeader header;
					if (safe_recv333(recv_sock, reinterpret_cast<char*>(&header), sizeof(MsgHeader)) <= 0) { lose_tcp(); return; }
					if (header.msg_size_ > 0x00100000 || header.msg_size_ < 0) { lose_tcp(); return; }
					recv_msg.resize(header.msg_size_);
					recv_msg.header() = header;
					if (recv_msg.size() > 0 && safe_recv333(recv_sock, recv_msg.data(), recv_msg.size()) <= 0) { lose_tcp(); return; }
					if (onReceivedMsg)onReceivedMsg(recv_sock, recv_msg);
					break;
				}
				case Type::TCP_RAW: {
					char data[1024];
					int ret = recv(recv_sock, data, 1024, 0);
					if (ret <= 0) {
						aris_close(recv_sock);
						lose_tcp(); return;
					}
					//if (ret > 0 && onReceivedData)onReceivedData(socket_, data, ret);
					break;
				}
				case Type::WEB: {
					std::string payload_data;
					for (bool fin{ false }; !fin;) {
						// 接受头 //
						char web_head[2];
						if (safe_recv333(recv_sock, web_head, 2) <= 0) { lose_tcp(); return; }

						// 是否最后一帧 //
						fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

						// 获取opcode //
						std::int8_t op_code = web_head[0] & 0x0f;
						if (op_code == 0x08) { lose_tcp(); return; }

						// 获取数据长度
						std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
						if (payload_len == 126) {
							char length_char[2];
							if (safe_recv333(recv_sock, length_char, 2) <= 0) { lose_tcp(); return; }

							union {
								std::uint16_t length;
								char reverse_char[2];
							};
							for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
							payload_len = length;
						}
						else if (payload_len == 127) {
							char length_char[8];
							if (safe_recv333(recv_sock, length_char, 8) <= 0) { lose_tcp(); return; }

							char reverse_char[8];
							for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
							std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
						}

						//////////////////////////////////保护，数据不能太大///////////////////////////////
						if (payload_len < 0 || payload_len > 0x00080000 || payload_len + payload_data.size() > 0x00100000) {
							ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
							lose_tcp();
							return;
						}

						// 获取掩码
						bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
						char masks[4];
						if (mask_flag && safe_recv333(recv_sock, masks, 4) <= 0) { lose_tcp(); return; }

						// 用掩码读取出数据 //
						auto last_size = payload_data.size();
						payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
						if (safe_recv333(recv_sock, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0) { lose_tcp(); return; }

						if (mask_flag) {
							for (int i{ 0 }; i < payload_len; ++i) {
								payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
							}
						}
					}

					//////////////////////////////////保护，最短长度不能小于MsgHeader的数据长度///////////////////////////////
					if (payload_data.size() < sizeof(aris::core::MsgHeader)) {
						ARIS_LOG(WEBSOCKET_RECEIVE_RAW);
						break;
					}

					// 把web sock 的东西转成 msg //
					recv_msg.resize(static_cast<aris::core::MsgSize>(payload_data.size() - sizeof(aris::core::MsgHeader)));
					std::copy(payload_data.data(), payload_data.data() + payload_data.size(), reinterpret_cast<char*>(&recv_msg.header()));

					if (recv_msg.size() != payload_data.size() - sizeof(aris::core::MsgHeader)) {
						ARIS_LOG(WEBSOCKET_RECEIVE_WRONG_MSG_SIZE, recv_msg.size(), payload_data.size());
						break;
					}

					//if (onReceivedMsg)onReceivedMsg(socket_, recv_msg);
					break;
				}
				case Type::WEB_RAW: {
					std::int64_t real_length{ 0 };
					std::string payload_data;

					for (bool fin{ false }; !fin;) {
						// 接受头 //
						char web_head[2];
						if (safe_recv333(recv_sock, web_head, 2) <= 0) { lose_tcp(); return; }

						// 是否最后一帧 //
						fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

						// 获取opcode //
						std::int8_t op_code = web_head[0] & 0x0f;
						if (op_code == 0x08) { lose_tcp(); return; }

						// 获取数据长度
						std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
						if (payload_len == 126) {
							char length_char[2];
							if (safe_recv333(recv_sock, length_char, 2) <= 0) { lose_tcp(); return; }

							union {
								std::uint16_t length;
								char reverse_char[2];
							};
							for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
							payload_len = length;
						}
						else if (payload_len == 127) {
							char length_char[8];
							if (safe_recv333(recv_sock, length_char, 8) <= 0) { lose_tcp(); return; }

							char reverse_char[8];
							for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
							std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
						}

						//////////////////////////////////保护，数据不能太大///////////////////////////////
						if (payload_len > 0x00100000 || payload_len + payload_data.size() > 0x00200000) {
							ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
							lose_tcp();
							return;
						}

						// 获取掩码
						bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
						char masks[4];
						if (mask_flag && safe_recv333(recv_sock, masks, 4) <= 0) { lose_tcp(); return; }

						// 用掩码读取出数据 //
						auto last_size = payload_data.size();
						payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
						if (safe_recv333(recv_sock, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0) { lose_tcp(); return; }

						if (mask_flag) {
							for (int i{ 0 }; i < payload_len; ++i) {
								payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
							}
						}
					}

					//if (onReceivedData)onReceivedData(socket_, payload_data.data(), static_cast<int>(payload_data.size()));

					break;
				}
				case Type::UDP: {
					int ret = recvfrom(recv_sock, reinterpret_cast<char*>(&recv_msg.header()), 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size_);

					std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
					if (ret <= 0 && !close_lck.try_lock()) return;
					if (ret != sizeof(MsgHeader) + recv_msg.size()) {
						ARIS_LOG(SOCKET_UDP_WRONG_MSG_SIZE);
						continue;
					}
					//if (ret > 0 && onReceivedMsg)onReceivedMsg(socket_, recv_msg);
					break;
				}
				case Type::UDP_RAW: {
					char data[1024];
					int ret = recvfrom(recv_sock, data, 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size_);
					std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
					if (ret <= 0 && !close_lck.try_lock()) return;
					//if (ret > 0 && onReceivedData)onReceivedData(socket_, data, ret);
					break;
				}
				}
			}
		}
		auto recv_from_sock2(SOCKET_T recv_sock, SocketRecvData& recv_data) -> int {
			aris::core::Msg recv_msg;
			recv_msg.resize(1024);

			// 开启接受数据的循环 //
			//for (;;) {
				switch (type_) {
				case SocketMultiIo::Type::TCP: {
					if (safe_recv2(recv_sock, recv_data) <= 0) { 
						lose_tcp2(recv_sock);
						return -1;
					}
					if (recv_data.received_length_ >= sizeof(MsgHeader)) {
						auto msg_size = reinterpret_cast<MsgHeader*>(recv_data.str_.data())->msg_size_;
						if (msg_size > 0x00100000 || msg_size < 0) { lose_tcp2(recv_sock); return -1; }
						recv_data.required_length_ = msg_size + sizeof(MsgHeader);
					}
					if (recv_data.received_length_ == recv_data.required_length_ && onReceivedMsg) {
						recv_msg.resize(recv_data.required_length_ - sizeof(MsgHeader));
						std::copy(recv_data.str_.data(), recv_data.str_.data() + recv_data.received_length_, (char*)(&recv_msg.header()));
						onReceivedMsg(recv_sock, recv_msg);
						recv_data.required_length_ = sizeof(MsgHeader);
						recv_data.received_length_ = 0;
					}
					break;
				}
				case Type::TCP_RAW: {
					char data[1024];
					int ret = recv(recv_sock, data, 1024, 0);
					if (ret <= 0) {
						aris_close(recv_sock);
						lose_tcp(); return -1;
					}
					//if (ret > 0 && onReceivedData)onReceivedData(socket_, data, ret);
					break;
				}
				case Type::WEB: {
					std::string payload_data;
					for (bool fin{ false }; !fin;) {
						// 接受头 //
						char web_head[2];
						if (safe_recv333(recv_sock, web_head, 2) <= 0) { lose_tcp(); return -1; }

						// 是否最后一帧 //
						fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

						// 获取opcode //
						std::int8_t op_code = web_head[0] & 0x0f;
						if (op_code == 0x08) { lose_tcp(); return -1; }

						// 获取数据长度
						std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
						if (payload_len == 126) {
							char length_char[2];
							if (safe_recv333(recv_sock, length_char, 2) <= 0) { lose_tcp(); return -1; }

							union {
								std::uint16_t length;
								char reverse_char[2];
							};
							for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
							payload_len = length;
						}
						else if (payload_len == 127) {
							char length_char[8];
							if (safe_recv333(recv_sock, length_char, 8) <= 0) { lose_tcp(); return -1; }

							char reverse_char[8];
							for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
							std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
						}

						//////////////////////////////////保护，数据不能太大///////////////////////////////
						if (payload_len < 0 || payload_len > 0x00080000 || payload_len + payload_data.size() > 0x00100000) {
							ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
							lose_tcp();
							return -1;
						}

						// 获取掩码
						bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
						char masks[4];
						if (mask_flag && safe_recv333(recv_sock, masks, 4) <= 0) { lose_tcp(); return -1; }

						// 用掩码读取出数据 //
						auto last_size = payload_data.size();
						payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
						if (safe_recv333(recv_sock, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0) { lose_tcp(); return -1; }

						if (mask_flag) {
							for (int i{ 0 }; i < payload_len; ++i) {
								payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
							}
						}
					}

					//////////////////////////////////保护，最短长度不能小于MsgHeader的数据长度///////////////////////////////
					if (payload_data.size() < sizeof(aris::core::MsgHeader)) {
						ARIS_LOG(WEBSOCKET_RECEIVE_RAW);
						break;
					}

					// 把web sock 的东西转成 msg //
					recv_msg.resize(static_cast<aris::core::MsgSize>(payload_data.size() - sizeof(aris::core::MsgHeader)));
					std::copy(payload_data.data(), payload_data.data() + payload_data.size(), reinterpret_cast<char*>(&recv_msg.header()));

					if (recv_msg.size() != payload_data.size() - sizeof(aris::core::MsgHeader)) {
						ARIS_LOG(WEBSOCKET_RECEIVE_WRONG_MSG_SIZE, recv_msg.size(), payload_data.size());
						break;
					}

					//if (onReceivedMsg)onReceivedMsg(socket_, recv_msg);
					break;
				}
				case Type::WEB_RAW: {
					std::int64_t real_length{ 0 };
					std::string payload_data;

					for (bool fin{ false }; !fin;) {
						// 接受头 //
						char web_head[2];
						if (safe_recv333(recv_sock, web_head, 2) <= 0) { lose_tcp(); return -1; }

						// 是否最后一帧 //
						fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

						// 获取opcode //
						std::int8_t op_code = web_head[0] & 0x0f;
						if (op_code == 0x08) { lose_tcp(); return -1; }

						// 获取数据长度
						std::int64_t payload_len = web_head[1] & 0x7F; // 数据长度 
						if (payload_len == 126) {
							char length_char[2];
							if (safe_recv333(recv_sock, length_char, 2) <= 0) { lose_tcp(); return -1; }

							union {
								std::uint16_t length;
								char reverse_char[2];
							};
							for (int i = 0; i < 2; ++i)reverse_char[i] = length_char[1 - i];
							payload_len = length;
						}
						else if (payload_len == 127) {
							char length_char[8];
							if (safe_recv333(recv_sock, length_char, 8) <= 0) { lose_tcp(); return -1; }

							char reverse_char[8];
							for (int i = 0; i < 8; ++i)reverse_char[i] = length_char[7 - i];
							std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
						}

						//////////////////////////////////保护，数据不能太大///////////////////////////////
						if (payload_len > 0x00100000 || payload_len + payload_data.size() > 0x00200000) {
							ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
							lose_tcp();
							return -1;
						}

						// 获取掩码
						bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
						char masks[4];
						if (mask_flag && safe_recv333(recv_sock, masks, 4) <= 0) { lose_tcp(); return -1; }

						// 用掩码读取出数据 //
						auto last_size = payload_data.size();
						payload_data.resize(payload_data.size() + static_cast<std::size_t>(payload_len));
						if (safe_recv333(recv_sock, payload_data.data() + last_size, static_cast<int>(payload_len)) <= 0) { lose_tcp(); return -1; }

						if (mask_flag) {
							for (int i{ 0 }; i < payload_len; ++i) {
								payload_data[i + last_size] = payload_data[i + last_size] ^ masks[i % 4];
							}
						}
					}

					//if (onReceivedData)onReceivedData(socket_, payload_data.data(), static_cast<int>(payload_data.size()));

					break;
				}
				case Type::UDP: {
					int ret = recvfrom(recv_sock, reinterpret_cast<char*>(&recv_msg.header()), 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size_);

					std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
					if (ret <= 0 && !close_lck.try_lock()) return -1;
					if (ret != sizeof(MsgHeader) + recv_msg.size()) {
						ARIS_LOG(SOCKET_UDP_WRONG_MSG_SIZE);
						break;
					}
					//if (ret > 0 && onReceivedMsg)onReceivedMsg(socket_, recv_msg);
					break;
				}
				case Type::UDP_RAW: {
					char data[1024];
					int ret = recvfrom(recv_sock, data, 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size_);
					std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
					if (ret <= 0 && !close_lck.try_lock()) return -1;
					//if (ret > 0 && onReceivedData)onReceivedData(socket_, data, ret);
					break;
				}
				}
			//}
			return 0;
		}
	};
	auto SocketMultiIo::Imp::acceptThread(SocketMultiIo::Imp* imp, std::promise<void> accept_thread_ready)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		// 改变状态 //
		imp->state_ = State::WORKING;

		// 通知主线程,accept线程已经拷贝完毕,准备监听 //
		accept_thread_ready.set_value();

		auto nfds = imp->lisn_socket_ + 1;
		
		imp->recv_data_.clear();
		imp->recv_sockets_.clear();

		::fd_set f_s;
		for (;;) {
			FD_ZERO(&f_s);
			FD_SET(imp->lisn_socket_, &f_s);
			for (auto& fd : imp->recv_sockets_) {
				FD_SET(fd, &f_s);
			}
			nfds = imp->recv_sockets_.size() > 0 ? std::max(*std::max_element(imp->recv_sockets_.begin(), imp->recv_sockets_.end()), imp->lisn_socket_)+1 : imp->lisn_socket_+1;

			auto select_ret = ::select(nfds, &f_s, nullptr, nullptr, nullptr);

			if (select_ret > 0) {
				if (FD_ISSET(imp->lisn_socket_, &f_s) > 0) {
					SOCKET_T recv_sock = static_cast<SOCKET_T>(::accept(imp->lisn_socket_, (struct sockaddr*)(&imp->client_addr_), &imp->sin_size_));

					if (recv_sock == -1) {
						ARIS_LOG(SOCKET_FAILED_ACCEPT, (int)recv_sock);

						std::unique_lock<std::recursive_mutex> close_lck(imp->close_mutex_, std::defer_lock);
						if (!close_lck.try_lock()) {
							aris_close(imp->lisn_socket_);
							imp->state_ = State::IDLE;
							imp->accept_thread_.detach();
							return;
						}
						else {
							aris_close(imp->lisn_socket_);
							std::this_thread::sleep_for(std::chrono::seconds(1));
							continue;
						}

					}

					imp->recv_sockets_.push_back(recv_sock);
					imp->recv_data_.insert(std::pair<SOCKET_T, SocketRecvData>(recv_sock, SocketRecvData()));
					

					if (imp->type_ == Type::WEB || imp->type_ == Type::WEB_RAW) {
						std::this_thread::sleep_for(std::chrono::seconds(3));

						char recv_data[1024]{ 0 };
						int res = recv(recv_sock, recv_data, 1024, 0);
						if (res <= 0) {
							ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED, res);
							shutdown(recv_sock, 2);
							aris_close(recv_sock);
							continue;
						}

						auto header_map = make_header_map2(recv_data);
						std::string server_key;
						try {
							server_key = header_map.at("Sec-WebSocket-Key");
						}
						catch (std::exception&) {
							ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_INVALID_KEY);
							shutdown(recv_sock, 2);
							aris_close(recv_sock);
							continue;
						}
						server_key += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

						// 找到返回的key //
						SHA1 checksum;
						checksum.update(server_key);
						std::string hash = checksum.final();

						std::uint32_t message_digest[5]{};
						for (Size i = 0; i < 20; ++i) {
							char num[5] = "0x00";
							std::copy_n(hash.data() + i * 2, 2, num + 2);
							std::uint8_t n = std::stoi(num, 0, 16);
							*(reinterpret_cast<unsigned char*>(message_digest) + i) = n;
						}

						auto ret_hey = base64_encode2_2(reinterpret_cast<const unsigned char*>(message_digest), 20);

						std::string shake_hand;
						shake_hand = "HTTP/1.1 101 Switching Protocols\r\n"
							"Upgrade: websocket\r\n"
							"Connection: Upgrade\r\n"
							"Sec-WebSocket-Accept: " + ret_hey + std::string("\r\n\r\n");

						auto ret = aris_send(recv_sock, shake_hand.c_str(), static_cast<int>(shake_hand.size()), 0);

						if (ret == -1) {
							ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_LOOSE_CONNECTION);
							shutdown(recv_sock, 2);
							aris_close(recv_sock);
							continue;
						};
					}

					// 设置为 blocking 模式 //
#ifdef WIN32
					u_long block = 0;
					if (ioctlsocket(recv_sock, FIONBIO, &block) == SOCKET_ERROR) {
						imp->lose_tcp2(recv_sock);
					}
#endif
#ifdef UNIX
					long arg;
					if ((arg = fcntl(recv_sock, F_GETFL, NULL)) < 0) {
						imp->lose_tcp2(recv_sock);
					}
					arg &= (~O_NONBLOCK);
					if (fcntl(recv_sock, F_SETFL, arg) < 0) {
						imp->lose_tcp2(recv_sock);
					}
#endif

					// CALL BACK //
					if (imp->onReceivedConnection) {
						imp->onReceivedConnection(imp->socket_, inet_ntoa(imp->client_addr_.sin_addr), ntohs(imp->client_addr_.sin_port));
					}
				}

				for (auto recv_iter = imp->recv_sockets_.begin(); recv_iter != imp->recv_sockets_.end();) {
					if (FD_ISSET(*recv_iter, &f_s)) {
						if (imp->recv_from_sock2(*recv_iter, imp->recv_data_[*recv_iter]) < 0) {
							recv_iter = imp->recv_sockets_.erase(recv_iter);
							continue;
						}
					}

					recv_iter++;
				}
			}

		}
		
		return;
	}
	auto SocketMultiIo::Imp::receiveThread(SocketMultiIo::Imp* imp, std::promise<void> receive_thread_ready)->void{
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
		imp->state_ = SocketMultiIo::State::WORKING;

		// 通知accept或connect线程已经准备好,下一步开始收发数据 //
		receive_thread_ready.set_value();

		SocketRecvData data;
		imp->recv_from_sock(imp->recv_socket_, data);
	}
	auto SocketMultiIo::stop()->void{
		std::lock(imp_->state_mutex_, imp_->close_mutex_);
		std::unique_lock<std::recursive_mutex> lck1(imp_->state_mutex_, std::adopt_lock);
		std::unique_lock<std::recursive_mutex> lck2(imp_->close_mutex_, std::adopt_lock);

		switch (imp_->state_){
		case State::IDLE:
			break;
		case State::WAITING_FOR_CONNECTION:
		case State::WORKING:
			shutdown(imp_->lisn_socket_, 2);
			//aris_close(imp_->lisn_socket_);

			for (auto& recv_sock : imp_->recv_sockets_) {
				switch (connectType()) {
				case Type::TCP:
				case Type::TCP_RAW:
				case Type::WEB:
				case Type::WEB_RAW:
					if (shutdown(recv_sock, 2) < 0)
						ARIS_LOG(SOCKET_SHUT_DOWN_ERROR, errno);
					break;
				case Type::UDP:
				case Type::UDP_RAW:
					if (aris_close(recv_sock) < 0) 
						ARIS_LOG(SOCKET_SHUT_CLOSE_ERROR, errno);
					break;
				}
			}

			while (imp_->accept_thread_.joinable())
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			break;
		}

		imp_->state_ = SocketMultiIo::State::IDLE;
	}
	auto SocketMultiIo::startServer(const std::string &port)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		if (!port.empty())setPort(port);
		if (this->port().empty())THROW_FILE_LINE("SocketMultiIo can't Start as server, because it has empty port\n");

		switch (imp_->state_){
		case State::IDLE:
			break;
		default:
			THROW_FILE_LINE("SocketMultiIo can't Start as server, because it is not at idle state\n");
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
		if (static_cast<int>(imp_->lisn_socket_ = static_cast<SOCKET_T>(socket(AF_INET, sock_type, 0))) == -1)THROW_FILE_LINE("SocketMultiIo can't Start as server, because it can't socket\n");

		// linux 下设置keep alive
#ifdef UNIX
		if (sock_type == SOCK_STREAM){
			int tcp_timeout = 10000; //10 seconds before aborting a write()
			if (setsockopt(imp_->lisn_socket_, SOL_TCP, TCP_USER_TIMEOUT, &tcp_timeout, sizeof(int)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_USER_TIMEOUT FAILED");
			}

			// Set the option active //
			int keepAlive = 1; // 开启keepalive属性
			int keepIdle = 5; // 如该连接在5秒内没有任何数据往来,则进行探测 
			int keepInterval = 1; // 探测时发包的时间间隔为5 秒
			int keepCount = 5; // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发.

			if (setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepAlive, sizeof(keepAlive)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt SO_KEEPALIVE FAILED");
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPIDLE, (void*)&keepIdle, sizeof(keepIdle)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_KEEPIDLE FAILED");
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPINTVL, (void*)&keepInterval, sizeof(keepInterval)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_KEEPINTVL FAILED");
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPCNT, (void*)&keepCount, sizeof(keepCount)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("socket setsockopt TCP_KEEPCNT FAILED");
			}
		}
#endif


		// 设置socketopt选项,使得地址在程序结束后立即可用 //
		int nvalue = 1;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&nvalue), sizeof(int)) < 0) {
			aris_close(imp_->lisn_socket_);
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
			aris_close(imp_->lisn_socket_);
			THROW_FILE_LINE("SocketMultiIo can't Start as server, because it can't bind\n");
		}
		
		if (connectType() == Type::TCP || connectType() == Type::TCP_RAW || connectType() == Type::WEB || connectType() == Type::WEB_RAW){
			// 监听lisn_socket_描述符 //
			if (listen(imp_->lisn_socket_, 5) == -1) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("SocketMultiIo can't Start as server, because it can't listen\n");
			}

			// 启动等待连接的线程 //
			std::promise<void> accept_thread_ready;
			auto ready = accept_thread_ready.get_future();
			imp_->accept_thread_ = std::thread(SocketMultiIo::Imp::acceptThread, this->imp_.get(), std::move(accept_thread_ready));
			ready.wait();
		}
		else{
			// 因为UDP没法shutdown，所以用非阻塞模式 //
#ifdef WIN32
			DWORD read_timeout = 10;
			if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("setsockopt failed: SO_RCVTIMEO \n");
			}
#endif
#ifdef UNIX
			struct timeval read_timeout;
			read_timeout.tv_sec = 0;
			read_timeout.tv_usec = 10000;
			if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0) {
				aris_close(imp_->lisn_socket_);
				THROW_FILE_LINE("setsockopt failed: SO_RCVTIMEO \n");
			}
#endif
			imp_->recv_socket_ = imp_->lisn_socket_;
			
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(SocketMultiIo::Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();
		}

		return;
	}
	auto SocketMultiIo::connect(const std::string& remote_ip, const std::string& port)->void {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		// check ip & port //
		if (!remote_ip.empty())setRemoteIP(remote_ip);
		if (!port.empty())setPort(port);
		if (remoteIP().empty())THROW_FILE_LINE("SocketMultiIo can't connect, because it empty ip address\n");
		if (this->port().empty())THROW_FILE_LINE("SocketMultiIo can't connect, because it empty port\n");

		// check state //
		switch (imp_->state_) {
		case State::IDLE:
			break;
		default:
			THROW_FILE_LINE("SocketMultiIo can't connect, because it is busy now, please close it\n");
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
		if ((imp_->recv_socket_ = static_cast<SOCKET_T>(socket(AF_INET, sock_type, 0))) < 0)
			THROW_FILE_LINE("SocketMultiIo can't connect, because can't socket\n");

		// 设置 time_out //
		if (imp_->connect_time_out_ >= 0) {
#ifdef WIN32
			u_long block = 1;
			if (ioctlsocket(imp_->recv_socket_, FIONBIO, &block) == SOCKET_ERROR) {
#endif
#ifdef UNIX
			if (fcntl(imp_->recv_socket_, F_SETFL, O_NONBLOCK) < 0) {
#endif
				aris_close(imp_->recv_socket_);
				THROW_FILE_LINE("SocketMultiIo can't connect, because can't set time out\n");
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
				time_out.tv_sec = (decltype(time_out.tv_sec))imp_->connect_time_out_ / 1000;
				time_out.tv_usec = (imp_->connect_time_out_ % 1000) * 1000;

#ifdef WIN32
				if (WSAGetLastError() == WSAEWOULDBLOCK) {
					// connection pending
					int ret = select(0, NULL, &setW, &setE, &time_out);
					if (ret < 0){
						aris_close(imp_->recv_socket_);
						THROW_FILE_LINE("SocketMultiIo can't connect, because failed to select\n");
					}
					else if (ret == 0) {
						aris_close(imp_->recv_socket_);
						WSASetLastError(WSAETIMEDOUT);
						THROW_FILE_LINE("SocketMultiIo can't connect, because time out\n");
					}
					else {
					
					}

					if (FD_ISSET(imp_->recv_socket_, &setE)){
						// connection failed
						int err = 0, err_size = sizeof(int);
						getsockopt(imp_->recv_socket_, SOL_SOCKET, SO_ERROR, (char*) &err, &err_size);
						aris_close(imp_->recv_socket_);
						WSASetLastError(err);
						THROW_FILE_LINE("SocketMultiIo can't connect, because failed to FD_ISSET\n");
					}
				}
				else {
					aris_close(imp_->recv_socket_);
					THROW_FILE_LINE("SocketMultiIo can't connect, because can't connect\n");
				}
#endif
#ifdef UNIX
				do {
					auto ret = select(imp_->recv_socket_ + 1, NULL, &setW, &setE, &time_out);
					if (ret > 0) {
						int so_error;
						socklen_t len = sizeof(so_error);
						if (getsockopt(imp_->recv_socket_, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) {
							aris_close(imp_->recv_socket_);
							THROW_FILE_LINE("SocketMultiIo can't connect, because can't getsockopt\n");
						}
						if (so_error) {
							aris_close(imp_->recv_socket_);
							if (time_out.tv_sec == 0 && time_out.tv_usec == 0)
								THROW_FILE_LINE("SocketMultiIo can't connect, because getsockopt error\n");
							else
								continue;
						}
						else
							break;// 正常结束
					}
					else if (ret == 0) {
						aris_close(imp_->recv_socket_);
						THROW_FILE_LINE("SocketMultiIo can't connect, because time out\n");
					}
					else {
						aris_close(imp_->recv_socket_);
						THROW_FILE_LINE("SocketMultiIo can't connect, because failed to select\n");
					}
				} while (1);
#endif	
			}

			imp_->state_ = SocketMultiIo::State::WORKING;

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
				aris_close(imp_->recv_socket_);
				THROW_FILE_LINE("SocketMultiIo can't connect, because can't connect\n");
			}

			char handshake_text[]{
				"GET / HTTP/1.1\r\n"
				"Host: localhost:8080\r\n"
				"Origin: http://127.0.0.1:3000\r\n"
				"Connection: Upgrade\r\n"
				"Upgrade: websocket\r\n"
				"Sec-WebSocket-Version: 13\r\n"
				"Sec-WebSocket-Key: w4v7O6xFTi36lq3RNcgctw==\r\n\r\n" };

			if(aris_send(imp_->recv_socket_, handshake_text, static_cast<int>(std::strlen(handshake_text)), 0) == -1)
				THROW_FILE_LINE("SocketMultiIo can't connect, web sock error 1\n");

			char recv_data[1024]{ 0 };
			int res = recv(imp_->recv_socket_, recv_data, 1024, 0);
			if(res <= 0)THROW_FILE_LINE("SocketMultiIo can't connect, web sock error 2\n");

			auto header_map = make_header_map2(recv_data);

			////////////   这里应该check更多，tbd //
			if(header_map.find("Sec-WebSocket-Accept") == header_map.end())THROW_FILE_LINE("SocketMultiIo can't connect, web sock error 3\n");

			imp_->state_ = SocketMultiIo::State::WORKING;

			// Start Thread //
			std::promise<void> receive_thread_ready;
			auto fut = receive_thread_ready.get_future();
			imp_->recv_thread_ = std::thread(Imp::receiveThread, imp_.get(), std::move(receive_thread_ready));
			fut.wait();

			return;
		}
		case Type::UDP:
		case Type::UDP_RAW:
			imp_->state_ = SocketMultiIo::State::WORKING;
			return;
		}
	}
	auto SocketMultiIo::sendMultiIoMsg(const aris::core::MsgBase &data, SOCKET_T recv_socket)->void{
		#ifdef UNIX
			signal(SIGPIPE, SIG_IGN);
		#endif
		
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:{
			switch (imp_->type_){
			case Type::TCP:
				if (aris_send(recv_socket, reinterpret_cast<const char *>(&data.header()), data.size() + sizeof(MsgHeader), 0) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return;
				break;
			case Type::WEB:{
				auto packed_data = imp_->is_server_ 
					? pack_data_server2(reinterpret_cast<const char*>(&data.header()), data.size() + sizeof(aris::core::MsgHeader))
					: pack_data_client2(reinterpret_cast<const char*>(&data.header()), data.size() + sizeof(aris::core::MsgHeader));
				if (aris_send(imp_->recv_socket_, packed_data.data(), static_cast<int>(packed_data.size()), 0) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
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
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return;

				break;
			}
			default:
				THROW_FILE_LINE("SocketMultiIo failed send msg, because SocketMultiIo is not at right MODE\n");
			}
		}
		default:
			THROW_FILE_LINE("SocketMultiIo failed sending data, because SocketMultiIo is not at right STATE\n");
		}
	}
	
	auto SocketMultiIo::sendMsg(const aris::core::MsgBase &data)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif

		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:{
			switch (imp_->type_){
			case Type::TCP:
				if (aris_send(imp_->recv_socket_, reinterpret_cast<const char *>(&data.header()), data.size() + sizeof(MsgHeader), 0) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return;
				break;
			case Type::WEB:{
				auto packed_data = imp_->is_server_ 
					? pack_data_server2(reinterpret_cast<const char*>(&data.header()), data.size() + sizeof(aris::core::MsgHeader))
					: pack_data_client2(reinterpret_cast<const char*>(&data.header()), data.size() + sizeof(aris::core::MsgHeader));
				if (aris_send(imp_->recv_socket_, packed_data.data(), static_cast<int>(packed_data.size()), 0) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
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
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return;

				break;
			}
			default:
				THROW_FILE_LINE("SocketMultiIo failed send msg, because SocketMultiIo is not at right MODE\n");
			}
		}
		default:
			THROW_FILE_LINE("SocketMultiIo failed sending data, because SocketMultiIo is not at right STATE\n");
		}
	}
	auto SocketMultiIo::sendRawData(const char *data, int size)->void{
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:{
			switch (imp_->type_){
			case Type::TCP_RAW: {
				if (aris_send(imp_->recv_socket_, data, size, 0) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return;
				break;
			}
			case Type::WEB_RAW:{
				auto packed_data = imp_->is_server_ ? pack_data_server2(data, size) : pack_data_client2(data, size);
				if (aris_send(imp_->recv_socket_, packed_data.data(), static_cast<int>(packed_data.size()), 0) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
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
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return;

				break;
			}
			default:
				THROW_FILE_LINE("SocketMultiIo failed send raw data, because SocketMultiIo is not at right MODE\n");
			}
		}
		default:
			THROW_FILE_LINE("SocketMultiIo failed send raw data, because SocketMultiIo is not at right STATE\n");
		}
	}
	auto SocketMultiIo::isConnected()->bool{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_){
		case State::WORKING:
			return true;
		default:
			return false;
		}
	}
	auto SocketMultiIo::state()->State{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->state_;
	};
	
	auto SocketMultiIo::port()const->const std::string&{
		//std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->port_;
	}
	auto SocketMultiIo::setPort(const std::string &port)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->port_ = port;
	}
	auto SocketMultiIo::remoteIP()const->const std::string &{
		//std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->remote_ip_;
	}
	auto SocketMultiIo::setRemoteIP(const std::string &remote_ip)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->remote_ip_ = remote_ip;
	}
	auto SocketMultiIo::connectType()const->Type { return imp_->type_; }
	auto SocketMultiIo::connectTimeoutMs()const->std::int64_t {
		return imp_->connect_time_out_;
	}
	auto SocketMultiIo::setConnectTimeoutMs(std::int64_t time_out_ms)->void {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->connect_time_out_ = time_out_ms;
	}

	auto SocketMultiIo::setConnectType(const Type type)->void { imp_->type_ = type; }
	auto SocketMultiIo::setOnReceivedMsg(std::function<int(SOCKET_T, aris::core::Msg&)> OnReceivedData)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedMsg = OnReceivedData;
	}
	auto SocketMultiIo::setOnReceivedRawData(std::function<int(SocketMultiIo*, const char *data, int size)> func)->void{ 
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedData = func;
	}
	auto SocketMultiIo::setOnReceivedConnection(std::function<int(SocketMultiIo*, const char*, int)> OnReceivedConnection)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onReceivedConnection = OnReceivedConnection;
	}
	auto SocketMultiIo::setOnLoseConnection(std::function<int(SocketMultiIo*)> OnLoseConnection)->void{
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		imp_->onLoseConnection = OnLoseConnection;
	}
	SocketMultiIo::~SocketMultiIo(){ 
		if (imp_){
			stop();
#ifdef WIN32 
			WSACleanup();
#endif
		}
	}
	SocketMultiIo::SocketMultiIo(const std::string &name, const std::string& remote_ip, const std::string& port, Type type) :imp_(new Imp(this)){
		// 启动服务器 //
#ifdef WIN32 
		if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)THROW_FILE_LINE("SocketMultiIo can't Start as server, because it can't WSAstartup\n");
#endif
		setRemoteIP(remote_ip);
		setPort(port);
		setConnectType(type);
	}
	SocketMultiIo::SocketMultiIo(SocketMultiIo&&s)noexcept{
		imp_ = std::move(s.imp_);
		imp_->socket_ = this;
	};
	SocketMultiIo& SocketMultiIo::operator=(SocketMultiIo&&s)noexcept {
		imp_ = std::move(s.imp_);
		imp_->socket_ = this;
		return *this;
	}


	
	struct SockData {
		SOCKET_T sock_{ 0 };

		// recv data // 
		int required_length_{ 0 };
		int received_length_{ 0 };
		std::vector<char> mem_;
		bool is_first{ true };
		aris::core::MsgSize max_msg_length_{0x10000};

		// pack data，多个数据帧（frame）拼成完整的数据后，触发回调 //
		int datapack_received_length_{ 0 };
		std::vector<char> datapack_mem_;

		// remote & local info //
		struct sockaddr_in remote_addr_ {}, local_addr_{};

		// 适用于所有情况 //
		auto safe_recv() -> int {
			mem_.resize(std::max(required_length_, 1024));
			auto ret = ::recv(sock_, mem_.data() + received_length_, std::max(required_length_ - received_length_, 1024 - received_length_), 0);
			if (ret >= 0) received_length_ += ret;
			return ret;
		}

		//
		// 适用于 WEB 和 WEB_RAW
		// 将websocket数据帧合并成用户数据包，合并后的数据如果是 WEB，那就是MSG，如果是WEB_RAW,那就是二进制
		// 用户数据储存在 datapack_mem_ 中
		// 
		// ret:
		// -1 : 数据错误
		//  0 : 获取数据帧成功，转成用户数据包成功（websocket 多个帧合成一个数据包）
		//  1 : 获取数据帧成功，但并未转成用户数据包
		//  2 : 获取数据帧不成功
		auto unpackWebData() -> int {
			bool fin{false};
			char web_head[2]{};
			std::int64_t payload_len{ 0 };
			
			// 接受头 //
			int frame_length = 0;
			required_length_ = frame_length + 2;
			if (received_length_ >= required_length_) {
				std::copy_n(mem_.data(), 2, web_head);

				// 是否最后一帧 //
				fin = (web_head[0] & 0x80) == 0x80; // 1bit，1表示最后一帧    

				// 获取opcode //
				std::int8_t op_code = web_head[0] & 0x0f;
				if (op_code == 0x08)
					return -1;

				// 获取数据长度
				payload_len = web_head[1] & 0x7F; // 数据长度

				frame_length += 2;
			}

			required_length_ = frame_length + 2;
			if (payload_len == 126 && received_length_ > required_length_) {
				// 似乎有大小端的问题 //
				char reverse_char[2];
				for (int i = 0; i < 2; ++i)reverse_char[i] = mem_[3 - i];
				payload_len = *reinterpret_cast<std::uint16_t*>(reverse_char);

				frame_length += 2;
			}

			required_length_ = frame_length + 8;
			if (payload_len == 127 && received_length_ > required_length_) {
				char reverse_char[8];
				for (int i = 0; i < 8; ++i)reverse_char[i] = mem_[9 - i];
				payload_len = *reinterpret_cast<std::int64_t*>(reverse_char);

				frame_length += 8;
			}

			// 保护，数据不能太大 //
			if (payload_len < 0 || payload_len > max_msg_length_ || payload_len + datapack_received_length_ > max_msg_length_) {
				ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
				return -1;
			}

			// 获取掩码
			bool mask_flag = (web_head[1] & 0x80) == 0x80; // 是否包含掩码    
			char masks[4];
			required_length_ = frame_length + 4;
			if (mask_flag && received_length_ > required_length_) {
				std::copy_n(mem_.data() + frame_length, 4, masks);
				frame_length += 4;
			}

			// 本帧没有收取完 //
			required_length_ = frame_length + payload_len;
			if (received_length_ < required_length_)
				return 2;

			// 读取数据 
			datapack_mem_.resize(payload_len + datapack_received_length_);
			if (mask_flag) {
				for (int i{ 0 }; i < payload_len; ++i) {
					datapack_mem_[i + datapack_received_length_] = mem_[i + frame_length] ^ masks[i % 4];
				}
			}
			else {
				std::copy_n(mem_.data(), payload_len, datapack_mem_.data() + datapack_received_length_);
			}

			datapack_received_length_ += payload_len;
			frame_length += payload_len;

			// 本帧已经收取完毕，准备下一帧 //
			std::copy_n(mem_.data() + required_length_, received_length_ - required_length_, mem_.data());
			received_length_ -= required_length_;
			required_length_ = 2;

			// 数据包由多个数据帧组成
			// 数据包完全收好返回0
			// 数据帧收好，但还组合不成数据包，返回1
			return fin ? 0 : 1;
		}
		auto initWebConnection() -> int {
			required_length_ = 1024;
			received_length_ = 0;

			int ret = 0;
			if ((ret = safe_recv()) <= 0) {
				ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED, ret);
				return ret;
			}

			auto recv_str = std::string_view(mem_.data(), received_length_);

			// 首次握手数据不应该太长 //
			if (received_length_ > 1023) {
				ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED, -1);
				return ret;
			}

			if (recv_str.find("\r\n\r\n") == std::string::npos)
				return ret;

			auto header_map = make_header_map2(std::string(recv_str));
			std::string server_key;
			try {
				server_key = header_map.at("Sec-WebSocket-Key");
			}
			catch (std::exception&) {
				ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_INVALID_KEY);
				return -1;
			}
			server_key += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

			// 找到返回的key //
			SHA1 checksum;
			checksum.update(server_key);
			std::string hash = checksum.final();

			std::uint32_t message_digest[5]{};
			for (Size i = 0; i < 20; ++i) {
				char num[5] = "0x00";
				std::copy_n(hash.data() + i * 2, 2, num + 2);
				std::uint8_t n = std::stoi(num, 0, 16);
				*(reinterpret_cast<unsigned char*>(message_digest) + i) = n;
			}

			auto ret_hey = base64_encode2_2(reinterpret_cast<const unsigned char*>(message_digest), 20);

			std::string shake_hand;
			shake_hand = "HTTP/1.1 101 Switching Protocols\r\n"
				"Upgrade: websocket\r\n"
				"Connection: Upgrade\r\n"
				"Sec-WebSocket-Accept: " + ret_hey + std::string("\r\n\r\n");

			auto send_ret = aris_send(sock_, shake_hand.c_str(), static_cast<int>(shake_hand.size()), 0);

			if (send_ret == -1) {
				ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_LOOSE_CONNECTION);
				return send_ret;
			};

			required_length_ = 2;
			received_length_ = 0;
			is_first = false;

			return ret;
		}

		// RAII to close sock
		~SockData() {
			if (sock_) {
				if (shutdown(sock_, 2) < 0)
					ARIS_LOG(SOCKET_SHUT_CLOSE_ERROR, errno);
				aris_close(sock_);
			}
		}
	};
	struct SocketServer::Imp {
		SocketServer::Type type_{ Type::TCP };
		std::string port_;
		aris::core::MsgSize max_msg_length_{ 0x1000000 };

		SocketServer* socket_server_;
		SocketServer::State state_{ State::IDLE };
		
		SOCKET_T lisn_socket_;
		std::map<SOCKET_T, SockData> sock_datas_;

		// callbacks //
		ReceiveMsgCallback on_receive_msg_;
		ReceiveRawDataCallback on_receive_raw_data_;
		ReceiveConnectionCallback on_receive_connection_;
		LoseConnectionCallback on_lose_connection_;

		// for udp ... //
		struct sockaddr_in client_addr_{};
		socklen_t sin_size_;

		// 线程同步变量 //
		std::recursive_mutex state_mutex_;
		std::thread accept_thread_;

		// 连接的socket //
#ifdef WIN32
		WSADATA wsa_data_;             //windows下才用,linux下无该项
#endif
		~Imp() = default;
		Imp(SocketServer* sock) :socket_server_(sock), lisn_socket_(0), sin_size_(sizeof(struct sockaddr_in)), state_(SocketServer::State::IDLE)
			, on_receive_msg_(nullptr), on_receive_raw_data_(nullptr), on_receive_connection_(nullptr), on_lose_connection_(nullptr) {}

		static void acceptThread(SocketServer::Imp* imp, std::promise<void> accept_thread_ready);

		auto is_tcp() -> bool {
			return type_ == Type::TCP || type_ == Type::TCP_RAW || type_ == Type::WEB || type_ == Type::WEB_RAW;
		}

		auto init_all_socks() -> void {
			lisn_socket_ = 0;
			sock_datas_.clear();
		}
		auto close_all_socks() -> void {
			sock_datas_.clear();

			if (shutdown(lisn_socket_, 2) < 0)
				ARIS_LOG(SOCKET_SHUT_CLOSE_ERROR, errno);
			aris_close(lisn_socket_);
		}
		auto recv_from_sock(SockData& recv_data) -> int {
			// 开启接受数据的循环 //
			switch (type_) {
			case Type::TCP: {
				if (recv_data.is_first) {
					recv_data.required_length_ = 40;
					recv_data.is_first = false;
				}

				int ret = 0;
				if ((ret = recv_data.safe_recv()) <= 0)
					return ret;

				while (recv_data.received_length_ >= recv_data.required_length_) {
					if (recv_data.received_length_ >= sizeof(MsgHeader)) {
						auto msg_size = reinterpret_cast<MsgHeader*>(recv_data.mem_.data())->msg_size_;
						if (msg_size > max_msg_length_ || msg_size < 0)
							return -1;
						recv_data.required_length_ = msg_size + sizeof(MsgHeader);
					}
					if (recv_data.received_length_ >= recv_data.required_length_) {
						aris::core::Msg recv_msg;
						recv_msg.resize(recv_data.required_length_ - sizeof(MsgHeader));
						std::copy(recv_data.mem_.data(), recv_data.mem_.data() + recv_msg.size() + sizeof(MsgHeader), (char*)(&recv_msg.header()));
						recv_msg.setRemoteSockaddrin(&recv_data.remote_addr_);

						if(on_receive_msg_)
							on_receive_msg_(socket_server_, recv_data.sock_, recv_msg);
						std::copy_n(recv_data.mem_.data() + recv_data.required_length_, recv_data.received_length_ - recv_data.required_length_, recv_data.mem_.data());
						recv_data.received_length_ -= recv_data.required_length_;
						recv_data.required_length_ = sizeof(MsgHeader);
					}
				}
				
				return ret;
			}
			case Type::TCP_RAW: {
				int ret = 0;
				if ((ret = recv_data.safe_recv()) <= 0)
					return ret;

				aris::core::Msg msg;
				msg.copy(recv_data.mem_.data(), recv_data.received_length_);
				msg.setRemoteSockaddrin(&recv_data.remote_addr_);
				//msg.setDestinationSockaddrin(&recv_data.local_addr_);

				// 触发回调 //
				if (on_receive_msg_)
					on_receive_msg_(socket_server_, recv_data.sock_, msg);

				recv_data.received_length_ = 0;
				recv_data.required_length_ = 1024;
				
				return ret;
			}
			case Type::WEB: {
				if (recv_data.is_first) {
					return recv_data.initWebConnection();
				}
				else {
					int ret = 0;
					if ((ret = recv_data.safe_recv()) <= 0)
						return ret;

					int state;
					for (state = recv_data.unpackWebData(); state == 0; state = recv_data.unpackWebData()) {
						// 把web sock 的东西转成 msg //
						aris::core::Msg recv_msg;
						recv_msg.resize(static_cast<aris::core::MsgSize>(recv_data.datapack_received_length_ - sizeof(aris::core::MsgHeader)));
						std::copy_n(recv_data.datapack_mem_.data(), recv_data.datapack_received_length_, reinterpret_cast<char*>(&recv_msg.header()));
						recv_msg.setRemoteSockaddrin(&recv_data.remote_addr_);

						if (recv_msg.size() != recv_data.datapack_received_length_ - sizeof(aris::core::MsgHeader)) {
							ARIS_LOG(WEBSOCKET_RECEIVE_WRONG_MSG_SIZE, recv_msg.size(), recv_data.datapack_received_length_);
							return -1;
						}

						// 触发回调 //
						if (on_receive_msg_)
							on_receive_msg_(this->socket_server_, recv_data.sock_, recv_msg);

						// 数据清零 //
						recv_data.datapack_received_length_ = 0;
					}

					return state < 0 ? state : ret;
				}

				break;
			}
			case Type::WEB_RAW: {
				if (recv_data.is_first) {
					return recv_data.initWebConnection();
				}
				else {
					int ret = 0;
					if ((ret = recv_data.safe_recv()) <= 0)
						return ret;

					int state;
					for (state = recv_data.unpackWebData(); state == 0; state = recv_data.unpackWebData()) {
						aris::core::Msg msg;
						msg.copy(recv_data.datapack_mem_.data(), recv_data.datapack_received_length_);
						msg.setRemoteSockaddrin(&recv_data.remote_addr_);
						//msg.setDestinationSockaddrin(&recv_data.local_addr_);
						
						// 触发回调 //
						if (on_receive_msg_)
							on_receive_msg_(socket_server_, recv_data.sock_, msg);
						
						// 数据清零 //
						recv_data.datapack_received_length_ = 0;
					}

					return state < 0 ? state : ret;
				}

				break;
			}
			case Type::UDP: {
				socklen_t sin_size = sizeof(struct sockaddr_in);

				aris::core::Msg recv_msg;
				recv_msg.resize(1024);
				int ret = recvfrom(recv_data.sock_, reinterpret_cast<char*>(&recv_msg.header()), 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size);

				if (ret != sizeof(MsgHeader) + recv_msg.size()) {
					ARIS_LOG(SOCKET_UDP_WRONG_MSG_SIZE);
					return -1;
				}

				if (ret > 0 && on_receive_msg_) {
					recv_msg.setRemoteSockaddrin(&client_addr_);
					on_receive_msg_(socket_server_, recv_data.sock_, recv_msg);
					return ret;
				}
				return 0; // error
				break;
			}
			case Type::UDP_RAW: {
				socklen_t sin_size = sizeof(struct sockaddr_in);

				aris::core::Msg recv_msg;
				recv_msg.resize(1024);
				int ret = recvfrom(recv_data.sock_, recv_msg.data(), 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size);

				if (ret > 0 && on_receive_msg_) {
					recv_msg.setRemoteSockaddrin(&client_addr_);
					recv_msg.resize(ret);
					on_receive_msg_(socket_server_, recv_data.sock_, recv_msg);
					return ret;
				}
				return 0; // error
				break;

				//char data[1024];
				//int ret = recvfrom(recv_data.sock_, data, 1024, 0, (struct sockaddr*)(&client_addr_), &sin_size_);
				//std::unique_lock<std::recursive_mutex> close_lck(close_mutex_, std::defer_lock);
				//if (ret <= 0 && !close_lck.try_lock()) return -1;
				//if (ret > 0 && onReceivedData)onReceivedData(socket_, data, ret);
				break;
			}
			}

			return 0;
		}
	};
	auto SocketServer::Imp::acceptThread(SocketServer::Imp* imp, std::promise<void> accept_thread_ready)->void {
		// 通知主线程,accept线程已经拷贝完毕,准备监听 //
		accept_thread_ready.set_value();

#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif

		::fd_set f_s;
		for (;;) {
			// map is sorted, so use map's last element
			auto nfds = imp->sock_datas_.size() > 0 ? std::max(imp->sock_datas_.crbegin()->first, imp->lisn_socket_) + 1 : imp->lisn_socket_ + 1;

			FD_ZERO(&f_s);
			FD_SET(imp->lisn_socket_, &f_s);
			for (auto& fd : imp->sock_datas_) {
				FD_SET(fd.first, &f_s);
			}
			
			struct timeval tv;
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			auto select_ret = ::select(nfds, &f_s, nullptr, nullptr, &tv);

			// lock data //
			std::unique_lock<std::recursive_mutex> lck(imp->state_mutex_);

			// close all sock //
			if (imp->state_ == State::IDLE) {
				imp->close_all_socks();
				return;
			}
			
			if (select_ret > 0) {
				if (FD_ISSET(imp->lisn_socket_, &f_s) > 0 && imp->is_tcp()) {
					struct sockaddr_in remote_addr;
					socklen_t sin_size = sizeof(struct sockaddr_in);

					auto recv_sock = static_cast<SOCKET_T>(::accept(imp->lisn_socket_, (struct sockaddr*)(&remote_addr), &sin_size));
					if (recv_sock == -1) {
						ARIS_LOG(SOCKET_FAILED_ACCEPT, (int)recv_sock);
						continue;
					}

					imp->sock_datas_.insert(std::pair<SOCKET_T, SockData>(recv_sock, SockData()));
					imp->sock_datas_[recv_sock].sock_ = recv_sock;
					imp->sock_datas_[recv_sock].remote_addr_ = remote_addr;
					imp->sock_datas_[recv_sock].max_msg_length_ = imp->max_msg_length_;

					if (::getsockname(recv_sock, (struct sockaddr*)&imp->sock_datas_[recv_sock].local_addr_, &sin_size)) {
						imp->sock_datas_.erase(recv_sock);
						continue;
					}
					
#ifdef WIN32
					u_long block = 0;
					if (::ioctlsocket(recv_sock, FIONBIO, &block) == SOCKET_ERROR) {
						imp->sock_datas_.erase(recv_sock);
						continue;
					}
#endif
#ifdef UNIX
					long arg;
					if ((arg = fcntl(recv_sock, F_GETFL, NULL)) < 0) {
						imp->sock_datas_.erase(recv_sock);
						continue;
					}
					arg &= (~O_NONBLOCK);
					if (fcntl(recv_sock, F_SETFL, arg) < 0) {
						imp->sock_datas_.erase(recv_sock);
						continue;
					}
#endif

					// CALL BACK //
					if (imp->on_receive_connection_) {
						imp->on_receive_connection_(imp->socket_server_, recv_sock, inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port));
					}
				}

				// 处理断连接 //
				for (auto recv_iter = imp->sock_datas_.begin(); recv_iter != imp->sock_datas_.end(); recv_iter++) {
					auto &recv_sock = recv_iter->first;
					auto &recv_sock_data = recv_iter->second;
					
					if (FD_ISSET(recv_sock, &f_s)) {
						if (imp->recv_from_sock(recv_sock_data) <= 0 && imp->is_tcp()) {
							imp->sock_datas_.erase(recv_sock);
							break;
						}
					}
				}
			}
		}

		return;
	}
	auto SocketServer::port()const->const std::string& {
		return imp_->port_;
	}
	auto SocketServer::setPort(const std::string& port)->void {
		imp_->port_ = port;
	}
	auto SocketServer::connectType()const->Type { return imp_->type_; }
	auto SocketServer::setConnectType(const Type type)->void { imp_->type_ = type; }
	auto SocketServer::maxMsgLength()const -> aris::core::MsgSize {
		return imp_->max_msg_length_;
	}
	auto SocketServer::setMaxMsgLength(aris::core::MsgSize length) -> void {
		imp_->max_msg_length_ = length;
	}

	auto SocketServer::setOnReceivedMsg(ReceiveMsgCallback on_receive_msg_func) -> void {
		imp_->on_receive_msg_ = on_receive_msg_func;
	}
	auto SocketServer::setOnReceivedRawData(ReceiveRawDataCallback on_receive_raw_data_func) -> void {
		imp_->on_receive_raw_data_ = on_receive_raw_data_func;
	}
	auto SocketServer::setOnReceivedConnection(ReceiveConnectionCallback on_receive_connection) -> void {
		imp_->on_receive_connection_ = on_receive_connection;
	}
	auto SocketServer::setOnLoseConnection(LoseConnectionCallback on_lose_connection_func) -> void {
		imp_->on_lose_connection_ = on_lose_connection_func;
	}

	auto SocketServer::state() -> State {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
		return imp_->state_;
	}
	auto SocketServer::startServer(const std::string& port) -> int {
		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		if (imp_->accept_thread_.joinable()) {
			ARIS_LOG(SOCKET_SERVER_START_ERROR, "already started");
			return -1;
		}

		switch (imp_->state_) {
		case State::IDLE:
			break;
		default:
			ARIS_LOG(SOCKET_SERVER_START_ERROR, "wrong state");
			return -2;
		}

		if (!port.empty())setPort(port);
		if (this->port().empty()) {
			ARIS_LOG(SOCKET_SERVER_START_ERROR, "wrong port");
			return -3;
		}

		imp_->init_all_socks();

		//////////////////////////////////////////////////////////////////////////////////////////////
		int sock_type = imp_->is_tcp() ? SOCK_STREAM : SOCK_DGRAM;
		///////////////////////////////////////////////////////////////////////////////////////////////

		// 服务器端开始建立socket描述符 //
		if (static_cast<int>(imp_->lisn_socket_ = static_cast<SOCKET_T>(socket(AF_INET, sock_type, 0))) == -1) {
			ARIS_LOG(SOCKET_SERVER_START_ERROR, "failed socket");
			return -4;
		}

		// linux 下设置keep alive
#ifdef UNIX
		if (sock_type == SOCK_STREAM) {
			int tcp_timeout = 10000; //10 seconds before aborting a write()
			if (setsockopt(imp_->lisn_socket_, SOL_TCP, TCP_USER_TIMEOUT, &tcp_timeout, sizeof(int)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_USER_TIMEOUT FAILED");
				return -5;
			}

			// Set the option active //
			int keepAlive = 1; // 开启keepalive属性
			int keepIdle = 5; // 如该连接在5秒内没有任何数据往来,则进行探测 
			int keepInterval = 1; // 探测时发包的时间间隔为5 秒
			int keepCount = 5; // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发.

			if (setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepAlive, sizeof(keepAlive)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_KEEPALIVE FAILED");
				return -6;
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPIDLE, (void*)&keepIdle, sizeof(keepIdle)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_KEEPIDLE FAILED");
				return -7;
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPINTVL, (void*)&keepInterval, sizeof(keepInterval)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_KEEPINTVL FAILED");
				return -8;
			}
			if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPCNT, (void*)&keepCount, sizeof(keepCount)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_KEEPCNT FAILED");
				return -9;
			}
		}
#endif

		// 设置socketopt选项,使得地址在程序结束后立即可用 //
		int nvalue = 1;
		if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&nvalue), sizeof(int)) < 0) {
			aris_close(imp_->lisn_socket_);
			ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_REUSEADDR FAILED");
			return -10;
		}

		// 服务器端填充server_addr_结构,并且bind //
		struct sockaddr_in server_addr;
		memset(&server_addr, 0, sizeof(struct sockaddr_in));
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		server_addr.sin_port = htons(std::stoi(imp_->port_));
		if (::bind(imp_->lisn_socket_, (struct sockaddr*)(&server_addr), sizeof(struct sockaddr)) == -1) {
#ifdef WIN32
			int err = WSAGetLastError();
#endif
			aris_close(imp_->lisn_socket_);
			ARIS_LOG(SOCKET_SERVER_START_ERROR, "bind FAILED");
			return -11;
		}

		if (imp_->is_tcp()) {
			// 监听lisn_socket_描述符 //
			if (::listen(imp_->lisn_socket_, 5) == -1) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "listen FAILED");
				return -12;
			}
		}
		else {
			// 因为UDP没法shutdown，所以用非阻塞模式 //
			imp_->sock_datas_.insert(std::pair<SOCKET_T, SockData>(imp_->lisn_socket_, SockData()));
			imp_->sock_datas_[imp_->lisn_socket_].sock_ = imp_->lisn_socket_;
#ifdef WIN32
			DWORD read_timeout = 10;
			if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_RCVTIMEO FAILED");
				return -13;
			}
#endif
#ifdef UNIX
			struct timeval read_timeout;
			read_timeout.tv_sec = 0;
			read_timeout.tv_usec = 10000;
			if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&read_timeout), sizeof(read_timeout)) < 0) {
				aris_close(imp_->lisn_socket_);
				ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_RCVTIMEO FAILED");
				return -13;
			}
#endif
		}

		// 改变状态 //
		imp_->state_ = State::WORKING;

		// 启动等待连接的线程 //
		std::promise<void> accept_thread_ready;
		auto ready = accept_thread_ready.get_future();

		imp_->accept_thread_ = std::thread(Imp::acceptThread, this->imp_.get(), std::move(accept_thread_ready));
		
		try {
			ready.get();
			return 0;
		}
		catch(...){
			imp_->accept_thread_.join();
			std::rethrow_exception(std::current_exception());
		}
	}
	auto SocketServer::stop() -> int {
		{
			std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
			imp_->state_ = State::IDLE;
		}

		if(imp_->accept_thread_.joinable())
			imp_->accept_thread_.join();

		return 0;
	}
	auto SocketServer::sendMsg(SOCKET_T sock, const aris::core::MsgBase& data) -> int {
#ifdef UNIX
		signal(SIGPIPE, SIG_IGN);
#endif

		std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

		switch (imp_->state_) {
		case State::WORKING: {
			switch (imp_->type_) {
			case Type::TCP: {
				aris::core::Msg send_msg = data;
				send_msg.setRemoteSockaddrin(&imp_->sock_datas_.at(sock).remote_addr_);
				//send_msg.setSourceSockaddrin(&imp_->sock_datas_.at(sock).local_addr_);
				auto ret = aris_send(sock, reinterpret_cast<const char*>(&send_msg.header()), send_msg.size() + sizeof(MsgHeader), 0);
				if (ret < 0)
					ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, ret);
				return ret;
			
			}
			case Type::WEB: {
				aris::core::Msg send_msg = data;
				send_msg.setRemoteSockaddrin(&imp_->sock_datas_.at(sock).remote_addr_);
				//send_msg.setSourceSockaddrin(&imp_->sock_datas_.at(sock).local_addr_);
				auto packed_data = pack_data_server2(reinterpret_cast<const char*>(&send_msg.header()), send_msg.size() + sizeof(aris::core::MsgHeader));
				auto ret = aris_send(sock, packed_data.data(), static_cast<int>(packed_data.size()), 0);
				if (ret < 0)
					ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, ret);
				return ret;
			}
			case Type::UDP: {
				aris::core::Msg send_msg = data;
				//send_msg.setSourceSockaddrin(&imp_->sock_datas_.at(sock).local_addr_);

				const struct sockaddr_in* addr = (send_msg.remoteIpStr() == "0.0.0.0") ? &imp_->client_addr_ : (const struct sockaddr_in*)send_msg.remoteSockaddrin();

				if (sendto(sock, reinterpret_cast<const char*>(&send_msg.header()), send_msg.size() + sizeof(MsgHeader), 0, (const struct sockaddr*)addr, sizeof(imp_->client_addr_)) == -1)
					THROW_FILE_LINE("SocketMultiIo failed sending data, because network failed\n");
				else
					return 0;

				break;
			}
			case Type::TCP_RAW: {
				if (auto ret = aris_send(sock, data.data(), data.size(), 0); ret < 0) {
					ARIS_LOG(SOCKET_SERVER_SEND_RAW_DATA_ERROR, ret);
					return ret;
				}
				break;
			}
			case Type::WEB_RAW: {
				auto packed_data = pack_data_client2(data.data(), data.size());
				if (auto ret = aris_send(sock, packed_data.data(), static_cast<int>(packed_data.size()), 0); ret < 0) {
					ARIS_LOG(SOCKET_SERVER_SEND_RAW_DATA_ERROR, ret);
					return ret;
				}
				break;
			}
			case Type::UDP_RAW: {
				const struct sockaddr_in* addr = (data.remoteIpStr() == "0.0.0.0") ? &imp_->client_addr_ : (const struct sockaddr_in*)data.remoteSockaddrin();

				if (sendto(sock, data.data(), data.size(), 0, (const struct sockaddr*)addr, sizeof(imp_->client_addr_)) == -1)
					THROW_FILE_LINE("SocketServer failed sending data, because network failed\n");
				else
					return 0;

				break;
			}
			default:
				ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, -1);
				return -1;
			}
		}
		default:
			ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, -2);
			return -1;
		}

		return 0;
	}

	SocketServer::~SocketServer() {
		if (imp_) {
			stop();
#ifdef WIN32 
			WSACleanup();
#endif
		}
	}
	SocketServer::SocketServer(const std::string& name, const std::string& port, Type type) :imp_(new Imp(this)) {
		// 启动服务器 //
#ifdef WIN32 
		if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)
			THROW_FILE_LINE("SocketServer can't Start as server, because it can't WSAstartup\n");
#endif
		setPort(port);
		setConnectType(type);
	}
	SocketServer::SocketServer(SocketServer&& s)noexcept {
		imp_ = std::move(s.imp_);
		imp_->socket_server_ = this;
	};
	SocketServer& SocketServer::operator=(SocketServer&& s)noexcept {
		imp_ = std::move(s.imp_);
		imp_->socket_server_ = this;
		return *this;
	}

	ARIS_REGISTRATION{
		aris::core::class_<SocketMultiIo::Type>("SocketMultiIo::connect_type")
			.textMethod([](SocketMultiIo::Type *v)->std::string{
				auto type = *reinterpret_cast<SocketMultiIo::Type*>(v);
				if (type == SocketMultiIo::Type::TCP)return "TCP";
				else if (type == SocketMultiIo::Type::TCP_RAW)return "TCP_RAW";
				else if (type == SocketMultiIo::Type::WEB)return "WEB";
				else if (type == SocketMultiIo::Type::WEB_RAW)return "WEB_RAW";
				else if (type == SocketMultiIo::Type::UDP)return "UDP";
				else if (type == SocketMultiIo::Type::UDP_RAW)return "UDP_RAW";
				else THROW_FILE_LINE("unknown connect type");
			},[](SocketMultiIo::Type *v,std::string_view str)->void{
				if (str == "TCP")*reinterpret_cast<SocketMultiIo::Type*>(v) = SocketMultiIo::Type::TCP;
				else if (str == "TCP_RAW")*reinterpret_cast<SocketMultiIo::Type*>(v) = SocketMultiIo::Type::TCP_RAW;
				else if (str == "WEB")*reinterpret_cast<SocketMultiIo::Type*>(v) = SocketMultiIo::Type::WEB;
				else if (str == "WEB_RAW")*reinterpret_cast<SocketMultiIo::Type*>(v) = SocketMultiIo::Type::WEB_RAW;
				else if (str == "UDP")*reinterpret_cast<SocketMultiIo::Type*>(v) = SocketMultiIo::Type::UDP;
				else if (str == "UDP_RAW")*reinterpret_cast<SocketMultiIo::Type*>(v) = SocketMultiIo::Type::UDP_RAW;
				else THROW_FILE_LINE("unknown connect type");
			});

		class_<SocketMultiIo>("SocketMultiIo")
			.prop("connect_type", &SocketMultiIo::setConnectType, &SocketMultiIo::connectType)
			.prop("remote_ip", &SocketMultiIo::setRemoteIP, &SocketMultiIo::remoteIP)
			.prop("port", &SocketMultiIo::setPort, &SocketMultiIo::port);

		aris::core::class_<SocketServer::Type>("SocketServer::connect_type")
			.textMethod([](SocketServer::Type* v)->std::string {
			auto type = *reinterpret_cast<SocketServer::Type*>(v);
			if (type == SocketServer::Type::TCP)return "TCP";
			else if (type == SocketServer::Type::TCP_RAW)return "TCP_RAW";
			else if (type == SocketServer::Type::WEB)return "WEB";
			else if (type == SocketServer::Type::WEB_RAW)return "WEB_RAW";
			else if (type == SocketServer::Type::UDP)return "UDP";
			else if (type == SocketServer::Type::UDP_RAW)return "UDP_RAW";
			else THROW_FILE_LINE("unknown connect type");
				}, [](SocketServer::Type* v, std::string_view str)->void {
					if (str == "TCP")*reinterpret_cast<SocketServer::Type*>(v) = SocketServer::Type::TCP;
					else if (str == "TCP_RAW")*reinterpret_cast<SocketServer::Type*>(v) = SocketServer::Type::TCP_RAW;
					else if (str == "WEB")*reinterpret_cast<SocketServer::Type*>(v) = SocketServer::Type::WEB;
					else if (str == "WEB_RAW")*reinterpret_cast<SocketServer::Type*>(v) = SocketServer::Type::WEB_RAW;
					else if (str == "UDP")*reinterpret_cast<SocketServer::Type*>(v) = SocketServer::Type::UDP;
					else if (str == "UDP_RAW")*reinterpret_cast<SocketServer::Type*>(v) = SocketServer::Type::UDP_RAW;
					else THROW_FILE_LINE("unknown connect type");
					});

		class_<SocketServer>("SocketServer")
			.prop("connect_type", &SocketServer::setConnectType, &SocketServer::connectType)
			.prop("port", &SocketServer::setPort, &SocketServer::port)
			.prop("max_msg_length", &SocketServer::setMaxMsgLength, &SocketServer::maxMsgLength);
	}
}