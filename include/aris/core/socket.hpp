#ifndef ARIS_CORE_SOCKET_H_
#define ARIS_CORE_SOCKET_H_

#include <aris/core/object.hpp>
#include <aris/core/msg.hpp>

namespace aris::core
{
	class Socket : public Object
	{
	public:
		enum State
		{
			IDLE = 0,
			WAITING_FOR_CONNECTION,
			WORKING,
		};
		enum TYPE
		{
			TCP,
			UDP,
			WEB,
			UDP_RAW,
			WEB_RAW
		};

	public:
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto isConnected()->bool;
		auto state()->State;
		auto startServer(const std::string &port = std::string())->void;
		auto connect(const std::string &remote_ip = std::string(), const std::string &port = std::string())->void;
		auto stop()->void;
		auto sendMsg(const aris::core::MsgBase &data)->void;
		auto sendRawData(const char *data, int size)->void;
		auto port()const->const std::string &;
		auto setPort(const std::string &port)->void;
		auto remoteIP()const->const std::string &;
		auto setRemoteIP(const std::string &remote_ip)->void;
		auto connectType()const->TYPE;
		auto setConnectType(const TYPE type)->void;
		auto setOnReceivedMsg(std::function<int(Socket*, aris::core::Msg &)> = nullptr)->void;
		auto setOnReceivedRawData(std::function<int(Socket*, const char *data, int size)> = nullptr)->void;
		auto setOnReceivedConnection(std::function<int(Socket*, const char* remote_ip, int remote_port)> = nullptr)->void;
		auto setOnLoseConnection(std::function<int(Socket*)> = nullptr)->void;

		virtual ~Socket();
		Socket(const std::string &name = "socket", const std::string& remote_ip = "", const std::string& port = "", TYPE type = TCP);
		Socket(const Socket & other) = delete;
		Socket(Socket && other) = default;
		Socket &operator=(const Socket& other) = delete;
		Socket &operator=(Socket&& other) = default;
		ARIS_REGISTER_TYPE(Socket)

	private:
		struct Imp;
		const std::unique_ptr<Imp> imp_;
	};
}


#endif
