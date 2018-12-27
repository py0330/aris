#ifndef ARIS_CORE_SOCKET_H_
#define ARIS_CORE_SOCKET_H_

#include <functional>
#include <memory>
#include <stdexcept>

#include <aris_core_object.h>
#include <aris_core_msg.h>

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
		static auto Type()->const std::string & { static const std::string type("Socket"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
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

	private:
		struct Imp;
		const std::unique_ptr<Imp> imp_;
	};
}


#endif
