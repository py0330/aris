#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/interface.hpp"
#include "aris/server/control_server.hpp"

#include "json.hpp"
namespace aris::server
{
	auto parse_ret_value(std::vector<std::pair<std::string, std::any>> &ret)->std::string
	{
		nlohmann::json js;
		
		for (auto &key_value : ret)
		{
#define ARIS_SET_TYPE(TYPE) if (auto value = std::any_cast<TYPE>(&key_value.second)) js[key_value.first] = *value; else

			ARIS_SET_TYPE(bool)
			ARIS_SET_TYPE(int)
			ARIS_SET_TYPE(double)
			ARIS_SET_TYPE(std::string)
			ARIS_SET_TYPE(std::vector<bool>)
			ARIS_SET_TYPE(std::vector<int>)
			ARIS_SET_TYPE(std::vector<double>)
			ARIS_SET_TYPE(std::vector<std::string>)
			{
				std::cout << "unrecognized return value" << std::endl;
			}

#undef ARIS_SET_TYPE
		}
		
		return  js.dump(2);
	}

	Interface::Interface(const std::string &name) :Object(name) {}
	
	auto onReceivedMsg(aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		LOG_INFO << "receive cmd:"
			<< msg.header().msg_size_ << "&"
			<< msg.header().msg_id_ << "&"
			<< msg.header().msg_type_ << "&"
			<< msg.header().reserved1_ << "&"
			<< msg.header().reserved2_ << "&"
			<< msg.header().reserved3_ << ":"
			<< msg_data << std::endl;

		try
		{
			aris::server::ControlServer::instance().executeCmd(aris::core::Msg(msg), [socket, msg](aris::plan::Plan &plan)->void
			{
				// make return msg
				aris::core::Msg ret_msg(msg);

				// only copy if it is a str
				if (auto str = std::any_cast<std::string>(&plan.ret()))
				{
					ret_msg.copy(*str);
				}
				else if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
				{
					js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
					js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
					ret_msg.copy(parse_ret_value(*js));
				}

				// return back to source
				try
				{
					socket->sendMsg(ret_msg);
				}
				catch (std::exception &e)
				{
					std::cout << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			});
		}
		catch (std::exception &e)
		{
			std::vector<std::pair<std::string, std::any>> ret_pair;
			ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
			ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
			std::string ret_str = parse_ret_value(ret_pair);

			std::cout << ret_str << std::endl;
			LOG_ERROR << ret_str << std::endl;

			try
			{
				aris::core::Msg m = msg;
				m.copy(ret_str);
				socket->sendMsg(m);
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}

		return 0;
	}
	auto onReceivedConnection(aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	}
	auto onLoseConnection(aris::core::Socket *socket)->int
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer(socket->port());
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	}
	auto WebInterface::open()->void { sock_->startServer(); }
	auto WebInterface::close()->void { sock_->stop(); }
	auto WebInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);

		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
	WebInterface::WebInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type):Interface(name)
	{
		sock_ = &add<aris::core::Socket>("socket", "", port, type);
		
		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
}