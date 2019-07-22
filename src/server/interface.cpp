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

namespace aris::server
{
	auto Interface::executeCmd(const aris::core::Msg &cmd_string)->std::shared_ptr<aris::plan::PlanTarget>
	{
		auto result = aris::server::ControlServer::instance().executeCmd(cmd_string);

		return result;
	}
	
	auto WebInterface::open()->void { sock_.startServer(); }
	auto WebInterface::close()->void { sock_.stop(); }
	auto WebInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		sock_.setPort(attributeString(xml_ele, "port", std::string()));

		Interface::loadXml(xml_ele);
	}
	auto WebInterface::saveXml(aris::core::XmlElement &xml_ele)const->void
	{
		Interface::saveXml(xml_ele);

		if (!sock_.port().empty())xml_ele.SetAttribute("port", sock_.port().c_str());
	}
	WebInterface::WebInterface(const std::string &port):Interface()
	{
		sock_.setPort(port);
		sock_.setOnReceivedMsg([this](aris::core::Socket *socket, aris::core::Msg &msg)->int
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
				auto result = aris::server::ControlServer::instance().executeCmd(aris::core::Msg(msg));


			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;

				try
				{
					aris::core::Msg m;
					m.setMsgID(msg.header().msg_id_);
					m.setType(msg.header().msg_type_);
					m.header().reserved1_ = msg.header().reserved1_;
					m.header().reserved2_ = msg.header().reserved2_;
					m.header().reserved3_ = msg.header().reserved3_;
					socket->sendMsg(m);
				}
				catch (std::exception &e)
				{
					std::cout << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			}

			return 0;
		});
		sock_.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
		{
			std::cout << "socket receive connection" << std::endl;
			LOG_INFO << "socket receive connection:\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
			return 0;
		});
		sock_.setOnLoseConnection([](aris::core::Socket *socket)
		{
			std::cout << "socket lose connection" << std::endl;
			LOG_INFO << "socket lose connection" << std::endl;
			for (;;)
			{
				try
				{
					socket->startServer("5866");
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
		});
		//
	}
}
