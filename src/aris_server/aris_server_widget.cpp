#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>

#include "aris_core.h"
#include "aris_control.h"
#include "aris_server.h"
#include "aris_server_widget.h"

namespace aris
{
	namespace server
    {
		aris::dynamic::SimpleModel m;
		
		auto sendStringToAris(const char *data_send, int send_size, char *data_return, int *return_size)->void
		{
			aris::core::XmlDocument doc;
			if(doc.Parse(data_send, send_size)) std::cout << "received data is not xml format" << std::endl;

			if (doc.RootElement()->Name() == std::string("sendDataToMotion"))
			{
				auto list = doc.RootElement()->FirstChildElement();
				auto ms_pause_time = list->NextSiblingElement();

				sendDataToMotion(list->GetText(), std::atoi(ms_pause_time->GetText()));

				*return_size = 0;
			}
			else if (doc.RootElement()->Name() == std::string("addPart"))
			{

			}
			else
			{
				std::cout << "unknown cmd" << std::endl;
			}
		}

		auto sendDataToMotion(const char *data, int ms_pause_time)->void
		{
			static aris::core::Socket2 sock;
			static bool first_time{ true };
			
			if (first_time)
			{
				first_time = false;

				sock.setOnReceivedMsg([](aris::core::Socket2* sock, aris::core::Msg &msg)->int
				{
					std::cout << "receive:" << msg.data() << std::endl;
					return 0;
				});

				sock.connect("192.168.4.1", "6000");

				std::cout << "connected" << std::endl;
			}

			std::string s;
			s.assign(data, std::strlen(data));
			std::stringstream ss(s);
			char cmd[1024];
			while (ss.getline(cmd, 1024))
			{
				cmd[std::strlen(cmd) + 2] = '\0';
				cmd[std::strlen(cmd) + 1] = '\r';
				cmd[std::strlen(cmd) + 0] = '\n';

				std::cout << "send:" << cmd << std::endl;
				std::cout << "pause:" << ms_pause_time << std::endl;

				sock.sendMsg(cmd);

				std::this_thread::sleep_for(std::chrono::milliseconds(ms_pause_time));
			}

		}
		
		class WidgetRoot::Imp
		{
		public:
			aris::core::CommandParser *cmd_parser_{ nullptr };
			aris::core::Socket *cmd_socket_{ nullptr };
		};
		auto WidgetRoot::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Object::loadXml(xml_ele);

			imp_->cmd_parser_ = findOrInsert<aris::core::CommandParser>("command_parser");
			imp_->cmd_socket_ = findOrInsert<aris::core::Socket>("command_socket");
		}
		auto WidgetRoot::cmdParser()->aris::core::CommandParser& { return *imp_->cmd_parser_; }
		auto WidgetRoot::cmdSocket()->aris::core::Socket& { return *imp_->cmd_socket_; }
		
		WidgetRoot::~WidgetRoot() = default;
		WidgetRoot::WidgetRoot(const std::string &name) :imp_{ new Imp }, aris::core::Object(name)
		{
			registerType<aris::core::CommandParser>();

			registerType<aris::core::Socket>();

			registerType<aris::core::Pipe>();

			imp_->cmd_parser_ = &add<aris::core::CommandParser>("command_parser");
			imp_->cmd_socket_ = &add<aris::core::Socket>("command_socket");
		};
	}
}








