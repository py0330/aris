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








