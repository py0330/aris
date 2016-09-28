#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

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
			aris::core::Pipe *cmd_pipe_{ nullptr };
			aris::core::Pipe *msg_pipe_{ nullptr };
		};
		auto WidgetRoot::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			auto root_xml_ele = xml_doc.RootElement()->FirstChildElement("widget_root");

			if (!root_xml_ele)throw std::runtime_error("can't find controller element in xml file");

			loadXml(*root_xml_ele);
		}
		auto WidgetRoot::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Root::loadXml(xml_ele);

			imp_->cmd_parser_ = findOrInsert<aris::core::CommandParser>("command_parser");
			imp_->cmd_pipe_ = findOrInsert<aris::core::Pipe>("command_pipe", 16384);
			imp_->msg_pipe_ = findOrInsert<aris::core::Pipe>("message_pipe", 16384);
		}
		auto WidgetRoot::cmdParser()->aris::core::CommandParser& { return *imp_->cmd_parser_; }
		auto WidgetRoot::cmdParser()const->const aris::core::CommandParser&{ return *imp_->cmd_parser_; }
		auto WidgetRoot::cmdPipe()->aris::core::Pipe & { return *imp_->cmd_pipe_; }
		auto WidgetRoot::cmdPipe()const->const aris::core::Pipe &{ return *imp_->cmd_pipe_; }
		auto WidgetRoot::msgPipe()->aris::core::Pipe & { return *imp_->msg_pipe_; }
		auto WidgetRoot::msgPipe()const->const aris::core::Pipe &{ return *imp_->msg_pipe_; }
		
		WidgetRoot::~WidgetRoot() = default;
		WidgetRoot::WidgetRoot() :imp_{ new Imp }
		{
			registerChildType<aris::core::Param>();
			registerChildType<aris::core::UniqueParam>();
			registerChildType<aris::core::GroupParam>();
			registerChildType<aris::core::Command>();
			registerChildType<aris::core::ObjectPool<aris::core::Command> >();
			registerChildType<aris::core::CommandParser>();

			registerChildType<aris::core::Socket>();

			registerChildType<aris::core::Pipe>();
		};
	}
}








