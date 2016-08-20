﻿#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
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
			aris::core::Socket *cmd_socket_{ nullptr };
			aris::core::CommandParser *cmd_parser_{ nullptr };
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

			imp_->cmd_socket_ = findOrInsert<aris::core::Socket>("command_socket");
			imp_->cmd_parser_ = findOrInsert<aris::core::CommandParser>("command_parser");
			imp_->msg_pipe_ = findOrInsert<aris::core::Pipe>("msg_pipe", 16384);
		}
		auto WidgetRoot::commandSocket()->aris::core::Socket& { return *imp_->cmd_socket_; }
		auto WidgetRoot::commandSocket()const->const aris::core::Socket&{ return *imp_->cmd_socket_; }
		auto WidgetRoot::commandParser()->aris::core::CommandParser& { return *imp_->cmd_parser_; }
		auto WidgetRoot::commandParser()const->const aris::core::CommandParser&{ return *imp_->cmd_parser_; }
		auto WidgetRoot::msgPipe()->aris::core::Pipe & { return *imp_->msg_pipe_; }
		auto WidgetRoot::msgPipe()const->const aris::core::Pipe &{ return *imp_->msg_pipe_; }
		WidgetRoot::~WidgetRoot() = default;
		WidgetRoot::WidgetRoot() :imp_{new Imp} 
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








