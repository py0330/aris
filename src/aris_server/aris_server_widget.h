#ifndef ARIS_SERVER_WIDGET_H
#define ARIS_SERVER_WIDGET_H

#include <aris_core.h>
#include <aris_control.h>
#include <aris_sensor.h>
#include <aris_dynamic.h>



namespace aris
{
	namespace server
	{
		class WidgetRoot :public aris::core::Root
		{
		public:
			using Root::loadXml;
			virtual auto loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto cmdParser()->aris::core::CommandParser&;
			auto cmdParser()const->const aris::core::CommandParser&;
			auto cmdPipe()->aris::core::Pipe &;
			auto cmdPipe()const->const aris::core::Pipe &;
			auto msgPipe()->aris::core::Pipe &;
			auto msgPipe()const->const aris::core::Pipe &;
			auto msgOut()->aris::core::MsgBase& { static aris::core::MsgFix<8192> msg; return msg; };
			auto msgOut()const->const aris::core::MsgBase&{ return const_cast<WidgetRoot*>(this)->msgOut(); };
			auto mout()->aris::core::MsgStream & { static aris::core::MsgStream stream(msgOut()); return stream; };
			auto mout()const->const aris::core::MsgStream &{ return const_cast<WidgetRoot*>(this)->mout(); };
			
			virtual ~WidgetRoot();
			WidgetRoot();
			WidgetRoot(const WidgetRoot &other) = delete;
			WidgetRoot(WidgetRoot &&other) = delete;
			WidgetRoot& operator=(const WidgetRoot &other) = delete;
			WidgetRoot& operator=(WidgetRoot &&other) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};
	}
}

#endif

