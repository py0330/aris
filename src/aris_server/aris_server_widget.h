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
			auto msgPipe()->aris::core::Pipe &;
			auto msgPipe()const->const aris::core::Pipe &;
			
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

