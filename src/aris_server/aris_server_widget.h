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
			auto commandSocket()->aris::core::Socket&;
			auto commandSocket()const->const aris::core::Socket&;
			auto commandParser()->aris::core::CommandParser&;
			auto commandParser()const->const aris::core::CommandParser&;
			
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

