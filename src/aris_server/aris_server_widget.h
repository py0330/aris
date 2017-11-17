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
		class WidgetRoot :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("WidgetRoot"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto cmdParser()->aris::core::CommandParser&;
			auto cmdParser()const->const aris::core::CommandParser&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->cmdParser(); }
			auto cmdSocket()->aris::core::Socket&;
			auto cmdSocket()const->const aris::core::Socket&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->cmdSocket(); }

			virtual ~WidgetRoot();
			WidgetRoot(const std::string &name = "widget_root");
			WidgetRoot(const WidgetRoot &other) = delete;
			WidgetRoot(WidgetRoot &&other) = delete;
			WidgetRoot& operator=(const WidgetRoot &other) = delete;
			WidgetRoot& operator=(WidgetRoot &&other) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};


		auto sendStringToAris(const char *data_to_aris, int data_to_size, char *data_from_aris, int *data_from_size)->void;
		auto sendDataToMotion(const char *data, int ms_pause_time)->void;

	}
}

#endif

