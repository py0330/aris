#ifndef ARIS_SERVER_H
#define ARIS_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <future>

#include <aris_core.h>
#include <aris_control.h>
#include <aris_sensor.h>
#include <aris_dynamic.h>
#include <aris_plan.h>

#include <aris_server_widget.h>


namespace aris
{
	namespace server
	{
		class ControlServer : public aris::core::Object
		{
		public:
			static auto instance()->ControlServer &;
			static auto Type()->const std::string & { static const std::string type("ControlServer"); return type; }
			auto virtual type() const->const std::string& override { return Type(); }
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			template<typename T = aris::dynamic::Model, typename... Args>
			auto makeModel(Args&&... args)->void { this->resetModel(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::control::Controller, typename... Args>
			auto makeController(Args&&... args)->void { this->resetController(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::sensor::SensorRoot, typename... Args>
			auto makeSensorRoot(Args&&... args)->void { this->resetSensorRoot(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::plan::PlanRoot, typename... Args>
			auto makePlanRoot(Args&&... args)->void { this->resetPlanRoot(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::server::WidgetRoot, typename... Args>
			auto makeWidgetRoot(Args&&... args)->void { this->resetWidgetRoot(new T(std::forward<Args>(args)...)); }
			auto resetModel(dynamic::Model *model)->void;
			auto resetController(control::Controller *controller)->void;
			auto resetSensorRoot(sensor::SensorRoot *sensor_root)->void;
			auto resetPlanRoot(plan::PlanRoot *sensor_root)->void;
			auto resetWidgetRoot(server::WidgetRoot *widget_root)->void;
			auto model()->dynamic::Model&;
			auto model()const->const dynamic::Model& { return const_cast<ControlServer *>(this)->model(); }
			auto controller()->control::Controller&;
			auto controller()const->const control::Controller& { return const_cast<ControlServer *>(this)->controller(); }
			auto sensorRoot()->sensor::SensorRoot&;
			auto sensorRoot()const->const sensor::SensorRoot& { return const_cast<ControlServer *>(this)->sensorRoot(); }
			auto planRoot()->plan::PlanRoot&;
			auto planRoot()const->const plan::PlanRoot& { return const_cast<ControlServer *>(this)->planRoot(); }
			auto widgetRoot()->WidgetRoot&;
			auto widgetRoot()const->const WidgetRoot& { return const_cast<ControlServer *>(this)->widgetRoot(); }

			auto executeCmd(const aris::core::Msg &cmd_string)->std::int64_t;
			auto start()->void;
			auto stop()->void;
			auto currentCmdId()->std::int64_t;
			auto getPartPm()->std::vector<double>;

		private:
			~ControlServer();
			ControlServer();
			ControlServer(const ControlServer &) = delete;
			ControlServer &operator=(const ControlServer &) = delete;

		private:
			struct Imp;
			std::unique_ptr<Imp> imp_;
		};
	}
}

#endif

