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

#include <aris_server_widget.h>


namespace aris
{
	namespace server
	{
		enum { MAX_MOTOR_NUM = 100 };

		class ControlServer;
		
		using ParseFunc = std::function<void(ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)>;

		// for enable, disable, and home //
		struct BasicFunctionParam
		{
			bool active_motor_[MAX_MOTOR_NUM];
            BasicFunctionParam() { std::fill(active_motor_, active_motor_ + MAX_MOTOR_NUM, true); }
		};
		// for all ordinary gaits //
		struct GaitParamBase :BasicFunctionParam
		{
			bool if_check_pos_min_{ true };
            bool if_check_pos_max_{ true };
            bool if_check_pos_continuous_{ true };
			bool if_check_pos_target_and_feedback_{ true };
		};

		class ControlServer
		{
		public:
			enum { MAX_RTOUT_MSG_SIZE = 8192 };
			enum { MAX_PLAN_PARAM_SIZE = 8192 };
			static auto instance()->ControlServer &;
			template<typename T = aris::dynamic::Model, typename... Args>
			auto makeModel(Args&&... args)->void { this->resetModel(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::control::Controller, typename... Args>
			auto makeController(Args&&... args)->void { this->resetController(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::sensor::SensorRoot, typename... Args>
			auto makeSensorRoot(Args&&... args)->void { this->resetSensorRoot(new T(std::forward<Args>(args)...)); }
			template<typename T = aris::server::WidgetRoot, typename... Args>
			auto makeWidgetRoot(Args&&... args)->void { this->resetWidgetRoot(new T(std::forward<Args>(args)...)); }
			auto resetModel(dynamic::Model *model)->void;
			auto resetController(control::Controller *controller)->void;
			auto resetSensorRoot(sensor::SensorRoot *sensor_root)->void;
			auto resetWidgetRoot(server::WidgetRoot *widget_root)->void;
			auto model()->dynamic::Model&;
			auto model()const->const dynamic::Model&{ return const_cast<ControlServer *>(this)->model(); }
			auto controller()->control::Controller&;
			auto controller()const->const control::Controller&{ return const_cast<ControlServer *>(this)->controller(); }
			auto sensorRoot()->sensor::SensorRoot&;
			auto sensorRoot()const->const sensor::SensorRoot&{ return const_cast<ControlServer *>(this)->sensorRoot(); }
			auto widgetRoot()->WidgetRoot&;
			auto widgetRoot()const->const WidgetRoot&{ return const_cast<ControlServer *>(this)->widgetRoot(); }
			auto loadXml(const char *file_name)->void;
			auto loadXml(const aris::core::XmlDocument &xml_doc)->void;
			auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunction &gait_func)->void;
			auto executeCmd(const std::string &cmd_string, bool if_wait_finish = true)->void;
			auto setOnExit(std::function<void(void)> callback_func)->void;

		private:
			~ControlServer();
			ControlServer();
			ControlServer(const ControlServer &) = delete;
			ControlServer &operator=(const ControlServer &) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};
	}
}

#endif

