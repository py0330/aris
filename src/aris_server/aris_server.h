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
		
		using ParseFunc = std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)>;

		class ControlServer
		{
		public:
			// Msg reserved 1 //
			enum 
			{ 
				CHECK_POS_MIN = 0x0001, 
				CHECK_POS_MAX = 0x0002, 
				CHECK_POS_PLAN_CONTINUOUS = 0x0004, 
				CHECK_POS_FOLLOWING_ERROR = 0x0008,
				CHECK_VEL_PLAN_CONTINUOUS = 0x0010,
				CHECK_VEL_FOLLOWING_ERROR = 0x0020,
				USING_TARGET_POS = 0x0100,
				USING_TARGET_VEL = 0x0200,
				USING_TARGET_CUR = 0x0400,
				USING_VEL_OFFSET = 0x0800,
				USING_CUR_OFFSET = 0x1000,
			};
			// Msg reserved 2 //
			enum
			{
				EXECUTE_RT_PLAN = 0x0001,
			};
			// Msg reserved 3 //
			enum
			{
				WAIT_FOR_RT_PLAN_FINISHED = 0x0001
			};
			static auto instance()->ControlServer &;
			auto saveXml(const char *file_name)->void;
			auto saveXml(aris::core::XmlDocument &xml_doc)->void;
			auto saveXml(aris::core::XmlElement &xml_ele)->void;
			auto loadXml(const char *file_name)->void;
			auto loadXml(const aris::core::XmlDocument &xml_doc)->void;
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

			auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunction &gait_func)->void;
			auto executeCmd(const std::string &cmd_string)->void;
			auto start()->void;
			auto stop()->void;


		private:
			~ControlServer();
			ControlServer();
			ControlServer(const ControlServer &) = delete;
			ControlServer &operator=(const ControlServer &) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};

		struct DefaultParam
		{
			bool active_motor_[MAX_MOTOR_NUM];
			DefaultParam() { std::fill(active_motor_, active_motor_ + MAX_MOTOR_NUM, true); }
		};
		auto default_parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto default_enable_plan(const aris::dynamic::PlanParam &plan_param)->int;
		auto default_disable_plan(const aris::dynamic::PlanParam &plan_param)->int;
		auto default_mode_plan(const aris::dynamic::PlanParam &plan_param)->int;
		auto default_home_plan(const aris::dynamic::PlanParam &plan_param)->int;
		auto default_enable_command()->const aris::core::Command &;
		auto default_disable_command()->const aris::core::Command &;
		auto default_home_command()->const aris::core::Command &;
		auto default_mode_command()->const aris::core::Command &;
	}
}

#endif

