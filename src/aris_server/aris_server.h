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
		enum { MAX_MOTOR_NUM = 100 };

		using ParseFunc = std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)>;

		class ControlServer : public aris::core::Object
		{
		public:
			// Msg reserved 1 //
			enum 
			{ 
				PARSE_EVEN_IF_CMD_POOL_IS_FULL = 0x0001,
				PARSE_WHEN_ALL_PLAN_FINISHED = 0x0002,
				NOT_EXECUTE_RT_PLAN = 0x0004,
				EXECUTE_EVEN_IF_CMD_POOL_IS_FULL = 0X0008,
				EXECUTE_WHEN_ALL_PLAN_FINISHED = 0x0010,
				WAIT_FOR_RT_PLAN_EXECUTION = 0x0020
			};
			// Msg reserved 2 //
			enum
			{
				NOT_CHECK_POS_MIN = 0x0001,
				NOT_CHECK_POS_MAX = 0x0002,
				NOT_CHECK_POS_PLAN_CONTINUOUS = 0x0004,
				NOT_CHECK_POS_FOLLOWING_ERROR = 0x0008,
				NOT_CHECK_VEL_PLAN_CONTINUOUS = 0x0010,
				NOT_CHECK_VEL_FOLLOWING_ERROR = 0x0020,
				USING_TARGET_POS = 0x0100,
				USING_TARGET_VEL = 0x0200,
				USING_TARGET_CUR = 0x0400,
				USING_VEL_OFFSET = 0x0800,
				USING_CUR_OFFSET = 0x1000,
			};
			static auto instance()->ControlServer &;
			static auto Type()->const std::string &{ static const std::string type("ControlServer"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
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
			auto executeCmd(const aris::core::Msg &cmd_string)->void;
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
			std::uint32_t limit_time_;
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



		class ControlServer2 : public aris::core::Object
		{
		public:
			// Msg reserved 1 //
			enum
			{
				PARSE_EVEN_IF_CMD_POOL_IS_FULL = 0x0001,
				PARSE_WHEN_ALL_PLAN_FINISHED = 0x0002,
				NOT_EXECUTE_RT_PLAN = 0x0004,
				EXECUTE_EVEN_IF_CMD_POOL_IS_FULL = 0X0008,
				EXECUTE_WHEN_ALL_PLAN_FINISHED = 0x0010,
				WAIT_FOR_RT_PLAN_EXECUTION = 0x0020
			};
			// Msg reserved 2 //
			enum
			{
				NOT_CHECK_POS_MIN = 0x0001,
				NOT_CHECK_POS_MAX = 0x0002,
				NOT_CHECK_POS_PLAN_CONTINUOUS = 0x0004,
				NOT_CHECK_POS_FOLLOWING_ERROR = 0x0008,
				NOT_CHECK_VEL_PLAN_CONTINUOUS = 0x0010,
				NOT_CHECK_VEL_FOLLOWING_ERROR = 0x0020,
				USING_TARGET_POS = 0x0100,
				USING_TARGET_VEL = 0x0200,
				USING_TARGET_CUR = 0x0400,
				USING_VEL_OFFSET = 0x0800,
				USING_CUR_OFFSET = 0x1000,
			};
			static auto instance()->ControlServer2 &;
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
			auto model()const->const dynamic::Model& { return const_cast<ControlServer2 *>(this)->model(); }
			auto controller()->control::Controller&;
			auto controller()const->const control::Controller& { return const_cast<ControlServer2 *>(this)->controller(); }
			auto sensorRoot()->sensor::SensorRoot&;
			auto sensorRoot()const->const sensor::SensorRoot& { return const_cast<ControlServer2 *>(this)->sensorRoot(); }
			auto planRoot()->plan::PlanRoot&;
			auto planRoot()const->const plan::PlanRoot& { return const_cast<ControlServer2 *>(this)->planRoot(); }
			auto widgetRoot()->WidgetRoot&;
			auto widgetRoot()const->const WidgetRoot& { return const_cast<ControlServer2 *>(this)->widgetRoot(); }

			auto executeCmd(const aris::core::Msg &cmd_string)->void;
			auto start()->void;
			auto stop()->void;

		private:
			~ControlServer2();
			ControlServer2();
			ControlServer2(const ControlServer2 &) = delete;
			ControlServer2 &operator=(const ControlServer2 &) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};




	}
}

#endif

