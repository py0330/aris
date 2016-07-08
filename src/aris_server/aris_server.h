#ifndef ARIS_SERVER_H
#define ARIS_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>

#include <aris_core.h>
#include <aris_control.h>
#include <aris_sensor.h>
#include <aris_dynamic.h>



namespace aris
{
	namespace server
	{
		enum { MAX_MOTOR_NUM = 100 };

		class ControlServer;
		
		using ParseFunc = std::function<void(const ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)>;

		//for enable, disable, and home
		struct BasicFunctionParam :aris::dynamic::PlanParamBase
		{
			bool active_motor_[MAX_MOTOR_NUM];
            BasicFunctionParam() { std::fill(active_motor_, active_motor_ + MAX_MOTOR_NUM, true); }
		};
		//for all ordinary gaits
		struct GaitParamBase :BasicFunctionParam
		{
			ControlServer* cs_;
			std::int32_t gait_id_;

			bool if_check_pos_min_{ true };
            bool if_check_pos_max_{ true };
            bool if_check_pos_continuous_{ true };
		};

		class ControlServer
		{
		public:
			static ControlServer &instance();

			template<typename T>
			auto createModel()->void { this->createModel(new T); }
			auto createModel(dynamic::Model *model)->void;
			template<typename T>
			auto createController()->void { this->createController(new T); }
			auto createController(control::Controller *controller)->void;
			template<typename T>
			auto createSensorRoot()->void { this->createSensorRoot(new T); }
			auto createSensorRoot(sensor::SensorRoot *sensor_root)->void;
			
			auto model()->dynamic::Model&;
			auto model()const->const dynamic::Model&{ return const_cast<ControlServer *>(this)->model(); }
			auto controller()->control::Controller&;
			auto controller()const->const control::Controller&{ return const_cast<ControlServer *>(this)->controller(); }
			auto sensorRoot()->sensor::SensorRoot&;
			auto sensorRoot()const->const sensor::SensorRoot&{ return const_cast<ControlServer *>(this)->sensorRoot(); }
			auto parser()->core::CommandParser&;
			auto parser()const->const core::CommandParser&{ return const_cast<ControlServer *>(this)->parser(); }

			auto loadXml(const char *fileName)->void;
			auto loadXml(const aris::core::XmlDocument &xmlDoc)->void;
			auto open()->void;
			auto close()->void;
			auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunc &gait_func)->void;
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

