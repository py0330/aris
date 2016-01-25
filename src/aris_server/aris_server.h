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



namespace Aris
{
	namespace Server
	{
		const int MAX_MOTOR_NUM = 100;

		/*for enable, disable, and home*/
		struct BasicFunctionParam final :Aris::Dynamic::PlanParamBase
		{
			bool active_motor[MAX_MOTOR_NUM]{ false };
		};

		/*for all ordinary gaits*/
		struct GaitParamBase :Dynamic::PlanParamBase
		{
			std::int32_t gait_id;
			const Aris::Sensor::ImuData *imu_data;
			const std::vector<Aris::Control::EthercatForceSensor::Data> *force_data;
			const std::vector<Aris::Control::EthercatMotion::RawData> *motion_raw_data;
			const std::vector<double> *motion_feedback_pos;
		};

		typedef std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)> ParseFunc;

		class ControlServer
		{
		public:
			static ControlServer &instance();

			template<typename T>
			void createModel() { this->createModel(new T); };
			void createModel(Dynamic::Model *pModel);

			void loadXml(const char *fileName);
			void loadXml(const Aris::Core::XmlDocument &xmlDoc);
			void addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const Aris::Dynamic::PlanFunc &gait_func);
			void start();
			void stop();

		private:
			ControlServer();
			~ControlServer();
			ControlServer(const ControlServer &) = delete;
			ControlServer &operator=(const ControlServer &) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> pImp;
		};
	}
}

#endif

