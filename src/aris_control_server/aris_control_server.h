#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>

#include <aris_core.h>
#include <aris_motion.h>
#include <aris_imu.h>
#include <aris_dyn_model.h>



namespace Aris
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
		const std::vector<Aris::Control::EthercatMotion::Data> *motion_raw_data;
		const std::vector<double> *motion_feedback_pos;
	};
	
	typedef std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)> ParseFunc;

	class ControlServer
	{
	public:
		static ControlServer &Instance();

		template<typename T>
		void CreateModel() { this->CreateModel(new T); };
		void CreateModel(Dynamic::Model *pModel);

		void LoadXml(const char *fileName);
		void LoadXml(const Aris::Core::XmlDocument &xmlDoc);
		void AddCmd(const std::string &cmd_name, const ParseFunc &parse_func, const Aris::Dynamic::PlanFunc &gait_func);
		void Start();
		void Stop();

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

#endif

