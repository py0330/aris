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
	struct BasicFunctionParam final :Aris::DynKer::PlanParamBase
	{
		bool isMotorActive[MAX_MOTOR_NUM]{ false };
	};

	/*for recover*/
	struct RecoverParam final :Aris::DynKer::PlanParamBase
	{
		std::int32_t alignCount{ 3000 };
		std::int32_t recoverCount{ 3000 };
		double beginPin[18]{ 0 };
		double alignPin[18]{ 0 };
		double alignPee[18]{ 0 };
		double recoverPee[18]{ 0 };
		bool isLegActive[6]{ true,true,true,true,true,true };
	};
	
	struct GaitParamBase :DynKer::PlanParamBase
	{
		const Aris::Sensor::ImuData *imuData;
		const std::vector<Aris::Control::EthercatForceSensor::Data> *pForceData;
		double beginPee[18]{ 0 };
		double beginVee[18]{ 0 };
		double beginPeb[6]{ 0 };
		double beginVb[6]{ 0 };
	};
	
	
	typedef std::function<Aris::Core::Msg(const std::string &cmd, const std::map<std::string, std::string> &params)> ParseFunc;

	class ControlServer
	{
	public:
		static ControlServer &Instance();

		template<typename T>
		void CreateRobot()
		{
			if (pRobot)
				throw std::logic_error("already has a robot instance");
			else
				pRobot.reset(new T);
		};
		void LoadXml(const char *fileName);
		void LoadXml(const Aris::Core::XmlDocument &xmlDoc);
		void AddGait(std::string cmdName, Aris::DynKer::PlanFunc gaitFunc, ParseFunc parseFunc);
		void Start();
		void Stop();

	private:
		ControlServer();
		virtual ~ControlServer();
		ControlServer(const ControlServer &) = delete;
		ControlServer &operator=(const ControlServer &) = delete;

		virtual void ParseEnableMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out);
		virtual void ParseDisableMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out);
		virtual void ParseHomeMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out);

		std::unique_ptr<DynKer::Model> pRobot;
	private:
		class Imp;
		std::unique_ptr<Imp> pImp;
	};
}

#endif

