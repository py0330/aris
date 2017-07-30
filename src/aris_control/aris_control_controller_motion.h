#ifndef ARIS_CONTROL_CONTROLLER_MOTION_H
#define ARIS_CONTROL_CONTROLLER_MOTION_H

#include <functional>
#include <thread>
#include <atomic>

#include <aris_core.h>
#include <aris_control_master_slave.h>

namespace aris
{
	namespace control
	{
		class Motion : public virtual Slave
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Motion"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto maxPos()->double;
			auto minPos()->double;
			auto maxVel()->double;
			auto posOffset()->double;
			auto posFactor()->double;

			auto virtual modeOfOperation()const->std::uint8_t = 0;
			auto virtual targetPos()const->double = 0;
			auto virtual targetVel()const->double = 0;
			auto virtual targetCur()const->double = 0;
			auto virtual offsetVel()const->double = 0;
			auto virtual offsetCur()const->double = 0;

			auto virtual setModeOfOperation(std::uint8_t mode)->void = 0;
			auto virtual setTargetPos(double pos)->void = 0;
			auto virtual setTargetVel(double vel)->void = 0;
			auto virtual setTargetCur(double cur)->void = 0;
			auto virtual setOffsetVel(double vel)->void = 0;
			auto virtual setOffsetCur(double cur)->void = 0;

			auto virtual modeOfDisplay()->std::uint8_t = 0;
			auto virtual actualPos()->double = 0;
			auto virtual actualVel()->double = 0;
			auto virtual actualCur()->double = 0;

			auto virtual disable()->int = 0;
			auto virtual enable()->int = 0;
			auto virtual home()->int = 0;
			auto virtual mode(std::uint8_t md)->int = 0;

			virtual ~Motion();
			explicit Motion(const std::string &name, const SlaveType &st, std::int32_t pos_factor, double max_pos, double min_pos, double max_vel, double home_pos = 0, double pos_offset = 0);
			explicit Motion(Object &father, const aris::core::XmlElement &xml_ele);
			Motion(const Motion &other) = delete;
			Motion(Motion &&other) = delete;
			Motion& operator=(const Motion &other) = delete;
			Motion& operator=(Motion &&other) = delete;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Controller : public virtual Master
		{
		public:
			auto motionPool()->aris::core::RefPool<Motion>&;

			virtual ~Controller();
			Controller();
			Controller(const Controller &other) = delete;
			Controller(Controller &&other) = delete;
			Controller& operator=(const Controller &other) = delete;
			Controller& operator=(Controller &&other) = delete;

		protected:
			auto virtual init()->void override;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

		};
	}
}

#endif
