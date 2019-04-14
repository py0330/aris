#ifndef ARIS_CONTROL_CONTROLLER_MOTION_H
#define ARIS_CONTROL_CONTROLLER_MOTION_H

#include <functional>
#include <thread>
#include <atomic>

#include <aris/core/core.hpp>
#include <aris/control/master_slave.hpp>

namespace aris::control
{
	class Motion : public virtual Slave
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto motId()const->aris::Size;
		auto maxPos()const->double;
		auto minPos()const->double;
		auto maxVel()const->double;
		auto minVel()const->double;
		auto maxAcc()const->double;
		auto minAcc()const->double;
		auto maxPosFollowingError()const->double;
		auto maxVelFollowingError()const->double;
		auto posOffset()const->double;
		auto posFactor()const->double;
		auto homePos()const->double;

		auto virtual controlWord()const->std::uint16_t = 0;
		auto virtual modeOfOperation()const->std::uint8_t = 0;
		auto virtual targetPos()const->double = 0;
		auto virtual targetVel()const->double = 0;
		auto virtual targetCur()const->double = 0;
		auto virtual offsetVel()const->double = 0;
		auto virtual offsetCur()const->double = 0;

		auto virtual setControlWord(std::uint16_t control_word)->void = 0;
		auto virtual setModeOfOperation(std::uint8_t mode)->void = 0;
		auto virtual setTargetPos(double pos)->void = 0;
		auto virtual setTargetVel(double vel)->void = 0;
		auto virtual setTargetCur(double cur)->void = 0;
		auto virtual setOffsetVel(double vel)->void = 0;
		auto virtual setOffsetCur(double cur)->void = 0;

		auto virtual statusWord()->std::uint16_t = 0;
		auto virtual modeOfDisplay()->std::uint8_t = 0;
		auto virtual actualPos()->double = 0;
		auto virtual actualVel()->double = 0;
		auto virtual actualCur()->double = 0;

		auto virtual disable()->int = 0;
		auto virtual enable()->int = 0;
		auto virtual home()->int = 0;
		auto virtual mode(std::uint8_t md)->int = 0;

		virtual ~Motion();
		explicit Motion(const std::string &name = "motion", std::uint16_t phy_id = 0
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0);
		ARIS_REGISTER_TYPE(Motion);
		ARIS_DELETE_BIG_FOUR(Motion);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class Controller;
	};
	class Controller : public virtual Master
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Controller"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto motionPool()->aris::core::RefPool<Motion>&;
		auto motionPool()const->const aris::core::RefPool<Motion>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->motionPool(); }
		auto motionAtSla(aris::Size id)->Motion&;
		auto motionAtSla(aris::Size id)const->const Motion& { return const_cast<std::decay_t<decltype(*this)> *>(this)->motionAtSla(id); }
		auto motionAtPhy(aris::Size id)->Motion&;
		auto motionAtPhy(aris::Size id)const->const Motion& { return const_cast<std::decay_t<decltype(*this)> *>(this)->motionAtPhy(id); }
		auto motionAtAbs(aris::Size id)->Motion&;
		auto motionAtAbs(aris::Size id)const->const Motion& { return const_cast<std::decay_t<decltype(*this)> *>(this)->motionAtAbs(id); }

		virtual ~Controller();
		Controller(const std::string &name = "controller");
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

#endif
