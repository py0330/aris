#ifndef ARIS_CONTROL_CONTROLLER_MOTION_H_
#define ARIS_CONTROL_CONTROLLER_MOTION_H_

#include <aris_lib_export.h>
#include <aris/control/master_slave.hpp>

namespace aris::control
{
	class ARIS_API Motor{
	public:
		auto motId()const->aris::Size;
		auto maxPos()const->double;
		auto setMaxPos(double max_pos)->void;
		auto minPos()const->double;
		auto setMinPos(double min_pos)->void;
		auto maxVel()const->double;
		auto setMaxVel(double max_vel)->void;
		auto minVel()const->double;
		auto setMinVel(double min_vel)->void;
		auto maxAcc()const->double;
		auto setMaxAcc(double max_acc)->void;
		auto minAcc()const->double;
		auto setMinAcc(double min_acc)->void;
		auto maxPosFollowingError()const->double;
		auto setMaxPosFollowingError(double max_pos_following_error)->void;
		auto maxVelFollowingError()const->double;
		auto setMaxVelFollowingError(double max_vel_following_error)->void;
		auto posOffset()const->double;
		auto setPosOffset(double pos_offset)->void;
		auto posFactor()const->double;
		auto setPosFactor(double pos_factor)->void;
		auto homePos()const->double;
		auto setHomePos(double home_pos)->void;

		auto virtual controlWord()const->std::uint16_t = 0;
		auto virtual modeOfOperation()const->std::uint8_t = 0;
		auto virtual targetPos()const->double = 0;
		auto virtual targetVel()const->double = 0;
		auto virtual targetToq()const->double = 0;
		auto virtual offsetVel()const->double = 0;
		auto virtual offsetCur()const->double = 0;

		auto virtual setControlWord(std::uint16_t control_word)->void = 0;
		auto virtual setModeOfOperation(std::uint8_t mode)->void = 0;
		auto virtual setTargetPos(double pos)->void = 0;
		auto virtual setTargetVel(double vel)->void = 0;
		auto virtual setTargetToq(double toq)->void = 0;
		auto virtual setOffsetVel(double vel)->void = 0;
		auto virtual setOffsetToq(double toq)->void = 0;

		auto virtual statusWord()const->std::uint16_t = 0;
		auto virtual modeOfDisplay()const->std::uint8_t = 0;
		auto virtual actualPos()const->double = 0;
		auto virtual actualVel()const->double = 0;
		auto virtual actualToq()const->double = 0;
		auto virtual actualCur()const->double = 0;

		auto virtual disable()->int = 0;
		auto virtual enable()->int = 0;
		auto virtual home()->int = 0;
		auto virtual mode(std::uint8_t md)->int = 0;

		virtual ~Motor();
		explicit Motor(double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0);
		Motor(const Motor &other);
		Motor(Motor &&other) = delete;
		Motor& operator=(const Motor &other);
		Motor& operator=(Motor &&other) = delete;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class Controller;
	};
	class ARIS_API DigitalIo {
	public:
		auto numOfDi() const->std::uint16_t;
		auto setNumOfDi(std::uint16_t num_of_di)->void;

		auto numOfDo() const->std::uint16_t;
		auto setNumOfDo(std::uint16_t num_of_do)->void;

		virtual auto getDi(const std::uint16_t index)->bool = 0;
		virtual auto getDo(const std::uint16_t index)->bool = 0;
		virtual auto setDo(const std::uint16_t index, const bool status)->void = 0;

		virtual ~DigitalIo();
		explicit DigitalIo(std::uint16_t num_of_di = 8, std::uint16_t num_of_do = 8);
		DigitalIo(const DigitalIo &other);
		DigitalIo(DigitalIo &&other) = delete;
		DigitalIo& operator=(const DigitalIo &other);
		DigitalIo& operator=(DigitalIo &&other) = delete;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class Controller;
	};
	class ARIS_API FtSensor {
	public:
		virtual auto getFtData(double *data_address)->void = 0;
	};

	class ARIS_API Controller {
	public:
		auto virtual init()->void;

		auto resetMotorPool(aris::core::PointerArray<Motor> *pool);
		auto motorPool()->aris::core::PointerArray<Motor>&;
		auto motorPool()const->const aris::core::PointerArray<Motor>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->motorPool(); }

		auto resetDigitalIoPool(aris::core::PointerArray<DigitalIo> *pool);
		auto digitalIoPool()->aris::core::PointerArray<DigitalIo>&;
		auto digitalIoPool()const->const aris::core::PointerArray<DigitalIo>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->digitalIoPool(); }

		auto resetFtSensorPool(aris::core::PointerArray<FtSensor> *pool);
		auto ftSensorPool()->aris::core::PointerArray<FtSensor>&;
		auto ftSensorPool()const->const aris::core::PointerArray<FtSensor>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->ftSensorPool(); }

		virtual ~Controller();
		Controller(const std::string &name = "controller");
		Controller(const Controller &other) = delete;
		Controller(Controller &&other) = delete;
		Controller& operator=(const Controller &other) = delete;
		Controller& operator=(Controller &&other) = delete;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif
