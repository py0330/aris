#include <string>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <thread>
#include <future>

#include "aris/core/reflection.hpp"

#include "aris/control/controller_motion.hpp"

namespace aris::control{
	struct Motor::Imp{
		double max_pos_, min_pos_, max_vel_, min_vel_, max_acc_, min_acc_, max_pos_following_error_, max_vel_following_error_;
		double pos_factor_, pos_offset_, vel_factor_;
		double home_pos_;
		aris::Size mot_id_;
	};
	auto Motor::motId()const->aris::Size { return imp_->mot_id_; }
	auto Motor::maxPos()const->double { return imp_->max_pos_; }
	auto Motor::setMaxPos(double max_pos)->void { imp_->max_pos_ = max_pos; }
	auto Motor::minPos()const->double { return imp_->min_pos_; }
	auto Motor::setMinPos(double min_pos)->void { imp_->min_pos_ = min_pos; }
	auto Motor::maxVel()const->double { return imp_->max_vel_; }
	auto Motor::setMaxVel(double max_vel)->void { imp_->max_vel_ = max_vel; }
	auto Motor::minVel()const->double { return imp_->min_vel_; }
	auto Motor::setMinVel(double min_vel)->void { imp_->min_vel_ = min_vel; }
	auto Motor::maxAcc()const->double { return imp_->max_acc_; }
	auto Motor::setMaxAcc(double max_acc)->void { imp_->max_acc_ = max_acc; }
	auto Motor::minAcc()const->double { return imp_->min_acc_; }
	auto Motor::setMinAcc(double min_acc)->void { imp_->min_acc_ = min_acc; }
	auto Motor::maxPosFollowingError()const->double { return imp_->max_pos_following_error_; }
	auto Motor::setMaxPosFollowingError(double max_pos_following_error)->void { imp_->max_pos_following_error_ = max_pos_following_error; }
	auto Motor::maxVelFollowingError()const->double { return imp_->max_vel_following_error_; }
	auto Motor::setMaxVelFollowingError(double max_vel_following_error)->void { imp_->max_vel_following_error_ = max_vel_following_error; }
	auto Motor::posOffset()const->double { return imp_->pos_offset_; }
	auto Motor::setPosOffset(double pos_offset)->void { imp_->pos_offset_ = pos_offset; }
	auto Motor::posFactor()const->double { return imp_->pos_factor_; }
	auto Motor::setPosFactor(double pos_factor)->void { imp_->pos_factor_ = pos_factor; }
	auto Motor::homePos()const->double { return imp_->home_pos_; }
	auto Motor::setHomePos(double home_pos)->void { imp_->home_pos_ = home_pos; }
	auto Motor::velFactor()const->double { return imp_->vel_factor_; }
	auto Motor::setVelFactor(double vel_factor)->void { imp_->vel_factor_ = vel_factor; }
	Motor::~Motor() = default;
	Motor::Motor(double max_pos, double min_pos, double max_vel, double min_vel, double max_acc, double min_acc
		, double max_pos_following_error, double max_vel_following_error, double pos_factor, double pos_offset, double home_pos, double vel_factor)
	{
		imp_->max_pos_ = max_pos;
		imp_->min_pos_ = min_pos;
		imp_->max_vel_ = max_vel;
		imp_->min_vel_ = min_vel;
		imp_->max_acc_ = max_acc;
		imp_->min_acc_ = min_acc;
		imp_->max_pos_following_error_ = max_pos_following_error;
		imp_->max_vel_following_error_ = max_vel_following_error;
		imp_->pos_factor_ = pos_factor;
		imp_->pos_offset_ = pos_offset;
		imp_->vel_factor_ = vel_factor;
		imp_->home_pos_ = home_pos;
	}
	Motor::Motor(const Motor &other) = default;
	Motor& Motor::operator=(const Motor &other) = default;

	struct DigitalIo::Imp{
		std::uint16_t num_of_di_, num_of_do_;
	};
	auto DigitalIo::numOfDi() const->std::uint16_t {
		return imp_->num_of_di_;
	}
	auto DigitalIo::setNumOfDi(std::uint16_t num_of_di)->void {
		imp_->num_of_di_ = num_of_di;
	}
	auto DigitalIo::numOfDo() const->std::uint16_t {
		return imp_->num_of_do_;
	}
	auto DigitalIo::setNumOfDo(std::uint16_t num_of_do)->void {
		imp_->num_of_do_ = num_of_do;
	}
	DigitalIo::~DigitalIo() = default;
	DigitalIo::DigitalIo(std::uint16_t num_of_di, std::uint16_t num_of_do){
		imp_->num_of_di_ = num_of_di;
		imp_->num_of_do_ = num_of_do;
	}
	DigitalIo::DigitalIo(const DigitalIo &other) = default;
	DigitalIo& DigitalIo::operator=(const DigitalIo &other) = default;

	struct AnalogIo::Imp{
		std::uint16_t num_of_ai_, num_of_ao_;
	};
	auto AnalogIo::numOfAi() const->std::uint16_t {
		return imp_->num_of_ai_;
	}
	auto AnalogIo::setNumOfAi(std::uint16_t num_of_ai)->void {
		imp_->num_of_ai_ = num_of_ai;
	}
	auto AnalogIo::numOfAo() const->std::uint16_t {
		return imp_->num_of_ao_;
	}
	auto AnalogIo::setNumOfAo(std::uint16_t num_of_ao)->void {
		imp_->num_of_ao_ = num_of_ao;
	}
	AnalogIo::~AnalogIo() = default;
	AnalogIo::AnalogIo(std::uint16_t num_of_ai, std::uint16_t num_of_ao){
		imp_->num_of_ai_ = num_of_ai;
		imp_->num_of_ao_ = num_of_ao;
	}
	AnalogIo::AnalogIo(const AnalogIo &other) = default;
	AnalogIo& AnalogIo::operator=(const AnalogIo &other) = default;

	struct Controller::Imp { 
		std::unique_ptr<aris::core::PointerArray<Motor>> motor_pool_{ new aris::core::PointerArray<Motor> };
		std::unique_ptr<aris::core::PointerArray<DigitalIo>> digital_io_pool_{ new aris::core::PointerArray<DigitalIo> };
		std::unique_ptr<aris::core::PointerArray<AnalogIo>> analog_io_pool_{ new aris::core::PointerArray<AnalogIo> };
		std::unique_ptr<aris::core::PointerArray<FtSensor>> ft_sensor_pool_{ new aris::core::PointerArray<FtSensor> };
	};
	auto Controller::resetMotorPool(aris::core::PointerArray<Motor> *pool) { imp_->motor_pool_.reset(pool); }
	auto Controller::motorPool()->aris::core::PointerArray<Motor>& { return *imp_->motor_pool_; }
	auto Controller::resetDigitalIoPool(aris::core::PointerArray<DigitalIo> *pool) { imp_->digital_io_pool_.reset(pool); }
	auto Controller::digitalIoPool()->aris::core::PointerArray<DigitalIo>& { return *imp_->digital_io_pool_; }
	auto Controller::resetAnalogIoPool(aris::core::PointerArray<AnalogIo> *pool) { imp_->analog_io_pool_.reset(pool); }
	auto Controller::analogIoPool()->aris::core::PointerArray<AnalogIo>& { return *imp_->analog_io_pool_; }
	auto Controller::resetFtSensorPool(aris::core::PointerArray<FtSensor> *pool) { imp_->ft_sensor_pool_.reset(pool); }
	auto Controller::ftSensorPool()->aris::core::PointerArray<FtSensor>& { return *imp_->ft_sensor_pool_; }
	auto Controller::init()->void{
		for (auto &motor : motorPool())	 motor.init();
		for (auto &digital_io : digitalIoPool()) digital_io.init();
		for (auto &analog_io : analogIoPool()) analog_io.init();
		for (auto &ft_sensor : ftSensorPool()) ft_sensor.init();
	}
	Controller::~Controller() = default;
	Controller::Controller(const std::string &name) :imp_(new Imp) {}

	ARIS_REGISTRATION{
		aris::core::class_<Motor>("Motor")
			.prop("max_pos", &Motor::setMaxPos, &Motor::maxPos)
			.prop("min_pos", &Motor::setMinPos, &Motor::minPos)
			.prop("max_vel", &Motor::setMaxVel, &Motor::maxVel)
			.prop("min_vel", &Motor::setMinVel, &Motor::minVel)
			.prop("max_acc", &Motor::setMaxAcc, &Motor::maxAcc)
			.prop("min_acc", &Motor::setMinAcc, &Motor::minAcc)
			.prop("max_pos_following_error", &Motor::setMaxPosFollowingError, &Motor::maxPosFollowingError)
			.prop("max_vel_following_error", &Motor::setMaxVelFollowingError, &Motor::maxVelFollowingError)
			.prop("pos_factor", &Motor::setPosFactor, &Motor::posFactor)
			.prop("pos_offset", &Motor::setPosOffset, &Motor::posOffset)
			.prop("home_pos", &Motor::setHomePos, &Motor::homePos)
			.prop("vel_factor", &Motor::setVelFactor, &Motor::velFactor)
			;

		aris::core::class_<DigitalIo>("DigitalIo")
			.prop("num_of_di", &DigitalIo::setNumOfDi, &DigitalIo::numOfDi)
			.prop("num_of_do", &DigitalIo::setNumOfDo, &DigitalIo::numOfDo)
			;

		aris::core::class_<AnalogIo>("AnalogIo")
			.prop("num_of_ai", &AnalogIo::setNumOfAi, &AnalogIo::numOfAi)
			.prop("num_of_ao", &AnalogIo::setNumOfAo, &AnalogIo::numOfAo)
			;

		aris::core::class_<FtSensor>("FtSensor")
			;

		aris::core::class_<aris::core::PointerArray<Motor>>("MotorPoolObject")
			.asRefArray();

		aris::core::class_<aris::core::PointerArray<DigitalIo>>("DigitalIoPoolObject")
			.asRefArray();

		aris::core::class_<aris::core::PointerArray<AnalogIo>>("AnalogIoPoolObject")
			.asRefArray();

		aris::core::class_<aris::core::PointerArray<FtSensor>>("FtSensorPoolObject")
			.asRefArray();

		typedef aris::core::PointerArray<Motor>&(Controller::*MotorPoolFunc)();
		typedef aris::core::PointerArray<DigitalIo>&(Controller::*DigitalIoPoolFunc)();
		typedef aris::core::PointerArray<AnalogIo>&(Controller::*AnalogIoPoolFunc)();
		typedef aris::core::PointerArray<FtSensor>&(Controller::*FtSensorPoolFunc)();
		aris::core::class_<Controller>("Controller")
			.prop("motor_pool", &Controller::resetMotorPool, MotorPoolFunc(&Controller::motorPool))
			.prop("digital_io_pool", &Controller::resetDigitalIoPool, DigitalIoPoolFunc(&Controller::digitalIoPool))
			.prop("analog_io_pool", &Controller::resetAnalogIoPool, AnalogIoPoolFunc(&Controller::analogIoPool))
			.prop("ft_sensor_pool", &Controller::resetFtSensorPool, FtSensorPoolFunc(&Controller::ftSensorPool))
			;
	}
}
