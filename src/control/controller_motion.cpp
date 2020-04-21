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

namespace aris::control
{
	struct Motor::Imp
	{
		double max_pos_, min_pos_, max_vel_, min_vel_, max_acc_, min_acc_, max_pos_following_error_, max_vel_following_error_;
		double pos_factor_, pos_offset_;
		double home_pos_;
		aris::Size mot_id_;
	};
	auto Motor::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Slave::saveXml(xml_ele);
		xml_ele.SetAttribute("max_pos", maxPos());
		xml_ele.SetAttribute("min_pos", minPos());
		xml_ele.SetAttribute("max_vel", maxVel());
		xml_ele.SetAttribute("min_vel", minVel());
		xml_ele.SetAttribute("max_acc", maxAcc());
		xml_ele.SetAttribute("min_acc", minAcc());
		xml_ele.SetAttribute("max_pos_following_error", maxPosFollowingError());
		xml_ele.SetAttribute("max_vel_following_error", maxVelFollowingError());
		xml_ele.SetAttribute("pos_factor", posFactor());
		xml_ele.SetAttribute("pos_offset", posOffset());
		xml_ele.SetAttribute("home_pos", homePos());
	}
	auto Motor::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		imp_->max_pos_ = attributeDouble(xml_ele, "max_pos", 1.0);
		imp_->min_pos_ = attributeDouble(xml_ele, "min_pos", -1.0);
		imp_->max_vel_ = attributeDouble(xml_ele, "max_vel", 1.0);
		imp_->min_vel_ = attributeDouble(xml_ele, "min_vel", -imp_->max_vel_);
		imp_->max_acc_ = attributeDouble(xml_ele, "max_acc", 1.0);
		imp_->min_acc_ = attributeDouble(xml_ele, "min_acc", -imp_->max_acc_);
		imp_->max_pos_following_error_ = attributeDouble(xml_ele, "max_pos_following_error", 1.0);
		imp_->max_vel_following_error_ = attributeDouble(xml_ele, "max_vel_following_error", 1.0);
		imp_->pos_factor_ = attributeDouble(xml_ele, "pos_factor", 1.0);
		imp_->pos_offset_ = attributeDouble(xml_ele, "pos_offset", 0.0);
		imp_->home_pos_ = attributeDouble(xml_ele, "home_pos", 0.0);
		Slave::loadXml(xml_ele);
	}
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
	Motor::~Motor() = default;
	Motor::Motor(const std::string &name, std::uint16_t phy_id
		, double max_pos, double min_pos, double max_vel, double min_vel, double max_acc, double min_acc
		, double max_pos_following_error, double max_vel_following_error, double pos_factor, double pos_offset, double home_pos) :Slave(name, phy_id)
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
		imp_->home_pos_ = home_pos;
	}
	Motor::Motor(const Motor &other) = default;
	Motor& Motor::operator=(const Motor &other) = default;

	struct Controller::Imp { aris::core::SubRefPool<Motor, aris::core::ObjectPool<Slave>> motion_pool_{ nullptr }; };
	auto Controller::motionPool()->aris::core::SubRefPool<Motor, aris::core::ObjectPool<Slave>>& { return imp_->motion_pool_; }
	auto Controller::init()->void
	{
		Master::init();
		imp_->motion_pool_ = aris::core::SubRefPool<Motor, aris::core::ObjectPool<Slave>>(&slavePool());
		motionPool().update();
		for (int i = 0; i < motionPool().size(); ++i)motionPool()[i].imp_->mot_id_ = i;
	}
	auto Controller::motionAtAbs(aris::Size id)->Motor& { return imp_->motion_pool_.at(id); }
	auto Controller::motionAtPhy(aris::Size id)->Motor& { return dynamic_cast<Motor&>(slaveAtPhy(id)); }
	auto Controller::motionAtSla(aris::Size id)->Motor& { return dynamic_cast<Motor&>(slavePool().at(id)); }
	Controller::~Controller() = default;
	Controller::Controller(const std::string &name) :imp_(new Imp), Master(name) 
	{
		this->registerType<aris::core::ObjectPool<Slave, aris::core::Object> >();
	}

	//xml_ele.SetAttribute("max_pos", maxPos());
	//xml_ele.SetAttribute("min_pos", minPos());
	//xml_ele.SetAttribute("max_vel", maxVel());
	//xml_ele.SetAttribute("min_vel", minVel());
	//xml_ele.SetAttribute("max_acc", maxAcc());
	//xml_ele.SetAttribute("min_acc", minAcc());
	//xml_ele.SetAttribute("max_pos_following_error", maxPosFollowingError());
	//xml_ele.SetAttribute("max_vel_following_error", maxVelFollowingError());
	//xml_ele.SetAttribute("pos_factor", posFactor());
	//xml_ele.SetAttribute("pos_offset", posOffset());
	//xml_ele.SetAttribute("home_pos", homePos());

	ARIS_REGISTRATION
	{
		aris::core::class_<Motor>("Motor")
			.inherit<aris::core::Object>()
			.property("max_pos", &Motor::setMaxPos, &Motor::maxPos)
			.property("min_pos", &Motor::setMinPos, &Motor::minPos)
			.property("max_vel", &Motor::setMaxVel, &Motor::maxVel)
			.property("min_vel", &Motor::setMinVel, &Motor::minVel)
			.property("max_acc", &Motor::setMaxAcc, &Motor::maxAcc)
			.property("min_acc", &Motor::setMinAcc, &Motor::minAcc)
			.property("max_pos_following_error", &Motor::setMaxPosFollowingError, &Motor::maxPosFollowingError)
			.property("max_vel_following_error", &Motor::setMaxVelFollowingError, &Motor::maxVelFollowingError)
			.property("pos_factor", &Motor::setPosFactor, &Motor::posFactor)
			.property("pos_offset", &Motor::setPosOffset, &Motor::posOffset)
			.property("home_pos", &Motor::setHomePos, &Motor::homePos)
			;

		aris::core::class_<Controller>("Controller")
			.inherit<Master>()
			;
			//.property("port", &Socket::setPort, &Socket::port);
	}
}
