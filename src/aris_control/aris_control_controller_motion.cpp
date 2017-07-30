#include <string>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <thread>
#include <future>

#include "aris_control_controller_motion.h"


namespace aris
{
	namespace control
	{
		struct Motion::Imp 
		{
			double max_pos_;
			double min_pos_;
			double max_vel_;
			double pos_offset_;
			double pos_factor_;
			double home_pos;
		};
		auto Motion::maxPos()->double { return imp_->max_pos_; }
		auto Motion::minPos()->double { return imp_->min_pos_; }
		auto Motion::maxVel()->double { return imp_->max_vel_; }
		auto Motion::posOffset()->double { return imp_->pos_offset_; }
		auto Motion::posFactor()->double { return imp_->pos_factor_; }
		Motion::~Motion() = default;
		Motion::Motion(const std::string &name, std::int32_t input_ratio, double max_pos, double min_pos, double max_vel, double home_pos, double pos_offset) :Slave(name), imp_(new Imp)
		{
			imp_->pos_factor_ = input_ratio;
			imp_->max_pos_ = max_pos;
			imp_->min_pos_ = min_pos;
			imp_->max_vel_ = max_vel;
			imp_->home_pos = home_pos;
			imp_->pos_offset_ = pos_offset;
		}
		Motion::Motion(Object &father, const aris::core::XmlElement &xml_ele) : Slave(father, xml_ele), imp_(new Imp) {}

		struct Controller::Imp{	aris::core::RefPool<Motion> motion_pool_; };
		auto Controller::motionPool()->aris::core::RefPool<Motion>& { return imp_->motion_pool_; }
		auto Controller::init()->void
		{
			motionPool().clear();
			for (auto &s : slavePool())if (dynamic_cast<Motion*>(&s))motionPool().push_back_ptr(dynamic_cast<Motion*>(&s));
		}
		Controller::~Controller() = default;
		Controller::Controller() :imp_(new Imp) {}
    }
}
