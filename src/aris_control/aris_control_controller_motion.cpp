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
			double max_pos_, min_pos_, max_vel_, max_acc_;
			double pos_factor_, pos_offset_;
			double home_pos_;
			aris::Size mot_id_;
		};
		auto Motion::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Slave::saveXml(xml_ele);
			xml_ele.SetAttribute("max_pos", maxPos());
			xml_ele.SetAttribute("min_pos", minPos());
			xml_ele.SetAttribute("max_vel", maxVel());
			xml_ele.SetAttribute("max_acc", maxAcc());
			xml_ele.SetAttribute("pos_factor", posFactor());
			xml_ele.SetAttribute("pos_offset", posOffset());
			xml_ele.SetAttribute("home_pos", homePos());
		}
		auto Motion::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			imp_->max_pos_ = attributeDouble(xml_ele, "max_pos", 0.0);
			imp_->min_pos_ = attributeDouble(xml_ele, "min_pos", 0.0);
			imp_->max_vel_ = attributeDouble(xml_ele, "max_vel", 0.0);
			imp_->max_acc_ = attributeDouble(xml_ele, "max_acc", 0.0);
			imp_->pos_factor_ = attributeDouble(xml_ele, "pos_factor", 1.0);
			imp_->pos_offset_ = attributeDouble(xml_ele, "pos_offset", 0.0);
			imp_->home_pos_ = attributeDouble(xml_ele, "home_pos", 0.0);
			Slave::loadXml(xml_ele);
		}
		auto Motion::motId()const->aris::Size { return imp_->mot_id_; }
		auto Motion::maxPos()const->double { return imp_->max_pos_; }
		auto Motion::minPos()const->double { return imp_->min_pos_; }
		auto Motion::maxVel()const->double { return imp_->max_vel_; }
		auto Motion::maxAcc()const->double { return imp_->max_acc_; }
		auto Motion::posOffset()const->double { return imp_->pos_offset_; }
		auto Motion::posFactor()const->double { return imp_->pos_factor_; }
		auto Motion::homePos()const->double { return imp_->home_pos_; }
		Motion::~Motion() = default;
		Motion::Motion(const std::string &name, const SlaveType *st, std::uint16_t phy_id, double max_pos, double min_pos, double max_vel, double max_acc, double pos_factor, double pos_offset, double home_pos):Slave(name, st, phy_id)
		{
			imp_->max_pos_ = max_pos;
			imp_->min_pos_ = min_pos;
			imp_->max_vel_ = max_vel;
			imp_->max_acc_ = max_acc;
			imp_->pos_factor_ = pos_factor;
			imp_->pos_offset_ = pos_offset;
			imp_->home_pos_ = home_pos;
		}

		struct Controller::Imp{	aris::core::RefPool<Motion> motion_pool_;};
		auto Controller::motionPool()->aris::core::RefPool<Motion>& { return imp_->motion_pool_; }
		auto Controller::init()->void
		{
			motionPool().clear();
			for (auto &s : slavePool()) 
			{
				if (dynamic_cast<Motion*>(&s))motionPool().push_back_ptr(dynamic_cast<Motion*>(&s));
				motionPool().back().imp_->mot_id_ = motionPool().size() - 1;
			}
		}
		auto Controller::motionAtAbs(aris::Size id)->Motion& { return imp_->motion_pool_.at(id); }
		auto Controller::motionAtPhy(aris::Size id)->Motion& { return dynamic_cast<Motion&>(slaveAtPhy(id)); }
		auto Controller::motionAtSla(aris::Size id)->Motion& { return dynamic_cast<Motion&>(slavePool().at(id)); }
		Controller::~Controller() = default;
		Controller::Controller(const std::string &name) :imp_(new Imp), Master(name) { }
    }
}
