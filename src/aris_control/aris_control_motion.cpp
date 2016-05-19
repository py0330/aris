#ifdef UNIX
#include <ecrt.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <sys/mman.h>
#endif
#ifdef WIN32
#define rt_printf printf
#endif


#include <string>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>

#include "aris_control_motion.h"


namespace aris
{
	namespace control
	{
		class Motion::Imp
		{
		public:
			std::int32_t input2count_;
			std::int32_t home_count_;
			std::int32_t max_pos_count_;
			std::int32_t min_pos_count_;
			std::int32_t max_vel_count_;
		};
		auto Motion::readUpdate()->void 
		{
			std::int32_t value;
			readPdoIndex(0x6064, 0x0000, value);
			rxData().pos = static_cast<double>(value) / imp_->input2count_;
		}
		auto Motion::writeUpdate()->void 
		{

		}

		Motion::Motion(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :SlaveTemplate(father, id, xml_ele), imp_(new Imp)
		{
			imp_->input2count_ = attributeInt32(xml_ele, "input2count");
			imp_->max_pos_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "max_pos") * imp_->input2count_);
			imp_->min_pos_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "min_pos") * imp_->input2count_);
			imp_->max_vel_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "max_vel") * imp_->input2count_);
			imp_->home_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "home_pos") * imp_->input2count_);
			configSdo(9, static_cast<std::int32_t>(-imp_->home_count_));
		}
	}
}
