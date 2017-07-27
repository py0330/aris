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
			std::int32_t pos_factor_;
			std::int32_t home_count_;
			double max_pos_;
			double min_pos_;
			double max_vel_;
			double pos_offset_{ 0.0 };

			double target_pos_, target_vel_, target_cur_, offset_vel_, offset_cur_;
			std::uint8_t mode_of_operation;

			bool is_fake{ true };//for not all motor can be run
			bool is_waiting_mode{ false };
			int waiting_count_left{ 0 };

			int enable_period{ 0 };
			int home_period{ 0 };
			std::uint8_t running_mode{ 9 };
		};
		auto Motion::maxPos()->double { return imp_->max_pos_; }
		auto Motion::minPos()->double { return imp_->min_pos_; }
		auto Motion::maxVel()->double { return imp_->max_vel_; }
		auto Motion::posOffset()->double { return imp_->pos_offset_; }
		auto Motion::posFactor()->std::int32_t { return imp_->pos_factor_; }
		auto Motion::modeOfOperation()const->std::uint8_t { return imp_->mode_of_operation; }
		auto Motion::targetPos()const->double { return imp_->target_pos_; }
		auto Motion::targetVel()const->double { return imp_->target_vel_; }
		auto Motion::targetCur()const->double { return imp_->target_cur_; }
		auto Motion::offsetVel()const->double { return imp_->offset_vel_; }
		auto Motion::offsetCur()const->double { return imp_->offset_cur_; }
		auto Motion::modeOfDisplay()->std::uint8_t
		{
			std::uint8_t mode;
			readPdoIndex(0x6061, 0x00, mode);
			return mode;
		}
		auto Motion::actualPos()->double 
		{
			std::int32_t pos_count;
			readPdoIndex(0x6064, 0x00, pos_count);
			return static_cast<double>(pos_count) / posFactor() - posOffset();
		}
		auto Motion::actualVel()->double 
		{
			std::int32_t vel_count;
			readPdoIndex(0x606C, 0x00, vel_count);
			return static_cast<double>(vel_count) / posFactor();
		}
		auto Motion::actualCur()->double
		{
			std::int16_t cur_count;
			readPdoIndex(0x6078, 0x00, cur_count);
			return static_cast<double>(cur_count);
		}
		auto Motion::setModeOfOperation(std::uint8_t mode)->void 
		{ 
			imp_->mode_of_operation = mode;
			writePdoIndex(0x6060, 0x00, mode);
		}
		auto Motion::setTargetPos(double pos)->void 
		{
			imp_->target_pos_ = pos;
			writePdoIndex(0x607A, 0x00, static_cast<std::int32_t>((pos + posOffset()) * posFactor())); 
		}
		auto Motion::setTargetVel(double vel)->void 
		{ 
			imp_->target_vel_ = vel;
			writePdoIndex(0x60FF, 0x00, static_cast<std::int32_t>(vel * posFactor())); 
		}
		auto Motion::setTargetCur(double cur)->void 
		{
			imp_->target_cur_ = cur;
			writePdoIndex(0x6071, 0x00, static_cast<std::int16_t>(cur));
		}
		auto Motion::setOffsetVel(double vel)->void 
		{
			imp_->offset_vel_ = vel;
			writePdoIndex(0x60B1, 0x00, static_cast<std::int32_t>(vel * posFactor())); 
		}
		auto Motion::setOffsetCur(double cur)->void 
		{
			imp_->offset_cur_ = cur;
			writePdoIndex(0x60B2, 0x00, static_cast<std::int16_t>(cur));
		}
		auto Motion::disable()->int
		{
			// control word
			// 0x06    0b xxxx xxxx 0xxx 0110    A: transition 2,6,8       Shutdown
			// 0x07    0b xxxx xxxx 0xxx 0111    B: transition 3           Switch ON
			// 0x0F    0b xxxx xxxx 0xxx 1111    C: transition 3           Switch ON
			// 0x00    0b xxxx xxxx 0xxx 0000    D: transition 7,9,10,12   Disable Voltage
			// 0x02    0b xxxx xxxx 0xxx 0000    E: transition 7,10,11     Quick Stop
			// 0x07    0b xxxx xxxx 0xxx 0111    F: transition 5           Disable Operation
			// 0x0F    0b xxxx xxxx 0xxx 1111    G: transition 4,16        Enable Operation
			// 0x80    0b xxxx xxxx 1xxx xxxx    H: transition 15          Fault Reset

			// status word
			// 0x00    0b xxxx xxxx x0xx 0000    A: not ready to switch on     
			// 0x40    0b xxxx xxxx x1xx 0000    B: switch on disabled         
			// 0x21    0b xxxx xxxx x01x 0001    C: ready to switch on         
			// 0x23    0b xxxx xxxx x01x 0011    D: switch on                  
			// 0x27    0b xxxx xxxx x01x 0111    E: operation enabled          
			// 0x07    0b xxxx xxxx x00x 0111    F: quick stop active
			// 0x0F    0b xxxx xxxx x0xx 1111    G: fault reaction active
			// 0x08    0b xxxx xxxx x0xx 1000    H: fault

			// 0x6F    0b 0000 0000 0110 1111
			// 0x4F    0b 0000 0000 0100 1111
			// disable change state to A/B/C/D to E

			std::uint16_t status_word;
			readPdoIndex(0x6041, 0x00, status_word);

			// check status A
			if ((status_word & 0x4F) == 0x00)
			{
				return 1;
			}
			// check status B, now transition 2
			else if ((status_word & 0x4F) == 0x40)
			{
				// transition 2 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x06));
				return 1;
			}
			// check status C, now transition 3
			else if ((status_word & 0x6F) == 0x21)
			{
				// transition 3 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x07));
				return 1;
			}
			// check status D, now keep and return
			else if ((status_word & 0x6F) == 0x23)
			{
				return 0;
			}
			// check status E, now transition 5
			else if ((status_word & 0x6F) == 0x27)
			{
				// transition 5 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x06));
				return 1;
			}
			// check status F, now transition 12
			else if ((status_word & 0x6F) == 0x07)
			{
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
				return 1;
			}
			// check status G, now transition 14
			else if ((status_word & 0x4F) == 0x0F)
			{
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
				return 1;
			}
			// check status H, now transition 13
			else if ((status_word & 0x4F) == 0x08)
			{
				// transition 4 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x80));
				return 1;
			}
			// unknown status
			else
			{
				return -1;
			}
		}
		auto Motion::enable()->int
		{
			// control word
			// 0x06    0b xxxx xxxx 0xxx 0110    A: transition 2,6,8       Shutdown
			// 0x07    0b xxxx xxxx 0xxx 0111    B: transition 3           Switch ON
			// 0x0F    0b xxxx xxxx 0xxx 1111    C: transition 3           Switch ON
			// 0x00    0b xxxx xxxx 0xxx 0000    D: transition 7,9,10,12   Disable Voltage
			// 0x02    0b xxxx xxxx 0xxx 0000    E: transition 7,10,11     Quick Stop
			// 0x07    0b xxxx xxxx 0xxx 0111    F: transition 5           Disable Operation
			// 0x0F    0b xxxx xxxx 0xxx 1111    G: transition 4,16        Enable Operation
			// 0x80    0b xxxx xxxx 1xxx xxxx    H: transition 15          Fault Reset

			// status word
			// 0x00    0b xxxx xxxx x0xx 0000    A: not ready to switch on     
			// 0x40    0b xxxx xxxx x1xx 0000    B: switch on disabled         
			// 0x21    0b xxxx xxxx x01x 0001    C: ready to switch on         
			// 0x23    0b xxxx xxxx x01x 0011    D: switch on                  
			// 0x27    0b xxxx xxxx x01x 0111    E: operation enabled          
			// 0x07    0b xxxx xxxx x00x 0111    F: quick stop active
			// 0x0F    0b xxxx xxxx x0xx 1111    G: fault reaction active
			// 0x08    0b xxxx xxxx x0xx 1000    H: fault

			// 0x6F    0b 0000 0000 0110 1111
			// 0x4F    0b 0000 0000 0100 1111
			// enable change state to A/B/C/D to E
			std::uint16_t status_word;
			readPdoIndex(0x6041, 0x00, status_word);

			// check status A
			if ((status_word & 0x4F) == 0x00)
			{
				return 1;
			}
			// check status B, now transition 2
			else if ((status_word & 0x4F) == 0x40)
			{
				// transition 2 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x06));
				return 2;
			}
			// check status C, now transition 3
			else if ((status_word & 0x6F) == 0x21)
			{
				// transition 3 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x07));
				return 3;
			}
			// check status D, now transition 4
			else if ((status_word & 0x6F) == 0x23)
			{
				// transition 4 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x0F));
				imp_->waiting_count_left = 20;
				
				// check mode to set correct pos, vel or cur //
				switch (modeOfDisplay())
				{
				case 0x08: setTargetPos(actualPos()); break;
				case 0x09: setTargetVel(0.0); break;
				case 0x10: setTargetPos(0.0); break;
				default: setTargetPos(actualPos()); setTargetVel(0.0); setTargetVel(0.0);
				}

				return 4;
			}
			// check status E, now keep status
			else if ((status_word & 0x6F) == 0x27)
			{
				// check if need wait //
				if (--imp_->waiting_count_left > 0) return 5;
				// now return normal
				else return 0;
			}
			// check status F, now transition 12
			else if ((status_word & 0x6F) == 0x07)
			{
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
				return 6;
			}
			// check status G, now transition 14
			else if ((status_word & 0x4F) == 0x0F)
			{
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
				return 7;
			}
			// check status H, now transition 13
			else if ((status_word & 0x4F) == 0x08)
			{
				// transition 4 //
				writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x80));
				return 8;
			}
			// unknown status
			else
			{
				return -1;
			}
		}
		auto Motion::home()->int
		{
			// control word
			// 0x06    0b xxxx xxxx 0xxx 0110    A: transition 2,6,8       Shutdown
			// 0x07    0b xxxx xxxx 0xxx 0111    B: transition 3           Switch ON
			// 0x0F    0b xxxx xxxx 0xxx 1111    C: transition 3           Switch ON
			// 0x00    0b xxxx xxxx 0xxx 0000    D: transition 7,9,10,12   Disable Voltage
			// 0x02    0b xxxx xxxx 0xxx 0000    E: transition 7,10,11     Quick Stop
			// 0x07    0b xxxx xxxx 0xxx 0111    F: transition 5           Disable Operation
			// 0x0F    0b xxxx xxxx 0xxx 1111    G: transition 4,16        Enable Operation
			// 0x80    0b xxxx xxxx 1xxx xxxx    H: transition 15          Fault Reset

			// status word
			// 0x00    0b xxxx xxxx x0xx 0000    A: not ready to switch on     
			// 0x40    0b xxxx xxxx x1xx 0000    B: switch on disabled         
			// 0x21    0b xxxx xxxx x01x 0001    C: ready to switch on         
			// 0x23    0b xxxx xxxx x01x 0011    D: switch on                  
			// 0x27    0b xxxx xxxx x01x 0111    E: operation enabled          
			// 0x07    0b xxxx xxxx x00x 0111    F: quick stop active
			// 0x0F    0b xxxx xxxx x0xx 1111    G: fault reaction active
			// 0x08    0b xxxx xxxx x0xx 1000    H: fault

			// 0x6F    0b 0000 0000 0110 1111
			// 0x4F    0b 0000 0000 0100 1111
			// enable change state to A/B/C/D to E
			//if (is_waiting_mode)
			//{
			//	auto ret = this->enable(running_mode);
			//	is_waiting_mode = (ret == 0 ? false : true);
			//	return ret;
			//}

			//std::uint16_t statusWord;
			//pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);
			//std::uint8_t modeRead;
			//pFather->readPdoIndex(MODEOPERATIONDIS, 0x00, modeRead);
			//int motorState = (statusWord & 0x3400);

			//if (modeRead != Motion::HOME_MODE) {
			//	pFather->writePdoIndex(MODEOPERATION, 0x00, static_cast<std::uint8_t>(Motion::HOME_MODE));
			//	return Motion::MODE_CHANGE;
			//}
			//else if (motorState == 0x0400) {
			//	// homing procedure is interrupted or not started
			//	if (home_period<10) {
			//		// write 15 to controlword, make the bit4 equal to 0, 10 times
			//		pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x1F));
			//		home_period++;
			//		return Motion::NOT_START;
			//	}
			//	else if (home_period<20)
			//	{
			//		pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0F));
			//		home_period++;
			//		return Motion::NOT_START;
			//	}
			//	else
			//	{
			//		home_period = 0;
			//		return Motion::NOT_START;
			//	}
			//}
			//else if (motorState == 0x0000) {
			//	//in progress
			//	home_period = 0;
			//	pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x1F));
			//	pFather->writePdoIndex(TARGETPOSITION, 0x00, home_count_);
			//	return Motion::EXECUTING;
			//}
			//else if (motorState == 0x2000 || motorState == 0x2400)
			//{
			//	//homing error occurred, velocity is not 0 , or homing error occurred, velocity is 0, should halt
			//	pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0100));
			//	home_period = 0;
			//	return Motion::HOME_ERROR;
			//}
			//else if (motorState == 0x1400)
			//{
			//	//homing procedure is completed successfully, home method 35<->0x1400,
			//	pFather->writePdoIndex(TARGETPOSITION, 0x00, home_count_);
			//	home_period = 0;
			//	is_waiting_mode = true;
			//	return Motion::EXECUTING;
			//}
			//else {
			//	//other statusworld
			//	home_period = 0;
			//	return Motion::EXECUTING;
			//}

			return 0;
		}
		auto Motion::mode(std::uint8_t md)->int
		{
			setModeOfOperation(md);
			return md == modeOfOperation() ? 0 : 1;
		}
		Motion::~Motion() = default;
		Motion::Motion(Object &father, const aris::core::XmlElement &xml_ele) :Slave(father, xml_ele), imp_(new Imp)
		{
			imp_->pos_factor_ = attributeInt32(xml_ele, "pos_factor");
			imp_->max_pos_ = attributeDouble(xml_ele, "max_pos");
			imp_->min_pos_ = attributeDouble(xml_ele, "min_pos");
			imp_->max_vel_ = attributeDouble(xml_ele, "max_vel");
			imp_->pos_offset_ = attributeDouble(xml_ele, "pos_offset", 0.0);
			imp_->home_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "home_pos") * imp_->pos_factor_);
			// this should be 
			//configSdoIndex(Imp::HOMEOFFSET, 0x00, static_cast<std::int32_t>(-imp_->home_count_));
		}
		Motion::Motion(const std::string &name, const SlaveType &slave_type, std::int32_t input_ratio, double max_pos, double min_pos, double max_vel, double home_pos , double pos_offset)
			:Slave(name, slave_type), imp_(new Imp)
		{
			imp_->pos_factor_ = input_ratio;
			imp_->max_pos_ = max_pos;
			imp_->min_pos_ = min_pos;
			imp_->max_vel_ = max_vel;
			imp_->home_count_ = static_cast<std::int32_t>(home_pos * imp_->pos_factor_);
			imp_->pos_offset_ = pos_offset;

			//configSdoIndex(Imp::HOMEOFFSET, 0x00, static_cast<std::int32_t>(-imp_->home_count_));
		}
    }
}
