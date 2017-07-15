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
			enum PDO_Entry
			{
				// following is 0x1600 RX//
				CONTROL_WORD = 0x6040,
				MODE_OF_OPERATION = 0x6060,
				TARGET_POSITION = 0x607A,
				TARGET_VELOCITY = 0x60FF,
				TARGET_TORQUE = 0x6071,


				// following is 0x1a00 TX //
				STATUS_WORD = 0x6041,
				MODE_OF_DISPLAY = 0x6061,
				ACTUAL_POSITION = 0x6064,
				ACTUAL_VELOCITY = 0x606C,
				ACTUAL_TORQUE = 0x6078,
			};
			enum SDO_Entry_Index
			{
				HOMEOFFSET = 0x607C,
			};

			Imp(Motion *mot) :father(mot) {}
			~Imp() = default;

			std::int16_t enable(const std::uint8_t mode)
			{
				is_fake = false;

				std::uint16_t status_word;
				father->readPdoIndex(STATUS_WORD, 0x00, status_word);

				std::uint8_t mode_read;
				father->readPdoIndex(MODE_OF_DISPLAY, 0x00, mode_read);

				int state = (status_word & 0x006F);

				if (state != 0x0027) {
					switch (mode)
					{
					case Motion::POSITION:
						// target position should be equal to actual position
						std::int32_t actualPosition;
						father->readPdoIndex(ACTUAL_POSITION, 0x00, actualPosition);
						father->writePdoIndex(TARGET_POSITION, 0x00, actualPosition);
						break;
					case Motion::VELOCITY:
						// velocity loop to set velocity of 0
						father->writePdoIndex(TARGET_VELOCITY, 0x00, static_cast<std::int32_t>(0));
						break;
					case Motion::TORQUE:
						father->writePdoIndex(TARGET_TORQUE, 0x00, static_cast<std::int16_t>(0));
						break;
					}
				}

				if (mode_read != mode)
				{
					// state is RUNNING, now change it to desired mode //
					father->writePdoIndex(MODE_OF_OPERATION, 0x00, static_cast<std::uint8_t>(mode));
					return Motion::MODE_CHANGE;
				}

				if (state == 0x0060 || state == 0x0040)
				{
					// switch on disable //
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x06));
					return Motion::EXECUTING;
				}
				else if (state == 0x0021)
				{
					// ready to switch on //
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x07));
					return Motion::EXECUTING;
				}
				else if (state == 0x0023)
				{
					// switch on //
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x0F));
					return Motion::EXECUTING;
				}
				else if (state == 0x0027)
				{
					// successfull, but still need to wait for 10 more cycles to make it stable //
					if (++enable_period >= 10)
					{
						running_mode = mode;
						enable_period = 0;
						return Motion::SUCCESS;
					}
					else
					{
						return Motion::EXECUTING;
					}
				}
				else
				{
					// the motor is in fault //
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x80));
					return Motion::EXE_FAULT;
				}
			}
			std::int16_t disable()
			{
				is_fake = false;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUS_WORD, 0x00, statusWord);

				int motorState = (statusWord & 0x006F);
				if (motorState == 0x0021)
				{
					// alReady disabled //
					return Motion::SUCCESS;
				}
				else if (motorState == 0x0023 || motorState == 0x0027 || motorState == 0x0060 || motorState == 0x0040)
				{
					// try to disable //
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x06));
					return Motion::EXECUTING;
				}
				else
				{
					// the motor is in fault //
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x80));
					return Motion::EXE_FAULT;
				}

			}
			std::int16_t home()
			{
				if (is_waiting_mode)
				{
					auto ret = this->enable(running_mode);
					is_waiting_mode = (ret == 0 ? false : true);
					return ret;
				}

				std::uint16_t statusWord;
				father->readPdoIndex(STATUS_WORD, 0x00, statusWord);
				std::uint8_t modeRead;
				father->readPdoIndex(MODE_OF_DISPLAY, 0x00, modeRead);
				int motorState = (statusWord & 0x3400);

				if (modeRead != Motion::HOME_MODE) {
					father->writePdoIndex(MODE_OF_OPERATION, 0x00, static_cast<std::uint8_t>(Motion::HOME_MODE));
					return Motion::MODE_CHANGE;
				}
				else if (motorState == 0x0400) {
					// homing procedure is interrupted or not started
					if (home_period < 10) {
						// write 15 to controlword, make the bit4 equal to 0, 10 times
						father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x1F));
						home_period++;
						return Motion::NOT_START;
					}
					else if (home_period < 20)
					{
						father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x0F));
						home_period++;
						return Motion::NOT_START;
					}
					else
					{
						home_period = 0;
						return Motion::NOT_START;
					}
				}
				else if (motorState == 0x0000) {
					//in progress
					home_period = 0;
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x1F));
					father->writePdoIndex(TARGET_POSITION, 0x00, home_count_);
					return Motion::EXECUTING;
				}
				else if (motorState == 0x2000 || motorState == 0x2400)
				{
					// homing error occurred, velocity is not 0 , or homing error occurred, velocity is 0, should halt
					father->writePdoIndex(CONTROL_WORD, 0x00, static_cast<std::uint16_t>(0x0100));
					home_period = 0;
					return Motion::HOME_ERROR;
				}
				else if (motorState == 0x1400)
				{
					// homing procedure is completed successfully, home method 35<->0x1400, //
					father->writePdoIndex(TARGET_POSITION, 0x00, home_count_);
					home_period = 0;
					is_waiting_mode = true;
					return Motion::EXECUTING;
				}
				else {
					// other status word //
					home_period = 0;
					return Motion::EXECUTING;
				}
			}
			std::int16_t runPos(const std::int32_t pos, const std::int32_t velocity_offset, const std::int16_t torque_offset)
			{
				if (is_fake) return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUS_WORD, 0x00, statusWord);
				int motorState = (statusWord & 0x006F);

				std::uint8_t modeRead;
				father->readPdoIndex(MODE_OF_DISPLAY, 0x00, modeRead);
				if (motorState != 0x0027)
				{
					return Motion::ENABLE_ERROR;
				}
				else if (modeRead != Motion::POSITION)
				{
					return Motion::MODE_ERROR;
				}
				else
				{
					father->writePdoIndex(TARGET_POSITION, 0x00, pos);
					return Motion::SUCCESS;
				}
			}
			std::int16_t runVel(const std::int32_t vel, const std::int32_t velocity_offset, const std::int16_t torque_offset)
			{
				if (is_fake)return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUS_WORD, 0x00, statusWord);
				int motorState = (statusWord & 0x006F);

				std::uint8_t modeRead;
				father->readPdoIndex(MODE_OF_DISPLAY, 0x00, modeRead);
				if (motorState != 0x0027)
				{
					return Motion::ENABLE_ERROR;
				}
				else if (modeRead != Motion::VELOCITY)
				{
					return Motion::MODE_ERROR;
				}
				else
				{
					father->writePdoIndex(TARGET_VELOCITY, 0x00, vel);
					return Motion::SUCCESS;
				}
			}
			std::int16_t runTor(const std::int16_t tor, const std::int16_t torque_offset)
			{
				if (is_fake)return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUS_WORD, 0x00, statusWord);
				int motorState = (statusWord & 0x006F);

				std::uint8_t modeRead;
				father->readPdoIndex(MODE_OF_DISPLAY, 0x00, modeRead);
				if (motorState != 0x0027)
				{
					return Motion::ENABLE_ERROR;
				}
				else if (modeRead != Motion::TORQUE)
				{
					return Motion::MODE_ERROR;
				}
				else
				{
					father->writePdoIndex(TARGET_TORQUE, 0x00, tor);
					return Motion::SUCCESS;
				}
			}
			std::int32_t pos()
			{
				if (is_fake)return static_cast<std::int32_t>((father->txData().target_pos + pos_offset_) * father->pos2countRatio());

				std::int32_t pos;
				father->readPdoIndex(ACTUAL_POSITION, 0x00, pos);
				return pos;
			}
			std::int32_t vel()
			{
				std::int32_t vel;
				father->readPdoIndex(ACTUAL_VELOCITY, 0x00, vel);
				return vel;
			}
			std::int32_t tor() { std::int16_t tor; father->readPdoIndex(ACTUAL_TORQUE, 0x00, tor); return tor; }
			std::uint8_t modeDisplay()
			{
				if (is_fake)return father->txData().mode;

				std::uint8_t mode;
				father->readPdoIndex(MODE_OF_DISPLAY, 0x00, mode);
				return mode;
			}
			std::int8_t hasFault()
			{
				if (is_fake)return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUS_WORD, 0x00, statusWord);
				int motorState = (statusWord & 0x0088);
				if (motorState == 0x0000)
					//no fault and no warning
					return 0;
				else if (motorState == 0x0008)
					//fault
					return -1;
				else if (motorState == 0x0080)
					//warning
					return -2;
				else
					//fault and warning
					return -3;
			}

			std::int32_t input2count_;
			std::int32_t home_count_;
			double max_pos_;
			double min_pos_;
			double max_vel_;
			double pos_offset_{ 0.0 };

			Motion *father;

			bool is_fake{ true };//for not all motor can be run
			bool is_waiting_mode{ false };

			int enable_period{ 0 };
			int home_period{ 0 };
			std::uint8_t running_mode{ 9 };
		};
		auto Motion::readUpdate()->void
		{
			rxData().feedback_tor = static_cast<double>(imp_->tor());
			rxData().feedback_pos = static_cast<double>(imp_->pos()) / imp_->input2count_ - posOffset();
			rxData().feedback_vel = static_cast<double>(imp_->vel()) / imp_->input2count_;
			rxData().fault_warning = imp_->hasFault();
			rxData().mode = imp_->modeDisplay();
		}
		auto Motion::writeUpdate()->void
		{
			switch (txData().cmd)
			{
			case IDLE:
				rxData().ret = 0;
				return;
			case ENABLE:
				rxData().ret = imp_->enable(txData().mode);
				return;
			case DISABLE:
				rxData().ret = imp_->disable();
				return;
			case HOME:
				rxData().ret = imp_->home();
				return;
			case RUN:
				switch (txData().mode)
				{
				case POSITION:
					rxData().ret = imp_->runPos(static_cast<std::int32_t>((txData().target_pos + posOffset()) * imp_->input2count_), static_cast<std::int32_t>(txData().vel_offset * imp_->input2count_), static_cast<std::int16_t>(txData().tor_offset));
					return;
				case VELOCITY:
					rxData().ret = imp_->runVel(static_cast<std::int32_t>(txData().target_vel * imp_->input2count_), static_cast<std::int32_t>(txData().vel_offset * imp_->input2count_), static_cast<std::int16_t>(txData().tor_offset));
					return;
				case TORQUE:
					rxData().ret = imp_->runTor(static_cast<std::int16_t>(txData().target_tor), static_cast<std::int16_t>(txData().tor_offset));
					return;
				default:
					rxData().ret = -1;
					return;
				}
			default:
				rxData().ret = -1;
				return;
			}

		}
		auto Motion::maxPos()->double { return imp_->max_pos_; }
		auto Motion::minPos()->double { return imp_->min_pos_; }
		auto Motion::maxVel()->double { return imp_->max_vel_; }
		auto Motion::posOffset()->double { return imp_->pos_offset_; }
		auto Motion::pos2countRatio()->std::int32_t { return imp_->input2count_; }
		Motion::~Motion() = default;
		Motion::Motion(Object &father, const aris::core::XmlElement &xml_ele) :SlaveTemplate(father, xml_ele), imp_(new Motion::Imp(this))
		{
			imp_->input2count_ = attributeInt32(xml_ele, "input2count");
			imp_->max_pos_ = attributeDouble(xml_ele, "max_pos");
			imp_->min_pos_ = attributeDouble(xml_ele, "min_pos");
			imp_->max_vel_ = attributeDouble(xml_ele, "max_vel");
			imp_->pos_offset_ = attributeDouble(xml_ele, "pos_offset", 0.0);
			imp_->home_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "home_pos") * imp_->input2count_);
			configSdoIndex(Imp::HOMEOFFSET, 0x00, static_cast<std::int32_t>(-imp_->home_count_));
		}

		class MyMotion::Imp
		{
		public:
			enum PDO_Entry
			{
				// following is 0x1600 RX//
				CONTROL_WORD = 0x6040,
				MODE_OF_OPERATION = 0x6060,
				TARGET_POSITION = 0x607A,
				TARGET_VELOCITY = 0x60FF,
				TARGET_TORQUE = 0x6071,


				// following is 0x1a00 TX //
				STATUS_WORD = 0x6041,
				MODE_OF_DISPLAY = 0x6061,
				ACTUAL_POSITION = 0x6064,
				ACTUAL_VELOCITY = 0x606C,
				ACTUAL_TORQUE = 0x6078,
			};
			enum SDO_Entry_Index
			{
				HOMEOFFSET = 0x607C,
			};

			Imp(MyMotion *mot) :father(mot) {}
			~Imp() = default;

			std::int32_t input2count_;
			std::int32_t home_count_;
			double max_pos_;
			double min_pos_;
			double max_vel_;
			double pos_offset_{ 0.0 };

			MyMotion *father;

			bool is_fake{ true };//for not all motor can be run
			bool is_waiting_mode{ false };
			int waiting_count_left{ 0 };

			int enable_period{ 0 };
			int home_period{ 0 };
			std::uint8_t running_mode{ 9 };
		};
		auto MyMotion::readUpdate()->void
		{

		}
		auto MyMotion::writeUpdate()->void
		{


		}
		auto MyMotion::maxPos()->double { return imp_->max_pos_; }
		auto MyMotion::minPos()->double { return imp_->min_pos_; }
		auto MyMotion::maxVel()->double { return imp_->max_vel_; }
		auto MyMotion::posOffset()->double { return imp_->pos_offset_; }
		auto MyMotion::pos2countRatio()->std::int32_t { return imp_->input2count_; }

		auto MyMotion::actualPos()->double 
		{
			std::int32_t pos_count;
			readPdoIndex(0x6064, 0x00, pos_count);
			return static_cast<double>(pos_count) / imp_->input2count_ - posOffset();
		}
		auto MyMotion::actualVel()->double 
		{
			std::int32_t vel_count;
			readPdoIndex(0x606C, 0x00, vel_count);
			return static_cast<double>(vel_count) / imp_->input2count_;
		}
		auto MyMotion::actualCur()->double
		{
			std::int16_t cur_count;
			readPdoIndex(0x6078, 0x00, cur_count);
			return static_cast<double>(cur_count);
		}
		auto MyMotion::disable()->int
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
		auto MyMotion::enable(std::uint8_t mode)->int
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
				std::uint8_t mode_read;
				readPdoIndex(0x6061, 0x00, mode_read);

				// mode is correct, now transition 4 //
				if (mode_read == mode) 
				{
					writePdoIndex(0x6040, 0x00, static_cast<std::uint16_t>(0x0F));

					// write postion //
					std::int32_t actual_pos;
					readPdoIndex(0x6064, 0x00, actual_pos);
					writePdoIndex(0x607A, 0x00, actual_pos);

					imp_->waiting_count_left = 20;
				}
				// change mode //
				else
				{
					writePdoIndex(0x6060, 0x00, mode);
				}

				return 4;
			}
			// check status E, now keep status
			else if ((status_word & 0x6F) == 0x27)
			{
				std::uint8_t mode_read;
				readPdoIndex(0x6061, 0x00, mode_read);

				// check if mode is correct //
				if (mode_read != mode) return -1;
				// check if need wait //
				else if (--imp_->waiting_count_left > 0) return 5;
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
		auto MyMotion::setTargetPos(double pos)->int
		{
			writePdoIndex(0x607A, 0x00, static_cast<std::int32_t>((pos + posOffset()) *pos2countRatio()));
			return 0;
		}
		auto MyMotion::setTargetVel(double vel)->int
		{
			writePdoIndex(0x60FF, 0x00, static_cast<std::int32_t>(vel * pos2countRatio()));
			return 0;
		}
		auto MyMotion::setTargetCur(double cur)->int
		{
			writePdoIndex(0x6071, 0x00, static_cast<std::int32_t>(cur));
			return 0;
		}

		MyMotion::~MyMotion() = default;
		MyMotion::MyMotion(Object &father, const aris::core::XmlElement &xml_ele) :Slave(father, xml_ele), imp_(new Imp(this))
		{
			imp_->input2count_ = attributeInt32(xml_ele, "input2count");
			imp_->max_pos_ = attributeDouble(xml_ele, "max_pos");
			imp_->min_pos_ = attributeDouble(xml_ele, "min_pos");
			imp_->max_vel_ = attributeDouble(xml_ele, "max_vel");
			imp_->pos_offset_ = attributeDouble(xml_ele, "pos_offset", 0.0);
			imp_->home_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "home_pos") * imp_->input2count_);
			configSdoIndex(Imp::HOMEOFFSET, 0x00, static_cast<std::int32_t>(-imp_->home_count_));
		}
		MyMotion::MyMotion(const std::string &name, const SlaveType &slave_type, std::int32_t input_ratio, double max_pos, double min_pos, double max_vel, double home_pos , double pos_offset)
			:Slave(name, slave_type), imp_(new Imp(this))
		{
			imp_->input2count_ = input_ratio;
			imp_->max_pos_ = max_pos;
			imp_->min_pos_ = min_pos;
			imp_->max_vel_ = max_vel;
			imp_->home_count_ = static_cast<std::int32_t>(home_pos * imp_->input2count_);
			imp_->pos_offset_ = pos_offset;

			//configSdoIndex(Imp::HOMEOFFSET, 0x00, static_cast<std::int32_t>(-imp_->home_count_));
		}
    }
}
