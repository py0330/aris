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
				CONTROLWORD = 0x6040,
				TARGETPOSITION = 0x607A,
				TARGETVELOCITY = 0x60FF,
				TARGETTORQUE = 0x6071,
				MODEOPERATION = 0x6060,

				// following is 0x1a00 TX //
				STATUSWORD = 0x6041,
				ACTUALPOSITION = 0x6064,
				ACTUALVELOCITY = 0x606C,
				ACTUALTORQUE = 0x6078,
				MODEOPERATIONDIS = 0x6061,
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

				std::uint16_t statusWord;
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);

				std::uint8_t modeRead;
				father->readPdoIndex(MODEOPERATIONDIS, 0x00, modeRead);

				int motorState = (statusWord & 0x006F);

				if (motorState != 0x0027) {
					switch (mode)
					{
					case Motion::POSITION:
						//targetposition should be equal to actualposition
						std::int32_t actualPosition;
						father->readPdoIndex(ACTUALPOSITION, 0x00, actualPosition);
						father->writePdoIndex(TARGETPOSITION, 0x00, actualPosition);
						break;
					case Motion::VELOCITY:
						//velocity loop to set velocity of 0
						father->writePdoIndex(TARGETVELOCITY, 0x00, static_cast<std::int32_t>(0));
						break;
					case Motion::TORQUE:
						father->writePdoIndex(TARGETTORQUE, 0x00, static_cast<std::int16_t>(0));
						break;
					}
				}

				if (modeRead != mode)
				{
					/*state is RUNNING, now change it to desired mode*/
					father->writePdoIndex(MODEOPERATION, 0x00, static_cast<std::uint8_t>(mode));
					return Motion::MODE_CHANGE;
				}

				if (motorState == 0x0060 || motorState == 0x0040)
				{
					// switch on disable //
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x06));
					return Motion::EXECUTING;
				}
				else if (motorState == 0x0021)
				{
					// ready to switch on //
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x07));
					return Motion::EXECUTING;
				}
				else if (motorState == 0x0023)
				{
					// switch on //
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0F));
					return Motion::EXECUTING;
				}
				else if (motorState == 0x0027)
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
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x80));
					return Motion::EXE_FAULT;
				}
			}
			std::int16_t disable()
			{
				is_fake = false;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);

				int motorState = (statusWord & 0x006F);
				if (motorState == 0x0021)
				{
					/*alReady disabled*/
					return Motion::SUCCESS;
				}
				else if (motorState == 0x0023 || motorState == 0x0027 || motorState == 0x0060 || motorState == 0x0040)
				{
					/*try to disable*/
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x06));
					return Motion::EXECUTING;
				}
				else
				{
					/*the motor is in fault*/
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x80));
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
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);
				std::uint8_t modeRead;
				father->readPdoIndex(MODEOPERATIONDIS, 0x00, modeRead);
				int motorState = (statusWord & 0x3400);

				if (modeRead != Motion::HOME_MODE) {
					father->writePdoIndex(MODEOPERATION, 0x00, static_cast<std::uint8_t>(Motion::HOME_MODE));
					return Motion::MODE_CHANGE;
				}
				else if (motorState == 0x0400) {
					// homing procedure is interrupted or not started
					if (home_period < 10) {
						// write 15 to controlword, make the bit4 equal to 0, 10 times
						father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x1F));
						home_period++;
						return Motion::NOT_START;
					}
					else if (home_period < 20)
					{
						father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0F));
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
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x1F));
					father->writePdoIndex(TARGETPOSITION, 0x00, home_count_);
					return Motion::EXECUTING;
				}
				else if (motorState == 0x2000 || motorState == 0x2400)
				{
					//homing error occurred, velocity is not 0 , or homing error occurred, velocity is 0, should halt
					father->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0100));
					home_period = 0;
					return Motion::HOME_ERROR;
				}
				else if (motorState == 0x1400)
				{
					//homing procedure is completed successfully, home method 35<->0x1400,
					father->writePdoIndex(TARGETPOSITION, 0x00, home_count_);
					home_period = 0;
					is_waiting_mode = true;
					return Motion::EXECUTING;
				}
				else {
					//other statusworld
					home_period = 0;
					return Motion::EXECUTING;
				}
			}
			std::int16_t runPos(const std::int32_t pos, const std::int32_t velocity_offset, const std::int16_t torque_offset)
			{
				if (is_fake) return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);
				int motorState = (statusWord & 0x006F);

				std::uint8_t modeRead;
				father->readPdoIndex(MODEOPERATIONDIS, 0x00, modeRead);
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
					father->writePdoIndex(TARGETPOSITION, 0x00, pos);
					return Motion::SUCCESS;
				}
			}
			std::int16_t runVel(const std::int32_t vel, const std::int32_t velocity_offset, const std::int16_t torque_offset)
			{
				if (is_fake)return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);
				int motorState = (statusWord & 0x006F);

				std::uint8_t modeRead;
				father->readPdoIndex(MODEOPERATIONDIS, 0x00, modeRead);
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
					father->writePdoIndex(TARGETVELOCITY, 0x00, vel);
					return Motion::SUCCESS;
				}
			}
			std::int16_t runTor(const std::int16_t tor, const std::int16_t torque_offset)
			{
				if (is_fake)return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);
				int motorState = (statusWord & 0x006F);

				std::uint8_t modeRead;
				father->readPdoIndex(MODEOPERATIONDIS, 0x00, modeRead);
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
					father->writePdoIndex(TARGETTORQUE, 0x00, tor);
					return Motion::SUCCESS;
				}
			}
			std::int32_t pos()
			{
				if (is_fake)return static_cast<std::int32_t>((father->txData().target_pos + pos_offset_) * father->pos2countRatio());

				std::int32_t pos;
				father->readPdoIndex(ACTUALPOSITION, 0x00, pos);
				return pos;
			}
			std::int32_t vel()
			{
				std::int32_t vel;
				father->readPdoIndex(ACTUALVELOCITY, 0x00, vel);
				return vel;
			}
			std::int32_t tor() { std::int16_t tor; father->readPdoIndex(ACTUALTORQUE, 0x00, tor); return tor; }
			std::uint8_t modeDisplay()
			{
				if (is_fake)return father->txData().mode;

				std::uint8_t mode;
				father->readPdoIndex(MODEOPERATIONDIS, 0x00, mode);
				return mode;
			}
			std::int8_t hasFault()
			{
				if (is_fake)return 0;

				std::uint16_t statusWord;
				father->readPdoIndex(STATUSWORD, 0x00, statusWord);
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
		auto Motion::logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void
		{
			auto &rx_motiondata = static_cast<const RxType &>(rx_data);
			auto &tx_motiondata = static_cast<const TxType &>(tx_data);
			file << rx_motiondata.feedback_pos << " " << tx_motiondata.target_pos << " " << rx_motiondata.feedback_vel << " " << tx_motiondata.target_vel << " " << rx_motiondata.feedback_tor;
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
    }
}
