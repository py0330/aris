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


namespace Aris
{
	namespace Control
	{
		class EthercatMotion::Imp 
		{
		public:
			Imp(EthercatMotion *mot) :pFather(mot) {};
			~Imp() = default;

			std::int16_t Enable(const std::uint8_t mode)
			{
				isFake = false;				

				std::uint16_t statusWord;
				pFather->readPdo(1, 3, statusWord);

				std::uint8_t modeRead;
				pFather->readPdo(4, 0, modeRead);

				int motorState = (statusWord & 0x000F);

				if (motorState == 0x0000)
				{
					/*state is POWERED_OFF, now set it to STOPPED*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x06));
					return 1;
				}
				else if (motorState == 0x0001)
				{
					/*state is STOPPED, now set it to ENABLED*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x07));
					return 1;
				}
				else if (motorState == 0x0003)
				{
					/*state is ENABLED, now set it to RUNNING*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x0F));
					return 1;
				}
				else if ((mode == POSITION) && (modeRead != VELOCITY))
				{
					/*state is RUNNING, now to set desired mode*/
					/*desired mode is POSITION, but we need to use our own velocity loop*/
					pFather->writePdo(0, 5, static_cast<std::uint8_t>(VELOCITY));
					return 1;
				}
				else if ((mode != POSITION) && (modeRead != mode))
				{
					/*state is RUNNING, now change it to desired mode*/
					pFather->writePdo(0, 5, static_cast<std::uint8_t>(mode));
					return 1;
				}
				else if (motorState == 0x0007)
				{
					/*successfull, but still need to wait for 10 more cycles to make it stable*/
					switch (mode)
					{
					case POSITION:
					case VELOCITY:
						/*velocity loop to set velocity of 0*/
						pFather->writePdo(0, 1, static_cast<std::int32_t>(0));
						break;
					case CURRENT:
						pFather->writePdo(0, 2, static_cast<std::int16_t>(0));
						pFather->writePdo(0, 3, static_cast<std::int16_t>(1500));
						break;
					}

					if (++enablePeriod >= 10)
					{	
						runningMode = mode;
						enablePeriod = 0;
						return 0;
					}
					else
					{
						return 1;
					}
				}
				else
				{
					/*the motor is in fault*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x80));
					return 1;
				}
			}
			std::int16_t Disable()
			{
				isFake = false;					

				std::uint16_t statusWord;
				pFather->readPdo(1, 3, statusWord);

				int motorState = (statusWord & 0x000F);
				if (motorState == 0x0001)
				{
					/*alReady disabled*/
					return 0;
				}
				else if (motorState == 0x0003 || motorState == 0x0007 || motorState == 0x0000)
				{
					/*try to disable*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x06));
					return 1;
				}
				else
				{
					/*the motor is in fault*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x80));
					return 1;
				}

			}
			std::int16_t Home()
			{
				isFake = false;					

				if(isWaitingMode)
				{
					auto ret = this->Enable(runningMode);	
					isWaitingMode = (ret==0?false:true);
					return ret;
				}
				
				std::uint16_t statusWord;
				pFather->readPdo(1, 3, statusWord);
				int motorState = (statusWord & 0x000F);
				if (motorState != 0x0007)
				{
					return -1;
				}
				else
				{
					/*motor is in running state*/
					std::uint8_t mode_Read;
					pFather->readPdo(4, 0, mode_Read);
					if (mode_Read != 0x0006)
					{
						/*set motor to mode 0x006, which is homing mode*/
						pFather->writePdo(0, 5, static_cast<std::uint8_t>(0x006));
						return 1;
					}
					else
					{
						if (statusWord & 0x1000)
						{
							/*home finished, set mode to running mode, whose value is decided by 
							enable function, also write velocity to 0*/
							pFather->writePdo(0, 5, static_cast<uint8_t>(runningMode));
							isWaitingMode = true;
							return 1;
						}
						else
						{
							/*still homing*/
							pFather->writePdo(0, 4, static_cast<uint16_t>(0x1F));
							return 1;
						}
					}
				}
			}
			std::int16_t RunPos(const std::int32_t pos)
			{
				if (isFake)return 0;

				std::uint16_t statusword;
				pFather->readPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->readPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != VELOCITY)
				{
					return -1;
				}
				else
				{
					std::int32_t current_pos = this->pos();
					double Kp = 200;
					std::int32_t desired_vel = static_cast<std::int32_t>(Kp*(pos - current_pos));
					
					/*保护上下限*/
					desired_vel = std::max(desired_vel, -pFather->max_vel_count_);
					desired_vel = std::min(desired_vel, pFather->max_vel_count_);
					
					pFather->writePdo(0, 1, desired_vel);
					return 0;
				}
			}
			std::int16_t RunVel(const std::int32_t vel)
			{
				if (isFake)return 0;

				std::uint16_t statusword;
				pFather->readPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->readPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != VELOCITY)
				{
					return -1;
				}
				else
				{
					pFather->writePdo(0, 1, vel);
					return 0;
				}
			}
			std::int16_t RunCur(const std::int16_t cur)
			{
				if (isFake)return 0;
								
				std::uint16_t statusword;
				pFather->readPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->readPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != CURRENT) //need running and cur mode
				{
					return -1;
				}
				else
				{
					pFather->writePdo(0, 2, cur);
					return 0;
				}
			}
			std::int32_t pos() { std::int32_t pos; pFather->readPdo(1, 0, pos); return pos; };
			std::int32_t vel() { std::int32_t vel; pFather->readPdo(1, 2, vel); return vel; };
			std::int32_t Cur() { std::int16_t cur; pFather->readPdo(2, 0, cur); return cur; };
		private:
			EthercatMotion *pFather;
			
			bool isWaitingMode{ false };
			bool isFake{ true };
			int enablePeriod{ 0 };
			std::uint8_t runningMode{ 9 };
		};
		EthercatMotion::~EthercatMotion() {}
		EthercatMotion::EthercatMotion(const Aris::Core::XmlElement &xml_ele) :EthercatSlave(xml_ele), pImp(new EthercatMotion::Imp(this))
		{
		};
		void EthercatMotion::init()
		{
			this->EthercatSlave::init();
		}
		void EthercatMotion::doCommand(const RawData &data)
		{		
			switch (data.cmd)
			{
			case IDLE:
				data.ret = 0;
				return;
			case ENABLE:
				data.ret = pImp->Enable(data.mode);
				return;
			case DISABLE:
				data.ret = pImp->Disable();
				return;
			case HOME:
				data.ret = pImp->Home();
				return;
			case RUN:
				switch (data.mode)
				{
				case POSITION:
					data.ret = pImp->RunPos(data.target_pos);
					return;
				case VELOCITY:
					data.ret = pImp->RunVel(data.target_vel);
					return;
				case CURRENT:
					data.ret = pImp->RunCur(data.target_cur);
					return;
				default:
					data.ret = -1;
					return;
				}
			default:
				data.ret = -1;
				return;
			}
		}
		void EthercatMotion::readFeedback(RawData &data)
		{
			data.feedback_cur = pImp->Cur();
			data.feedback_pos = pImp->pos();
			data.feedback_vel = pImp->vel();
		}
		bool EthercatMotion::hasFault()
		{
			std::uint16_t statusword;
			this->readPdo(1, 3, statusword);
			int motorState = (statusword & 0x000F);
			if (motorState != 0x0003 && motorState != 0x0007 && motorState != 0x0001 && motorState != 0x0000)
				return true;
			else
				return false;
		}

		void EthercatForceSensor::readData(Data &data)
		{
			std::int32_t value;

			this->readPdo(0, 0, value);
			data.Fx = value / 1000.0;

			this->readPdo(0, 1, value);
			data.Fy = value / 1000.0;

			this->readPdo(0, 2, value);
			data.Fz = value / 1000.0;

			this->readPdo(0, 3, value);
			data.Mx = value / 1000.0;

			this->readPdo(0, 4, value);
			data.My = value / 1000.0;

			this->readPdo(0, 5, value);
			data.Mz = value / 1000.0;
		}

		void EthercatController::loadXml(const Aris::Core::XmlElement &xml_ele)
		{
			/*Load EtherCat slave types*/
			std::map<std::string, const Aris::Core::XmlElement *> slaveTypeMap;

			auto pSlaveTypes = xml_ele.FirstChildElement("SlaveType");
			for (auto pType = pSlaveTypes->FirstChildElement(); pType != nullptr; pType = pType->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(pType->name()), pType));
			}

			/*Load all slaves*/
			motion_vec_.clear();
			force_sensor_vec_.clear();

			auto pSlaves = xml_ele.FirstChildElement("Slave");
			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				std::string type{ pSla->Attribute("type") };
				if (type == "ElmoSoloWhistle")
				{
					motion_vec_.push_back(addSlave<EthercatMotion>(std::ref(*slaveTypeMap.at(type))));
					
					if (pSla->QueryIntAttribute("input2count", &motion_vec_.back()->input2count_) != tinyxml2::XML_NO_ERROR)
					{
						throw std::runtime_error("failed to find motion attribute \"input2count\"");
					}

					double value;
					if (pSla->QueryDoubleAttribute("maxVel", &value) != tinyxml2::XML_NO_ERROR)
					{
						throw std::runtime_error("failed to find motion attribute \"maxVel\"");
					}
					motion_vec_.back()->max_vel_count_ = static_cast<std::int32_t>(value * motion_vec_.back()->input2count_);

					if (pSla->QueryDoubleAttribute("homePos", &value) != tinyxml2::XML_NO_ERROR)
					{
						throw std::runtime_error("failed to find motion attribute \"homePos\"");
					}
					motion_vec_.back()->home_count_ = static_cast<std::int32_t>(pSla->DoubleAttribute("homePos")*motion_vec_.back()->input2count_);

					if (pSla->QueryIntAttribute("absID", &motion_vec_.back()->abs_id_) != tinyxml2::XML_NO_ERROR)
					{
						throw std::runtime_error("failed to find motion attribute \"input2count\"");
					}

					motion_vec_.back()->writeSdo(9, static_cast<std::int32_t>(-motion_vec_.back()->home_count_));

				}
				else if (type == "AtiForceSensor")
				{
					force_sensor_vec_.push_back(addSlave<EthercatForceSensor>(std::ref(*slaveTypeMap.at(type))));
				}
				else
				{
					throw std::runtime_error(std::string("unknown slave type of \"") + type + "\"");
				}
			}

			/*update map*/
			map_phy2abs_.resize(motion_vec_.size());
			map_abs2phy_.resize(motion_vec_.size());

			std::cout << "p2a:";
			for (std::size_t i = 0; i < motion_vec_.size(); ++i)
			{
				map_phy2abs_[i] = motion_vec_[i]->absID();
				std::cout << map_phy2abs_[i] <<"  ";
			}
			std::cout << std::endl;

			for (std::size_t i = 0; i < motion_vec_.size(); ++i)
			{
				map_abs2phy_[i] = std::find(map_phy2abs_.begin(), map_phy2abs_.end(), i) - map_phy2abs_.begin();
			}

			/*resize other var*/
			this->motion_rawdata_.resize(this->motion_vec_.size());
			this->last_motion_rawdata_.resize(this->motion_vec_.size());
			this->force_sensor_data_.resize(this->force_sensor_vec_.size());


			record_pipe_.reset(new Pipe<std::vector<EthercatMotion::RawData> >(1, true, motion_vec_.size()));
		}
		void EthercatController::setControlStrategy(std::function<int(Data&)> strategy)
		{
			if (this->strategy_)
			{
				throw std::runtime_error("failed to set control strategy, because it alReady has one");
			}


			this->strategy_ = strategy;
		}
		void EthercatController::start()
		{
			is_stopping_ = false;

			/*begin thread which will save data*/
			record_thread_ = std::thread([this]()
			{
				static std::fstream file;
				std::string name = Aris::Core::logFileName();
				name.replace(name.rfind("log.txt"), std::strlen("data.txt"), "data.txt");
				file.open(name.c_str(), std::ios::out | std::ios::trunc);
				
				std::vector<EthercatMotion::RawData> data;
				data.resize(this->motion_vec_.size());

				
				long long count = -1;
				while (!is_stopping_)
				{
					this->record_pipe_->recvInNrt(data);

					file << ++count << " ";

					for (auto &d : data)
					{
						file << d.feedback_pos << " ";
						file << d.target_pos << " ";
						file << d.feedback_cur << " ";
					}
					file << std::endl;
				}

				file.close();
			});
			
			this->EthercatMaster::start();
		}
		void EthercatController::stop()
		{
			this->EthercatMaster::stop();

			this->is_stopping_ = true;
			this->record_thread_.join();
		}
		void EthercatController::controlStrategy()
		{
			/*构造传入strategy的参数*/
			Data data{ &last_motion_rawdata_, &motion_rawdata_, &force_sensor_data_, nullptr, nullptr };
			
			/*收取消息*/
			if (this->msgPipe().recvInRT(Aris::Core::MsgRT::instance[0]) > 0)
			{
				data.msg_recv = &Aris::Core::MsgRT::instance[0];
			};
			
			/*读取反馈*/
			for (std::size_t i = 0; i < motion_vec_.size(); ++i)
			{
				motionAtAbs(i).readFeedback(motion_rawdata_[i]);
			}
			for (std::size_t i = 0; i < force_sensor_vec_.size(); ++i)
			{
				force_sensor_vec_.at(i)->readData(force_sensor_data_[i]);
			}
			
			/*执行自定义的控制策略*/
			if (strategy_)
			{
				strategy_(data);
			}

			/*重新读取反馈信息，因为strategy可能修改已做好的反馈信息，之后写入PDO，之后放进lastMotionData中*/
			for (std::size_t i = 0; i < motion_rawdata_.size(); ++i)
			{
				motionAtAbs(i).readFeedback(motion_rawdata_[i]);
				motionAtAbs(i).doCommand(motion_rawdata_[i]);
				last_motion_rawdata_[i] = motion_rawdata_[i];
			}

			/*发送数据到记录的线程*/
			record_pipe_->sendToNrt(motion_rawdata_);

			/*向外发送消息*/
			if (data.pMsgSend)
			{
				this->msgPipe().sendToNrt(*data.pMsgSend);
			}
		}
	}
}
