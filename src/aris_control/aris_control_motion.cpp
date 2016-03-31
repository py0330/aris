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
		class EthercatMotion::Imp 
		{
		public:
			Imp(EthercatMotion *mot) :pFather(mot) {};
			~Imp() = default;

			std::int16_t enable(const std::uint8_t mode)
			{
				is_fake = false;				

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

					if (++enable_period >= 10)
					{	
						running_mode = mode;
						enable_period = 0;
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
			std::int16_t disable()
			{
				is_fake = false;					

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
			std::int16_t home()
			{
				is_fake = false;					

				if(is_waiting_mode)
				{
					auto ret = this->enable(running_mode);
					is_waiting_mode = (ret == 0 ? false : true);
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
							pFather->writePdo(0, 5, static_cast<uint8_t>(running_mode));
							is_waiting_mode = true;
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
			std::int16_t runPos(const std::int32_t pos)
			{
				if (is_fake)return 0;

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
					desired_vel = std::max(desired_vel, -max_vel_count_);
					desired_vel = std::min(desired_vel, max_vel_count_);
					
					pFather->writePdo(0, 1, desired_vel);
					return 0;
				}
			}
			std::int16_t runVel(const std::int32_t vel)
			{
				if (is_fake)return 0;

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
			std::int16_t runCur(const std::int16_t cur)
			{
				if (is_fake)return 0;
								
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
			std::int32_t pos() { std::int32_t pos; pFather->readPdo(1, 0, pos); return pos + pos_offset_; };
			std::int32_t vel() { std::int32_t vel; pFather->readPdo(1, 2, vel); return vel; };
			std::int32_t cur() { std::int16_t cur; pFather->readPdo(2, 0, cur); return cur; };
		
			std::int32_t input2count_;
			std::int32_t home_count_;
			std::int32_t max_pos_count_;
			std::int32_t min_pos_count_;
			std::int32_t max_vel_count_;
			std::int32_t abs_id_;
			std::int32_t phy_id_;
			
			EthercatMotion *pFather;
			
			std::int32_t pos_offset_{0};

			bool is_fake{ true };
			bool is_waiting_mode{ false };
			
			int enable_period{ 0 };
			std::uint8_t running_mode{ 9 };
		};
		EthercatMotion::~EthercatMotion() {}
		EthercatMotion::EthercatMotion(const aris::core::XmlElement &xml_ele, const aris::core::XmlElement &type_xml_ele) 
			:EthercatSlave(type_xml_ele), imp_(new EthercatMotion::Imp(this))
		{
			if (xml_ele.QueryIntAttribute("input2count", &imp_->input2count_) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"input2count\"");
			}

			double value;
			if (xml_ele.QueryDoubleAttribute("max_pos", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"max_pos\"");
			}
			imp_->max_pos_count_ = static_cast<std::int32_t>(value * imp_->input2count_);
			if (xml_ele.QueryDoubleAttribute("min_pos", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"min_pos\"");
			}
			imp_->min_pos_count_ = static_cast<std::int32_t>(value * imp_->input2count_);
			if (xml_ele.QueryDoubleAttribute("max_vel", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"max_vel\"");
			}
			imp_->max_vel_count_ = static_cast<std::int32_t>(value * imp_->input2count_);

			if (xml_ele.QueryDoubleAttribute("home_pos", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"home_pos\"");
			}
			imp_->home_count_ = static_cast<std::int32_t>(value * imp_->input2count_);

			if (xml_ele.QueryIntAttribute("abs_id", &imp_->abs_id_) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"abs_id\"");
			}

			configSdo(9, static_cast<std::int32_t>(-imp_->home_count_));
		};
		auto EthercatMotion::writeCommand(const RawData &data)->void
		{		
			switch (data.cmd)
			{
			case IDLE:
				data.ret = 0;
				return;
			case ENABLE:
				data.ret = imp_->enable(data.mode);
				return;
			case DISABLE:
				data.ret = imp_->disable();
				return;
			case HOME:
				data.ret = imp_->home();
				return;
			case RUN:
				switch (data.mode)
				{
				case POSITION:
					data.ret = imp_->runPos(data.target_pos);
					return;
				case VELOCITY:
					data.ret = imp_->runVel(data.target_vel);
					return;
				case CURRENT:
					data.ret = imp_->runCur(data.target_cur);
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
		auto EthercatMotion::readFeedback(RawData &data)->void
		{
			data.feedback_cur = imp_->cur();
			data.feedback_pos = imp_->pos();
			data.feedback_vel = imp_->vel();
		}
		auto EthercatMotion::hasFault()->bool
		{
			std::uint16_t statusword;
			this->readPdo(1, 3, statusword);
			int motorState = (statusword & 0x000F);
			return (motorState != 0x0003 && motorState != 0x0007 && motorState != 0x0001 && motorState != 0x0000) ? true : false;
		}
		auto EthercatMotion::absID()->std::int32_t { return imp_->abs_id_; };
		auto EthercatMotion::phyID()->std::int32_t { return imp_->phy_id_; };
		auto EthercatMotion::maxPosCount()->std::int32_t { return imp_->max_pos_count_; };
		auto EthercatMotion::minPosCount()->std::int32_t { return imp_->min_pos_count_; };
		auto EthercatMotion::maxVelCount()->std::int32_t { return imp_->max_vel_count_; };
		auto EthercatMotion::pos2countRatio()->std::int32_t { return imp_->input2count_; };
		auto EthercatMotion::setPosOffset(std::int32_t offset)->void
		{
			imp_->pos_offset_ = offset;
		};
		auto EthercatMotion::posOffset()const->std::int32_t
		{
			return imp_->pos_offset_;
		}

		auto EthercatForceSensor::readData(Data &data)->void
		{
			std::int32_t value;

			this->readPdo(0, 0, value);
			data.Fx = static_cast<double>(value) / force_ratio_;

			this->readPdo(0, 1, value);
			data.Fy = static_cast<double>(value) / force_ratio_;

			this->readPdo(0, 2, value);
			data.Fz = static_cast<double>(value) / force_ratio_;

			this->readPdo(0, 3, value);
			data.Mx = static_cast<double>(value) / torque_ratio_;

			this->readPdo(0, 4, value);
			data.My = static_cast<double>(value) / torque_ratio_;

			this->readPdo(0, 5, value);
			data.Mz = static_cast<double>(value) / torque_ratio_;
		}

		struct EthercatController::Imp
		{
			std::vector<int> map_phy2abs_, map_abs2phy_;

			std::function<int(Data&)> strategy_;
			Pipe<aris::core::Msg> msg_pipe_;
			std::atomic_bool is_stopping_;

			std::vector<EthercatMotion *> motion_vec_;
			std::vector<EthercatMotion::RawData> motion_rawdata_, last_motion_rawdata_;

			std::vector<EthercatForceSensor *> force_sensor_vec_;
			std::vector<EthercatForceSensor::Data> force_sensor_data_;

			std::unique_ptr<Pipe<std::vector<EthercatMotion::RawData> > > record_pipe_;
			std::thread record_thread_;
		};
		EthercatController::~EthercatController() {};
		EthercatController::EthercatController() :EthercatMaster(),imp_(new Imp) {};
		auto EthercatController::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			/*Load EtherCat slave types*/
			std::map<std::string, const aris::core::XmlElement *> slaveTypeMap;

			auto pSlaveTypes = xml_ele.FirstChildElement("SlaveType");
			for (auto type_xml_ele = pSlaveTypes->FirstChildElement(); type_xml_ele; type_xml_ele = type_xml_ele->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(type_xml_ele->name()), type_xml_ele));
			}

			/*Load all slaves*/
			imp_->motion_vec_.clear();
			imp_->force_sensor_vec_.clear();

			auto slave_xml = xml_ele.FirstChildElement("Slave");
			for (auto sla = slave_xml->FirstChildElement(); sla; sla = sla->NextSiblingElement())
			{
				std::string type{ sla->Attribute("type") };
				if (type == "ElmoSoloWhistle")
				{
					imp_->motion_vec_.push_back(addSlave<EthercatMotion>(std::ref(*sla), std::ref(*slaveTypeMap.at(type))));
				}
				else if (type == "AtiForceSensor")
				{
					imp_->force_sensor_vec_.push_back(addSlave<EthercatForceSensor>(std::ref(*slaveTypeMap.at(type))));
				}
				else
				{
					throw std::runtime_error(std::string("unknown slave type of \"") + type + "\"");
				}
			}

			/*update map*/
			imp_->map_phy2abs_.resize(imp_->motion_vec_.size());
			imp_->map_abs2phy_.resize(imp_->motion_vec_.size());

			for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
			{
				imp_->map_phy2abs_[i] = imp_->motion_vec_[i]->absID();
				motionAtPhy(i).imp_->phy_id_ = i;
			}

			for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
			{
				imp_->map_abs2phy_[i] = std::find(imp_->map_phy2abs_.begin(), imp_->map_phy2abs_.end(), i) - imp_->map_phy2abs_.begin();
			}

			/*resize other var*/
			imp_->motion_rawdata_.resize(imp_->motion_vec_.size());
			imp_->last_motion_rawdata_.resize(imp_->motion_vec_.size());
			imp_->force_sensor_data_.resize(imp_->force_sensor_vec_.size());


			imp_->record_pipe_.reset(new Pipe<std::vector<EthercatMotion::RawData> >(true, imp_->motion_vec_.size()));
		}
		auto EthercatController::setControlStrategy(std::function<int(Data&)> strategy)->void
		{
			if (imp_->strategy_)
			{
				throw std::runtime_error("failed to set control strategy, because it alReady has one");
			}


			imp_->strategy_ = strategy;
		}
		auto EthercatController::start()->void
		{
			imp_->is_stopping_ = false;

			/*begin thread which will save data*/
			if(!imp_->record_thread_.joinable())
			imp_->record_thread_ = std::thread([this]()
			{
				static std::fstream file;
				std::string name = aris::core::logFileName();
				name.replace(name.rfind("log.txt"), std::strlen("data.txt"), "data.txt");
				file.open(name.c_str(), std::ios::out | std::ios::trunc);
				
				std::vector<EthercatMotion::RawData> data;
				data.resize(imp_->motion_vec_.size());

				long long count = -1;
				while (!imp_->is_stopping_)
				{
					imp_->record_pipe_->recvInNrt(data);

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
		auto EthercatController::stop()->void
		{
			this->EthercatMaster::stop();
		}
		auto EthercatController::motionNum()->std::size_t { return imp_->motion_vec_.size(); };
		auto EthercatController::motionAtAbs(int i)->EthercatMotion & { return *imp_->motion_vec_.at(imp_->map_abs2phy_[i]); };
		auto EthercatController::motionAtPhy(int i)->EthercatMotion & { return *imp_->motion_vec_.at(i); };
		auto EthercatController::forceSensorNum()->std::size_t { return imp_->force_sensor_vec_.size(); };
		auto EthercatController::forceSensorAt(int i)->EthercatForceSensor & { return *imp_->force_sensor_vec_.at(i); };
		auto EthercatController::msgPipe()->Pipe<aris::core::Msg>& { return imp_->msg_pipe_; };
		auto EthercatController::controlStrategy()->void
		{
			/*构造传入strategy的参数*/
			Data data{ &imp_->last_motion_rawdata_, &imp_->motion_rawdata_, &imp_->force_sensor_data_, nullptr, nullptr };
			
			/*收取消息*/
			if (this->msgPipe().recvInRT(aris::core::MsgRT::instance[0]) > 0)
			{
				data.msg_recv = &aris::core::MsgRT::instance[0];
			};
			
			/*读取反馈*/
			for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
			{
				motionAtAbs(i).readFeedback(imp_->motion_rawdata_[i]);
			}
			for (std::size_t i = 0; i < imp_->force_sensor_vec_.size(); ++i)
			{
				imp_->force_sensor_vec_.at(i)->readData(imp_->force_sensor_data_[i]);
			}
			
			/*执行自定义的控制策略*/
			if (imp_->strategy_)
			{
				imp_->strategy_(data);
			}

			/*重新读取反馈信息，因为strategy可能修改已做好的反馈信息，之后写入PDO，之后放进lastMotionData中*/
			for (std::size_t i = 0; i < imp_->motion_rawdata_.size(); ++i)
			{
				motionAtAbs(i).readFeedback(imp_->motion_rawdata_[i]);
				motionAtAbs(i).writeCommand(imp_->motion_rawdata_[i]);
				imp_->last_motion_rawdata_[i] = imp_->motion_rawdata_[i];
			}

			/*发送数据到记录的线程*/
			imp_->record_pipe_->sendToNrt(imp_->motion_rawdata_);

			/*向外发送消息*/
			if (data.msg_send)
			{
				this->msgPipe().sendToNrt(*data.msg_send);
			}
		}
	}
}
