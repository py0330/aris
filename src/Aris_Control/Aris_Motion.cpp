
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
#include <Aris_Motion.h>
#include <fstream>
#include <algorithm>


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
				pFather->ReadPdo(1, 3, statusWord);

				std::uint8_t modeRead;
				pFather->ReadPdo(4, 0, modeRead);

				int motorState = (statusWord & 0x000F);

				if (motorState == 0x0000)
				{
					/*state is POWERED_OFF, now set it to STOPPED*/
					pFather->WritePdo(0, 4, static_cast<std::uint16_t>(0x06));
					return 1;
				}
				else if (motorState == 0x0001)
				{
					/*state is STOPPED, now set it to ENABLED*/
					pFather->WritePdo(0, 4, static_cast<std::uint16_t>(0x07));
					return 1;
				}
				else if (motorState == 0x0003)
				{
					/*state is ENABLED, now set it to RUNNING*/
					pFather->WritePdo(0, 4, static_cast<std::uint16_t>(0x0F));
					return 1;
				}
				else if ((mode == POSITION) && (modeRead != VELOCITY))
				{
					/*state is RUNNING, now to set desired mode*/
					/*desired mode is POSITION, but we need to use our own velocity loop*/
					pFather->WritePdo(0, 5, static_cast<std::uint8_t>(VELOCITY));
					return 1;
				}
				else if ((mode != POSITION) && (modeRead != mode))
				{
					/*state is RUNNING, now change it to desired mode*/
					pFather->WritePdo(0, 5, static_cast<std::uint8_t>(mode));
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
						pFather->WritePdo(0, 1, static_cast<std::int32_t>(0));
						break;
					case CURRENT:
						pFather->WritePdo(0, 2, static_cast<std::int16_t>(0));
						pFather->WritePdo(0, 3, static_cast<std::int16_t>(1500));
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
					pFather->WritePdo(0, 4, static_cast<std::uint16_t>(0x80));
					return 1;
				}
			}
			std::int16_t Disable()
			{
				isFake = false;					

				std::uint16_t statusWord;
				pFather->ReadPdo(1, 3, statusWord);

				int motorState = (statusWord & 0x000F);
				if (motorState == 0x0001)
				{
					/*alReady disabled*/
					return 0;
				}
				else if (motorState == 0x0003 || motorState == 0x0007 || motorState == 0x0000)
				{
					/*try to disable*/
					pFather->WritePdo(0, 4, static_cast<std::uint16_t>(0x06));
					return 1;
				}
				else
				{
					/*the motor is in fault*/
					pFather->WritePdo(0, 4, static_cast<std::uint16_t>(0x80));
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
				pFather->ReadPdo(1, 3, statusWord);
				int motorState = (statusWord & 0x000F);
				if (motorState != 0x0007)
				{
					return -1;
				}
				else
				{
					/*motor is in running state*/
					std::uint8_t mode_Read;
					pFather->ReadPdo(4, 0, mode_Read);
					if (mode_Read != 0x0006)
					{
						/*set motor to mode 0x006, which is homing mode*/
						pFather->WritePdo(0, 5, static_cast<std::uint8_t>(0x006));
						return 1;
					}
					else
					{
						if (statusWord & 0x1000)
						{
							/*home finished, set mode to running mode, whose value is decided by 
							enable function, also write velocity to 0*/
							pFather->WritePdo(0, 5, static_cast<uint8_t>(runningMode));
							isEverHomed = true;
							isWaitingMode = true;
							return 1;
						}
						else
						{
							/*still homing*/
							pFather->WritePdo(0, 4, static_cast<uint16_t>(0x1F));
							return 1;
						}
					}
				}
			}
			std::int16_t RunPos(const std::int32_t pos)
			{
				if (isFake)return 0;

				std::uint16_t statusword;
				pFather->ReadPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->ReadPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != VELOCITY)
				{
					return -1;
				}
				else
				{
					std::int32_t current_pos = this->Pos();
					double Kp = 200;
					std::int32_t desired_vel = static_cast<std::int32_t>(Kp*(pos - current_pos));
					
					/*保护上下限*/
					desired_vel = std::max(desired_vel, -maxSpeed);
					desired_vel = std::min(desired_vel, maxSpeed);
					
					pFather->WritePdo(0, 1, desired_vel);
					return 0;
				}
			}
			std::int16_t RunVel(const std::int32_t vel)
			{
				if (isFake)return 0;

				std::uint16_t statusword;
				pFather->ReadPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->ReadPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != VELOCITY)
				{
					return -1;
				}
				else
				{
					pFather->WritePdo(0, 1, vel);
					return 0;
				}
			}
			std::int16_t RunCur(const std::int16_t cur)
			{
				if (isFake)return 0;
								
				std::uint16_t statusword;
				pFather->ReadPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->ReadPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != CURRENT) //need running and cur mode
				{
					return -1;
				}
				else
				{
					pFather->WritePdo(0, 2, cur);
					return 0;
				}
			}
			std::int32_t Pos() { std::int32_t pos; pFather->ReadPdo(1, 0, pos); return pos; };
			std::int32_t Vel() { std::int32_t vel; pFather->ReadPdo(1, 2, vel); return vel; };
			std::int32_t Cur() { std::int16_t cur; pFather->ReadPdo(2, 0, cur); return cur; };

			std::int32_t homeOffSet{ 0 };
			std::int32_t maxSpeed{ 0 };
		private:
			EthercatMotion *pFather;
			
			bool isWaitingMode{ false };
			bool isEverHomed{ false };
			bool isFake{ true };
			int enablePeriod{ 0 };
			std::uint8_t runningMode{ 9 };
		};
		EthercatMotion::~EthercatMotion() {}
		EthercatMotion::EthercatMotion(const Aris::Core::XmlElement *ele) :EthercatSlave(ele), pImp(new EthercatMotion::Imp(this))
		{
		};
		void EthercatMotion::Init()
		{
			this->EthercatSlave::Init();
		}
		void EthercatMotion::DoCommand(const Data &data)
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
					data.ret = pImp->RunPos(data.targetPos);
					return;
				case VELOCITY:
					data.ret = pImp->RunVel(data.targetVel);
					return;
				case CURRENT:
					data.ret = pImp->RunCur(data.targetCur);
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
		void EthercatMotion::ReadFeedback(Data &data)
		{
			data.feedbackCur = pImp->Cur();
			data.feedbackPos = pImp->Pos();
			data.feedbackVel = pImp->Vel();
		}
		bool EthercatMotion::HasFault()
		{
			std::uint16_t statusword;
			this->ReadPdo(1, 3, statusword);
			int motorState = (statusword & 0x000F);
			if (motorState != 0x0003 && motorState != 0x0007 && motorState != 0x0001 && motorState != 0x0000)
				return true;
			else
				return false;
		}

		void EthercatForceSensor::ReadData(Data &data)
		{
			std::int32_t value;

			this->ReadPdo(0, 0, value);
			data.Fx = value / 1000.0;

			this->ReadPdo(0, 1, value);
			data.Fy = value / 1000.0;

			this->ReadPdo(0, 2, value);
			data.Fz = value / 1000.0;

			this->ReadPdo(0, 3, value);
			data.Mx = value / 1000.0;

			this->ReadPdo(0, 4, value);
			data.My = value / 1000.0;

			this->ReadPdo(0, 5, value);
			data.Mz = value / 1000.0;
		}

		void EthercatController::LoadXml(const Aris::Core::XmlElement *ele)
		{
			/*Load EtherCat slave types*/
			std::map<std::string, const Aris::Core::XmlElement *> slaveTypeMap;

			auto pSlaveTypes = ele->FirstChildElement("SlaveType");
			for (auto pType = pSlaveTypes->FirstChildElement(); pType != nullptr; pType = pType->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(pType->Name()), pType));
			}

			/*Load all slaves*/
			auto pSlaves = ele->FirstChildElement("Slave");
			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				std::string type{ pSla->Attribute("type") };
				if (type == "ElmoSoloWhistle")
				{
					pMotions.push_back(AddSlave<EthercatMotion>(slaveTypeMap.at(type)));
					if (pSla->QueryIntAttribute("maxSpeed", &pMotions.back()->pImp->maxSpeed) != tinyxml2::XML_NO_ERROR)
					{						
						throw std::runtime_error("failed to find motion attribute \"maxSpeed\"");
					}

				}
				else if (type == "AtiForceSensor")
				{
					pForceSensors.push_back(AddSlave<EthercatForceSensor>(slaveTypeMap.at(type)));
				}
				else
				{
					throw std::runtime_error(std::string("unknown slave type of \"") + type + "\"");
				}
				
			}
			this->motionData.resize(this->pMotions.size());
			this->lastMotionData.resize(this->pMotions.size());
			this->forceSensorData.resize(this->pForceSensors.size());


			pMotDataPipe.reset(new Pipe<std::vector<EthercatMotion::Data> >(1, true, pMotions.size()));

			
		}
		void EthercatController::SetControlStrategy(std::function<int(Data&)> strategy)
		{
			if (this->strategy)
			{
				throw std::runtime_error("failed to set control strategy, because it alReady has one");
			}


			this->strategy = strategy;
		}
		void EthercatController::Start()
		{
			isStoping = false;

			/*begin thRead which will save data*/
			motionDataThread = std::thread([this]()
			{
				static std::fstream file;
				std::string name = Aris::Core::logFileName();
				name.replace(name.rfind("log.txt"), std::strlen("data.txt"), "data.txt");
				file.open(name.c_str(), std::ios::out | std::ios::trunc);
				
				std::vector<EthercatMotion::Data> data;
				data.resize(this->pMotions.size());

				
				long long count = -1;
				while (!isStoping)
				{
					this->pMotDataPipe->RecvInNRT(data);

					file << ++count << " ";

					for (auto &d : data)
					{
						file << d.feedbackPos << " ";
						file << d.targetPos << " ";
						file << d.feedbackCur << " ";
					}
					file << std::endl;
				}

				file.close();
			});
			
			this->EthercatMaster::Start();
		}
		void EthercatController::Stop()
		{
			this->EthercatMaster::Stop();

			this->isStoping = true;
			this->motionDataThread.join();
		}
		void EthercatController::ControlStrategy()
		{
			/*构造传入strategy的参数*/
			Data data{ &lastMotionData, &motionData, &forceSensorData, nullptr, nullptr };
			
			/*收取消息*/
			if (this->MsgPipe().RecvInRT(Aris::Core::MsgRT::instance[0]) > 0)
			{
				data.pMsgRecv = &Aris::Core::MsgRT::instance[0];
			};
			
			/*读取反馈*/
			for (std::size_t i = 0; i < pMotions.size(); ++i)
			{
				pMotions.at(i)->ReadFeedback(motionData[i]);
			}
			for (std::size_t i = 0; i < pForceSensors.size(); ++i)
			{
				pForceSensors.at(i)->ReadData(forceSensorData[i]);
			}
			
			/*执行自定义的控制策略*/
			if (strategy)
			{
				strategy(data);
			}

			/*重新读取反馈信息，因为strategy可能修改，之后写入PDO，之后放进lastMotionData中*/
			for (std::size_t i = 0; i < motionData.size(); ++i)
			{
				pMotions.at(i)->ReadFeedback(motionData[i]);
				pMotions.at(i)->DoCommand(motionData[i]);
				lastMotionData[i] = motionData[i];
			}

			/*发送数据到记录的线程*/
			pMotDataPipe->SendToNRT(motionData);

			/*向外发送消息*/
			if (data.pMsgSend)
			{
				this->MsgPipe().SendToNRT(*data.pMsgSend);
			}
		}
	}
}
