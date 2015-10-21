#include <Platform.h>
#ifdef PLATFORM_IS_WINDOWS
#include <ecrt_windows_py.h>//just for IDE vs2015, it does not really work
#endif
#ifdef PLATFORM_IS_LINUX
#include <ecrt.h>
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
// following for pipe
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <rtdm/rtdm.h>
#include <rtdm/rtipc.h>
#include <mutex>
#endif

#include <mutex>
#include <string>
#include <iostream>
#include <map>
#include <atomic>
#include <memory>

#include <Aris_EtherCat.h>


namespace Aris
{
	namespace Control
	{
		class ETHERCAT_SLAVE::IMP 
		{
		public:
			IMP(const Aris::Core::ELEMENT *ele);
			void Initialize();
			void Read();
			void Write();

		private:
			/*data object, can be PDO or SDO*/
			class DO
			{
			public:
				IMP *pImp;
				std::uint16_t index;
				std::uint8_t subIndex;
				std::uint8_t size;
				std::uint32_t offset;

				std::int32_t value;//only for sdo
				
				virtual ~DO() = default;
				virtual void ReadValue(std::int8_t &value) const { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual void ReadValue(std::int16_t &value) const { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type");};
				virtual void ReadValue(std::int32_t &value) const { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual void ReadValue(std::uint8_t &value) const { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual void ReadValue(std::uint16_t &value) const { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual void ReadValue(std::uint32_t &value) const { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual void WriteValue(std::int8_t value) { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual void WriteValue(std::int16_t value) { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual void WriteValue(std::int32_t value) { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual void WriteValue(std::uint8_t value) { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual void WriteValue(std::uint16_t value) { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual void WriteValue(std::uint32_t value) { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
			};
			template<typename TYPE>
			class PDO_TYPE_TX :public DO
			{
			public:
				virtual ~PDO_TYPE_TX() = default;
				virtual void ReadValue(TYPE &value) const override { value = *reinterpret_cast<const TYPE*>(pImp->pDomainPd + offset); };
			};
			template<typename TYPE>
			class PDO_TYPE_RX :public DO
			{
			public:
				virtual ~PDO_TYPE_RX() = default;
				virtual void WriteValue(TYPE value) override { *reinterpret_cast<TYPE*>(pImp->pDomainPd + offset) = value; };
				//virtual void ReadValue(TYPE &value) const override { value = *reinterpret_cast<const TYPE*>(pImp->pDomainPd + offset); };

			};

			class PDO_GROUP
			{
			public:
				bool isTx;
				uint16_t index;
				std::vector<std::unique_ptr<DO> > pdos;
				std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec;
			};
			std::vector<PDO_GROUP> pdoGroups;
			std::vector<std::unique_ptr<DO> > sdos;
			std::uint32_t productCode, venderID;
			std::uint16_t position, alias;
			std::unique_ptr<int> distributedClock;

			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec;
			std::vector<ec_pdo_info_t> ec_pdo_info_vec_Tx, ec_pdo_info_vec_Rx;
			ec_sync_info_t ec_sync_info[5];
			ec_slave_config_t* ec_slave_config;
			
			ec_domain_t* pDomain;
			std::uint8_t* pDomainPd;

			friend class ETHERCAT_MASTER::IMP;
			friend class ETHERCAT_SLAVE;
		};
		class ETHERCAT_MASTER::IMP
		{
		public:
			void Read();
			void Write();
			void Start();
			void Stop();
			
		private:
			void Initialize();
			void Sync(uint64_t nanoSecond);
			static void RealTimeCore(void *);

		private:
			std::vector<std::unique_ptr<ETHERCAT_SLAVE> > slaves;
			ec_master_t* pEcMaster;

			static const int samplePeriodNs;
			static std::atomic_bool isStopping;

#ifdef PLATFORM_IS_LINUX
			static RT_TASK realtimeCore;
#endif
			friend class ETHERCAT_SLAVE::IMP;
			friend class ETHERCAT_MASTER;
		};

#ifdef PLATFORM_IS_LINUX
		RT_TASK ETHERCAT_MASTER::IMP::realtimeCore;
		const int ETHERCAT_MASTER::IMP::samplePeriodNs = 1000000;
#endif
		ETHERCAT_SLAVE::IMP::IMP(const Aris::Core::ELEMENT *ele)
		{
			/*load product id...*/
			this->productCode = std::stoi(ele->Attribute("productCode"), nullptr, 0);
			this->venderID = std::stoi(ele->Attribute("venderID"), nullptr, 0);
			this->alias = std::stoi(ele->Attribute("alias"), nullptr, 0);
			this->distributedClock.reset(new std::int32_t);
						
			if (ele->Attribute("distributedClock"))
			{						
				*distributedClock.get() = std::stoi(ele->Attribute("distributedClock"),nullptr,0);				
			}
			else
			{
				distributedClock.reset();
			}


			/*load PDO*/
			auto AddDoType = [](const Aris::Core::ELEMENT *ele, bool isTx)-> DO*
			{
				DO* ret;
				if (ele->Attribute("type", "int8"))
				{
					if(isTx)
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_TX<std::int8_t>();
					else
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_RX<std::int8_t>();
					ret->size = 8;
				}
				else if (ele->Attribute("type", "uint8"))
				{
					if (isTx)
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_TX<std::uint8_t>();
					else
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_RX<std::uint8_t>();
					ret->size = 8;
				}
				else if (ele->Attribute("type", "int16"))
				{
					if (isTx)
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_TX<std::int16_t>();
					else
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_RX<std::int16_t>();
					ret->size = 16;
				}
				else if (ele->Attribute("type", "uint16"))
				{
					if (isTx)
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_TX<std::uint16_t>();
					else
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_RX<std::uint16_t>();
					ret->size = 16;
				}
				else if (ele->Attribute("type", "int32"))
				{
					if (isTx)
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_TX<std::int32_t>();
					else
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_RX<std::int32_t>();
					ret->size = 32;
				}
				else if (ele->Attribute("type", "uint32"))
				{
					if (isTx)
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_TX<std::uint32_t>();
					else
						ret = new ETHERCAT_SLAVE::IMP::PDO_TYPE_RX<std::uint32_t>();
					ret->size = 32;
				}
				else
				{
					throw std::runtime_error("invalid type of pdo");
				}

				return ret;
			};
			auto PDO = ele->FirstChildElement("PDO");
			for (auto p_g = PDO->FirstChildElement(); p_g != nullptr; p_g = p_g->NextSiblingElement())
			{
				PDO_GROUP pdo_group;
				pdo_group.index = std::stoi(p_g->Attribute("index"), nullptr, 0);
				pdo_group.isTx = p_g->Attribute("isTx", "true") ? true : false;
				for (auto p = p_g->FirstChildElement(); p != nullptr; p = p->NextSiblingElement())
				{
					pdo_group.pdos.push_back(std::unique_ptr<DO>(AddDoType(p,pdo_group.isTx)));				
					pdo_group.pdos.back()->index = std::stoi(p->Attribute("index"), nullptr, 0);
					pdo_group.pdos.back()->subIndex = std::stoi(p->Attribute("subIndex"), nullptr, 0);
					pdo_group.pdos.back()->pImp = this;
				}
				pdoGroups.push_back(std::move(pdo_group));
			}


			/*load SDO*/
			auto SDO = ele->FirstChildElement("SDO");
			for (auto s = SDO->FirstChildElement(); s != nullptr; s = s->NextSiblingElement())
			{			
				sdos.push_back(std::unique_ptr<DO>(new DO));
				sdos.back()->size = 32;
				sdos.back()->index = std::stoi(s->Attribute("index"), nullptr, 0);
				sdos.back()->subIndex = std::stoi(s->Attribute("subIndex"), nullptr, 0);
				sdos.back()->pImp = this;
				sdos.back()->value = std::stoi(s->Attribute("value"), nullptr, 0);
			}

			/*create ecrt structs*/
			for (auto &p_g : pdoGroups)
			{
				for (auto &p : p_g.pdos)
				{
					ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ alias,position,venderID,productCode,p->index,p->subIndex,&p->offset });
					p_g.ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ p->index,p->subIndex,p->size });
				}

				if (p_g.isTx)
				{
					ec_pdo_info_vec_Tx.push_back(ec_pdo_info_t{ p_g.index, static_cast<std::uint8_t>(p_g.pdos.size()), p_g.ec_pdo_entry_info_vec.data() });
				}
				else
				{
					ec_pdo_info_vec_Rx.push_back(ec_pdo_info_t{ p_g.index, static_cast<std::uint8_t>(p_g.pdos.size()), p_g.ec_pdo_entry_info_vec.data() });
				}
			}
			ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ });

			ec_sync_info[0] = ec_sync_info_t{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE };
			ec_sync_info[1] = ec_sync_info_t{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE };
			ec_sync_info[2] = ec_sync_info_t{ 2, EC_DIR_OUTPUT, static_cast<unsigned int>(ec_pdo_info_vec_Rx.size()), ec_pdo_info_vec_Rx.data(), EC_WD_ENABLE };
			ec_sync_info[3] = ec_sync_info_t{ 3, EC_DIR_INPUT, static_cast<unsigned int>(ec_pdo_info_vec_Tx.size()), ec_pdo_info_vec_Tx.data(), EC_WD_ENABLE };
			ec_sync_info[4] = ec_sync_info_t{ 0xff };
		};
		void ETHERCAT_SLAVE::IMP::Initialize()
		{		
			auto pEcMaster = ETHERCAT_MASTER::GetInstance()->pImp->pEcMaster;

			for(auto &reg:ec_pdo_entry_reg_vec)
			{
				reg.position = this->position;
 			}
            		pDomain = ecrt_master_create_domain(pEcMaster);
			if (!pDomain)
			{
				throw std::runtime_error("failed to create domain");
			}
			// Get the slave configuration 
			if (!(ec_slave_config = ecrt_master_slave_config(pEcMaster, alias, position, venderID, productCode)))
			{
				throw std::runtime_error("failed to slave config");
			}
			/*Set Sdo*/
			for (auto &sdo : sdos)
			{
				ecrt_slave_config_sdo32(ec_slave_config, sdo->index, sdo->subIndex, sdo->value);
			}
			// Configure the slave's PDOs and sync masters
			if (ecrt_slave_config_pdos(ec_slave_config, 4, ec_sync_info))
			{
				throw std::runtime_error("failed to slave config pdos");
			}
			// Configure the slave's domain
			if (ecrt_domain_reg_pdo_entry_list(pDomain, ec_pdo_entry_reg_vec.data()))
			{
				throw std::runtime_error("failed domain_reg_pdo_entry");
			}
			// Configure the slave's discrete clock			
			if(this->distributedClock)
			{
				ecrt_slave_config_dc(ec_slave_config, *distributedClock.get(), 1000000, 4400000, 0, 0);
			}
			
		}
		void ETHERCAT_SLAVE::IMP::Read()
		{
			ecrt_domain_process(pDomain);
		}
		void ETHERCAT_SLAVE::IMP::Write()
		{
			ecrt_domain_queue(pDomain);
		}

		ETHERCAT_SLAVE::ETHERCAT_SLAVE(const Aris::Core::ELEMENT *ele):pImp(new IMP{ ele })
		{
		}
		ETHERCAT_SLAVE::~ETHERCAT_SLAVE() {};
		void ETHERCAT_SLAVE::Initialize()
		{
			this->pImp->Initialize();
		}
		void ETHERCAT_SLAVE::ReadPdo(int pdoGroupID, int pdoID, std::int8_t &value) const
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->ReadValue(value);
		}
		void ETHERCAT_SLAVE::ReadPdo(int pdoGroupID, int pdoID, std::int16_t &value) const
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->ReadValue(value);
		}
		void ETHERCAT_SLAVE::ReadPdo(int pdoGroupID, int pdoID, std::int32_t &value) const
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->ReadValue(value);
		}
		void ETHERCAT_SLAVE::ReadPdo(int pdoGroupID, int pdoID, std::uint8_t &value) const
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->ReadValue(value);
		}
		void ETHERCAT_SLAVE::ReadPdo(int pdoGroupID, int pdoID, std::uint16_t &value) const
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->ReadValue(value);
		}
		void ETHERCAT_SLAVE::ReadPdo(int pdoGroupID, int pdoID, std::uint32_t &value) const
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->ReadValue(value);
		}
		void ETHERCAT_SLAVE::WritePdo(int pdoGroupID, int pdoID, std::int8_t value)
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->WriteValue(value);
		}
		void ETHERCAT_SLAVE::WritePdo(int pdoGroupID, int pdoID, std::int16_t value)
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->WriteValue(value);
		}
		void ETHERCAT_SLAVE::WritePdo(int pdoGroupID, int pdoID, std::int32_t value)
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->WriteValue(value);
		}
		void ETHERCAT_SLAVE::WritePdo(int pdoGroupID, int pdoID, std::uint8_t value)
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->WriteValue(value);
		}
		void ETHERCAT_SLAVE::WritePdo(int pdoGroupID, int pdoID, std::uint16_t value)
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->WriteValue(value);
		}
		void ETHERCAT_SLAVE::WritePdo(int pdoGroupID, int pdoID, std::uint32_t value)
		{
			pImp->pdoGroups[pdoGroupID].pdos[pdoID]->WriteValue(value);
		}
		void ETHERCAT_SLAVE::ReadSdo(int sdoID, std::int32_t &value) const
		{
			value = this->pImp->sdos[sdoID]->value;
		}
		void ETHERCAT_SLAVE::WriteSdo(int sdoID, std::int32_t value)
		{
			this->pImp->sdos[sdoID]->value = value;
		}

		
		std::atomic_bool ETHERCAT_MASTER::IMP::isStopping;
		void ETHERCAT_MASTER::IMP::Read()
		{
			ecrt_master_receive(pEcMaster);
			for (auto &pSla : slaves)
			{
				pSla->pImp->Read();
			}
		}
		void ETHERCAT_MASTER::IMP::Write()
		{

			for (auto &pSla : slaves)
			{
				pSla->pImp->Write();
			}


			ecrt_master_send(pEcMaster);
		}
		void ETHERCAT_MASTER::IMP::Start()
		{
			static bool isFirstTime{ true };
			if (!isFirstTime)
			{
				throw std::runtime_error("master already running");
			}
			isFirstTime = false;
			
			this->Initialize();
#ifdef PLATFORM_IS_LINUX
			rt_print_auto_init(1);		

			const int priority = 99;
			
			isStopping = false;

			rt_task_create(&ETHERCAT_MASTER::IMP::realtimeCore, "realtime core", 0, priority, T_FPU);
			rt_task_start(&ETHERCAT_MASTER::IMP::realtimeCore, &ETHERCAT_MASTER::IMP::RealTimeCore, NULL);
#endif
		};
		void ETHERCAT_MASTER::IMP::Stop()
		{
			isStopping = true;
#ifdef PLATFORM_IS_LINUX
			rt_task_join(&ETHERCAT_MASTER::IMP::realtimeCore);
#endif
		}
		void ETHERCAT_MASTER::IMP::Initialize()
		{
#ifdef PLATFORM_IS_LINUX
			if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
			{
				throw std::runtime_error("lock failed");
			}
#endif
			pEcMaster = ecrt_request_master(0);
			if (!pEcMaster)
			{
				throw std::runtime_error("master request failed!");
			}
			for (size_t i = 0; i < slaves.size(); ++i)
			{				


				slaves[i]->pImp->position = static_cast<std::uint16_t>(i);
				slaves[i]->Initialize();

			}
			
			ecrt_master_activate(pEcMaster);

			for (auto &pSla : slaves)
			{
				pSla->pImp->pDomainPd = ecrt_domain_data(pSla->pImp->pDomain);
			}
		}
		void ETHERCAT_MASTER::IMP::Sync(uint64_t nanoSecond)
		{
			ecrt_master_application_time(pEcMaster, nanoSecond);
			ecrt_master_sync_reference_clock(pEcMaster);
			ecrt_master_sync_slave_clocks(pEcMaster);
		}
		void ETHERCAT_MASTER::IMP::RealTimeCore(void *)
		{
#ifdef PLATFORM_IS_LINUX
			rt_task_set_periodic(NULL, TM_NOW, samplePeriodNs);

			while (!isStopping)
			{
				rt_task_wait_period(NULL);

				ETHERCAT_MASTER::GetInstance()->pImp->Read();//motors and sensors get data
				
				/*Here is tg*/
				ETHERCAT_MASTER::GetInstance()->ControlStrategy();
				/*tg finished*/

				ETHERCAT_MASTER::GetInstance()->pImp->Sync(rt_timer_read());
				ETHERCAT_MASTER::GetInstance()->pImp->Write();//motor data write and state machine/mode transition
			}
#endif
		};
		
		std::unique_ptr<ETHERCAT_MASTER> ETHERCAT_MASTER::pInstance;
		ETHERCAT_MASTER * ETHERCAT_MASTER::GetInstance()
		{
			if (!pInstance)
			{
				throw std::runtime_error("please first create an instance fo ETHERCAT_MASTER");
			}
			
			return pInstance.get();
		}
		ETHERCAT_MASTER::ETHERCAT_MASTER():pImp(new IMP){}
		ETHERCAT_MASTER::~ETHERCAT_MASTER()	{}
		void ETHERCAT_MASTER::LoadXml(const Aris::Core::ELEMENT *ele)
		{
			/*Load EtherCat slave types*/
			std::map<std::string, const Aris::Core::ELEMENT *> slaveTypeMap;
			
			auto pSlaveTypes = ele->FirstChildElement("SlaveType");
			for (auto pType = pSlaveTypes->FirstChildElement(); pType != nullptr; pType = pType->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(pType->Name()), pType));
			}
			
			/*Load all slaves*/
			auto pSlaves = ele->FirstChildElement("Slave");
			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				this->AddSlave<ETHERCAT_SLAVE>(slaveTypeMap.at(std::string(pSla->Attribute("type"))));
			}
		}
		void ETHERCAT_MASTER::Start()
		{
			this->pImp->Start();
		}
		void ETHERCAT_MASTER::Stop()
		{
			this->pImp->Stop();
		}
		void ETHERCAT_MASTER::AddSlavePtr(ETHERCAT_SLAVE *pSla)
		{
			pImp->slaves.push_back(std::unique_ptr<ETHERCAT_SLAVE>(pSla));
		}

		


	}
}
