#include <Platform.h>
#ifdef PLATFORM_IS_WINDOWS
#include <ecrt_windows_py.h>
#endif
#ifdef PLATFORM_IS_LINUX
#include <ecrt.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#endif


#include <string>

#include <Aris_Device.h>


namespace Aris
{
	namespace Control
	{
		class ETHERCAT_SLAVE::IMP 
		{
		public:
			IMP(Aris::Core::ELEMENT *ele);
			void Initialize();
			void Read();
			void Write();

		private:
			struct PDO
			{
				std::uint16_t index;
				std::uint8_t subIndex;
				std::uint32_t offset;
				std::uint8_t size;
			};
			class POD_GROUP
			{
			public:
				bool isTx;
				uint16_t index;
				std::vector<PDO> pdos;
				std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec;
			};
			std::vector<POD_GROUP> pdoGroups;
			std::uint32_t productCode, venderID;
			std::uint16_t position, alias;

			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec;
			std::vector<ec_pdo_info_t> ec_pdo_info_vec_Tx, ec_pdo_info_vec_Rx;
			ec_sync_info_t ec_sync_info[5];
			ec_slave_config_t* ec_slave_config;
			
			ec_domain_t* pDomain;
			std::uint8_t* pDomainPd;

			friend class ETHERCAT_MASTER::IMP;
		};
		class ETHERCAT_MASTER::IMP
		{
		public:
			void LoadXml(Aris::Core::ELEMENT *ele);
			void Initialize();
			void Read();
			void Write();
			void Run();
			
		private:
			void Sync(uint64_t nanoSecond);
			static void RealTimeCore(void *);

		private:
			std::vector<std::unique_ptr<ETHERCAT_SLAVE> > devices;
			ec_master_t* pEcMaster;
			static RT_TASK realtimeCore;
			
			friend class ETHERCAT_SLAVE::IMP;
		};
		RT_TASK ETHERCAT_MASTER::IMP::realtimeCore;


		

		ETHERCAT_SLAVE::IMP::IMP(Aris::Core::ELEMENT *ele)
		{
			/*load product id...*/
			this->productCode = std::stoi(ele->Attribute("productCode"), nullptr, 0);
			this->venderID = std::stoi(ele->Attribute("venderID"), nullptr, 0);
			this->alias = std::stoi(ele->Attribute("alias"), nullptr, 0);

			/*load pdos*/
			auto PDO = ele->FirstChildElement("PDO");
			for (auto p_g = PDO->FirstChildElement(); p_g != nullptr; p_g = p_g->NextSiblingElement())
			{
				POD_GROUP pdo_group;
				pdo_group.index = std::stoi(p_g->Attribute("index"), nullptr, 0);
				pdo_group.isTx = p_g->Attribute("isTx", "true") ? true : false;
				for (auto p = p_g->FirstChildElement(); p != nullptr; p = p->NextSiblingElement())
				{
					IMP::PDO pdo;
					pdo.index = std::stoi(p->Attribute("index"), nullptr, 0);
					pdo.subIndex = std::stoi(p->Attribute("subIndex"), nullptr, 0);
					pdo.size = std::stoi(p->Attribute("size"), nullptr, 0);

					pdo_group.pdos.push_back(pdo);
				}
				pdoGroups.push_back(pdo_group);
			}


			/*create ecrt structs*/
			for (auto &p_g : pdoGroups)
			{
				for (auto &p : p_g.pdos)
				{
					ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ alias,position,venderID,productCode,p.index,p.subIndex,&p.offset });
					p_g.ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ p.index,p.subIndex,p.size });
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
			ec_sync_info[2] = ec_sync_info_t{ 2, EC_DIR_OUTPUT, 1, ec_pdo_info_vec_Rx.data(), EC_WD_ENABLE };
			ec_sync_info[3] = ec_sync_info_t{ 3, EC_DIR_INPUT, 4, ec_pdo_info_vec_Tx.data(), EC_WD_ENABLE };
			ec_sync_info[4] = ec_sync_info_t{ 0xff };
		};
		void ETHERCAT_SLAVE::IMP::Initialize()
		{
			auto pEcMaster = ETHERCAT_MASTER::GetInstance()->pImp->pEcMaster;

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

			/*
			// Configure initial parameters by writing SDO
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x6098, 0, settings.homeMode);     //homing mode
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x609A, 0, settings.homeAccel);      //homing acc
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x6099, 1, settings.homeHighSpeed); //high speed
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x6099, 2, settings.homeLowSpeed); //low speed which we don't use here
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x2020, 1, settings.homeTorqueLimit);  //torque limit force

																							  //    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6095,1,1);
																							  //    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6095,2,1);
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x6096, 1, 1);   // Ratio of Position/Velocity, Numerator
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x6096, 2, 1);   // Ratio of Position/Velocity, Divisor


																		//set ptp max profile speed
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x607F, 0, settings.p2pMaxSpeed);
			//set ptp profile speed
			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x6081, 0, settings.p2pSpeed);

			ecrt_slave_config_sdo32(m_slaveConfigElmo, 0x607C, 0, homeOffset);
			*/

			// Configure the slave's PDOs and sync masters
			if (ecrt_slave_config_pdos(ec_slave_config, 4, ec_sync_info))
			{
				throw std::runtime_error("failed to slave config pdos");
			}

			// Configure the slave's domain
			if (ecrt_domain_reg_pdo_entry_list(pDomain, ec_pdo_entry_reg_vec.data()))
			{
				throw std::runtime_error("failed to slave config pdos");
			}

			// Configure the slave's discrete clock
			ecrt_slave_config_dc(ec_slave_config, 0x0300, 1000000, 4400000, 0, 0);
		}
		void ETHERCAT_SLAVE::IMP::Read()
		{
			ecrt_domain_process(pDomain);
			auto vel = EC_READ_S32(pDomainPd + this->pdoGroups[1].pdos[0].offset);

			static int i = 0;

			if (i % 500 == 0)
			{
#ifdef PLATFORM_IS_LINUX
				rt_printf("position is : %d\n", vel);
#endif
			}

			++i;
		}
		void ETHERCAT_SLAVE::IMP::Write()
		{
			EC_WRITE_S32(pDomainPd + this->pdoGroups[0].pdos[0].offset, 0);
			EC_WRITE_S16(pDomainPd + this->pdoGroups[0].pdos[1].offset, 0);
			EC_WRITE_S32(pDomainPd + this->pdoGroups[0].pdos[2].offset, 0);
			EC_WRITE_S16(pDomainPd + this->pdoGroups[0].pdos[3].offset, 0);
			EC_WRITE_U16(pDomainPd + this->pdoGroups[0].pdos[4].offset, 0);
			EC_WRITE_U8(pDomainPd + this->pdoGroups[0].pdos[5].offset, 0);
			ecrt_domain_queue(pDomain);
		}

		ETHERCAT_SLAVE::ETHERCAT_SLAVE(Aris::Core::ELEMENT *ele)
		{
			this->pImp = new IMP{ ele };
		}
		ETHERCAT_SLAVE::~ETHERCAT_SLAVE()
		{
			delete this->pImp;
		}
		void ETHERCAT_SLAVE::Initialize()
		{
			this->pImp->Initialize();
		}
		void ETHERCAT_SLAVE::Read()
		{
			pImp->Read();
		}
		void ETHERCAT_SLAVE::Write()
		{
			pImp->Write();
		}

		void ETHERCAT_MASTER::IMP::LoadXml(Aris::Core::ELEMENT *ele)
		{
			auto pSlaves = ele->FirstChildElement("Slave");

			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				this->devices.push_back(std::unique_ptr<ETHERCAT_SLAVE>(new ETHERCAT_SLAVE(pSla)));
				this->devices.back()->pImp->position = static_cast<std::int16_t>(this->devices.size() - 1);
			}
		}
		void ETHERCAT_MASTER::IMP::Initialize()
		{
			static bool isFirstTime{ true };
			if (isFirstTime)
			{
				
#ifdef PLARFORM_IS_LINUX
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
				isFirstTime = false;
			}

			for (auto &pSla : devices)
			{
				pSla->Initialize();
			}

			ecrt_master_activate(pEcMaster);

			for (auto &pSla : devices)
			{
				pSla->pImp->pDomainPd = ecrt_domain_data(pSla->pImp->pDomain);
			}
		}
		void ETHERCAT_MASTER::IMP::Read()
		{
			ecrt_master_receive(pEcMaster);
			for (auto &pSla : devices)
			{
				pSla->Read();
			}
		}
		void ETHERCAT_MASTER::IMP::Write()
		{
			for (auto &pSla : devices)
			{
				pSla->Write();
			}
			ecrt_master_send(pEcMaster);
		}
		void ETHERCAT_MASTER::IMP::Run()
		{
#define TASK_PRIO_CORE 99
			int ret;
			ret = rt_task_create(&ETHERCAT_MASTER::IMP::realtimeCore, "realtime core", 0, TASK_PRIO_CORE, T_FPU);
			ret = rt_task_start(&ETHERCAT_MASTER::IMP::realtimeCore, &ETHERCAT_MASTER::IMP::RealTimeCore, NULL);
		};
		void ETHERCAT_MASTER::IMP::Sync(uint64_t nanoSecond)
		{
			ecrt_master_application_time(pEcMaster, nanoSecond);
			ecrt_master_sync_reference_clock(pEcMaster);
			ecrt_master_sync_slave_clocks(pEcMaster);
		}
		void ETHERCAT_MASTER::IMP::RealTimeCore(void *)
		{
			rt_printf("Realtime Core started.\n");

			//float period;
#define PERIOD_NS_CORE 1000000
			rt_task_set_periodic(NULL, TM_NOW, PERIOD_NS_CORE);

			while (1)
			{
				rt_task_wait_period(NULL);

				ETHERCAT_MASTER::GetInstance()->pImp->Read();//motors and sensors get data
				ETHERCAT_MASTER::GetInstance()->pImp->Sync(rt_timer_read());
				ETHERCAT_MASTER::GetInstance()->pImp->Write();//motor data write and state machine/mode transition
			}
		};
		

		ETHERCAT_MASTER ETHERCAT_MASTER::instance;
		ETHERCAT_MASTER * ETHERCAT_MASTER::GetInstance()
		{
			return &ETHERCAT_MASTER::instance;
		}
		ETHERCAT_MASTER::ETHERCAT_MASTER()
		{
			pImp = new IMP;
		}
		ETHERCAT_MASTER::~ETHERCAT_MASTER()
		{
			delete pImp;
		}
		void ETHERCAT_MASTER::LoadXml(Aris::Core::ELEMENT *ele)
		{
			pImp->LoadXml(ele);
		}
		void ETHERCAT_MASTER::Initialize()
		{
			pImp->Initialize();
		}
		void ETHERCAT_MASTER::Read()
		{
			pImp->Read();
		}
		void ETHERCAT_MASTER::Write()
		{
			pImp->Write();
		}
		void ETHERCAT_MASTER::Run()
		{
			pImp->Run();
		}


		


	}
}