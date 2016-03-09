#ifdef WIN32
#include <ecrt_windows_py.h>//just for IDE vs2015, it does not really work
#endif
#ifdef UNIX
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

#include "aris_control_ethercat.h"

namespace Aris
{
	namespace Control
	{
		struct EthercatSlave::Imp
		{
		public:
			Imp() = default;
			void read() { ecrt_domain_process(domain); };
			void write() { ecrt_domain_queue(domain); };

		private:
			/*data object, can be PDO or SDO*/
			class DataObject
			{
			public:
				virtual ~DataObject() = default;
				virtual auto readValue(std::int8_t &value) const->void { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual auto readValue(std::int16_t &value) const->void { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type");};
				virtual auto readValue(std::int32_t &value) const->void { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual auto readValue(std::uint8_t &value) const->void { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual auto readValue(std::uint16_t &value) const->void { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual auto readValue(std::uint32_t &value) const->void { throw std::runtime_error("Param type of ReadValue is not the same with pdo data type"); };
				virtual auto writeValue(std::int8_t value)->void { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual auto writeValue(std::int16_t value)->void { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual auto writeValue(std::int32_t value)->void { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual auto writeValue(std::uint8_t value)->void { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual auto writeValue(std::uint16_t value)->void { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };
				virtual auto writeValue(std::uint32_t value)->void { throw std::runtime_error("Param type of WriteValue is not the same with pdo data type"); };

				Imp *imp;
				std::uint16_t index;
				std::uint8_t subindex;
				std::uint8_t size;
				std::uint32_t offset;

				std::int32_t value;//only for sdo
			};
			template<typename Type>
			class PdoTx :public DataObject
			{
			public:
				virtual ~PdoTx() = default;
				virtual auto readValue(Type &value) const->void override { value = *reinterpret_cast<const Type*>(imp->domain_pd + offset); };
			};
			template<typename Type>
			class PdoRx :public DataObject
			{
			public:
				virtual ~PdoRx() = default;
				virtual auto writeValue(Type value)->void override { *reinterpret_cast<Type*>(imp->domain_pd + offset) = value; };
			};
			class PdoGroup
			{
			public:
				bool is_tx;
				uint16_t index;
				std::vector<std::unique_ptr<DataObject> > pdo_vec;
				std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec;
			};

			std::vector<PdoGroup> pdo_group_vec;
			std::vector<std::unique_ptr<DataObject> > sdo_vec;

			std::uint32_t product_code, vender_id;
			std::uint16_t position, alias;
			std::unique_ptr<int> distributed_clock;

			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec;
			std::vector<ec_pdo_info_t> ec_pdo_info_vec_tx, ec_pdo_info_vec_rx;
			ec_sync_info_t ec_sync_info[5];
			ec_slave_config_t* ec_slave_config;
			
			ec_domain_t* domain;
			std::uint8_t* domain_pd;

			friend class EthercatMaster::Imp;
			friend class EthercatSlave;
		};
		class EthercatMaster::Imp
		{
		public:
			auto read()->void { ecrt_master_receive(ec_master);	for (auto &pSla : slaves)pSla->imp->read(); };
			auto write()->void { for (auto &pSla : slaves)pSla->imp->write();	ecrt_master_send(ec_master); };

		private:
			void init();
			void sync(uint64_t ns)
			{
				ecrt_master_application_time(ec_master, ns);
				ecrt_master_sync_reference_clock(ec_master);
				ecrt_master_sync_slave_clocks(ec_master);
			};
			static void RealTimeCore(void *);

		private:
			std::vector<std::unique_ptr<EthercatSlave> > slaves;
			ec_master_t* ec_master;

			static const int samplePeriodNs;
			static std::atomic_bool isStopping;

#ifdef UNIX
			static RT_TASK realtimeCore;
#endif
			friend class EthercatSlave;
			friend class EthercatMaster;
		};

#ifdef UNIX
		RT_TASK EthercatMaster::Imp::realtimeCore;
		const int EthercatMaster::Imp::samplePeriodNs = 1000000;
#endif

		EthercatSlave::EthercatSlave(const Aris::Core::XmlElement &xml_ele):imp(new Imp)
		{
			//load product id...
			imp->product_code = std::stoi(xml_ele.Attribute("product_code"), nullptr, 0);
			imp->vender_id = std::stoi(xml_ele.Attribute("vender_id"), nullptr, 0);
			imp->alias = std::stoi(xml_ele.Attribute("alias"), nullptr, 0);
			imp->distributed_clock.reset(new std::int32_t);

			if (xml_ele.Attribute("distributed_clock"))
			{
				*imp->distributed_clock.get() = std::stoi(xml_ele.Attribute("distributed_clock"), nullptr, 0);
			}
			else
			{
				imp->distributed_clock.reset();
			}


			//load PDO
			auto addDoType = [](const Aris::Core::XmlElement &ele, bool is_tx)-> Imp::DataObject*
			{
				Imp::DataObject* ret;
				if (ele.Attribute("type", "int8"))
				{
					if (is_tx)
						ret = new EthercatSlave::Imp::PdoTx<std::int8_t>();
					else
						ret = new EthercatSlave::Imp::PdoRx<std::int8_t>();
					ret->size = 8;
				}
				else if (ele.Attribute("type", "uint8"))
				{
					if (is_tx)
						ret = new EthercatSlave::Imp::PdoTx<std::uint8_t>();
					else
						ret = new EthercatSlave::Imp::PdoRx<std::uint8_t>();
					ret->size = 8;
				}
				else if (ele.Attribute("type", "int16"))
				{
					if (is_tx)
						ret = new EthercatSlave::Imp::PdoTx<std::int16_t>();
					else
						ret = new EthercatSlave::Imp::PdoRx<std::int16_t>();
					ret->size = 16;
				}
				else if (ele.Attribute("type", "uint16"))
				{
					if (is_tx)
						ret = new EthercatSlave::Imp::PdoTx<std::uint16_t>();
					else
						ret = new EthercatSlave::Imp::PdoRx<std::uint16_t>();
					ret->size = 16;
				}
				else if (ele.Attribute("type", "int32"))
				{
					if (is_tx)
						ret = new EthercatSlave::Imp::PdoTx<std::int32_t>();
					else
						ret = new EthercatSlave::Imp::PdoRx<std::int32_t>();
					ret->size = 32;
				}
				else if (ele.Attribute("type", "uint32"))
				{
					if (is_tx)
						ret = new EthercatSlave::Imp::PdoTx<std::uint32_t>();
					else
						ret = new EthercatSlave::Imp::PdoRx<std::uint32_t>();
					ret->size = 32;
				}
				else
				{
					throw std::runtime_error("invalid type of pdo");
				}

				return ret;
			};
			auto PDO = xml_ele.FirstChildElement("PDO");
			for (auto p_g = PDO->FirstChildElement(); p_g != nullptr; p_g = p_g->NextSiblingElement())
			{
				Imp::PdoGroup pdo_group;
				pdo_group.index = std::stoi(p_g->Attribute("index"), nullptr, 0);
				pdo_group.is_tx = p_g->Attribute("is_tx", "true") ? true : false;
				for (auto p = p_g->FirstChildElement(); p != nullptr; p = p->NextSiblingElement())
				{
					pdo_group.pdo_vec.push_back(std::unique_ptr<Imp::DataObject>(addDoType(*p, pdo_group.is_tx)));
					pdo_group.pdo_vec.back()->index = std::stoi(p->Attribute("index"), nullptr, 0);
					pdo_group.pdo_vec.back()->subindex = std::stoi(p->Attribute("subindex"), nullptr, 0);
					pdo_group.pdo_vec.back()->imp = imp.get();
				}
				imp->pdo_group_vec.push_back(std::move(pdo_group));
			}

			//load SDO
			auto SDO = xml_ele.FirstChildElement("SDO");
			for (auto s = SDO->FirstChildElement(); s != nullptr; s = s->NextSiblingElement())
			{
				imp->sdo_vec.push_back(std::unique_ptr<Imp::DataObject>(new Imp::DataObject));
				imp->sdo_vec.back()->size = 32;
				imp->sdo_vec.back()->index = std::stoi(s->Attribute("index"), nullptr, 0);
				imp->sdo_vec.back()->subindex = std::stoi(s->Attribute("subindex"), nullptr, 0);
				imp->sdo_vec.back()->imp = imp.get();
				imp->sdo_vec.back()->value = std::stoi(s->Attribute("value"), nullptr, 0);
			}

			//create ecrt structs
			for (auto &p_g : imp->pdo_group_vec)
			{
				for (auto &p : p_g.pdo_vec)
				{
					imp->ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ imp->alias, imp->position, imp->vender_id, imp->product_code, p->index, p->subindex, &p->offset });
					p_g.ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ p->index, p->subindex, p->size });
				}

				if (p_g.is_tx)
				{
					imp->ec_pdo_info_vec_tx.push_back(ec_pdo_info_t{ p_g.index, static_cast<std::uint8_t>(p_g.pdo_vec.size()), p_g.ec_pdo_entry_info_vec.data() });
				}
				else
				{
					imp->ec_pdo_info_vec_rx.push_back(ec_pdo_info_t{ p_g.index, static_cast<std::uint8_t>(p_g.pdo_vec.size()), p_g.ec_pdo_entry_info_vec.data() });
				}
			}
			imp->ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{});

			imp->ec_sync_info[0] = ec_sync_info_t{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE };
			imp->ec_sync_info[1] = ec_sync_info_t{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE };
			imp->ec_sync_info[2] = ec_sync_info_t{ 2, EC_DIR_OUTPUT, static_cast<unsigned int>(imp->ec_pdo_info_vec_rx.size()), imp->ec_pdo_info_vec_rx.data(), EC_WD_ENABLE };
			imp->ec_sync_info[3] = ec_sync_info_t{ 3, EC_DIR_INPUT, static_cast<unsigned int>(imp->ec_pdo_info_vec_tx.size()),imp->ec_pdo_info_vec_tx.data(), EC_WD_ENABLE };
			imp->ec_sync_info[4] = ec_sync_info_t{ 0xff };
		}
		EthercatSlave::~EthercatSlave() {};
		auto EthercatSlave::init()->void
		{
			auto mst = EthercatMaster::instance().imp->ec_master;

			for (auto &reg : imp->ec_pdo_entry_reg_vec)	reg.position = imp->position;

			// Create domain
			if (!(imp->domain = ecrt_master_create_domain(mst)))throw std::runtime_error("failed to create domain");

			// Get the slave configuration 
			if (!(imp->ec_slave_config = ecrt_master_slave_config(mst, imp->alias, imp->position, imp->vender_id, imp->product_code)))
			{
				throw std::runtime_error("failed to slave config");
			}
			// Set Sdo
			for (auto &sdo : imp->sdo_vec)ecrt_slave_config_sdo32(imp->ec_slave_config, sdo->index, sdo->subindex, sdo->value);

			// Configure the slave's PDOs and sync masters
			if (ecrt_slave_config_pdos(imp->ec_slave_config, 4, imp->ec_sync_info))throw std::runtime_error("failed to slave config pdos");

			// Configure the slave's domain
			if (ecrt_domain_reg_pdo_entry_list(imp->domain, imp->ec_pdo_entry_reg_vec.data()))throw std::runtime_error("failed domain_reg_pdo_entry");

			// Configure the slave's discrete clock			
			if (imp->distributed_clock)ecrt_slave_config_dc(imp->ec_slave_config, *imp->distributed_clock.get(), 1000000, 4400000, 0, 0);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::int8_t &value) const->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->readValue(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::int16_t &value) const->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->readValue(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::int32_t &value) const->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->readValue(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::uint8_t &value) const->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->readValue(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::uint16_t &value) const->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->readValue(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::uint32_t &value) const->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->readValue(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::int8_t value)->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->writeValue(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::int16_t value)->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->writeValue(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::int32_t value)->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->writeValue(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::uint8_t value)->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->writeValue(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::uint16_t value)->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->writeValue(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::uint32_t value)->void
		{
			imp->pdo_group_vec[pdoGroupID].pdo_vec[pdoID]->writeValue(value);
		}
		auto EthercatSlave::readSdo(int sdoID, std::int32_t &value) const->void
		{
			value = this->imp->sdo_vec[sdoID]->value;
		}
		auto EthercatSlave::writeSdo(int sdoID, std::int32_t value)->void
		{
			this->imp->sdo_vec[sdoID]->value = value;
		}



		
		std::atomic_bool EthercatMaster::Imp::isStopping;
		void EthercatMaster::Imp::init()
		{
#ifdef UNIX
			if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
			{
				throw std::runtime_error("lock failed");
			}
#endif
			ec_master = ecrt_request_master(0);
			if (!ec_master)
			{
				throw std::runtime_error("master request failed!");
			}
			for (size_t i = 0; i < slaves.size(); ++i)
			{				
				slaves[i]->imp->position = static_cast<std::uint16_t>(i);
				slaves[i]->init();
			}
			
			ecrt_master_activate(ec_master);

			for (auto &pSla : slaves)
			{
				pSla->imp->domain_pd = ecrt_domain_data(pSla->imp->domain);
			}
		}
		void EthercatMaster::Imp::RealTimeCore(void *)
		{
#ifdef UNIX
			rt_task_set_periodic(NULL, TM_NOW, samplePeriodNs);

			while (!isStopping)
			{
				rt_task_wait_period(NULL);

				EthercatMaster::instance().imp->read();//motors and sensors get data
				
				/*Here is tg*/
				EthercatMaster::instance().controlStrategy();
				/*tg finished*/

				EthercatMaster::instance().imp->sync(rt_timer_read());
				EthercatMaster::instance().imp->write();//motor data write and state machine/mode transition
			}
#endif
		};
		
		std::unique_ptr<EthercatMaster> EthercatMaster::pInstance;
		EthercatMaster& EthercatMaster::instance()
		{
			if (!pInstance)
			{
				throw std::runtime_error("please first create an instance fo EthercatMaster");
			}
			
			return *pInstance.get();
		}
		EthercatMaster::EthercatMaster():imp(new Imp){}
		EthercatMaster::~EthercatMaster()	{}
		void EthercatMaster::loadXml(const Aris::Core::XmlElement &xml_ele)
		{
			/*Load EtherCat slave types*/
			std::map<std::string, const Aris::Core::XmlElement *> slaveTypeMap;
			
			auto pSlaveTypes = xml_ele.FirstChildElement("SlaveType");
			for (auto pType = pSlaveTypes->FirstChildElement(); pType != nullptr; pType = pType->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(pType->name()), pType));
			}
			
			/*Load all slaves*/
			auto pSlaves = xml_ele.FirstChildElement("Slave");
			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				this->addSlave<EthercatSlave>(std::ref(*slaveTypeMap.at(std::string(pSla->Attribute("type")))));
			}
		}
		void EthercatMaster::start() 
		{
			static bool is_first_time{ true };
			if (!is_first_time)
			{
				throw std::runtime_error("master already running");
			}
			is_first_time = false;

			imp->init();
#ifdef UNIX
			rt_print_auto_init(1);

			const int priority = 99;

			Imp::isStopping = false;

			rt_task_create(&EthercatMaster::Imp::realtimeCore, "realtime core", 0, priority, T_FPU);
			rt_task_start(&EthercatMaster::Imp::realtimeCore, &EthercatMaster::Imp::RealTimeCore, NULL);
#endif
		};
		void EthercatMaster::stop()
		{
			EthercatMaster::Imp::isStopping = true;
#ifdef UNIX
			rt_task_join(&EthercatMaster::Imp::realtimeCore);
#endif
		}
		void EthercatMaster::addSlavePtr(EthercatSlave *pSla)
		{
			imp->slaves.push_back(std::unique_ptr<EthercatSlave>(pSla));
		}
	}
}
