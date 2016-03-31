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
#include <sstream>
#include <map>
#include <atomic>
#include <memory>
#include <typeinfo>

#include "aris_control_ethercat.h"

namespace aris
{
	namespace control
	{
		class EthercatSlave::Imp
		{
		public:
			auto read()->void
			{
#ifdef UNIX 
				ecrt_domain_process(domain_);
#endif
			};
			auto write()->void
			{
#ifdef UNIX
				ecrt_domain_queue(domain_);
#endif
			};
			auto init()->void;

			class DataObject
			{
			public:
				// common data
				Imp *imp_;
				std::string type_name_;
				std::uint16_t index_;
				std::uint8_t subindex_;
				std::uint8_t size_;
				std::uint32_t offset_;

				DataObject(const aris::core::XmlElement &xml_ele, Imp *imp)
				{
					if ((!xml_ele.Attribute("type")) || DataObject::typeInfoMap().find(xml_ele.Attribute("type")) == DataObject::typeInfoMap().end())
						throw std::runtime_error("invalid type in ethercat configuration \"" + std::string(xml_ele.name()) + "\"");
					else type_name_ = xml_ele.Attribute("type");

					index_ = std::stoi(xml_ele.Attribute("index"), nullptr, 0);
					subindex_ = std::stoi(xml_ele.Attribute("subindex"), nullptr, 0);
					imp_ = imp;
					size_ = typeInfoMap().at(type_name_).size_;
				}

				class TypeInfo
				{
				public:
					const std::type_info *type_info_;
					std::uint8_t size_;
					std::function<void(const std::string &, void *)> queryFunc;

					template<typename DataType> static auto createTypeInfo()->TypeInfo
					{
						TypeInfo info;

						info.type_info_ = &typeid(DataType);
						info.size_ = sizeof(DataType) * 8;

						if (typeid(DataType) == typeid(std::int8_t) || typeid(DataType) == typeid(std::uint8_t))
						{
							info.queryFunc = [](const std::string &str, void *value)
							{
								std::int16_t loc_value;
								std::stringstream(str) >> loc_value;
								(*reinterpret_cast<DataType*>(value)) = DataType(loc_value);
							};
						}
						else
						{
							info.queryFunc = [](const std::string &str, void *value) {std::stringstream(str) >> (*reinterpret_cast<DataType*>(value)); };
						}

						return info;
					}
				};

				static auto typeInfoMap()->const std::map<std::string, TypeInfo>&
				{
					const static std::map<std::string, TypeInfo> info_map
					{
						std::make_pair(std::string("int8"), TypeInfo::createTypeInfo<std::int8_t>()),
						std::make_pair(std::string("uint8"), TypeInfo::createTypeInfo<std::uint8_t>()),
						std::make_pair(std::string("int16"), TypeInfo::createTypeInfo<std::int16_t>()),
						std::make_pair(std::string("uint16"), TypeInfo::createTypeInfo<std::uint16_t>()),
						std::make_pair(std::string("int32"), TypeInfo::createTypeInfo<std::int32_t>()),
						std::make_pair(std::string("uint32"), TypeInfo::createTypeInfo<std::uint32_t>()),
					};

					return std::ref(info_map);
				}

			};
			class Pdo :public DataObject
			{
			public:
				template<typename DataType>auto readPdo(DataType&data)->void
				{
					if (typeid(DataType) != *typeInfoMap().at(type_name_).type_info_) throw std::runtime_error("invalid pdo Tx type");
					data = *reinterpret_cast<const DataType*>(imp_->domain_pd_ + offset_);
				};
				template<typename DataType>auto writePdo(DataType data)->void
				{
					if (typeid(DataType) != *typeInfoMap().at(type_name_).type_info_) throw std::runtime_error("invalid pdo Rx type");
					*reinterpret_cast<DataType*>(imp_->domain_pd_ + offset_) = data;
				};

				Pdo(const aris::core::XmlElement &xml_ele, Imp *imp) :DataObject(xml_ele, imp) {};
			};
			class Sdo :public DataObject
			{
			public:
				enum Option
				{
					READ = 0x01,
					WRITE = 0x02,
					CONFIG = 0x04
				};
				unsigned option_;

				union
				{
					char config_value_[8];
					std::uint32_t config_value_uint32_;
					std::uint16_t config_value_uint16_;
					std::uint8_t config_value_uint8_;
					std::int32_t config_value_int32_;
					std::int16_t config_value_int16_;
					std::int8_t config_value_int8_;
				};

				template<typename DataType>auto readSdoConfig(DataType&data)->void
				{
					if (!(option_ & CONFIG)) throw std::runtime_error("sdo is not config-able");
					if (typeid(DataType) != *typeInfoMap().at(type_name_).type_info_)throw std::runtime_error("invalid sdo type when readSdoConfig sdo");
					data = (*reinterpret_cast<DataType *>(config_value_));
				};
				template<typename DataType>auto configSdo(DataType data)->void
				{
					if (!(option_ & CONFIG)) throw std::runtime_error("sdo is not config-able");
					if (typeid(DataType) != *typeInfoMap().at(type_name_).type_info_)throw std::runtime_error("invalid sdo type when config sdo");
					(*reinterpret_cast<DataType *>(config_value_)) = data;
				};
				template<typename DataType>auto readSdo(DataType &data)->void;
				template<typename DataType>auto writeSdo(DataType data)->void;

				Sdo(const aris::core::XmlElement &xml_ele, Imp *imp) :DataObject(xml_ele, imp), option_(READ | WRITE)
				{
					if ((!xml_ele.Attribute("read")) || xml_ele.Attribute("read", "true")) option_ |= READ;
					else if (xml_ele.Attribute("read", "false")) option_ &= ~READ;
					else throw std::runtime_error("invalid value of field READ in xml file for ethercat configuration \"" + std::string(xml_ele.name()) + "\"");

					if ((!xml_ele.Attribute("write")) || xml_ele.Attribute("write", "true")) option_ |= WRITE;
					else if (xml_ele.Attribute("write", "false")) option_ &= ~WRITE;
					else throw std::runtime_error("invalid value of field WRITE in xml file for ethercat configuration \"" + std::string(xml_ele.name()) + "\"");

					if (xml_ele.Attribute("config"))
					{
						if (!(option_ & WRITE))throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + std::string(xml_ele.name()) + "\" sdo");
						option_ |= CONFIG;
						DataObject::typeInfoMap().at(xml_ele.Attribute("type")).queryFunc(xml_ele.Attribute("config"), config_value_);
					}
				};
			};
			class PdoGroup
			{
			public:
				bool is_tx;
				uint16_t index;
				std::vector<std::unique_ptr<Pdo> > pdo_vec;
				std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec;

				PdoGroup(const aris::core::XmlElement &xml_ele, Imp *imp)
				{
					index = std::stoi(xml_ele.Attribute("index"), nullptr, 0);
					is_tx = xml_ele.Attribute("is_tx", "true") ? true : false;
					for (auto p = xml_ele.FirstChildElement(); p; p = p->NextSiblingElement())
					{
						pdo_vec.push_back(std::unique_ptr<Imp::Pdo>(new Pdo(*p, imp)));
					}
				}
			};

			std::vector<PdoGroup> pdo_group_vec_;
			std::vector<std::unique_ptr<Sdo> > sdo_vec_;

			std::uint32_t product_code_, vender_id_;
			std::uint16_t position_, alias_;
			std::unique_ptr<int> distributed_clock_;

			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec_;
			std::vector<ec_pdo_info_t> ec_pdo_info_vec_tx_, ec_pdo_info_vec_rx_;
			ec_sync_info_t ec_sync_info_[5];
			ec_slave_config_t* ec_slave_config_;

			ec_domain_t* domain_;
			std::uint8_t* domain_pd_;

			friend class EthercatMaster::Imp;
			friend class EthercatSlave;
			friend class EthercatMaster;
		};
		class EthercatMaster::Imp
		{
		public:
			static auto rt_task_func(void *)->void
			{
#ifdef UNIX
				auto &mst = EthercatMaster::instance();

				rt_task_set_periodic(NULL, TM_NOW, mst.imp_->sample_period_ns_);

				while (!mst.imp_->is_stopping_)
				{
					rt_task_wait_period(NULL);

					mst.imp_->read();//motors and sensors get data

									 //tg begin
					mst.controlStrategy();
					//tg end

					mst.imp_->sync(rt_timer_read());
					mst.imp_->write();//motor data write and state machine/mode transition
				}
#endif
			};
			auto read()->void { ecrt_master_receive(ec_master_);	for (auto &sla : slave_vec_)sla->imp_->read(); };
			auto write()->void { for (auto &sla : slave_vec_)sla->imp_->write(); ecrt_master_send(ec_master_); };
			auto sync(uint64_t ns)->void
			{
				ecrt_master_application_time(ec_master_, ns);
				ecrt_master_sync_reference_clock(ec_master_);
				ecrt_master_sync_slave_clocks(ec_master_);
			};

			std::vector<std::unique_ptr<EthercatSlave> > slave_vec_;
			ec_master_t* ec_master_;

			const int sample_period_ns_ = 1000000;

			std::atomic_bool is_running_{ false }, is_stopping_{ false };

#ifdef UNIX
			RT_TASK rt_task_;
#endif
			friend class EthercatSlave;
			friend class EthercatMaster;
		};

		auto EthercatSlave::Imp::init()->void
		{
			auto &ec_mst = EthercatMaster::instance().imp_->ec_master_;

#ifdef UNIX
			for (auto &reg : this->ec_pdo_entry_reg_vec_)	reg.position = this->position_;

			// Create domain
			if (!(this->domain_ = ecrt_master_create_domain(ec_mst)))throw std::runtime_error("failed to create domain");

			// Get the slave configuration 
			if (!(this->ec_slave_config_ = ecrt_master_slave_config(ec_mst, this->alias_, this->position_, this->vender_id_, this->product_code_)))
			{
				throw std::runtime_error("failed to slave config");
			}
			// Config Sdo
			for (auto &sdo : this->sdo_vec_)
			{
				if (!(sdo->option_ & Sdo::CONFIG)) continue;

				switch (sdo->size_)
				{
				case 8:		ecrt_slave_config_sdo8(this->ec_slave_config_, sdo->index_, sdo->subindex_, sdo->config_value_uint8_); break;
				case 16:	ecrt_slave_config_sdo16(this->ec_slave_config_, sdo->index_, sdo->subindex_, sdo->config_value_uint16_); break;
				case 32:	ecrt_slave_config_sdo32(this->ec_slave_config_, sdo->index_, sdo->subindex_, sdo->config_value_uint32_); break;
				default:    throw std::runtime_error("invalid size of sdo, it must be 8, 16 or 32");
				}
			}

			//for (auto &sdo : this->sdo_vec_)
			//{
			//	if (sdo->is_tx_ != Sdo::RX) continue;

			//	std::uint32_t code;
			//	std::size_t real_size;
			//	
			//	ecrt_master_sdo_upload(
			//		mst, /**< EtherCAT master. */
			//		this->position, /**< Slave position. */
			//		sdo->index_, /**< Index of the SDO. */
			//		sdo->subindex_, /**< Subindex of the SDO. */
			//		&sdo->config_value_uint8_, /**< Target buffer for the upload. */
			//		sdo->size_, /**< Size of the target buffer. */
			//		&real_size, /**< Uploaded data size. */
			//		&code /**< Abort code of the SDO upload. */
			//		);

			//	std::cout <<"sdo value:"<< static_cast<int>(sdo->config_value_int32_)<<std::endl;
			//}

			// Configure the slave's PDOs and sync masters
			if (ecrt_slave_config_pdos(this->ec_slave_config_, 4, this->ec_sync_info_))throw std::runtime_error("failed to slave config pdos");

			// Configure the slave's domain
			if (ecrt_domain_reg_pdo_entry_list(this->domain_, this->ec_pdo_entry_reg_vec_.data()))throw std::runtime_error("failed domain_reg_pdo_entry");

			// Configure the slave's discrete clock			
			if (this->distributed_clock_)ecrt_slave_config_dc(this->ec_slave_config_, *this->distributed_clock_.get(), 1000000, 4400000, 0, 0);
#endif
		};
		template<typename DataType>
		auto EthercatSlave::Imp::Sdo::readSdo(DataType &data)->void
		{
			if (!(option_ & READ)) throw std::runtime_error("sdo is not read-able");
			if (typeid(DataType) != *typeInfoMap().at(type_name_).type_info_)throw std::runtime_error("invalid sdo type when read sdo");

#ifdef UNIX
			auto &ec_mst = EthercatMaster::instance().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;

			ecrt_master_sdo_upload(ec_mst, imp_->position_, index_, subindex_, reinterpret_cast<std::uint8_t *>(&data), size_, &real_size, &abort_code);
#endif
		};
		template<typename DataType>
		auto EthercatSlave::Imp::Sdo::writeSdo(DataType data)->void
		{
			if (!(option_ & WRITE)) throw std::runtime_error("sdo is not write-able");
			if (typeid(DataType) != *typeInfoMap().at(type_name_).type_info_)throw std::runtime_error("invalid sdo type when read sdo");

#ifdef UNIX
			auto &ec_mst = EthercatMaster::instance().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;

			ecrt_master_sdo_download(ec_mst, imp_->position_, index_, subindex_, reinterpret_cast<std::uint8_t *>(&data), size_, &abort_code);
#endif
		}

		EthercatSlave::EthercatSlave(const aris::core::XmlElement &xml_ele) :imp_(new Imp)
		{
			//load product id...
			imp_->product_code_ = std::stoi(xml_ele.Attribute("product_code"), nullptr, 0);
			imp_->vender_id_ = std::stoi(xml_ele.Attribute("vender_id"), nullptr, 0);
			imp_->alias_ = std::stoi(xml_ele.Attribute("alias"), nullptr, 0);
			imp_->distributed_clock_.reset(new std::int32_t);

			if (xml_ele.Attribute("distributed_clock"))
			{
				*imp_->distributed_clock_.get() = std::stoi(xml_ele.Attribute("distributed_clock"), nullptr, 0);
			}
			else
			{
				imp_->distributed_clock_.reset();
			}


			// load PDO
			auto pdo_xml_ele = xml_ele.FirstChildElement("PDO");
			for (auto p = pdo_xml_ele->FirstChildElement(); p; p = p->NextSiblingElement())
			{
				imp_->pdo_group_vec_.push_back(Imp::PdoGroup(*p, imp_.get()));
			}

			// load SDO
			auto sdo_xml_ele = xml_ele.FirstChildElement("SDO");
			for (auto s = sdo_xml_ele->FirstChildElement(); s; s = s->NextSiblingElement())
			{
				imp_->sdo_vec_.push_back(std::unique_ptr<Imp::Sdo>(new Imp::Sdo(*s, imp_.get())));
			}

			// create ecrt structs
			for (auto &p_g : imp_->pdo_group_vec_)
			{
				for (auto &p : p_g.pdo_vec)
				{
					imp_->ec_pdo_entry_reg_vec_.push_back(ec_pdo_entry_reg_t{ imp_->alias_, imp_->position_, imp_->vender_id_, imp_->product_code_, p->index_, p->subindex_, &p->offset_ });
					p_g.ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ p->index_, p->subindex_, p->size_ });
				}

				if (p_g.is_tx)
				{
					imp_->ec_pdo_info_vec_tx_.push_back(ec_pdo_info_t{ p_g.index, static_cast<std::uint8_t>(p_g.pdo_vec.size()), p_g.ec_pdo_entry_info_vec.data() });
				}
				else
				{
					imp_->ec_pdo_info_vec_rx_.push_back(ec_pdo_info_t{ p_g.index, static_cast<std::uint8_t>(p_g.pdo_vec.size()), p_g.ec_pdo_entry_info_vec.data() });
				}
			}
			imp_->ec_pdo_entry_reg_vec_.push_back(ec_pdo_entry_reg_t{});

			imp_->ec_sync_info_[0] = ec_sync_info_t{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE };
			imp_->ec_sync_info_[1] = ec_sync_info_t{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE };
			imp_->ec_sync_info_[2] = ec_sync_info_t{ 2, EC_DIR_OUTPUT, static_cast<unsigned int>(imp_->ec_pdo_info_vec_rx_.size()), imp_->ec_pdo_info_vec_rx_.data(), EC_WD_ENABLE };
			imp_->ec_sync_info_[3] = ec_sync_info_t{ 3, EC_DIR_INPUT, static_cast<unsigned int>(imp_->ec_pdo_info_vec_tx_.size()),imp_->ec_pdo_info_vec_tx_.data(), EC_WD_ENABLE };
			imp_->ec_sync_info_[4] = ec_sync_info_t{ 0xff };
		}
		EthercatSlave::~EthercatSlave() {};
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::int8_t &value) const->void
		{
			if (!imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't read pdo in rx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->readPdo(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::int16_t &value) const->void
		{
			if (!imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't read pdo in rx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->readPdo(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::int32_t &value) const->void
		{
			if (!imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't read pdo in rx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->readPdo(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::uint8_t &value) const->void
		{
			if (!imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't read pdo in rx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->readPdo(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::uint16_t &value) const->void
		{
			if (!imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't read pdo in rx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->readPdo(value);
		}
		auto EthercatSlave::readPdo(int pdoGroupID, int pdoID, std::uint32_t &value) const->void
		{
			if (!imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't read pdo in rx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->readPdo(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::int8_t value)->void
		{
			if (imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't write pdo in tx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->writePdo(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::int16_t value)->void
		{
			if (imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't write pdo in tx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->writePdo(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::int32_t value)->void
		{
			if (imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't write pdo in tx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->writePdo(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::uint8_t value)->void
		{
			if (imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't write pdo in tx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->writePdo(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::uint16_t value)->void
		{
			if (imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't write pdo in tx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->writePdo(value);
		}
		auto EthercatSlave::writePdo(int pdoGroupID, int pdoID, std::uint32_t value)->void
		{
			if (imp_->pdo_group_vec_[pdoGroupID].is_tx) throw std::runtime_error("you can't write pdo in tx pdo");
			imp_->pdo_group_vec_[pdoGroupID].pdo_vec[pdoID]->writePdo(value);
		}
		auto EthercatSlave::readSdoConfig(int sdoID, std::int8_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdoConfig(value); }
		auto EthercatSlave::readSdoConfig(int sdoID, std::int16_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdoConfig(value); }
		auto EthercatSlave::readSdoConfig(int sdoID, std::int32_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdoConfig(value); }
		auto EthercatSlave::readSdoConfig(int sdoID, std::uint8_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdoConfig(value); }
		auto EthercatSlave::readSdoConfig(int sdoID, std::uint16_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdoConfig(value); }
		auto EthercatSlave::readSdoConfig(int sdoID, std::uint32_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdoConfig(value); }
		auto EthercatSlave::configSdo(int sdoID, std::int8_t value)->void { imp_->sdo_vec_[sdoID]->configSdo(value); }
		auto EthercatSlave::configSdo(int sdoID, std::int16_t value)->void { imp_->sdo_vec_[sdoID]->configSdo(value); }
		auto EthercatSlave::configSdo(int sdoID, std::int32_t value)->void { imp_->sdo_vec_[sdoID]->configSdo(value); }
		auto EthercatSlave::configSdo(int sdoID, std::uint8_t value)->void { imp_->sdo_vec_[sdoID]->configSdo(value); }
		auto EthercatSlave::configSdo(int sdoID, std::uint16_t value)->void { imp_->sdo_vec_[sdoID]->configSdo(value); }
		auto EthercatSlave::configSdo(int sdoID, std::uint32_t value)->void { imp_->sdo_vec_[sdoID]->configSdo(value); }
		auto EthercatSlave::readSdo(int sdoID, std::int8_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdo(value); }
		auto EthercatSlave::readSdo(int sdoID, std::int16_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdo(value); }
		auto EthercatSlave::readSdo(int sdoID, std::int32_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdo(value); }
		auto EthercatSlave::readSdo(int sdoID, std::uint8_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdo(value); }
		auto EthercatSlave::readSdo(int sdoID, std::uint16_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdo(value); }
		auto EthercatSlave::readSdo(int sdoID, std::uint32_t &value) const->void { imp_->sdo_vec_[sdoID]->readSdo(value); }
		auto EthercatSlave::writeSdo(int sdoID, std::int8_t value)->void { imp_->sdo_vec_[sdoID]->writeSdo(value); }
		auto EthercatSlave::writeSdo(int sdoID, std::int16_t value)->void { imp_->sdo_vec_[sdoID]->writeSdo(value); }
		auto EthercatSlave::writeSdo(int sdoID, std::int32_t value)->void { imp_->sdo_vec_[sdoID]->writeSdo(value); }
		auto EthercatSlave::writeSdo(int sdoID, std::uint8_t value)->void { imp_->sdo_vec_[sdoID]->writeSdo(value); }
		auto EthercatSlave::writeSdo(int sdoID, std::uint16_t value)->void { imp_->sdo_vec_[sdoID]->writeSdo(value); }
		auto EthercatSlave::writeSdo(int sdoID, std::uint32_t value)->void { imp_->sdo_vec_[sdoID]->writeSdo(value); }

		EthercatMaster::EthercatMaster() :imp_(new Imp) {}
		EthercatMaster::~EthercatMaster() {}
		auto EthercatMaster::instance()->EthercatMaster&
		{
			if (!instancePtr())throw std::runtime_error("please first create an instance fo EthercatMaster");
			return *instancePtr().get();
		};
		auto EthercatMaster::instancePtr() -> const std::unique_ptr<EthercatMaster> &
		{
			static std::unique_ptr<EthercatMaster> instance_ptr;
			return std::ref(instance_ptr);
		};
		auto EthercatMaster::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			// Load EtherCat slave types //
			std::map<std::string, const aris::core::XmlElement *> slaveTypeMap;

			auto pSlaveTypes = xml_ele.FirstChildElement("SlaveType");
			for (auto pType = pSlaveTypes->FirstChildElement(); pType != nullptr; pType = pType->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(pType->name()), pType));
			}

			// Load all slaves //
			auto pSlaves = xml_ele.FirstChildElement("Slave");
			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				this->addSlave<EthercatSlave>(std::ref(*slaveTypeMap.at(std::string(pSla->Attribute("type")))));
			}
		}
		auto EthercatMaster::start()->void
		{
			if (imp_->is_running_)throw std::runtime_error("master already running");
			imp_->is_running_ = true;

			// init begin //
#ifdef UNIX
			if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) { throw std::runtime_error("lock failed"); }

			imp_->ec_master_ = ecrt_request_master(0);
			if (!imp_->ec_master_) { throw std::runtime_error("master request failed!"); }

			for (size_t i = 0; i < imp_->slave_vec_.size(); ++i)
			{
				imp_->slave_vec_[i]->imp_->position_ = static_cast<std::uint16_t>(i);
				imp_->slave_vec_[i]->imp_->init();
				imp_->slave_vec_[i]->init();
			}

			ecrt_master_activate(imp_->ec_master_);

			for (auto &sla : imp_->slave_vec_)
			{
				sla->imp_->domain_pd_ = ecrt_domain_data(sla->imp_->domain_);
			}
			//init end

			rt_print_auto_init(1);
			const int priority = 99;

			rt_task_create(&imp_->rt_task_, "realtime core", 0, priority, T_FPU);
			rt_task_start(&imp_->rt_task_, &EthercatMaster::Imp::rt_task_func, NULL);
#endif
		};
		auto EthercatMaster::stop()->void
		{
			if (!imp_->is_running_)throw std::runtime_error("master is not running, so can't stop");

			imp_->is_stopping_ = true;
#ifdef UNIX
			rt_task_delete(&imp_->rt_task_);

			ecrt_master_deactivate(imp_->ec_master_);
			ecrt_release_master(imp_->ec_master_);
#endif
			imp_->is_stopping_ = false;
			imp_->is_running_ = false;
		}
		auto EthercatMaster::addSlavePtr(EthercatSlave *pSla)->void
		{
			imp_->slave_vec_.push_back(std::unique_ptr<EthercatSlave>(pSla));
		}
	}
}
