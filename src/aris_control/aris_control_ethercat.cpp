#include <mutex>
#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <atomic>
#include <memory>
#include <typeinfo>
#include <thread>
#include <chrono>
#include <future>

#include "aris_control_ethercat_kernel.h"
#include "aris_control_ethercat.h"
#include "aris_control_motion.h"

namespace aris
{
	namespace control
	{
		struct DO::Imp
		{
			std::uint16_t index_;
			std::uint8_t subindex_;
			aris::Size data_size_;

			Imp(std::uint16_t index = 0, std::uint8_t subindex = 0, aris::Size data_size= 0):data_size_(data_size), index_(index), subindex_(subindex){}
		};
		auto DO::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);

			std::stringstream s;
			s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
			xml_ele.SetAttribute("index", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(subindex());
			xml_ele.SetAttribute("subindex", s.str().c_str());

			xml_ele.SetAttribute("size", static_cast<std::int32_t>(size()));
		}
		auto DO::index()const->std::uint16_t { return imp_->index_; }
		auto DO::subindex()const->std::uint8_t { return imp_->subindex_; }
		auto DO::size()const->aris::Size { return imp_->data_size_; }
		DO::~DO() = default;
		DO::DO(const std::string &name, std::uint16_t index, std::uint8_t subindex, aris::Size data_size):Object(name), imp_(new Imp(index, subindex, data_size)){}
		DO::DO(Object &father, const aris::core::XmlElement &xml_ele) : Object(father, xml_ele)
		{
			imp_->index_ = attributeUint16(xml_ele, "index");
			imp_->subindex_ = attributeUint8(xml_ele, "subindex");
			imp_->data_size_ = attributeUint32(xml_ele, "size");
		}
		DO::DO(const DO &) = default;
		DO::DO(DO &&) = default;
		DO& DO::operator=(const DO &) = default;
		DO& DO::operator=(DO &&) = default;

		struct Sdo::Imp
		{
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

			Imp(unsigned opt = Sdo::READ | Sdo::WRITE | Sdo::CONFIG):option_(opt) {}
		};
		auto Sdo::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DO::saveXml(xml_ele);

			xml_ele.SetAttribute("read", option() & READ ? "true" : "false");
			xml_ele.SetAttribute("write", option() & WRITE ? "true" : "false");
		}
		auto Sdo::readable()const->bool { return (imp_->option_ & READ) != 0; }
		auto Sdo::writeable()const->bool { return (imp_->option_ & WRITE) != 0; }
		auto Sdo::configurable()const->bool { return (imp_->option_ & CONFIG) != 0;  }
		auto Sdo::option()const->unsigned { return imp_->option_; }
		auto Sdo::configBuffer()->char* { return imp_->config_value_; }
		Sdo::~Sdo() = default;
		Sdo::Sdo(const std::string &name, std::uint16_t index, std::uint8_t sub_index, aris::Size size, unsigned opt, std::int32_t config_value) :DO(name, index, sub_index, size), imp_(new Imp(opt)) 
		{
			if (opt & Sdo::CONFIG)
			{
				if(!(opt & Sdo::WRITE)) throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + name + "\" sdo");
				//
			}
		}
		Sdo::Sdo(Object &father, const aris::core::XmlElement &xml_ele) :DO(father, xml_ele)
		{
			if (attributeBool(xml_ele, "read", true))imp_->option_ |= READ; else imp_->option_ &= ~READ;
			if (attributeBool(xml_ele, "write", true))imp_->option_ |= WRITE; else imp_->option_ &= ~WRITE;
			if (xml_ele.Attribute("config"))
			{
				if (!writeable())throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + std::string(xml_ele.Name()) + "\" sdo");
				imp_->option_ |= CONFIG;
				imp_->config_value_int32_ = attributeInt64(xml_ele, "config");
			}
		}
		Sdo::Sdo(const Sdo &) = default;
		Sdo::Sdo(Sdo &&) = default;
		Sdo& Sdo::operator=(const Sdo &) = default;
		Sdo& Sdo::operator=(Sdo &&) = default;

		struct Pdo::Imp { aris::core::ImpPtr<Handle> ec_handle_; };
		auto Pdo::saveXml(aris::core::XmlElement &xml_ele) const->void{	DO::saveXml(xml_ele);}
		auto Pdo::ecHandle()->Handle* { return imp_->ec_handle_.get(); }
		auto Pdo::ecHandle()const->const Handle*{ return imp_->ec_handle_.get(); }
		Pdo::~Pdo() = default;
		Pdo::Pdo(const std::string &name, std::uint16_t index, std::uint8_t sub_index, aris::Size size):DO(name, index, sub_index, size){}
		Pdo::Pdo(Object &father, const aris::core::XmlElement &xml_ele) :DO(father, xml_ele) {}
		Pdo::Pdo(const Pdo &) = default;
		Pdo::Pdo(Pdo &&) = default;
		Pdo& Pdo::operator=(const Pdo &) = default;
		Pdo& Pdo::operator=(Pdo &&) = default;

		struct PdoGroup::Imp
		{
			aris::core::ImpPtr<Handle> handle_;
			bool is_tx_;
			std::uint16_t index_;

			Imp(std::uint16_t index = 0, bool is_tx = true):is_tx_(is_tx), index_(index) {}
		};
		auto PdoGroup::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);

			std::stringstream s;
			s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
			xml_ele.SetAttribute("index", s.str().c_str());

			xml_ele.SetAttribute("is_tx", tx());
		}
		auto PdoGroup::ecHandle()->Handle* { return imp_->handle_.get(); }
		auto PdoGroup::ecHandle()const->const Handle*{ return imp_->handle_.get(); }
		auto PdoGroup::tx()const->bool { return imp_->is_tx_; }
		auto PdoGroup::rx()const->bool { return !imp_->is_tx_; }
		auto PdoGroup::index()const->std::uint16_t { return imp_->index_; }
		PdoGroup::~PdoGroup() = default;
		PdoGroup::PdoGroup(const std::string &name, std::uint16_t index, bool is_tx):aris::core::ObjectPool<Pdo>(name), imp_(new Imp(index, is_tx)){}
		PdoGroup::PdoGroup(Object &father, const aris::core::XmlElement &xml_ele) :ObjectPool(father, xml_ele)
		{
			imp_->index_ = attributeUint16(xml_ele, "index");
			imp_->is_tx_ = attributeBool(xml_ele, "is_tx");
		}
		PdoGroup::PdoGroup(const PdoGroup &) = default;
		PdoGroup::PdoGroup(PdoGroup &&) = default;
		PdoGroup& PdoGroup::operator=(const PdoGroup &) = default;
		PdoGroup& PdoGroup::operator=(PdoGroup &&) = default;

		struct SlaveType::Imp
		{
			std::uint32_t product_code_, vender_id_;
			std::uint16_t alias_;
			std::uint32_t distributed_clock_;

			Imp(std::uint32_t product_code = 0, std::uint32_t vender_id = 0, std::uint16_t alias = 0, std::uint32_t distributed_clock = 0)
				:product_code_(product_code), vender_id_(vender_id), alias_(alias), distributed_clock_(distributed_clock){}
		};
		auto SlaveType::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);

			std::stringstream s;
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(productCode())) * 2) << std::hex << productCode();
			xml_ele.SetAttribute("product_code", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(venderID())) * 2) << std::hex << venderID();
			xml_ele.SetAttribute("vender_id", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(alias())) * 2) << std::hex << alias();
			xml_ele.SetAttribute("alias", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(distributedClock())) * 2) << std::hex << distributedClock();
			xml_ele.SetAttribute("distributed_clock", s.str().c_str());
		}
		auto SlaveType::productCode()const->std::uint32_t { return imp_->product_code_; }
		auto SlaveType::venderID()const->std::uint32_t { return imp_->vender_id_; }
		auto SlaveType::alias()const->std::uint16_t { return imp_->alias_; }
		auto SlaveType::distributedClock()const->std::uint32_t { return imp_->distributed_clock_; }
		SlaveType::~SlaveType() = default;
		SlaveType::SlaveType(const std::string &name, std::uint32_t product_code, std::uint32_t vender_id, std::uint16_t alias, std::uint32_t distributed_clock)
			:Object(name), imp_(new Imp(product_code, vender_id, alias, distributed_clock)){}
		SlaveType::SlaveType(Object &father, const aris::core::XmlElement &xml_ele) : Object(father, xml_ele)
		{
			imp_->product_code_ = attributeUint32(xml_ele, "product_code");
			imp_->vender_id_ = attributeUint32(xml_ele, "vender_id");
			imp_->alias_ = attributeUint16(xml_ele, "alias");
			imp_->distributed_clock_ = attributeUint32(xml_ele, "distributed_clock", 0);
		}
		SlaveType::SlaveType(const SlaveType &) = default;
		SlaveType::SlaveType(SlaveType &&) = default;
		SlaveType& SlaveType::operator=(const SlaveType &) = default;
		SlaveType& SlaveType::operator=(SlaveType &&) = default;
		
		struct EthercatSlave::Imp
		{
		public:
			Imp(EthercatSlave*slave, const SlaveType *st = nullptr) :slave_(slave), slave_type_(st) {}

			aris::core::ImpPtr<Handle> ec_handle_;

			const SlaveType *slave_type_;

			aris::core::ObjectPool<PdoGroup> *pdo_group_pool_;
			aris::core::ObjectPool<Sdo> *sdo_pool_;
			std::map<std::uint16_t, std::map<std::uint8_t, std::pair<int, int> > > pdo_map_;
			std::map<std::uint16_t, std::map<std::uint8_t, int>> sdo_map_;

			EthercatSlave *slave_;

			friend class EthercatSlave;
			friend class Master;
		};
		auto EthercatSlave::saveXml(aris::core::XmlElement &xml_ele) const->void { Slave::saveXml(xml_ele); }
		auto EthercatSlave::ecHandle()->Handle* { return imp_->ec_handle_.get(); }
		auto EthercatSlave::ecHandle()const->const Handle*{ return imp_->ec_handle_.get(); }
		auto EthercatSlave::productCode()const->std::uint32_t { return imp_->slave_type_->productCode(); }
		auto EthercatSlave::venderID()const->std::uint32_t { return imp_->slave_type_->venderID(); }
		auto EthercatSlave::alias()const->std::uint16_t { return imp_->slave_type_->alias(); }
		auto EthercatSlave::distributedClock()const->std::uint32_t { return imp_->slave_type_->distributedClock(); }
		auto EthercatSlave::pdoGroupPool()->aris::core::ObjectPool<PdoGroup>& { return *imp_->pdo_group_pool_; }
		auto EthercatSlave::pdoGroupPool()const->const aris::core::ObjectPool<PdoGroup>&{return *imp_->pdo_group_pool_; }
		auto EthercatSlave::sdoPool()->aris::core::ObjectPool<Sdo>& { return *imp_->sdo_pool_; }
		auto EthercatSlave::sdoPool()const->const aris::core::ObjectPool<Sdo>&{return *imp_->sdo_pool_; }
		auto EthercatSlave::readPdo(std::uint16_t index, std::uint8_t subindex, void *value, int byte_size)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			auto &pdo_group = pdoGroupPool().at(id_pair.first);
			auto &pdo = pdo_group.at(id_pair.second);
			if (!pdo_group.tx())throw std::runtime_error("failed to read pdo:\"" + pdo.name() + "\" because it is not tx");
			if (pdo.size() != byte_size)throw std::runtime_error("failed to read pdo:\"" + pdo.name() + "\" because byte size is not correct");
			aris_ecrt_pdo_read(ecHandle(), pdo.ecHandle(), value, byte_size);
		}
		auto EthercatSlave::writePdo(std::uint16_t index, std::uint8_t subindex, const void *value, int byte_size)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			auto &pdo_group = pdoGroupPool().at(id_pair.first);
			auto &pdo = pdo_group.at(id_pair.second);
			if (!pdo_group.rx())throw std::runtime_error("failed to write pdo:\"" + pdo.name() + "\" because it is not rx");
			if (pdo.size() != byte_size)throw std::runtime_error("failed to write pdo:\"" + pdo.name() + "\" because byte size is not correct");
			aris_ecrt_pdo_write(ecHandle(), pdoGroupPool().at(id_pair.first).at(id_pair.second).ecHandle(), value, byte_size);
		}
		auto EthercatSlave::readSdo(std::uint16_t index, std::uint8_t subindex, void *value, int byte_size)->void
		{
			std::size_t result_size;
			std::uint32_t abort_code;
			auto &sdo = sdoPool().at(imp_->sdo_map_.at(index).at(subindex));
			aris_ecrt_sdo_read(ecHandle(), position(), index, subindex, reinterpret_cast<std::uint8_t*>(value), byte_size, &result_size, &abort_code);
		}
		auto EthercatSlave::writeSdo(std::uint16_t index, std::uint8_t subindex, const void *value, int byte_size)->void
		{
			std::uint32_t abort_code;
			auto &sdo = sdoPool().at(imp_->sdo_map_.at(index).at(subindex));
			aris_ecrt_sdo_write(dynamic_cast<EthercatMaster&>(root()).ecHandle(), position(), index, subindex, const_cast<std::uint8_t*>(reinterpret_cast<const std::uint8_t*>(value)), byte_size, &abort_code);

		}
		auto EthercatSlave::configSdo(std::uint16_t index, std::uint8_t subindex, const void *value, int byte_size)->void
		{

		}
		EthercatSlave::~EthercatSlave() = default;
		EthercatSlave::EthercatSlave(const std::string &name, const SlaveType &slave_type) :Slave(name), imp_(new Imp(this, &slave_type))
		{
			imp_->pdo_group_pool_ = &add<aris::core::ObjectPool<PdoGroup> >("pdo_group_pool");
			imp_->sdo_pool_ = &add<aris::core::ObjectPool<Sdo> >("sdo_pool");
		}
		EthercatSlave::EthercatSlave(Object &father, const aris::core::XmlElement &xml_ele) : Slave(father, xml_ele), imp_(new Imp(this))
		{
			if (root().findByName("slave_type_pool") == root().children().end())throw std::runtime_error("you must insert \"slave_type_pool\" before insert \"slave_pool\" node");
			auto &slave_type_pool = static_cast<aris::core::ObjectPool<SlaveType> &>(*root().findByName("slave_type_pool"));

			if (slave_type_pool.findByName(attributeString(xml_ele, "slave_type")) == slave_type_pool.end())
			{
				throw std::runtime_error("can not find slave_type \"" + attributeString(xml_ele, "slave_type") + "\" in slave \"" + name() + "\"");
			}
			imp_->slave_type_ = &*slave_type_pool.findByName(attributeString(xml_ele, "slave_type"));
			imp_->pdo_group_pool_ = findOrInsert<aris::core::ObjectPool<PdoGroup> >("pdo_group_pool");
			imp_->sdo_pool_ = findOrInsert<aris::core::ObjectPool<Sdo> >("sdo_pool");
		}

		class EthercatMaster::Imp
		{
		public:
			aris::core::ImpPtr<Handle> ec_handle_;
			aris::core::ObjectPool<SlaveType> *slave_type_pool_;
			aris::core::RefPool<EthercatSlave> ec_slave_pool_;
		};
		auto EthercatMaster::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			auto root_xml_ele = xml_doc.RootElement()->FirstChildElement("controller");

			if (!root_xml_ele)throw std::runtime_error("can't find controller element in xml file");

			loadXml(*root_xml_ele);
		}
		auto EthercatMaster::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Master::loadXml(xml_ele);
		}
		auto EthercatMaster::init()->void
		{
			// make ec_slave_pool_ //
			imp_->ec_slave_pool_.clear();
			for (auto &sla : slavePool()) if (dynamic_cast<EthercatSlave*>(&sla)) imp_->ec_slave_pool_.push_back_ptr(dynamic_cast<EthercatSlave*>(&sla));
			
			// make pdo & sdo map for each slave //
			for (auto &sla : ecSlavePool()) 
			{
				// make PDO map and upd pdo's slave ptr //
				sla.imp_->pdo_map_.clear();
				for (int i = 0; i < static_cast<int>(sla.pdoGroupPool().size()); ++i)
				{
					auto &group = sla.pdoGroupPool().at(i);
					for (int j = 0; j < static_cast<int>(group.size()); ++j)
					{
						auto &pdo = group.at(j);
						if (sla.imp_->pdo_map_.find(pdo.index()) != sla.imp_->pdo_map_.end())
						{
							sla.imp_->pdo_map_.at(pdo.index()).insert(std::make_pair(pdo.subindex(), std::make_pair(i, j)));
						}
						else
						{
							std::map<std::uint8_t, std::pair<int, int> > subindex_map;
							subindex_map.insert(std::make_pair(pdo.subindex(), std::make_pair(i, j)));
							sla.imp_->pdo_map_.insert(std::make_pair(pdo.index(), subindex_map));
						}
					}
				}

				// make SDO map and upd pdo's slave ptr //
				sla.imp_->sdo_map_.clear();
				for (int i = 0; i < static_cast<int>(sla.sdoPool().size()); ++i)
				{
					auto &sdo = sla.sdoPool().at(i);
					if (sla.imp_->sdo_map_.find(sdo.index()) != sla.imp_->sdo_map_.end())
					{
						sla.imp_->sdo_map_.at(sdo.index()).insert(std::make_pair(sdo.subindex(), i));
					}
					else
					{
						std::map<std::uint8_t, int > subindex_map;
						subindex_map.insert(std::make_pair(sdo.subindex(), i));
						sla.imp_->sdo_map_.insert(std::make_pair(sdo.index(), subindex_map));
					}
				}
			}

			// init ethercat master, slave, pdo group, and pdo //
			imp_->ec_handle_.reset(aris_ecrt_master_init());
			for (auto &sla : ecSlavePool())
			{
				sla.imp_->ec_handle_.reset(aris_ecrt_slave_init());

				for (auto &pdo_group : sla.pdoGroupPool())
				{
					pdo_group.imp_->handle_.reset(aris_ecrt_pdo_group_init());
					for (auto &pdo : pdo_group)
					{
						pdo.imp_->ec_handle_.reset(aris_ecrt_pdo_init());
					}
				}
			}

			// config ethercat master, slave, pdo group, and pdo //
			for (auto &sla : ecSlavePool())
			{
				for (auto &pdo_group : sla.pdoGroupPool())
				{
					for (auto &pdo : pdo_group)
					{
						aris_ecrt_pdo_config(sla.ecHandle(), pdo_group.ecHandle(), pdo.ecHandle(), pdo.index(), pdo.subindex(), static_cast<std::uint8_t>(pdo.size() * 8));
					}
					aris_ecrt_pdo_group_config(sla.ecHandle(), pdo_group.ecHandle(), pdo_group.index(), pdo_group.tx());
				}
				aris_ecrt_slave_config(ecHandle(), sla.ecHandle(), sla.alias(), sla.position(), sla.venderID(), sla.productCode(), sla.distributedClock());
			}
			aris_ecrt_master_config(ecHandle());

			// config ethercat sdo //
			for (auto &sla : ecSlavePool())for (auto &sdo : sla.sdoPool())
				aris_ecrt_sdo_config(ecHandle(), sla.ecHandle(), sdo.index(), sdo.subindex(), reinterpret_cast<std::uint8_t*>(sdo.configBuffer()), sdo.size());

			// start ethercat master and slave //
			aris_ecrt_master_start(ecHandle());
			for (auto &sla : ecSlavePool())aris_ecrt_slave_start(sla.ecHandle());
		}
		auto EthercatMaster::release()->void { aris_ecrt_master_stop(ecHandle()); }
		auto EthercatMaster::send()->void 
		{
			for (auto &sla : ecSlavePool())aris_ecrt_slave_send(sla.ecHandle());
			aris_ecrt_master_send(ecHandle());
		}
		auto EthercatMaster::recv()->void
		{
			aris_ecrt_master_receive(ecHandle());
			for (auto &sla : ecSlavePool())aris_ecrt_slave_receive(sla.ecHandle());
		}
		auto EthercatMaster::sync()->void { aris_ecrt_master_sync(ecHandle(), aris_rt_timer_read()); }
		auto EthercatMaster::ecHandle()->Handle* { return imp_->ec_handle_.get(); }
		auto EthercatMaster::ecSlavePool()->aris::core::RefPool<EthercatSlave>& { return imp_->ec_slave_pool_; }
		auto EthercatMaster::slaveTypePool()->aris::core::ObjectPool<SlaveType>& { return *imp_->slave_type_pool_; }
		EthercatMaster::~EthercatMaster() = default;
		EthercatMaster::EthercatMaster() :imp_(new Imp)
		{
			registerChildType<Pdo>();
			registerChildType<Sdo>();
			registerChildType<PdoGroup>();
			registerChildType<aris::core::ObjectPool<Sdo> >();
			registerChildType<aris::core::ObjectPool<PdoGroup> >();
			registerChildType<aris::core::ObjectPool<SlaveType> >();

			registerChildType<SlaveType>();
			registerChildType<aris::core::ObjectPool<SlaveType> >();
			registerChildType<EthercatSlave>();
			registerChildType<aris::core::ObjectPool<EthercatSlave, aris::core::ObjectPool<Slave> > >();
			
			registerChildType<EthercatMotion>();
			imp_->slave_type_pool_ = &add<aris::core::ObjectPool<SlaveType> >("slave_type_pool");
		}
	}
}
