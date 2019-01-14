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

namespace aris::control
{
	struct Sdo::Imp
	{
		unsigned option_;
		std::uint16_t index_;
		std::uint8_t subindex_;
		std::uint8_t byte_size_;

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
	};
	auto Sdo::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
		xml_ele.SetAttribute("index", s.str().c_str());

		s = std::stringstream();
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(subindex());
		xml_ele.SetAttribute("subindex", s.str().c_str());

		xml_ele.SetAttribute("size", static_cast<std::int32_t>(byteSize()));

		xml_ele.SetAttribute("read", option() & READ ? "true" : "false");
		xml_ele.SetAttribute("write", option() & WRITE ? "true" : "false");
	}
	auto Sdo::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		if (attributeBool(xml_ele, "read", true))imp_->option_ |= READ; else imp_->option_ &= ~READ;
		if (attributeBool(xml_ele, "write", true))imp_->option_ |= WRITE; else imp_->option_ &= ~WRITE;
		if (xml_ele.Attribute("config"))
		{
			if (!writeable())throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + std::string(xml_ele.Name()) + "\" sdo");
			imp_->option_ |= CONFIG;
			imp_->config_value_int32_ = attributeInt32(xml_ele, "config");
		}

		imp_->index_ = attributeUint16(xml_ele, "index");
		imp_->subindex_ = attributeUint8(xml_ele, "subindex");
		imp_->byte_size_ = attributeUint8(xml_ele, "size");
	}
	auto Sdo::index()const->std::uint16_t { return imp_->index_; }
	auto Sdo::subindex()const->std::uint8_t { return imp_->subindex_; }
	auto Sdo::byteSize()const->std::uint8_t { return imp_->byte_size_; }
	auto Sdo::readable()const->bool { return (imp_->option_ & READ) != 0; }
	auto Sdo::writeable()const->bool { return (imp_->option_ & WRITE) != 0; }
	auto Sdo::configurable()const->bool { return (imp_->option_ & CONFIG) != 0; }
	auto Sdo::option()const->unsigned { return imp_->option_; }
	auto Sdo::configBuffer()->char* { return imp_->config_value_; }
	Sdo::~Sdo() = default;
	Sdo::Sdo(const std::string &name, std::uint16_t index, std::uint8_t sub_index, std::uint8_t byte_size, unsigned opt, std::int32_t config_value) :imp_(new Imp)
	{
		imp_->index_ = index;
		imp_->subindex_ = sub_index;
		imp_->byte_size_ = byte_size;
		imp_->option_ = opt;
		imp_->config_value_int32_ = config_value;
		
		if (opt & Sdo::CONFIG)
		{
			if (!(opt & Sdo::WRITE)) throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + name + "\" sdo");
		}
	}
	Sdo::Sdo(const Sdo &) = default;
	Sdo::Sdo(Sdo &&) = default;
	Sdo& Sdo::operator=(const Sdo &) = default;
	Sdo& Sdo::operator=(Sdo &&) = default;

	struct PdoEntry::Imp 
	{ 
		std::any ec_handle_; 
		std::uint16_t index_;
		std::uint8_t subindex_;
		aris::Size bit_size_;
	};
	auto PdoEntry::saveXml(aris::core::XmlElement &xml_ele) const->void 
	{ 
		Object::saveXml(xml_ele);

		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
		xml_ele.SetAttribute("index", s.str().c_str());

		s = std::stringstream();
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(subindex());
		xml_ele.SetAttribute("subindex", s.str().c_str());

		xml_ele.SetAttribute("size", static_cast<std::int32_t>(bitSize()));
	}
	auto PdoEntry::loadXml(const aris::core::XmlElement &xml_ele)->void 
	{ 
		imp_->index_ = attributeUint16(xml_ele, "index");
		imp_->subindex_ = attributeUint8(xml_ele, "subindex");
		imp_->bit_size_ = attributeUint32(xml_ele, "size");
		
		Object::loadXml(xml_ele);
	}
	auto PdoEntry::ecHandle()->std::any& { return imp_->ec_handle_; }
	auto PdoEntry::index()const->std::uint16_t { return imp_->index_; }
	auto PdoEntry::subindex()const->std::uint8_t { return imp_->subindex_; }
	auto PdoEntry::bitSize()const->aris::Size { return imp_->bit_size_; }
	PdoEntry::~PdoEntry() = default;
	PdoEntry::PdoEntry(const std::string &name, std::uint16_t index, std::uint8_t sub_index, aris::Size bit_size) :Object(name) 
	{
		imp_->index_ = index;
		imp_->subindex_ = sub_index;
		imp_->bit_size_ = bit_size;
	}
	PdoEntry::PdoEntry(const PdoEntry &) = default;
	PdoEntry::PdoEntry(PdoEntry &&) = default;
	PdoEntry& PdoEntry::operator=(const PdoEntry &) = default;
	PdoEntry& PdoEntry::operator=(PdoEntry &&) = default;

	struct Pdo::Imp
	{
		std::any handle_;
		std::uint16_t index_;
	};
	auto Pdo::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		aris::core::Object::saveXml(xml_ele);

		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
		xml_ele.SetAttribute("index", s.str().c_str());
	}
	auto Pdo::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		imp_->index_ = attributeUint16(xml_ele, "index");

		aris::core::Object::loadXml(xml_ele);
	}
	auto Pdo::ecHandle()->std::any& { return imp_->handle_; }
	auto Pdo::index()const->std::uint16_t { return imp_->index_; }
	Pdo::~Pdo() = default;
	Pdo::Pdo(const std::string &name, std::uint16_t index) :aris::core::ObjectPool<PdoEntry>(name), imp_(new Imp)
	{
		registerType<PdoEntry>();
		registerType<aris::core::ObjectPool<PdoEntry> >();

		imp_->index_ = index;
	}
	Pdo::Pdo(const Pdo &) = default;
	Pdo::Pdo(Pdo &&) = default;
	Pdo& Pdo::operator=(const Pdo &) = default;
	Pdo& Pdo::operator=(Pdo &&) = default;

	struct SyncManager::Imp { std::any handle_; bool is_tx_; };
	auto SyncManager::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		aris::core::ObjectPool<Pdo>::saveXml(xml_ele);
		xml_ele.SetAttribute("is_tx", tx());
	}
	auto SyncManager::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		imp_->is_tx_ = attributeBool(xml_ele, "is_tx");
		aris::core::ObjectPool<Pdo>::loadXml(xml_ele);
	}
	auto SyncManager::tx()const->bool { return imp_->is_tx_; }
	auto SyncManager::rx()const->bool { return !imp_->is_tx_; }
	SyncManager::~SyncManager() = default;
	SyncManager::SyncManager(const std::string &name, bool is_tx):ObjectPool(name), imp_(new Imp) {	imp_->is_tx_ = is_tx;}
	SyncManager::SyncManager(const SyncManager &) = default;
	SyncManager::SyncManager(SyncManager &&) = default;
	SyncManager& SyncManager::operator=(const SyncManager &) = default;
	SyncManager& SyncManager::operator=(SyncManager &&) = default;

	struct EthercatSlave::Imp
	{
	public:
		std::any ec_handle_;

		std::uint32_t vendor_id_, product_code_, revision_num_, dc_assign_activate_;
		aris::core::ObjectPool<SyncManager> *sm_pool_;
		aris::core::ObjectPool<Sdo> *sdo_pool_;
		std::map<std::uint16_t, std::map<std::uint8_t, PdoEntry* > > pdo_map_;
		std::map<std::uint16_t, std::map<std::uint8_t, int>> sdo_map_;
	};
	auto EthercatSlave::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Slave::saveXml(xml_ele);

		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(vendorID()) * 2) << std::hex << vendorID();
		xml_ele.SetAttribute("vendor_id", s.str().c_str());

		s = std::stringstream();
		s << "0x" << std::setfill('0') << std::setw(sizeof(productCode()) * 2) << std::hex << productCode();
		xml_ele.SetAttribute("product_code", s.str().c_str());

		s = std::stringstream();
		s << "0x" << std::setfill('0') << std::setw(sizeof(revisionNum()) * 2) << std::hex << revisionNum();
		xml_ele.SetAttribute("revision_num", s.str().c_str());

		s = std::stringstream();
		s << "0x" << std::setfill('0') << std::setw(sizeof(dcAssignActivate()) * 2) << std::hex << dcAssignActivate();
		xml_ele.SetAttribute("dc_assign_activate", s.str().c_str());
	}
	auto EthercatSlave::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		imp_->vendor_id_ = attributeUint32(xml_ele, "vendor_id");
		imp_->product_code_ = attributeUint32(xml_ele, "product_code");
		imp_->revision_num_ = attributeUint32(xml_ele, "revision_num");
		imp_->dc_assign_activate_ = attributeUint32(xml_ele, "dc_assign_activate");

		Slave::loadXml(xml_ele);
		imp_->sm_pool_ = findOrInsert<aris::core::ObjectPool<SyncManager> >("sm_pool");
		imp_->sdo_pool_ = findOrInsert<aris::core::ObjectPool<Sdo> >("sdo_pool");
	}
	auto EthercatSlave::ecHandle()->std::any& { return imp_->ec_handle_; }
	auto EthercatSlave::vendorID()const->std::uint32_t { return imp_->vendor_id_; }
	auto EthercatSlave::productCode()const->std::uint32_t { return imp_->product_code_; }
	auto EthercatSlave::revisionNum()const->std::uint32_t { return imp_->revision_num_; }
	auto EthercatSlave::dcAssignActivate()const->std::uint32_t { return imp_->dc_assign_activate_; }
	auto EthercatSlave::smPool()->aris::core::ObjectPool<SyncManager>& { return *imp_->sm_pool_; }
	auto EthercatSlave::sdoPool()->aris::core::ObjectPool<Sdo>& { return *imp_->sdo_pool_; }
	auto EthercatSlave::readPdo(std::uint16_t index, std::uint8_t subindex, void *value, aris::Size bit_size)->void
	{
		auto entry = imp_->pdo_map_.at(index).at(subindex);
		if (entry->bitSize() != bit_size)throw std::runtime_error("failed to read pdo entry:\"" + entry->name() + "\" because byte size is not correct");
		aris_ecrt_pdo_read(entry, value, static_cast<int>(bit_size));
	}
	auto EthercatSlave::writePdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size bit_size)->void
	{
		auto entry = imp_->pdo_map_.at(index).at(subindex);
		if (entry->bitSize() != bit_size)throw std::runtime_error("failed to write pdo_entry:\"" + entry->name() + "\" because byte size is not correct");
		aris_ecrt_pdo_write(entry, value, static_cast<int>(bit_size));
	}
	auto EthercatSlave::readSdo(std::uint16_t index, std::uint8_t subindex, void *value, aris::Size byte_size)->void
	{
		std::size_t result_size;
		std::uint32_t abort_code;
		aris_ecrt_sdo_read(dynamic_cast<EthercatMaster&>(root()).ecHandle(), phyId(), index, subindex, reinterpret_cast<std::uint8_t*>(value), byte_size, &result_size, &abort_code);
	}
	auto EthercatSlave::writeSdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size byte_size)->void
	{
		std::uint32_t abort_code;
		aris_ecrt_sdo_write(dynamic_cast<EthercatMaster&>(root()).ecHandle(), phyId(), index, subindex, const_cast<std::uint8_t*>(reinterpret_cast<const std::uint8_t*>(value)), byte_size, &abort_code);
	}
	auto EthercatSlave::configSdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size byte_size)->void{}
	EthercatSlave::~EthercatSlave() = default;
	EthercatSlave::EthercatSlave(const std::string &name, std::uint16_t phy_id, std::uint32_t vid, std::uint32_t p_code, std::uint32_t r_num, std::uint32_t dc) :Slave(name, phy_id), imp_(new Imp)
	{
		imp_->sm_pool_ = &add<aris::core::ObjectPool<SyncManager> >("sm_pool");
		imp_->sdo_pool_ = &add<aris::core::ObjectPool<Sdo> >("sdo_pool");
		imp_->vendor_id_ = vid;
		imp_->product_code_ = p_code;
		imp_->revision_num_ = r_num;
		imp_->dc_assign_activate_ = dc;

		registerType<Sdo>();
		registerType<Pdo>();
		registerType<aris::core::ObjectPool<Sdo> >();
		registerType<aris::core::ObjectPool<Pdo> >();

		registerType<SyncManager>();
		registerType<aris::core::ObjectPool<SyncManager> >();
	}

	class EthercatMaster::Imp
	{
	public:
		std::any ec_handle_;
		aris::core::RefPool<EthercatSlave> ec_slave_pool_;
	};
	auto EthercatMaster::scan()->void { aris_ecrt_scan(this); }
	auto EthercatMaster::init()->void
	{
		// make ec_slave_pool_ //
		imp_->ec_slave_pool_.clear();
		for (auto &sla : slavePool()) if (dynamic_cast<EthercatSlave*>(&sla)) imp_->ec_slave_pool_.push_back_ptr(dynamic_cast<EthercatSlave*>(&sla));

		// make pdo & sdo map for each slave //
		for (auto &sla : ecSlavePool())
		{
			// make PDO map //
			sla.imp_->pdo_map_.clear();
			for (auto &sm : sla.smPool())
			{
				for (auto &pdo : sm)
				{
					for (auto &entry : pdo)
					{
						if (sla.imp_->pdo_map_.find(entry.index()) != sla.imp_->pdo_map_.end())
						{
							sla.imp_->pdo_map_.at(entry.index()).insert(std::make_pair(entry.subindex(), &entry));
						}
						else
						{
							std::map<std::uint8_t, PdoEntry* > subindex_map;
							subindex_map.insert(std::make_pair(entry.subindex(), &entry));
							sla.imp_->pdo_map_.insert(std::make_pair(entry.index(), subindex_map));
						}
					}
				}
			}

			// make SDO map //
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

		aris_ecrt_master_request(this);
	}
	auto EthercatMaster::release()->void { aris_ecrt_master_stop(this); }
	auto EthercatMaster::send()->void { aris_ecrt_master_send(this); }
	auto EthercatMaster::recv()->void { aris_ecrt_master_recv(this); }
	auto EthercatMaster::sync()->void { aris_ecrt_master_sync(this, aris_rt_timer_read()); }
	auto EthercatMaster::ecHandle()->std::any& { return imp_->ec_handle_; }
	auto EthercatMaster::ecSlavePool()->aris::core::RefPool<EthercatSlave>& { return imp_->ec_slave_pool_; }
	EthercatMaster::~EthercatMaster() = default;
	EthercatMaster::EthercatMaster() :imp_(new Imp)
	{
		registerType<EthercatSlave>();
		registerType<EthercatMotion>();
	}

	class EthercatMotion::Imp
	{
	public:
		std::uint16_t control_word;
		std::uint8_t mode_of_operation;
		double target_pos_{ 0 }, target_vel_{ 0 }, target_cur_{ 0 }, offset_vel_{ 0 }, offset_cur_{ 0 };
		
		int waiting_count_left{ 0 }; // enable 在用
		int home_count{ 0 };
	};
	auto EthercatMotion::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		EthercatSlave::saveXml(xml_ele);
		Motion::saveXml(xml_ele);
	}
	auto EthercatMotion::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Motion::loadXml(xml_ele);
		EthercatSlave::loadXml(xml_ele);
	}
	auto EthercatMotion::controlWord()const->std::uint16_t { return imp_->control_word; }
	auto EthercatMotion::modeOfOperation()const->std::uint8_t { return imp_->mode_of_operation; }
	auto EthercatMotion::targetPos()const->double { return imp_->target_pos_; }
	auto EthercatMotion::targetVel()const->double { return imp_->target_vel_; }
	auto EthercatMotion::targetCur()const->double { return imp_->target_cur_; }
	auto EthercatMotion::offsetVel()const->double { return imp_->offset_vel_; }
	auto EthercatMotion::offsetCur()const->double { return imp_->offset_cur_; }
	auto EthercatMotion::setControlWord(std::uint16_t control_word)->void
	{
		imp_->control_word = control_word;
		writePdo(0x6040, 0x00, control_word);
	}
	auto EthercatMotion::setModeOfOperation(std::uint8_t mode)->void
	{
		imp_->mode_of_operation = mode;
		writePdo(0x6060, 0x00, mode);
	}
	auto EthercatMotion::setTargetPos(double pos)->void
	{
		imp_->target_pos_ = pos;
		writePdo(0x607A, 0x00, static_cast<std::int32_t>((pos + posOffset()) * posFactor()));
	}
	auto EthercatMotion::setTargetVel(double vel)->void
	{
		imp_->target_vel_ = vel;
		writePdo(0x60FF, 0x00, static_cast<std::int32_t>(vel * posFactor()));
	}
	auto EthercatMotion::setTargetCur(double cur)->void
	{
		imp_->target_cur_ = cur;
		writePdo(0x6071, 0x00, static_cast<std::int16_t>(cur));
	}
	auto EthercatMotion::setOffsetVel(double vel)->void
	{
		imp_->offset_vel_ = vel;
		writePdo(0x60B1, 0x00, static_cast<std::int32_t>(vel * posFactor()));
	}
	auto EthercatMotion::setOffsetCur(double cur)->void
	{
		imp_->offset_cur_ = cur;
		writePdo(0x60B2, 0x00, static_cast<std::int16_t>(cur));
	}
	auto EthercatMotion::statusWord()->std::uint16_t
	{
		std::uint16_t status_word;
		readPdo(0x6041, 0x00, status_word);
		return status_word;
	}
	auto EthercatMotion::modeOfDisplay()->std::uint8_t
	{
		std::uint8_t mode;
		readPdo(0x6061, 0x00, mode);
		return mode;
	}
	auto EthercatMotion::actualPos()->double
	{
		std::int32_t pos_count{ 0 };
		readPdo(0x6064, 0x00, pos_count);
		return static_cast<double>(pos_count) / posFactor() - posOffset();
	}
	auto EthercatMotion::actualVel()->double
	{
		std::int32_t vel_count{ 0 };
		readPdo(0x606C, 0x00, vel_count);
		return static_cast<double>(vel_count) / posFactor();
	}
	auto EthercatMotion::actualCur()->double
	{
		std::int16_t cur_count{ 0 };
		readPdo(0x6078, 0x00, cur_count);
		return static_cast<double>(cur_count);
	}
	auto EthercatMotion::disable()->int
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
		//
		// status word
		// 0x00    0b xxxx xxxx x0xx 0000    A: not ready to switch on     
		// 0x40    0b xxxx xxxx x1xx 0000    B: switch on disabled         
		// 0x21    0b xxxx xxxx x01x 0001    C: ready to switch on         
		// 0x23    0b xxxx xxxx x01x 0011    D: switch on                  
		// 0x27    0b xxxx xxxx x01x 0111    E: operation enabled          
		// 0x07    0b xxxx xxxx x00x 0111    F: quick stop active
		// 0x0F    0b xxxx xxxx x0xx 1111    G: fault reaction active
		// 0x08    0b xxxx xxxx x0xx 1000    H: fault
		//
		// 0x6F    0b 0000 0000 0110 1111
		// 0x4F    0b 0000 0000 0100 1111
		// disable change state to A/B/C/E to D

		std::uint16_t status_word;
		readPdo(0x6041, 0x00, status_word);

		// check status A
		if ((status_word & 0x4F) == 0x00)
		{
			return 1;
		}
		// check status B, now transition 2
		else if ((status_word & 0x4F) == 0x40)
		{
			// transition 2 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x06));
			return 1;
		}
		// check status C, now transition 3
		else if ((status_word & 0x6F) == 0x21)
		{
			// transition 3 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x07));
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
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x06));
			return 1;
		}
		// check status F, now transition 12
		else if ((status_word & 0x6F) == 0x07)
		{
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
			return 1;
		}
		// check status G, now transition 14
		else if ((status_word & 0x4F) == 0x0F)
		{
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
			return 1;
		}
		// check status H, now transition 13
		else if ((status_word & 0x4F) == 0x08)
		{
			// transition 4 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x80));
			return 1;
		}
		// unknown status
		else
		{
			return -1;
		}
	}
	auto EthercatMotion::enable()->int
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
		// 
		// status word
		// 0x00    0b xxxx xxxx x0xx 0000    A: not ready to switch on     
		// 0x40    0b xxxx xxxx x1xx 0000    B: switch on disabled         
		// 0x21    0b xxxx xxxx x01x 0001    C: ready to switch on         
		// 0x23    0b xxxx xxxx x01x 0011    D: switch on                  
		// 0x27    0b xxxx xxxx x01x 0111    E: operation enabled          
		// 0x07    0b xxxx xxxx x00x 0111    F: quick stop active
		// 0x0F    0b xxxx xxxx x0xx 1111    G: fault reaction active
		// 0x08    0b xxxx xxxx x0xx 1000    H: fault
		// 
		// 0x6F    0b 0000 0000 0110 1111
		// 0x4F    0b 0000 0000 0100 1111
		// enable change state to A/B/C/D/F/G/H to E
		std::uint16_t status_word;
		readPdo(0x6041, 0x00, status_word);

		// check status A
		if ((status_word & 0x4F) == 0x00)
		{
			return 1;
		}
		// check status B, now transition 2
		else if ((status_word & 0x4F) == 0x40)
		{
			// transition 2 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x06));
			return 2;
		}
		// check status C, now transition 3
		else if ((status_word & 0x6F) == 0x21)
		{
			// transition 3 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x07));
			return 3;
		}
		// check status D, now transition 4
		else if ((status_word & 0x6F) == 0x23)
		{
			// transition 4 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x0F));
			imp_->waiting_count_left = 20;

			// check mode to set correct pos, vel or cur //
			switch (modeOfDisplay())
			{
			case 0x08: setTargetPos(actualPos()); break;
			case 0x09: setTargetVel(0.0); break;
			case 0x10: setTargetCur(0.0); break;
			default: setTargetPos(actualPos()); setTargetVel(0.0); setTargetCur(0.0);
			}

			return 4;
		}
		// check status E, now keep status
		else if ((status_word & 0x6F) == 0x27)
		{
			// check if need wait //
			if (--imp_->waiting_count_left > 0) return 5;
			// now return normal
			else return 0;
		}
		// check status F, now transition 12
		else if ((status_word & 0x6F) == 0x07)
		{
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
			return 6;
		}
		// check status G, now transition 14
		else if ((status_word & 0x4F) == 0x0F)
		{
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x00));
			return 7;
		}
		// check status H, now transition 13
		else if ((status_word & 0x4F) == 0x08)
		{
			// transition 4 //
			writePdo(0x6040, 0x00, static_cast<std::uint16_t>(0x80));
			return 8;
		}
		// unknown status
		else
		{
			return -1;
		}
	}
	auto EthercatMotion::home()->int
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
		// 
		// status word
		// 0x00    0b xxxx xxxx x0xx 0000    A: not ready to switch on     
		// 0x40    0b xxxx xxxx x1xx 0000    B: switch on disabled         
		// 0x21    0b xxxx xxxx x01x 0001    C: ready to switch on         
		// 0x23    0b xxxx xxxx x01x 0011    D: switch on                  
		// 0x27    0b xxxx xxxx x01x 0111    E: operation enabled          
		// 0x07    0b xxxx xxxx x00x 0111    F: quick stop active
		// 0x0F    0b xxxx xxxx x0xx 1111    G: fault reaction active
		// 0x08    0b xxxx xxxx x0xx 1000    H: fault
		// 
		// 0x6F    0b 0000 0000 0110 1111
		// 0x4F    0b 0000 0000 0100 1111
		// enable change state to A/B/C/D to E

		std::uint16_t status_word;
		readPdo(0x6041, 0x00, status_word);

		// 确定是否已经使能（status ：operation enabled） //
		if ((status_word & 0x6F) != 0x27)
		{
			return 1;
		}
		// 更改mode //
		else if (modeOfDisplay() != 0x06)
		{
			setModeOfOperation(0x06);
			return 2;
		}
		// 将home已经到达的标志位去掉 //
		else if (!(controlWord() & 0x10) && (statusWord() & 0x0400))
		{
			setControlWord(0x0F);
			return 3;
		}
		// 开始执行home //
		else if (!(controlWord() & 0x10) && !(statusWord() & 0x0400))
		{
			setControlWord(0x1F);
			return 4;
		}
		// homing... //
		else if ((controlWord() & 0x10) && !(statusWord() & 0x0400))
		{
			return 5;
		}
		// home error //
		else if ((controlWord() & 0x10) && (statusWord() != 0x1400))
		{
			return -1;
		}
		// home successfull //
		else if ((controlWord() & 0x10) && (statusWord() != 0x2400))
		{
			return 0;
		}
		// unknown error //
		else
		{
			return -2;
		}
	}
	auto EthercatMotion::mode(std::uint8_t md)->int
	{
		setModeOfOperation(md);
		return md == modeOfDisplay() ? 0 : 1;
	}
	EthercatMotion::~EthercatMotion() = default;
	EthercatMotion::EthercatMotion(const std::string &name, std::uint16_t phy_id, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t revision_num, std::uint32_t dc_assign_activate
		, double max_pos, double min_pos, double max_vel, double min_vel, double max_acc, double min_acc, double max_pos_following_error, double max_vel_following_error, double pos_factor, double pos_offset, double home_pos)
		: EthercatSlave(name, phy_id, vendor_id, product_code, revision_num, dc_assign_activate)
		, Motion(name, phy_id, max_pos, min_pos, max_vel, min_vel, max_acc, min_acc, max_pos_following_error, max_vel_following_error, pos_factor, pos_offset, home_pos)
		, Slave(name, phy_id), imp_(new Imp)
	{
	}
}
