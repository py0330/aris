#ifndef ARIS_CONTROL_ETHERCAT_H
#define ARIS_CONTROL_ETHERCAT_H

#include <vector>
#include <memory>
#include <thread>
#include <fstream>
#include <cstdint>

#include <aris_core.h>
#include <aris_control_master_slave.h>
#include <aris_control_controller_motion.h>
#include <aris_control_ethercat_kernel.h>


namespace aris::control
{
	class Sdo :public aris::core::Object
	{
	public:
		enum Option
		{
			READ = 0x01,
			WRITE = 0x02,
			CONFIG = 0x04
		};
		static auto Type()->const std::string & { static const std::string type("Sdo"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto index()const->std::uint16_t;
		auto subindex()const->std::uint8_t;
		auto byteSize()const->std::uint8_t;
		auto readable()const->bool;
		auto writeable()const->bool;
		auto configurable()const->bool;
		auto option()const->unsigned;
		auto configBuffer()->char*;
		virtual ~Sdo();
		explicit Sdo(const std::string &name = "sdo", std::uint16_t index = 0x0000, std::uint8_t subindex = 0x00, std::uint8_t byte_size = 0, unsigned opt = 0, std::int32_t config_value = 0);
		Sdo(const Sdo &);
		Sdo(Sdo &&);
		Sdo& operator=(const Sdo &);
		Sdo& operator=(Sdo &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class PdoEntry :public aris::core::Object
	{
	public:
		static auto Type()->const std::string & { static const std::string type("PdoEntry"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto index()const->std::uint16_t;
		auto subindex()const->std::uint8_t;
		auto bitSize()const->aris::Size;

		virtual ~PdoEntry();
		explicit PdoEntry(const std::string &name = "entry", std::uint16_t index = 0x0000, std::uint8_t subindex = 0x00, aris::Size bit_size = 8);
		PdoEntry(const PdoEntry &);
		PdoEntry(PdoEntry &&);
		PdoEntry& operator=(const PdoEntry &);
		PdoEntry& operator=(PdoEntry &&);

	public:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class Pdo :public aris::core::ObjectPool<PdoEntry>
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Pdo"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto index()const->std::uint16_t;

		virtual ~Pdo();
		explicit Pdo(const std::string &name = "pdo", std::uint16_t index = 0x0000);
		Pdo(const Pdo &);
		Pdo(Pdo &&);
		Pdo& operator=(const Pdo &);
		Pdo& operator=(Pdo &&);

	public:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class SyncManager :public aris::core::ObjectPool<Pdo>
	{
	public:
		static auto Type()->const std::string & { static const std::string type("SyncManager"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto tx()const->bool;
		auto rx()const->bool;

		virtual ~SyncManager();
		explicit SyncManager(const std::string &name = "sm", bool is_tx = true);
		SyncManager(const SyncManager &);
		SyncManager(SyncManager &&);
		SyncManager& operator=(const SyncManager &);
		SyncManager& operator=(SyncManager &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class EthercatSlave : virtual public Slave
	{
	public:
		static auto Type()->const std::string & { static const std::string type("EthercatSlave"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto vendorID()const->std::uint32_t;
		auto productCode()const->std::uint32_t;
		auto revisionNum()const->std::uint32_t;
		auto dcAssignActivate()const->std::uint32_t;
		auto smPool()->aris::core::ObjectPool<SyncManager>&;
		auto smPool()const->const aris::core::ObjectPool<SyncManager>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->smPool(); }
		auto sdoPool()->aris::core::ObjectPool<Sdo>&;
		auto sdoPool()const->const aris::core::ObjectPool<Sdo>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->sdoPool(); }
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }

		template<typename ValueType>
		auto readPdo(std::uint16_t index, std::uint8_t subindex, ValueType &value)->void { readPdo(index, subindex, &value, sizeof(ValueType) * 8); }
		auto readPdo(std::uint16_t index, std::uint8_t subindex, void *value, aris::Size bit_size)->void;
		template<typename ValueType>
		auto writePdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { writePdo(index, subindex, &value, sizeof(ValueType) * 8); }
		auto writePdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size bit_size)->void;
		template<typename ValueType>
		auto readSdo(std::uint16_t index, std::uint8_t subindex, ValueType &value)->void { readSdo(index, subindex, &value, sizeof(ValueType)); }
		auto readSdo(std::uint16_t index, std::uint8_t subindex, void *value, aris::Size byte_size)->void;
		template<typename ValueType>
		auto writeSdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { writeSdo(index, subindex, &value, sizeof(ValueType)); }
		auto writeSdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size byte_size)->void;
		template<typename ValueType>
		auto configSdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { configSdo(index, subindex, &value, sizeof(ValueType)); }
		auto configSdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size byte_size)->void;

		virtual ~EthercatSlave();
		explicit EthercatSlave(const std::string &name = "ethercat_slave", std::uint16_t phy_id = 0, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000);
		EthercatSlave(const EthercatSlave &other) = delete;
		EthercatSlave(EthercatSlave &&other) = delete;
		EthercatSlave& operator=(const EthercatSlave &other) = delete;
		EthercatSlave& operator=(EthercatSlave &&other) = delete;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class EthercatMaster;
	};
	class EthercatMaster : virtual public Master
	{
	public:
		static auto Type()->const std::string & { static const std::string type("EthercatMaster"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)> *>(this)->ecHandle(); }
		auto ecSlavePool()->aris::core::RefPool<EthercatSlave>&;
		auto ecSlavePool()const->const aris::core::RefPool<EthercatSlave>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->ecSlavePool(); }
		auto scan()->void;

		virtual ~EthercatMaster();
		EthercatMaster(const std::string &name = "ethercat_master");
		EthercatMaster(const EthercatMaster &other) = delete;
		EthercatMaster(EthercatMaster &&other) = delete;
		EthercatMaster& operator=(const EthercatMaster &other) = delete;
		EthercatMaster& operator=(EthercatMaster &&other) = delete;

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual sync()->void override;
		auto virtual release()->void override;

	private:
		class Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class Sdo;
		friend class PdoEntry;
	};

	class EthercatMotion :public EthercatSlave, public Motion
	{
	public:
		static auto Type()->const std::string & { static const std::string type("EthercatMotion"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;

		auto virtual controlWord()const->std::uint16_t override;
		auto virtual modeOfOperation()const->std::uint8_t override;
		auto virtual targetPos()const->double override;
		auto virtual targetVel()const->double override;
		auto virtual targetCur()const->double override;
		auto virtual offsetVel()const->double override;
		auto virtual offsetCur()const->double override;
		// require pdo 0x6040 //
		auto virtual setControlWord(std::uint16_t control_word)->void override;
		// require pdo 0x6060 //
		auto virtual setModeOfOperation(std::uint8_t mode)->void override;
		// require pdo 0x607A //
		auto virtual setTargetPos(double pos)->void override;
		// require pdo 0x60FF //
		auto virtual setTargetVel(double vel)->void override;
		// require pdo 0x6071 //
		auto virtual setTargetCur(double cur)->void override;
		// require pdo 0x6071 //
		auto virtual setOffsetVel(double vel)->void override;
		// require pdo 0x6071 //
		auto virtual setOffsetCur(double cur)->void override;
		// require pdo 0x6041 //
		auto virtual statusWord()->std::uint16_t override;
		// require pdo 0x6061 //
		auto virtual modeOfDisplay()->std::uint8_t override;
		// require pdo 0x6064 //
		auto virtual actualPos()->double override;
		// require pdo 0x606C //
		auto virtual actualVel()->double override;
		// require pdo 0x6078 //
		auto virtual actualCur()->double override;


		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		virtual ~EthercatMotion();
		EthercatMotion(const std::string &name = "ethercat_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0);

	private:
		class Imp;
		std::unique_ptr<Imp> imp_;
	};
	class EthercatController :public EthercatMaster, public Controller
	{
	public:
		static auto Type()->const std::string & { static const std::string type("EthercatController"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }

		virtual ~EthercatController() = default;
		EthercatController(const std::string &name = "ethercat_controller");
		EthercatController(const EthercatController &other) = delete;
		EthercatController(EthercatController &&other) = delete;
		EthercatController& operator=(const EthercatController &other) = delete;
		EthercatController& operator=(EthercatController &&other) = delete;

	protected:
		auto virtual init()->void override { EthercatMaster::init(); Controller::init(); };
		auto virtual send()->void override { EthercatMaster::send(); };
		auto virtual recv()->void override { EthercatMaster::recv(); };
		auto virtual sync()->void override { EthercatMaster::sync(); };
		auto virtual release()->void override { EthercatMaster::release(); };
	};
}

#endif