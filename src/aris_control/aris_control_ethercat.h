#ifndef ARIS_CONTROL_ETHERCAT_H
#define ARIS_CONTROL_ETHERCAT_H

#include <vector>
#include <memory>
#include <thread>
#include <fstream>
#include <cstdint>

#include <aris_core.h>
#include <aris_control_master_slave.h>
#include <aris_control_ethercat_kernel.h>


namespace aris
{
	namespace control
	{
		class DO :public aris::core::Object
		{
		public:
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto index()const->std::uint16_t;
			auto subindex()const->std::uint8_t;
			auto size()const->aris::Size;
			virtual ~DO();
			explicit DO(const std::string &name, std::uint16_t index, std::uint8_t subindex, aris::Size data_size);
			explicit DO(Object &father, const aris::core::XmlElement &xml_ele);
			DO(const DO &);
			DO(DO &&);
			DO& operator=(const DO &);
			DO& operator=(DO &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
			friend class EthercatMaster;
		};
		class Pdo :public DO
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Pdo"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*;

			virtual ~Pdo();
			explicit Pdo(const std::string &name, std::uint16_t index, std::uint8_t subindex, aris::Size data_size);
			explicit Pdo(Object &father, const aris::core::XmlElement &xml_ele);
			Pdo(const Pdo &);
			Pdo(Pdo &&);
			Pdo& operator=(const Pdo &);
			Pdo& operator=(Pdo &&);

		public:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Sdo :public DO
		{
		public:
			enum Option
			{
				READ = 0x01,
				WRITE = 0x02,
				CONFIG = 0x04
			};
			static auto Type()->const std::string &{ static const std::string type("Sdo"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto readable()const->bool;
			auto writeable()const->bool;
			auto configurable()const->bool;
			auto option()const->unsigned;
			auto configBuffer()->char*;
			virtual ~Sdo();
			explicit Sdo(const std::string &name, std::uint16_t index, std::uint8_t subindex, aris::Size data_size, unsigned opt, std::int32_t config_value = 0);
			explicit Sdo(Object &father, const aris::core::XmlElement &xml_ele);
			Sdo(const Sdo &);
			Sdo(Sdo &&);
			Sdo& operator=(const Sdo &);
			Sdo& operator=(Sdo &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class PdoGroup :public aris::core::ObjectPool<Pdo>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("PdoGroup"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*;
			auto tx()const->bool;
			auto rx()const->bool;
			auto index()const->std::uint16_t;

			virtual ~PdoGroup();
			explicit PdoGroup(const std::string &name, std::uint16_t index, bool is_tx);
			explicit PdoGroup(Object &father, const aris::core::XmlElement &xml_ele);
			PdoGroup(const PdoGroup &);
			PdoGroup(PdoGroup &&);
			PdoGroup& operator=(const PdoGroup &);
			PdoGroup& operator=(PdoGroup &&);

		public:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class SlaveType :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("SlaveType"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto productCode()const->std::uint32_t;
			auto venderID()const->std::uint32_t;
			auto alias()const->std::uint16_t;
			auto distributedClock()const->std::uint32_t;

			virtual ~SlaveType();
			explicit SlaveType(const std::string &name, std::uint32_t product_code, std::uint32_t vender_id, std::uint16_t alias, std::uint32_t distributed_clock);
			explicit SlaveType(Object &father, const aris::core::XmlElement &xml_ele);
			SlaveType(const SlaveType &);
			SlaveType(SlaveType &&);
			SlaveType& operator=(const SlaveType &);
			SlaveType& operator=(SlaveType &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};

		class EthercatSlave : public Slave
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("EthercatSlave"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*;
			auto position()const ->std::uint16_t { return static_cast<std::uint16_t>(id()); }
			auto productCode()const->std::uint32_t;
			auto venderID()const->std::uint32_t;
			auto alias()const->std::uint16_t;
			auto distributedClock()const->std::uint32_t;
			auto pdoGroupPool()->aris::core::ObjectPool<PdoGroup>&;
			auto pdoGroupPool()const->const aris::core::ObjectPool<PdoGroup>&;
			auto sdoPool()->aris::core::ObjectPool<Sdo>&;
			auto sdoPool()const->const aris::core::ObjectPool<Sdo>&;

			template<typename ValueType>
			auto readPdo(std::uint16_t index, std::uint8_t subindex, ValueType &value)->void { readPdo(index, subindex, &value, sizeof(ValueType)); }
			auto readPdo(std::uint16_t index, std::uint8_t subindex, void *value, int byte_size)->void;
			template<typename ValueType>
			auto writePdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { writePdo(index, subindex, &value, sizeof(ValueType)); }
			auto writePdo(std::uint16_t index, std::uint8_t subindex, const void *value, int byte_size)->void;
			template<typename ValueType>
			auto readSdo(std::uint16_t index, std::uint8_t subindex, ValueType &value)->void { readSdo(index, subindex, &value, sizeof(ValueType)); }
			auto readSdo(std::uint16_t index, std::uint8_t subindex, void *value, int byte_size)->void;
			template<typename ValueType>
			auto writeSdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { writeSdo(index, subindex, &value, sizeof(ValueType)); }
			auto writeSdo(std::uint16_t index, std::uint8_t subindex, const void *value, int byte_size)->void;
			template<typename ValueType>
			auto configSdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { configSdo(index, subindex, &value, sizeof(ValueType)); }
			auto configSdo(std::uint16_t index, std::uint8_t subindex, const void *value, int byte_size)->void;

			virtual ~EthercatSlave();
			explicit EthercatSlave(const std::string &name, const SlaveType &slave_type);
			explicit EthercatSlave(Object &father, const aris::core::XmlElement &xml_ele);
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
			enum { MAX_MSG_SIZE = 8192 };

			using Root::loadXml;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*{ return const_cast<std::decay_t<decltype(*this)> *>(this)->ecHandle(); }
			auto ecSlavePool()->aris::core::RefPool<EthercatSlave>&;
			auto ecSlavePool()const->const aris::core::RefPool<EthercatSlave>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->ecSlavePool(); }
			auto slaveTypePool()->aris::core::ObjectPool<SlaveType>&;
			auto slaveTypePool()const->const aris::core::ObjectPool<SlaveType>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveTypePool(); }

			virtual ~EthercatMaster();
			EthercatMaster();
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
			friend class Pdo;
		};
	}
}

#endif
