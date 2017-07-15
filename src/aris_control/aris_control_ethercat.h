#ifndef ARIS_CONTROL_ETHERCAT_H
#define ARIS_CONTROL_ETHERCAT_H

#include <vector>
#include <memory>
#include <thread>
#include <fstream>
#include <cstdint>

#include <aris_core.h>
#include <aris_control_kernel.h>

namespace aris
{
	namespace control
	{
		class Master;
		class Slave;

		class Element :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Element"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto master()->Master &;
			auto master()const->const Master &;
			Element(const std::string &name) :Object(name) {}
			Element(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele) {}

		private:
			friend class Master;
		};
		class DataLogger :public Element
		{
		public:
			enum { MAX_LOG_DATA_SIZE = 8192 };
			static auto Type()->const std::string &{ static const std::string type("DataLogger"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto start(const std::string &log_file_name = std::string())->void;
			auto stop()->void;
			auto lout()->aris::core::MsgStream &;
			auto lout()const->const aris::core::MsgStream &{ return const_cast<DataLogger*>(this)->lout(); };
			auto send()->void;

			virtual ~DataLogger();
			DataLogger(const std::string &name);
			DataLogger(Object &father, const aris::core::XmlElement &xml_ele);
			DataLogger(const DataLogger &) = delete;
			DataLogger(DataLogger &&) = delete;
			DataLogger& operator=(const DataLogger &) = delete;
			DataLogger& operator=(DataLogger &&) = delete;

		private:
			struct Imp;
			std::unique_ptr<Imp> imp_;
		};
		class DO :public Element
		{
		public:
			enum DataType
			{
				INT32,
				INT16,
				INT8,
				UINT32,
				UINT16,
				UINT8
			};
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto slave()->Slave&;
			auto slave()const->const Slave&;
			auto index()const->std::uint16_t;
			auto subindex()const->std::uint8_t;
			auto dataBitSize()const->std::uint8_t;
			auto dataType()const->DataType;
			virtual ~DO();
			explicit DO(const std::string &name, DO::DataType data_type, std::uint16_t index, std::uint8_t sub_index);
			explicit DO(Object &father, const aris::core::XmlElement &xml_ele);
			DO(const DO &);
			DO(DO &&);
			DO& operator=(const DO &);
			DO& operator=(DO &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
			friend class Slave;
		};
		class Pdo :public DO
		{
		public:
			enum TransmitType {};
			static auto Type()->const std::string &{ static const std::string type("Pdo"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*;
			auto read(std::int32_t &value)->void;
			auto read(std::int16_t &value)->void;
			auto read(std::int8_t &value)->void;
			auto read(std::uint32_t &value)->void;
			auto read(std::uint16_t &value)->void;
			auto read(std::uint8_t &value)->void;
			auto write(std::int32_t value)->void;
			auto write(std::int16_t value)->void;
			auto write(std::int8_t value)->void;
			auto write(std::uint32_t value)->void;
			auto write(std::uint16_t value)->void;
			auto write(std::uint8_t value)->void;
			virtual ~Pdo();
			explicit Pdo(const std::string &name, DO::DataType data_type, std::uint16_t index, std::uint8_t sub_index);
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
			auto configValueInt32()const->std::int32_t;
			auto configValueInt16()const->std::int16_t;
			auto configValueInt8()const->std::int8_t;
			auto configValueUint32()const->std::uint32_t;
			auto configValueUint16()const->std::uint16_t;
			auto configValueUint8()const->std::uint8_t;
			auto getConfigValue(std::int32_t &value)const->void;
			auto getConfigValue(std::int16_t &value)const->void;
			auto getConfigValue(std::int8_t &value)const->void;
			auto getConfigValue(std::uint32_t &value)const->void;
			auto getConfigValue(std::uint16_t &value)const->void;
			auto getConfigValue(std::uint8_t &value)const->void;
			auto setConfigValue(std::int32_t value)->void;
			auto setConfigValue(std::int16_t value)->void;
			auto setConfigValue(std::int8_t value)->void;
			auto setConfigValue(std::uint32_t value)->void;
			auto setConfigValue(std::uint16_t value)->void;
			auto setConfigValue(std::uint8_t value)->void;
			auto read(std::int32_t &value)->void;
			auto read(std::int16_t &value)->void;
			auto read(std::int8_t &value)->void;
			auto read(std::uint32_t &value)->void;
			auto read(std::uint16_t &value)->void;
			auto read(std::uint8_t &value)->void;
			auto write(std::int32_t value)->void;
			auto write(std::int16_t value)->void;
			auto write(std::int8_t value)->void;
			auto write(std::uint32_t value)->void;
			auto write(std::uint16_t value)->void;
			auto write(std::uint8_t value)->void;
			virtual ~Sdo();
			explicit Sdo(const std::string &name, DO::DataType data_type, std::uint16_t index, std::uint8_t sub_index, unsigned opt, std::int32_t config_value = 0);
			explicit Sdo(Object &father, const aris::core::XmlElement &xml_ele);
			Sdo(const Sdo &);
			Sdo(Sdo &&);
			Sdo& operator=(const Sdo &);
			Sdo& operator=(Sdo &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Slave;
		};
		class PdoGroup :public aris::core::ObjectPool<Pdo, Element>
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

			friend class Slave;
		};
		class SlaveType :public Element
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
		class Slave : public Element
		{
		public:
			struct TxType {};
			struct RxType { std::int32_t ret{ 0 }; };
			static auto Type()->const std::string &{ static const std::string type("Slave"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual txData()->TxType&;
			auto virtual txData()const->const TxType&;
			auto virtual rxData()->RxType&;
			auto virtual rxData()const->const RxType&;
			auto virtual getTxData(TxType& tx_data)const->void { tx_data = txData(); }
			auto virtual setTxData(const TxType& tx_data)->void { txData() = tx_data; }
			auto virtual getRxData(RxType& rx_data)const->void { rx_data = rxData(); }
			auto virtual setTxData(const RxType& rx_data)->void { rxData() = rx_data; }
			auto virtual txTypeSize()const->std::size_t { return sizeof(TxType); }
			auto virtual rxTypeSize()const->std::size_t { return sizeof(RxType); }
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*;
			auto position()const ->std::uint16_t { return static_cast<std::uint16_t>(id()); }
			auto productCode()const->std::uint32_t;
			auto venderID()const->std::uint32_t;
			auto alias()const->std::uint16_t;
			auto distributedClock()const->std::uint32_t;
			auto pdoGroupPool()->aris::core::ObjectPool<PdoGroup, Element>&;
			auto pdoGroupPool()const->const aris::core::ObjectPool<PdoGroup, Element>&;
			auto sdoPool()->aris::core::ObjectPool<Sdo, Element>&;
			auto sdoPool()const->const aris::core::ObjectPool<Sdo, Element>&;

			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void;

			auto readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)->void;
			auto readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)->void;
			auto readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)->void;
			auto readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)->void;
			auto readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)->void;
			auto readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)->void;
			auto writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void;
			auto writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void;
			auto writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void;
			auto writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void;
			auto writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void;
			auto writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void;

			auto readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)const->void;
			auto readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)const->void;
			auto readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)const->void;
			auto readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)const->void;
			auto readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)const->void;
			auto readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)const->void;
			auto configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void;
			auto configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void;
			auto configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void;
			auto configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void;
			auto configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void;
			auto configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void;
			virtual ~Slave();
			explicit Slave(const std::string &name, const SlaveType &slave_type);
			explicit Slave(Object &father, const aris::core::XmlElement &xml_ele);
			Slave(const Slave &other) = delete;
			Slave(Slave &&other) = delete;
			Slave& operator=(const Slave &other) = delete;
			Slave& operator=(Slave &&other) = delete;

		protected:
			auto virtual init()->void;
			auto virtual readUpdate()->void {}
			auto virtual writeUpdate()->void {}

		private:
			struct Imp;
			std::unique_ptr<Imp> imp_;

			friend class Master;
			friend class Pdo;
		};
		class Master : public aris::core::Root
		{
		public:
			using Root::loadXml;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual start()->void;
			auto virtual stop()->void;
			auto setControlStrategy(std::function<void()> strategy)->void;
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*{ return const_cast<std::decay_t<decltype(*this)> *>(this)->ecHandle(); }
			auto rtHandle()->Handle*;
			auto rtHandle()const->const Handle*{return const_cast<std::decay_t<decltype(*this)> *>(this)->rtHandle(); }
			auto pipeIn()->aris::core::Pipe&;
			auto pipeIn()const->const aris::core::Pipe&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->pipeIn(); }
			auto pipeOut()->aris::core::Pipe&;
			auto pipeOut()const->const aris::core::Pipe&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->pipeIn(); }
			auto slaveTypePool()->aris::core::ObjectPool<SlaveType, Element>&;
			auto slaveTypePool()const->const aris::core::ObjectPool<SlaveType, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveTypePool(); }
			auto slavePool()->aris::core::ObjectPool<Slave, Element>&;
			auto slavePool()const->const aris::core::ObjectPool<Slave, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slavePool(); }
			auto dataLogger()->DataLogger&;
			auto dataLogger()const->const DataLogger&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->dataLogger(); }
			virtual ~Master();
			Master();
			Master(const Master &other) = delete;
			Master(Master &&other) = delete;
			Master& operator=(const Master &other) = delete;
			Master& operator=(Master &&other) = delete;

		protected:
			auto virtual controlStrategy()->void {}


		private:
			class Imp;
			std::unique_ptr<Imp> imp_;

			friend class Slave;
			friend class Sdo;
			friend class Pdo;
		};

		template<typename Tx, typename Rx>
		class SlaveTemplate :public Slave
		{
		public:
			using TxType = Tx;
			using RxType = Rx;
			auto virtual txData()->TxType & override { return tx_data_; }
			auto virtual txData()const->const TxType & override{ return tx_data_; }
			auto virtual rxData()->RxType & override { return rx_data_; }
			auto virtual rxData()const->const RxType & override{ return rx_data_; }
			auto virtual getTxData(Slave::TxType& tx_data)const->void override { static_cast<TxType &>(tx_data) = txData(); }
			auto virtual setTxData(const Slave::TxType& tx_data)->void override { txData() = static_cast<const TxType &>(tx_data); }
			auto virtual getRxData(Slave::RxType& rx_data)const->void override { static_cast<RxType &>(rx_data) = rxData(); }
			auto virtual setTxData(const Slave::RxType& rx_data)->void override { rxData() = static_cast<const RxType &>(rx_data); }
			auto virtual txTypeSize()const->std::size_t override { return sizeof(TxType); }
			auto virtual rxTypeSize()const->std::size_t override { return sizeof(RxType); }

			SlaveTemplate(Object &father, const aris::core::XmlElement &xml_ele) :Slave(father, xml_ele)
			{
				static_assert(std::is_base_of<Slave::TxType, TxType>(), "\"TxDataType\" must be derived from \"TxData\"");
				static_assert(std::is_base_of<Slave::RxType, RxType>(), "\"RxDataType\" must be derived from \"RxData\"");
			}
		private:
			TxType tx_data_;
			RxType rx_data_;
		};
	}
}

#endif
