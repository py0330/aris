#ifndef ARIS_CONTROL_ETHERCAT_H
#define ARIS_CONTROL_ETHERCAT_H

#include <vector>
#include <memory>
#include <thread>
#include <fstream>
#include <cstdint>

#include <aris_core.h>

namespace aris
{
	/// \brief 控制命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace control
	{	
		struct Handle { virtual ~Handle() = default; };

		class Master;
		class Slave;

		class Element :public aris::core::Object
		{
		public:
			auto master()->Master &;
			auto master()const->const Master &;
			Element(const std::string &name) :Object(name) {}
			Element(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele){}

		private:
			friend class Master;
		};
		class DataLogger :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("DataLogger"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto prepair(const std::string &log_file_name = std::string())->void;
			auto start()->void;
			auto stop()->void;
			auto logDataRT()->void;

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
		class DO:public Element
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

			auto slave()->Slave&;
			auto slave()const->const Slave&;
			auto index()const->std::uint16_t;
			auto subindex()const->std::uint8_t;
			auto dataBit()const->std::uint8_t;
			auto dataType()const->DataType;
			virtual ~DO();
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
			static auto Type()->const std::string &{ static const std::string type("Pdo"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
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
			virtual auto type() const->const std::string&{ return Type(); }
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
		class PdoGroup:public aris::core::ObjectPool<Pdo, Element>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("PdoGroup"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto ecHandle()->Handle*;
			auto ecHandle()const->const Handle*;
			auto tx()const->bool;
			auto rx()const->bool;
			auto index()const->std::uint16_t;

			virtual ~PdoGroup();
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
		class SlaveType:public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("SlaveType"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto productCode()const->std::uint32_t;
			auto venderID()const->std::uint32_t;
			auto alias()const->std::uint16_t;
			auto distributedClock()const->std::uint32_t;

			virtual ~SlaveType();
			explicit SlaveType(Object &father, const aris::core::XmlElement &xml_ele);
			SlaveType(const SlaveType &);
			SlaveType(SlaveType &&);
			SlaveType& operator=(const SlaveType &);
			SlaveType& operator=(SlaveType &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Slave: public Element
		{
		public:
			struct TxType {};
			struct RxType { std::int32_t ret{ 0 }; };
			static auto Type()->const std::string &{ static const std::string type("Slave"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto txData()->TxType&;
			virtual auto txData()const->const TxType&;
			virtual auto rxData()->RxType&;
			virtual auto rxData()const->const RxType&;
			virtual auto getTxData(TxType& tx_data)const->void { tx_data = txData(); }
			virtual auto setTxData(const TxType& tx_data)->void { txData() = tx_data; }
			virtual auto getRxData(RxType& rx_data)const->void { rx_data = rxData(); }
			virtual auto setTxData(const RxType& rx_data)->void { rxData() = rx_data; }
			virtual auto txTypeSize()const->std::size_t { return sizeof(TxType); }
			virtual auto rxTypeSize()const->std::size_t { return sizeof(RxType); }
            virtual auto logData(const TxType &tx_data, const RxType &rx_data, std::fstream &file)->void {}
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

            auto readPdo(int pdo_group_id, int pdo_id, std::int8_t &value)->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::int16_t &value)->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::int32_t &value)->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint8_t &value)->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint16_t &value)->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint32_t &value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int8_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int16_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int32_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint8_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint16_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint32_t value)->void;

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

			auto readSdo(int sdo_id, std::int8_t &value)->void;
			auto readSdo(int sdo_id, std::int16_t &value)->void;
			auto readSdo(int sdo_id, std::int32_t &value)->void;
			auto readSdo(int sdo_id, std::uint8_t &value)->void;
			auto readSdo(int sdo_id, std::uint16_t &value)->void;
			auto readSdo(int sdo_id, std::uint32_t &value)->void;
			auto writeSdo(int sdo_id, std::int8_t value)->void;
			auto writeSdo(int sdo_id, std::int16_t value)->void;
			auto writeSdo(int sdo_id, std::int32_t value)->void;
			auto writeSdo(int sdo_id, std::uint8_t value)->void;
			auto writeSdo(int sdo_id, std::uint16_t value)->void;
			auto writeSdo(int sdo_id, std::uint32_t value)->void;

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

			auto readSdoConfig(int sdo_id, std::int8_t &value) const->void;
			auto readSdoConfig(int sdo_id, std::int16_t &value) const->void;
			auto readSdoConfig(int sdo_id, std::int32_t &value) const->void;
			auto readSdoConfig(int sdo_id, std::uint8_t &value) const->void;
			auto readSdoConfig(int sdo_id, std::uint16_t &value) const->void;
			auto readSdoConfig(int sdo_id, std::uint32_t &value) const->void;
			auto configSdo(int sdo_id, std::int8_t value)->void;
			auto configSdo(int sdo_id, std::int16_t value)->void;
			auto configSdo(int sdo_id, std::int32_t value)->void;
			auto configSdo(int sdo_id, std::uint8_t value)->void;
			auto configSdo(int sdo_id, std::uint16_t value)->void;
			auto configSdo(int sdo_id, std::uint32_t value)->void;

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
			Slave(Object &father, const aris::core::XmlElement &xml_ele);
			Slave(const Slave &other) = delete;
			Slave(Slave &&other) = delete;
			Slave& operator=(const Slave &other) = delete;
			Slave& operator=(Slave &&other) = delete;

		protected:
            virtual auto init()->void {}
            virtual auto readUpdate()->void {}
            virtual auto writeUpdate()->void {}

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
			virtual auto loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
			virtual auto start()->void;
			virtual auto stop()->void;
			auto ecHandle()const->const Handle*;
			auto ecHandle()->Handle*;
			auto slaveTypePool()->aris::core::ObjectPool<SlaveType, Element>&;
			auto slaveTypePool()const->const aris::core::ObjectPool<SlaveType, Element>&;
			auto slavePool()->aris::core::ObjectPool<Slave, Element>&;
			auto slavePool()const->const aris::core::ObjectPool<Slave, Element>&;
			auto txDataPool()->aris::core::RefPool<Slave::TxType> &;
			auto txDataPool()const->const aris::core::RefPool<Slave::TxType> &;
			auto rxDataPool()->aris::core::RefPool<Slave::RxType> &;
			auto rxDataPool()const->const aris::core::RefPool<Slave::RxType> &;
			auto dataLogger()->DataLogger&;
			auto dataLogger()const->const DataLogger&;

		protected:
            virtual auto controlStrategy()->void {}
			virtual ~Master();
			Master();
			Master(const Master &other) = delete;
			Master(Master &&other) = delete;
			Master& operator=(const Master &other) = delete;
			Master& operator=(Master &&other) = delete;

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
			virtual auto txData()->TxType & override { return tx_data_; }
			virtual auto txData()const->const TxType & override{ return tx_data_; }
			virtual auto rxData()->RxType & override { return rx_data_; }
			virtual auto rxData()const->const RxType & override{ return rx_data_; }
			virtual auto getTxData(Slave::TxType& tx_data)const->void override { static_cast<TxType &>(tx_data) = txData(); }
			virtual auto setTxData(const Slave::TxType& tx_data)->void override { txData() = static_cast<const TxType &>(tx_data); }
			virtual auto getRxData(Slave::RxType& rx_data)const->void override { static_cast<RxType &>(rx_data) = rxData(); }
			virtual auto setTxData(const Slave::RxType& rx_data)->void override { rxData() = static_cast<const RxType &>(rx_data); }
			virtual auto txTypeSize()const->std::size_t override { return sizeof(TxType); }
			virtual auto rxTypeSize()const->std::size_t override { return sizeof(RxType); }

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
