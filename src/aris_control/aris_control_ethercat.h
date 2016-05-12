#ifndef ARIS_CONTROL_ETHERCAT_H
#define ARIS_CONTROL_ETHERCAT_H

#include <vector>
#include <memory>
#include <cstdint>

#include <aris_core.h>
#include <aris_control_pipe.h>

namespace aris
{
	/// \brief 控制命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace control
	{	
		class EthercatSlave
		{
		public:
			auto readPdo(int pdo_group_id, int pdo_id, std::int8_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::int16_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::int32_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint8_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint16_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint32_t &value)const->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int8_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int16_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int32_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint8_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint16_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint32_t value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)const->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void;
			auto readSdo(int sdo_id, std::int8_t &value) const->void;
			auto readSdo(int sdo_id, std::int16_t &value) const->void;
			auto readSdo(int sdo_id, std::int32_t &value) const->void;
			auto readSdo(int sdo_id, std::uint8_t &value) const->void;
			auto readSdo(int sdo_id, std::uint16_t &value) const->void;
			auto readSdo(int sdo_id, std::uint32_t &value) const->void;
			auto writeSdo(int sdo_id, std::int8_t value)->void;
			auto writeSdo(int sdo_id, std::int16_t value)->void;
			auto writeSdo(int sdo_id, std::int32_t value)->void;
			auto writeSdo(int sdo_id, std::uint8_t value)->void;
			auto writeSdo(int sdo_id, std::uint16_t value)->void;
			auto writeSdo(int sdo_id, std::uint32_t value)->void;
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
			virtual ~EthercatSlave();

		protected:
            EthercatSlave(const aris::core::XmlElement &xml_ele);
			virtual auto init()->void {};

		private:
			EthercatSlave(const EthercatSlave &other) = delete;
			EthercatSlave(EthercatSlave &&other) = delete;
			EthercatSlave & operator=(const EthercatSlave &other) = delete;
			EthercatSlave & operator=(EthercatSlave &&other) = delete;

			class Imp;
			std::unique_ptr<Imp> imp_;

			friend class EthercatMaster;
		};
		class EthercatMaster
		{
		public:
			template <class EthercatController>
			static auto createInstance()->EthercatController*
			{
				if (instancePtr())
				{
					throw std::runtime_error("EthercatMaster can not create a controller, because it already has one");
				}

				const_cast<std::unique_ptr<EthercatMaster> &>(instancePtr()).reset(new EthercatController);
				return static_cast<EthercatController*>(instancePtr().get());
			}
			static auto instance()->EthercatMaster &;
			static auto instancePtr()->const std::unique_ptr<EthercatMaster> &;
			virtual ~EthercatMaster();
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void;
			virtual auto start()->void;
			virtual auto stop()->void;
			template <class Slave, typename ...Args> 
			auto addSlave(Args ...args)->Slave*
			{
				auto sla = new Slave(args...);
				this->addSlavePtr(sla);
				return sla;
			}
			
		protected:
			EthercatMaster();
			virtual auto controlStrategy()->void {};
			
		private:
			EthercatMaster(const EthercatMaster &other) = delete;
			EthercatMaster(EthercatMaster &&other) = delete;
			EthercatMaster & operator=(const EthercatMaster &other) = delete;
			EthercatMaster & operator=(EthercatMaster &&other) = delete;
			auto addSlavePtr(EthercatSlave *pSla)->void;
			
		private:
			class Imp;
			std::unique_ptr<Imp> imp_;

			friend class EthercatSlave;
		};

		class Master;
		class Slave;

		class Element :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("element"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto master()->Master &;
			auto master()const->const Master &;
			Element(aris::core::Object &father, std::size_t id, const std::string &name) :Object(father, id, name) {}
			Element(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Object(father, id, xml_ele){}

		private:
			friend class Master;
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
			auto index()const->std::uint16_t { return index_; }
			auto subindex()const->std::uint8_t { return subindex_; }
			auto offset()const->std::uint32_t { return offset_; }
			auto dataSize()const->std::uint8_t { return data_size_; }
			auto dataType()const->DataType { return data_type_; }
			
			DataType data_type_;
			std::uint16_t index_;
			std::uint8_t subindex_;
			std::uint8_t data_size_;
			std::uint32_t offset_;
			Slave *slave_;

			DO(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele):Element(father, id, xml_ele)
			{
				index_ = attributeUint16(xml_ele, "index");
				subindex_ = attributeUint8(xml_ele, "subindex");

				if (!xml_ele.Attribute("datatype"))throw std::runtime_error("Data Object in slave must have \"datatype\" attribute");
				else if (xml_ele.Attribute("datatype", "int32"))
				{
					data_type_ = INT32;
					data_size_ = 32;
				}
				else if (xml_ele.Attribute("datatype", "int16"))
				{
					data_type_ = INT16;
					data_size_ = 16;
				}
				else if (xml_ele.Attribute("datatype", "int8"))
				{
					data_type_ = INT8;
					data_size_ = 8;
				}
				else if (xml_ele.Attribute("datatype", "uint32"))
				{
					data_type_ = UINT32;
					data_size_ = 32;
				}
				else if (xml_ele.Attribute("datatype", "uint16"))
				{
					data_type_ = UINT16;
					data_size_ = 16;
				}
				else if (xml_ele.Attribute("datatype", "uint8"))
				{
					data_type_ = UINT8;
					data_size_ = 8;
				}
				else
				{
					throw std::runtime_error("Data Object in slave has invalid \"datatype\" attribute");
				}
			}
		};
		class Pdo :public DO
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("pdo"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto read(std::int32_t &value)const->void;
			auto read(std::int16_t &value)const->void;
			auto read(std::int8_t &value)const->void;
			auto read(std::uint32_t &value)const->void;
			auto read(std::uint16_t &value)const->void;
			auto read(std::uint8_t &value)const->void;
			auto write(std::int32_t value)->void;
			auto write(std::int16_t value)->void;
			auto write(std::int8_t value)->void;
			auto write(std::uint32_t value)->void;
			auto write(std::uint16_t value)->void;
			auto write(std::uint8_t value)->void;
			Pdo(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele):DO(father, id, xml_ele){}
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
			static auto Type()->const std::string &{ static const std::string type("sdo"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto readable()const->bool;
			auto writeable()const->bool;
			auto configurable()const->bool;
			auto option()const->unsigned;
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
			auto read(std::int32_t &value)const->void;
			auto read(std::int16_t &value)const->void;
			auto read(std::int8_t &value)const->void;
			auto read(std::uint32_t &value)const->void;
			auto read(std::uint16_t &value)const->void;
			auto read(std::uint8_t &value)const->void;
			auto write(std::int32_t value)->void;
			auto write(std::int16_t value)->void;
			auto write(std::int8_t value)->void;
			auto write(std::uint32_t value)->void;
			auto write(std::uint16_t value)->void;
			auto write(std::uint8_t value)->void;
			Sdo(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Slave;
		};
		class PdoGroup:public aris::core::ObjectPool<Pdo, Element>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("pdo_group"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto tx()const->bool;
			auto rx()const->bool;
			auto index()const->std::uint16_t;
			PdoGroup(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Slave;
		};
		class SlaveType:public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("slave_type"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto productCode()const->std::uint32_t;
			auto venderID()const->std::uint32_t;
			auto alias()const->std::uint16_t;
			auto distributedClock()const->std::uint32_t;

			SlaveType(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Slave: public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("slave"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto position()const ->std::uint16_t { return static_cast<std::uint16_t>(id()); }
			auto pdoGroupPool()->aris::core::ObjectPool<PdoGroup, Element>&;
			auto pdoGroupPool()const->const aris::core::ObjectPool<PdoGroup, Element>&;
			auto sdoPool()->aris::core::ObjectPool<Sdo, Element>&;
			auto sdoPool()const->const aris::core::ObjectPool<Sdo, Element>&;
			auto readPdo(int pdo_group_id, int pdo_id, std::int8_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::int16_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::int32_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint8_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint16_t &value)const->void;
			auto readPdo(int pdo_group_id, int pdo_id, std::uint32_t &value)const->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int8_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int16_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::int32_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint8_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint16_t value)->void;
			auto writePdo(int pdo_group_id, int pdo_id, std::uint32_t value)->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)const->void;
			auto readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)const->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void;
			auto writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void;
			auto readSdo(int sdo_id, std::int8_t &value) const->void;
			auto readSdo(int sdo_id, std::int16_t &value) const->void;
			auto readSdo(int sdo_id, std::int32_t &value) const->void;
			auto readSdo(int sdo_id, std::uint8_t &value) const->void;
			auto readSdo(int sdo_id, std::uint16_t &value) const->void;
			auto readSdo(int sdo_id, std::uint32_t &value) const->void;
			auto writeSdo(int sdo_id, std::int8_t value)->void;
			auto writeSdo(int sdo_id, std::int16_t value)->void;
			auto writeSdo(int sdo_id, std::int32_t value)->void;
			auto writeSdo(int sdo_id, std::uint8_t value)->void;
			auto writeSdo(int sdo_id, std::uint16_t value)->void;
			auto writeSdo(int sdo_id, std::uint32_t value)->void;
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
			virtual ~Slave();
			Slave(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
		
		protected:
			virtual auto init()->void {};

		private:
			Slave(const Slave &other) = delete;
			Slave(Slave &&other) = delete;
			auto operator=(const Slave &other)->Slave & = delete;
			auto operator=(Slave &&other)->Slave & = delete;

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
			auto slaveTypePool()->aris::core::ObjectPool<SlaveType, Element>&;
			auto slaveTypePool()const->const aris::core::ObjectPool<SlaveType, Element>&;
			auto slavePool()->aris::core::ObjectPool<Slave, Element>&;
			auto slavePool()const->const aris::core::ObjectPool<Slave, Element>&;

			virtual ~Master();
			Master();
		protected:
			virtual auto controlStrategy()->void {};

		private:
			Master(const Master &other) = delete;
			Master(Master &&other) = delete;
			Master & operator=(const Master &other) = delete;
			Master & operator=(Master &&other) = delete;

			class Imp;
			std::unique_ptr<Imp> imp_;

			friend class Slave;
			friend class Sdo;
			friend class Pdo;
		};
	}
}



















#endif
