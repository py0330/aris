#ifndef ARIS_CONTROL_ETHERCAT_H
#define ARIS_CONTROL_ETHERCAT_H

#include <vector>
#include <memory>
#include <cstdint>

#include <aris_core.h>
#include <aris_control_pipe.h>

namespace Aris
{
	/// \brief 控制命名空间
	/// \ingroup Aris
	/// 
	///
	///
	namespace Control
	{	
		class EthercatSlave
		{
		public:
			virtual ~EthercatSlave();
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

		protected:
            EthercatSlave(const Aris::Core::XmlElement &xml_ele);

		private:
			EthercatSlave(const EthercatSlave &other) = delete;
			EthercatSlave(EthercatSlave &&other) = delete;
			EthercatSlave & operator=(const EthercatSlave &other) = delete;
			EthercatSlave & operator=(EthercatSlave &&other) = delete;

			struct Imp;
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
			virtual auto loadXml(const Aris::Core::XmlElement &xml_ele)->void;
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
	}
}



















#endif
