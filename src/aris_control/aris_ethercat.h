#ifndef ARIS_ETHERCAT_H
#define ARIS_ETHERCAT_H

#include <vector>
#include <memory>
#include <cstdint>

#include "aris_xml.h"
#include "aris_core.h"
#include "aris_pipe.h"

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
			void ReadPdo(int pdoGroupID, int pdoID, std::int8_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::int16_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::int32_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::uint8_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::uint16_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::uint32_t &value) const;
			void WritePdo(int pdoGroupID, int pdoID, std::int8_t value);
			void WritePdo(int pdoGroupID, int pdoID, std::int16_t value);
			void WritePdo(int pdoGroupID, int pdoID, std::int32_t value);
			void WritePdo(int pdoGroupID, int pdoID, std::uint8_t value);
			void WritePdo(int pdoGroupID, int pdoID, std::uint16_t value);
			void WritePdo(int pdoGroupID, int pdoID, std::uint32_t value);
			void ReadSdo(int sdoID, std::int32_t &value) const;
			void WriteSdo(int sdoID, std::int32_t value);

		protected:
            EthercatSlave(const Aris::Core::XmlElement &xml_ele);
			virtual void Init();

		private:
			EthercatSlave(const EthercatSlave &other) = delete;
			EthercatSlave(EthercatSlave &&other) = delete;
			EthercatSlave & operator=(const EthercatSlave &other) = delete;
			EthercatSlave & operator=(EthercatSlave &&other) = delete;

			class Imp;
			std::unique_ptr<Imp> pImp;

			friend class EthercatMaster;
		};
		class EthercatMaster
		{
		public:
			static EthercatMaster &Instance();
			virtual ~EthercatMaster();
			virtual void LoadXml(const Aris::Core::XmlElement &xml_ele);
			virtual void Start();
			virtual void Stop();
			template <class EthercatController>	static EthercatController* CreateMaster()
			{
				if (pInstance)
				{
					throw std::runtime_error("EthercatMaster can not create a controller, because it already has one");
				}

				pInstance.reset(new EthercatController);
				return static_cast<EthercatController*>(pInstance.get());
			}
			template <class Slave, typename ...Args> Slave* AddSlave(Args ...args)
			{
				auto pSla = new Slave(args...);
				this->AddSlavePtr(pSla);
				return pSla;
			}
			
		protected:
			EthercatMaster();
			virtual void ControlStrategy() {};
			
		private:
			EthercatMaster(const EthercatMaster &other) = delete;
			EthercatMaster(EthercatMaster &&other) = delete;
			EthercatMaster & operator=(const EthercatMaster &other) = delete;
			EthercatMaster & operator=(EthercatMaster &&other) = delete;
			void AddSlavePtr(EthercatSlave *pSla);
			
		private:
			static std::unique_ptr<EthercatMaster> pInstance;
			class Imp;
			std::unique_ptr<Imp> pImp;

			friend class EthercatSlave::Imp;
		};
	}
}



















#endif
