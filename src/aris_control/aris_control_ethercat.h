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
			void readPdo(int pdoGroupID, int pdoID, std::int8_t &value) const;
			void readPdo(int pdoGroupID, int pdoID, std::int16_t &value) const;
			void readPdo(int pdoGroupID, int pdoID, std::int32_t &value) const;
			void readPdo(int pdoGroupID, int pdoID, std::uint8_t &value) const;
			void readPdo(int pdoGroupID, int pdoID, std::uint16_t &value) const;
			void readPdo(int pdoGroupID, int pdoID, std::uint32_t &value) const;
			void writePdo(int pdoGroupID, int pdoID, std::int8_t value);
			void writePdo(int pdoGroupID, int pdoID, std::int16_t value);
			void writePdo(int pdoGroupID, int pdoID, std::int32_t value);
			void writePdo(int pdoGroupID, int pdoID, std::uint8_t value);
			void writePdo(int pdoGroupID, int pdoID, std::uint16_t value);
			void writePdo(int pdoGroupID, int pdoID, std::uint32_t value);
			void readSdo(int sdoID, std::int32_t &value) const;
			void writeSdo(int sdoID, std::int32_t value);

		protected:
            EthercatSlave(const Aris::Core::XmlElement &xml_ele);
			virtual void init();

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
			static EthercatMaster &instance();
			virtual ~EthercatMaster();
			virtual void loadXml(const Aris::Core::XmlElement &xml_ele);
			virtual void start();
			virtual void stop();
			template <class EthercatController>	static EthercatController* createInstance()
			{
				if (pInstance)
				{
					throw std::runtime_error("EthercatMaster can not create a controller, because it already has one");
				}

				pInstance.reset(new EthercatController);
				return static_cast<EthercatController*>(pInstance.get());
			}
			template <class Slave, typename ...Args> Slave* addSlave(Args ...args)
			{
				auto pSla = new Slave(args...);
				this->addSlavePtr(pSla);
				return pSla;
			}
			
		protected:
			EthercatMaster();
			virtual void controlStrategy() {};
			
		private:
			EthercatMaster(const EthercatMaster &other) = delete;
			EthercatMaster(EthercatMaster &&other) = delete;
			EthercatMaster & operator=(const EthercatMaster &other) = delete;
			EthercatMaster & operator=(EthercatMaster &&other) = delete;
			void addSlavePtr(EthercatSlave *pSla);
			
		private:
			static std::unique_ptr<EthercatMaster> pInstance;
			class Imp;
			std::unique_ptr<Imp> pImp;

			friend class EthercatSlave::Imp;
		};
	}
}



















#endif
