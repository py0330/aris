#ifndef ARIS_ETHERCAT_H
#define ARIS_ETHERCAT_H

#include "Aris_XML.h"

#include <vector>
#include <memory>
#include <cstdint>

namespace Aris
{
	namespace Control
	{	
		class ETHERCAT_SLAVE
		{
		public:
			virtual ~ETHERCAT_SLAVE();
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
			ETHERCAT_SLAVE(Aris::Core::ELEMENT *);
			virtual void Initialize();

		private:
			ETHERCAT_SLAVE(const ETHERCAT_SLAVE &other) = delete;
			ETHERCAT_SLAVE(ETHERCAT_SLAVE &&other) = delete;
			ETHERCAT_SLAVE & operator=(const ETHERCAT_SLAVE &other) = delete;
			ETHERCAT_SLAVE & operator=(ETHERCAT_SLAVE &&other) = delete;

			class IMP;
			IMP *pImp;

			friend class ETHERCAT_MASTER;
		};

		class ETHERCAT_MASTER
		{
		public:
			virtual ~ETHERCAT_MASTER();
			virtual void LoadXml(Aris::Core::ELEMENT *);
			static ETHERCAT_MASTER *GetInstance();
			template <class CONTROLLER>	static CONTROLLER* CreateMaster()
			{
				if (pInstance)
				{
					throw std::runtime_error("ETHERCAT_MASTER can not create a controller, because it already has one");
				}

				pInstance = std::unique_ptr<ETHERCAT_MASTER>(new CONTROLLER);
				return static_cast<CONTROLLER*>(pInstance.get());
			}
			template <class SLAVE, typename ...Args> SLAVE* AddSlave(Args ...args)
			{
				auto pSla = new SLAVE(args...);
				this->AddSlavePtr(pSla);
				return pSla;
			}
			void Run();
			
		protected:
			ETHERCAT_MASTER();
			virtual void ControlStrategy() {};
			
		private:
			ETHERCAT_MASTER(const ETHERCAT_MASTER &other) = delete;
			ETHERCAT_MASTER(ETHERCAT_MASTER &&other) = delete;
			ETHERCAT_MASTER & operator=(const ETHERCAT_MASTER &other) = delete;
			ETHERCAT_MASTER & operator=(ETHERCAT_MASTER &&other) = delete;
			void AddSlavePtr(ETHERCAT_SLAVE *pSla);
			
		private:
			static std::unique_ptr<ETHERCAT_MASTER> pInstance;
			class IMP;
			IMP *pImp;

			friend class ETHERCAT_SLAVE::IMP;
		};















	}
}



















#endif