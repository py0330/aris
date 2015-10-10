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
			virtual void Initialize();

			void ReadPdo(int pdoGroupID, int pdoID, std::uint8_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::uint16_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::uint32_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::int8_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::int16_t &value) const;
			void ReadPdo(int pdoGroupID, int pdoID, std::int32_t &value) const;
			void WritePdo(int pdoGroupID, int pdoID, const std::int16_t &value);






			void ReadPdo(int pdoGroupID, int pdoID, void *dataAddress);
			void WritePdo(int pdoGroupID, int pdoID, void *dataAddress);
			
		private:
			ETHERCAT_SLAVE(Aris::Core::ELEMENT *);
			ETHERCAT_SLAVE(const ETHERCAT_SLAVE &other) = delete;
			ETHERCAT_SLAVE(ETHERCAT_SLAVE &&other) = delete;
			ETHERCAT_SLAVE & operator=(const ETHERCAT_SLAVE &other) = delete;
			ETHERCAT_SLAVE & operator=(ETHERCAT_SLAVE &&other) = delete;

		private:
			class IMP;
			IMP *pImp;

			friend class ETHERCAT_MASTER;
		};

		class ETHERCAT_MASTER
		{
		public:
			static ETHERCAT_MASTER *GetInstance() ;

			void LoadXml(Aris::Core::ELEMENT *);
			void Initialize();
			void Run();
			
		private:
			ETHERCAT_MASTER();
			~ETHERCAT_MASTER();
			ETHERCAT_MASTER(const ETHERCAT_MASTER &other) = delete;
			ETHERCAT_MASTER(ETHERCAT_MASTER &&other) = delete;
			ETHERCAT_MASTER & operator=(const ETHERCAT_MASTER &other) = delete;
			ETHERCAT_MASTER & operator=(ETHERCAT_MASTER &&other) = delete;

			
		private:
			static ETHERCAT_MASTER instance;

		private:
			class IMP;
			IMP *pImp;

			friend class ETHERCAT_SLAVE::IMP;

		};















	}
}



















#endif