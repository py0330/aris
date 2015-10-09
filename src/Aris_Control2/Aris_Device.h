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
			void Read();
			void Write();
			
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
			void Read();
			void Write();
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