#ifndef ARIS_ETHERCAT_H
#define ARIS_ETHERCAT_H

#include "Aris_XML.h"
#include "Aris_Core.h"

#include <vector>
#include <memory>
#include <cstdint>

namespace Aris
{
	namespace Control
	{	
		class PIPE_BASE
		{
		protected:
			PIPE_BASE(int port, bool isBlock);
			~PIPE_BASE();

			int SendToRT_RawData(const void *pData, int size);
			int SendToNRT_RawData(const void* pData, int size);
			int RecvInRT_RawData(void* pData, int size);
			int RecvInNRT_RawData(void *pData, int size);

			class IMP;
			IMP *pImp;
		};

		template <typename STANDARD_LAYOUT_STRUCT>
		class PIPE:private PIPE_BASE
		{
			PIPE(int port, bool isBlock):PIPE_BASE(port,isBlock){};
			int SendToRT(const STANDARD_LAYOUT_STRUCT &data)
			{
				SendToRT_RawData(static_cast<const void*>(&data),sizeof(data));
			};
			int SendToNRT(const STANDARD_LAYOUT_STRUCT &data)
			{
				SendToNRT_RawData(static_cast<const void*>(&data),sizeof(data));
			};
			int RecvInRT(STANDARD_LAYOUT_STRUCT &data)
			{
				RecvInRT_RawData(static_cast<void*>(&data),sizeof(data));
			};
			int RecvInNRT(STANDARD_LAYOUT_STRUCT &data)
			{
				RecvInNRT_RawData(static_cast<void*>(&data),sizeof(data));
			};
		};

		template <>
		class PIPE<Aris::Core::MSG>:public PIPE_BASE
		{
		public:
			PIPE(int port, bool isBlock);
			int SendToRT(const Aris::Core::MSG &msg);
			int SendToNRT(const Aris::Core::RT_MSG &msg);
			int RecvInRT(Aris::Core::RT_MSG &msg);
			int RecvInNRT(Aris::Core::MSG &msg);
		};

		
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
