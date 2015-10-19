#ifndef ARIS_MOTION_H
#define ARIS_MOTION_H

#include "Aris_EtherCat.h"

#include <functional>
#include <thread>

#include <atomic>

namespace Aris
{
	namespace Control
	{	
		class MOTION :public ETHERCAT_SLAVE
		{
		public:
			enum CMD
			{
				IDLE = 0,
				ENABLE,
				DISABLE,
				HOME,
				RUN
			};
			enum MODE
			{
				POSITION = 0x0008,
				VELOCITY = 0x0009,
				CURRENT = 0x0010,
			};
			struct DATA
			{
				std::int32_t targetPos{ 0 }, feedbackPos{ 0 };
				std::int32_t targetVel{ 0 }, feedbackVel{ 0 };
				std::int16_t targetCur{ 0 }, feedbackCur{ 0 };
				std::uint8_t cmd{ IDLE };
				std::uint8_t mode{ POSITION };
				mutable std::int16_t ret{ 0 };
			};

			virtual ~MOTION();
			MOTION(const Aris::Core::ELEMENT *);
			
			bool HasFault();
			void ReadFeedback(DATA &data);
			void DoCommand(const DATA &data);
			void SetHomeOffSet(std::int32_t homeOffSet);

		protected:
			virtual void Initialize() override;

		private:
			class IMP;
			std::unique_ptr<IMP> pImp;

			friend class CONTROLLER;
		};

		class CONTROLLER :public ETHERCAT_MASTER
		{
		public:
			struct DATA
			{
				const std::vector<MOTION::DATA> *pLastMotionData;
				std::vector<MOTION::DATA> *pMotionData;
				const Aris::Core::RT_MSG *pMsgRecv;
				Aris::Core::RT_MSG *pMsgSend;
			};

			virtual ~CONTROLLER(){};
			virtual void LoadXml(const Aris::Core::ELEMENT *) override;
			virtual void Start();
			virtual void Stop();
			MOTION * Motion(int i) { return pMotions.at(i); };
			PIPE<Aris::Core::MSG>& MsgPipe(){return msgPipe;};
			void SetControlStrategy(std::function<int(DATA&)>);

		protected:
			CONTROLLER() :ETHERCAT_MASTER(),msgPipe(0, true) {};
			virtual void ControlStrategy() override;

		private:
			std::function<int(DATA&)> strategy;

			std::vector<MOTION *> pMotions;
			PIPE<Aris::Core::MSG> msgPipe;

			std::vector<MOTION::DATA> motionData, lastMotionData;
			std::unique_ptr<PIPE<std::vector<MOTION::DATA> > > pMotDataPipe;
			std::thread motionDataThread;

			std::atomic_bool isStoping;

			friend class ETHERCAT_MASTER;
		};

		struct CONTROL_DATA;













	}
}

#endif
