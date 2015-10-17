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
		struct MOTION_DATA
		{
			std::int32_t targetPos, feedbackPos, targetVel, feedbackVel;
			std::int16_t targetCur, feedbackCur;
		};
		
		class MOTION :public ETHERCAT_SLAVE
		{
		public:
			enum MODE
			{
				//POSITION=0x0008,
				VELOCITY = 0x0009,
				CURRENT = 0x0010,
			};

			virtual ~MOTION() {};
			MOTION(Aris::Core::ELEMENT *);
			/*return 0 means successful, 1 means still enabling, -1 means error*/
            int Enable(MODE mode = MOTION::VELOCITY);
			int Disable();
            int Home();
			int RunPos(std::int32_t pos);
			int RunVel(std::int32_t vel);
            int RunCur(std::int16_t cur);
            bool HasFault();
			std::int32_t Pos() { std::int32_t pos; this->ReadPdo(1, 0, pos); return pos; };
			std::int32_t Vel() { std::int32_t vel; this->ReadPdo(1, 2, vel); return vel; };
			std::int32_t Cur() { std::int32_t cur; this->ReadPdo(2, 0, cur); return cur; };

		protected:
			virtual void Initialize() override;
            bool isEverHomed{false};
            int enableCount{0};
            std::uint8_t runningMode{9};
			friend class CONTROLLER;
		};

		class CONTROLLER :public ETHERCAT_MASTER
		{
		public:
			virtual ~CONTROLLER(){};
			virtual void LoadXml(Aris::Core::ELEMENT *) override;
			virtual void Start();
			virtual void Stop();
			MOTION * Motion(int i) { return pMotions.at(i); };
			PIPE<Aris::Core::MSG>& MsgPipe(){return msgPipe;};
			void SetControlStrategy(std::function<void(CONTROLLER *pController)>);

		protected:
			CONTROLLER() :ETHERCAT_MASTER(),msgPipe(0, true) {};
			virtual void ControlStrategy() override;

		private:
			std::function<void(CONTROLLER *pController)> strategy;

			std::vector<MOTION *> pMotions;
			PIPE<Aris::Core::MSG> msgPipe;

			std::vector<MOTION_DATA> motionData;
			std::unique_ptr<PIPE<std::vector<MOTION_DATA> > > pMotDataPipe;
			std::thread motionDataThread;

			std::atomic_bool isStoping;

			friend class ETHERCAT_MASTER;
		};

		struct CONTROL_DATA;













	}
}

#endif
