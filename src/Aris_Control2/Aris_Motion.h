#ifndef ARIS_MOTION_H
#define ARIS_MOTION_H

#include "Aris_EtherCat.h"



namespace Aris
{
	namespace Control
	{	
		struct MOTION_DATA
		{
		public:
			int status;
			int command;
			int current;
			int position;
			int velocity;
		};
		
	
		class MOTION :public ETHERCAT_SLAVE
		{
		public:
			virtual ~MOTION() {};
			MOTION(Aris::Core::ELEMENT *);

		protected:
			virtual void Initialize() override;

			friend class CONTROLLER;
		};



		class CONTROLLER :public ETHERCAT_MASTER
		{
		public:
			virtual ~CONTROLLER() {};
			virtual void LoadXml(Aris::Core::ELEMENT *) override;
			
		protected:
			CONTROLLER() = default;
			virtual void ControlStrategy() override;

		private:
			MOTION *pMotions[1];

			friend class ETHERCAT_MASTER;
		};















	}
}



















#endif