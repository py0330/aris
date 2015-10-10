#include <Platform.h>
#ifdef PLATFORM_IS_LINUX
#include <ecrt.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <sys/mman.h>
#endif


#include <string>
#include <iostream>

#include <Aris_Motion.h>


namespace Aris
{
	namespace Control
	{
		MOTION::MOTION(Aris::Core::ELEMENT *ele)
			:ETHERCAT_SLAVE(ele)
		{
		};
		void MOTION::Initialize()
		{
			this->WriteSdo(9, 100);			
			this->ETHERCAT_SLAVE::Initialize();		
			
		}
		
		void CONTROLLER::LoadXml(Aris::Core::ELEMENT *ele)
		{
			auto pSlaves = ele->FirstChildElement("Slave");

			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{			
				static int i = 0;
				pMotions[i++] = AddSlave<MOTION>(pSla);
			}
		}
		void CONTROLLER::ControlStrategy()
		{
			std::int32_t pos;
			pMotions[0]->ReadPdo(1, 0, pos);
			pMotions[0]->WritePdo(0, 0, pos);


#ifdef PLATFORM_IS_LINUX
			static int i = 0;

			if ((i++ % 500) == 0)
			{
				rt_printf("pos is:%d\n", pos);
			}
#endif




		}


	}
}
