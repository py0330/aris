/*
 * ac_test.cpp

 *
 *  Created on: Nov 26, 2014
 *      Author: leo
 */
#include <iostream>
//#include "Aris_ControlData.h"
#include "Aris_Control.h"
#include "Aris_Message.h"
#include "Aris_Thread.h"
#include "Aris_Socket.h"

using namespace std;
using namespace Aris::RT_CONTROL;



Aris::RT_CONTROL::ACTUATION cs;
//CSysBase sysbase;
 Aris::RT_CONTROL::CSysInitParameters initParam;

 enum MACHINE_CMD
 {
         NOCMD=1000,
	 POWEROFF=1001,
	 STOP=1002,
	 ENABLE=1003,
	 RUNNING=1004,
	 GOHOME=1005,
 };

 int count;

/*
 * Trajectory Generator
 */

 void* Thread2(void *)
 {
	 cout<<"running msgloop"<<endl;
 	Aris::Core::RunMsgLoop();
 	return NULL;
 };
void* Thread1(void *)
{
	int cmd;

	while(!cs.IsSysStopped())
		{
			std::cin>>cmd;

			if(cmd == 0)
						{
 							Aris::Core::MSG data;
							data.SetMsgID(NOCMD);
							cs.NRT_PostMsg(data);
						}
			else if(cmd == 1)
			{
 				Aris::Core::MSG data;
				data.SetMsgID(POWEROFF);
				cs.NRT_PostMsg(data);
			}
			else if(cmd == 2)
			{
 				Aris::Core::MSG data;
				data.SetMsgID(STOP);
				cs.NRT_PostMsg(data);
			}

			else if(cmd == 3)
			{
 				Aris::Core::MSG data;
				data.SetMsgID(ENABLE);
				cs.NRT_PostMsg(data);
			}
			else if(cmd == 4)
			{
 				Aris::Core::MSG data;
				data.SetMsgID(RUNNING);
				cs.NRT_PostMsg(data);
			}
			else if(cmd == 5)
			{
 				Aris::Core::MSG data;
				data.SetMsgID(GOHOME );
				cs.NRT_PostMsg(data);
			}

			else
			{
				printf("Hi! I didn't get validate cmd\n");
			}
		}

	return 0;
};




int tg(Aris::RT_CONTROL::CMachineData& machineData,Aris::RT_CONTROL::RT_MSG& msg)
{

 	int CommandID;

	 CommandID=msg.GetMsgID();
 	switch(CommandID)
	{
	case NOCMD:

	    machineData.motorsCommands[0]=EMCMD_NONE;
	    rt_printf("NONE Command Get in NRT\n" );

	break;

	case ENABLE:

		machineData.motorsCommands[0]=EMCMD_ENABLE;
 		rt_printf("ENABLE Command Get in NRT\n" );

		break;
	case POWEROFF:
		machineData.motorsCommands[0]=EMCMD_POWEROFF;
		rt_printf("POWEROFF Command Get in NRT\n" );

		break;
	case STOP:

		machineData.motorsCommands[0]=EMCMD_STOP;
		rt_printf("STOP Command Get in NRT\n" );

		break;
	case RUNNING:

		machineData.motorsCommands[0]=EMCMD_RUNNING;
		rt_printf("RUNNING Command Get in NRT\n" );
		break;

	case GOHOME:
		machineData.motorsCommands[0]=EMCMD_GOHOME;
		rt_printf("RUNNING Command Get in NRT\n" );
		break;
	default:
		//DO NOTHING, CMD AND TRAJ WILL KEEP STILL
 		break;
	}


return 0;

};

//offsets driver order
static int HEXBOT_HOME_OFFSETS_RESOLVER= -15849882;

int main(int argc, char** argv)
{
	Aris::Core::THREAD T1,T2,T3 ;
 	 T1.SetFunction(Thread1);
	 T2.SetFunction(Thread2);

     T2.Start(0);

	cs.SetSysInitializer(NULL);

	cs.SetTrajectoryGenerator(tg);

	initParam.motorNum=1;
	initParam.homeHighSpeed=400000;
	initParam.homeLowSpeed=60000;

	initParam.homeOffsets=&HEXBOT_HOME_OFFSETS_RESOLVER;

 	cs.SysInit(initParam);

	cs.SysInitCommunication();

	cs.SysStart();

	T1.Start(0);

	printf("Will start\n");
	while(!cs.IsSysStopped())
	{

		count++;
		sleep(1);
	}


	return 0;

};


