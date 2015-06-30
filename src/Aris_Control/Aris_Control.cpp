#include "Aris_Control.h"
#include "Aris_SysBase.h"
using namespace Aris::RT_CONTROL;

CSysBase shadow;

CSysBase* Aris::RT_CONTROL::ACTUATION::sysBase;

//Aris::Core::MSG Aris::RT_CONTROL::ACTUATION::m_recvData;

 Aris::Core::THREAD Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher;

Aris::Core::RT_MSG*  Aris::RT_CONTROL::ACTUATION::DataRecv;
Aris::Core::RT_MSG*  Aris::RT_CONTROL::ACTUATION::DataSend;
Aris::Core::MSG Aris::RT_CONTROL::ACTUATION::nrt_DataRecv;

//Aris::RT_CONTROL::CMachineData  Aris::RT_CONTROL::ACTUATION::MachineData;

Aris::RT_CONTROL::ACTUATION::ACTUATION()
{
	rt_printf("actuation constructed\n");
   	sysBase=&shadow;

  	DataRecv=&Aris::Core::RT_MSG::instance[0];
   	DataSend=&Aris::Core::RT_MSG::instance[1];
   	shadow.m_rtDataRecv=&Aris::Core::RT_MSG::instance[1];
   	shadow.m_rtDataSend=&Aris::Core::RT_MSG::instance[1];

    //shadow.m_rtDataRecvBuffer=(char*)&Aris::Core::RT_MSG::instance[0];
    //shadow.m_rtDataSendBuffer=(char*)&Aris::Core::RT_MSG::instance[1];

 	//MachineData=&shadow.m_machineDataCore;
};

Aris::RT_CONTROL::ACTUATION::~ACTUATION()
{
};

int Aris::RT_CONTROL::ACTUATION::SetSysInitializer(FuncPtrInit p_Initializer)
{
	return sysBase->SetSysInitializer(p_Initializer);
};

int  Aris::RT_CONTROL::ACTUATION::SysInit(CSysInitParameters p_Param)
{
	 Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher.SetFunction(NRT_Watcher);
	return sysBase->SysInit(p_Param);
};

int Aris::RT_CONTROL::ACTUATION::SysStart()
{
	int ret;
	ret=sysBase->SysStart();
	 Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher.Start(0);
	return ret;
};

int Aris::RT_CONTROL::ACTUATION::SysStop()
{
	//Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher.Terminate();
	return sysBase->SysStop();
};

int Aris::RT_CONTROL::ACTUATION::SetTrajectoryGenerator(FuncPtrWork p_Generator)
{
	return sysBase->SetTrajectoryGenerator(p_Generator);
};

//int Aris::Control::CControlSystem::SetLogger(FuncPtrWork p_Logger)
//{
//	return sysBase->SetLogger(p_Logger);
//
//};

int Aris::RT_CONTROL::ACTUATION::SysInitCommunication()
{

	return sysBase->SysInitCommunication();

};

bool Aris::RT_CONTROL::ACTUATION::IsSysStopped()
{
	return sysBase->IsSysStopped();
};


int Aris::RT_CONTROL::ACTUATION::SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater)
{
	return sysBase->SetOnDataUpdateHandler(p_DataUpdater);
};


 void Aris::RT_CONTROL::ACTUATION::GetMachineState(Aris::RT_CONTROL::EMachineState p_state )
{
	p_state=sysBase->m_machineDataCore.machinestate;

}


 int Aris::RT_CONTROL::ACTUATION::NRT_PostMsg(Aris::Core::MSG &p_data)
 {
 	int ret;
   	ret=sysBase->NRT_SendDataRaw(p_data._pData,p_data.GetLength()+MSG_HEADER_LENGTH);
 	//printf("NRT_SendData is me :%d\n",ret);
 	return 0;
 }//ok


int Aris::RT_CONTROL::ACTUATION::RT_PostMsg(Aris::Core::RT_MSG &p_data)
{
	      int ret;
			ret=sysBase->RT_SendDataRaw( p_data._pData,RT_MSG_HEADER_LENGTH);


			if(ret<0)
				return ret;
 			ret=sysBase->RT_SendDataRaw(p_data.GetDataAddress(),p_data.GetLength());
 		    return ret;
 }

//not used by customers


int Aris::RT_CONTROL::ACTUATION::NRT_RecvMsg(Aris::Core::MSG &p_data)
{
 	int ret;
	char Head[MSG_HEADER_LENGTH];
	sysBase->NRT_RecvDataRaw(Head,MSG_HEADER_LENGTH);

	 p_data.SetLength(*((unsigned int*)Head));
	 p_data.SetMsgID(*((int*)(Head+4)));
	 p_data.SetType(*((long long*)(Head+8)));

     ret=sysBase->NRT_RecvDataRaw(p_data.GetDataAddress(),p_data.GetLength());
 	return ret;

};



void* Aris::RT_CONTROL::ACTUATION::NRT_Watcher(void*)
{

	printf("NRT_Watcher\n");
	while(!Aris::RT_CONTROL::ACTUATION::IsSysStopped())
	{
   		 int ret=Aris::RT_CONTROL::ACTUATION::NRT_RecvMsg(nrt_DataRecv);

 	 	if(ret>=0)
 	 	{
 	 	 	 Aris::Core::PostMsg(nrt_DataRecv);
 	  	 //    printf("posting msg from  NRT_WATCHER,msg ID %d\n",nrt_DataRecv.GetMsgID());
 	 	}
	}
};



int Aris::RT_CONTROL::ACTUATION::Load_XML_PrintMessage()
{
	return sysBase->Load_XML_PrintMessages();
}
























