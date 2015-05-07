#include "Aris_Control.h"
#include "Aris_SysBase.h"
using namespace Aris::RT_CONTROL;

CSysBase shadow;

CSysBase* Aris::RT_CONTROL::ACTUATION::sysBase;

//Aris::Core::MSG Aris::RT_CONTROL::ACTUATION::m_recvData;

//Aris::Core::THREAD Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher;

//Aris::RT_CONTROL::RT_MSG* Aris::RT_CONTROL::ACTUATION::DataRecv;
//Aris::RT_CONTROL::RT_MSG* Aris::RT_CONTROL::ACTUATION::DataSend;
//Aris::RT_CONTROL::CMachineData  Aris::RT_CONTROL::ACTUATION::MachineData;

Aris::RT_CONTROL::ACTUATION::ACTUATION()
{
	rt_printf("actuation constructed\n");
   	sysBase=&shadow;
  	//DataRecv=&shadow.m_rtDataRecv;
 	//DataSend=&shadow.m_rtDataSend;
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
	//Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher.SetFunction(NRT_Watcher);
	return sysBase->SysInit(p_Param);
};

int Aris::RT_CONTROL::ACTUATION::SysStart()
{
	int ret;
	ret=sysBase->SysStart();
	//Aris::RT_CONTROL::ACTUATION::m_threadNRTWatcher.Start(0);
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

/*
int Aris::RT_CONTROL::ACTUATION::SetModeP2P()
{
	return sysBase->SetModeP2P();

};
int Aris::RT_CONTROL::ACTUATION::SetModeCycPos()
{
	return sysBase->SetModeCycPos();
};
int Aris::RT_CONTROL::ACTUATION::SetModeCycVel()
{
	return sysBase->SetModeCycVel();
};
int Aris::RT_CONTROL::ACTUATION::SetModeCycTor()
{
	return sysBase->SetModeCycTor();
};*/

/*
int Aris::RT_CONTROL::ACTUATION::NRT_MCPowerOff()
{
	CSysBase::EInternCmd cmd = sysBase->EInternCmd::EIC_POWEROFF;

	Aris::Core::MSG data;
	data.SetType(sysBase->EInternDataType::EIMT_CMD);
	//memcpy(data._pData+MSG_HEADER_LENGTH,&cmd,sizeof(cmd));
	//printf("DataLength PO:%d\n",data.GetLength());
	data.Copy(&cmd,sizeof(cmd));
	return NRT_SendCommand(data);

	//return sysBase->NRT_MCPowerOff();
};

int Aris::RT_CONTROL::ACTUATION::NRT_MCHome()
{
	CSysBase::EInternCmd cmd = sysBase->EInternCmd::EIC_HOME;

	Aris::Core::MSG data;
	data.SetType(sysBase->EInternDataType::EIMT_CMD);
	//memcpy(data._pData+MSG_HEADER_LENGTH,&cmd,sizeof(cmd));
	data.Copy(&cmd,sizeof(cmd));
	return NRT_SendCommand(data);

	//return sysBase->NRT_MCHome();
};
int Aris::RT_CONTROL::ACTUATION::NRT_MCHomeToRunning()
{
	CSysBase::EInternCmd cmd = sysBase->EInternCmd::EIC_HOME2RUN;

	Aris::Core::MSG data;
	data.SetType(sysBase->EInternDataType::EIMT_CMD);
	//memcpy(data._pData+MSG_HEADER_LENGTH,&cmd,sizeof(cmd));
	data.Copy(&cmd,sizeof(cmd));
	return NRT_SendCommand(data);
	//return sysBase->NRT_MCHomeToRunning();
};
int Aris::RT_CONTROL::ACTUATION::NRT_MCEnable()
{
	CSysBase::EInternCmd cmd = sysBase->EInternCmd::EIC_ENABLE;

	Aris::Core::MSG data;
	data.SetType(sysBase->EInternDataType::EIMT_CMD);

	//memcpy(data.GetDataAddress(),&cmd,sizeof(cmd));
	data.Copy(&cmd,sizeof(cmd));
	return NRT_SendCommand(data);
	//return sysBase->NRT_MCEnable();
};
int Aris::RT_CONTROL::ACTUATION::NRT_MCStop()
{
	CSysBase::EInternCmd cmd = sysBase->EInternCmd::EIC_STOP;

	Aris::Core::MSG data;
	data.SetType(sysBase->EInternDataType::EIMT_CMD);
	//memcpy(data._pData+MSG_HEADER_LENGTH,&cmd,sizeof(cmd));
	data.Copy(&cmd,sizeof(cmd));

	return NRT_SendCommand(data);

	//return sysBase->NRT_MCStop();
};


*/



/*Aris::RT_CONTROL::EServoState Aris::RT_CONTROL::ACTUATION::NRT_MCMachineState()
{
	return sysBase->NRT_MCMachineState();
};*/

/*
bool Aris::RT_CONTROL::ACTUATION::RT_IsCusMsg()
{
	return sysBase->RT_IsCusMsg();
};*/

//bool Aris::Control::CONTROL_SYSTEM::NRT_IsCusMsg()
//{
//	return sysBase->NRT_IsCusMsg();
//};
//
//bool Aris::Control::CONTROL_SYSTEM::NRT_CheckMessage()
//{
//	return sysBase->NRT_CheckMessage();
//};
//bool Aris::Control::CONTROL_SYSTEM::NRT_IsSysMsg()
//{
//	return sysBase->NRT_IsSysMsg();
//};
//Aris::Control::EServoState Aris::Control::CONTROL_SYSTEM::NRT_GetSysStateFromRT()
//{
//	return sysBase->NRT_GetSysStateFromRT();
//};

/*
 *  TO BE DONE
 *  This function will related to Aris_Message,
 *  It will send some message to message loop
 *  So we first define some message here
 */
//int Aris::Control::CONTROL_SYSTEM::NRT_SysStateChangeHandler()
//{
//	if(NRT_CheckMessage())
//	{
//		if(NRT_IsSysMsg())
//		{
//			switch(NRT_GetSysStateFromRT())
//			{
//			case Aris::Control::EServoState::EMSTAT_NONE:
//				printf("NRT_Watcher:This is really weird!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_POWEROFF:
//				printf("NRT_Watcher:On state POWEROFF!\n");
//				if(sysBase->OnPOWEROFF!=NULL)
//					sysBase->OnPOWEROFF(NULL);
//				else
//					printf("NRT_Watcher:To State POWEROFF not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_POED:
//				printf("NRT_Watcher:On state POED!\n");
//				if(sysBase->OnPOED!=NULL)
//					sysBase->OnPOED(NULL);
//				else
//					printf("NRT_Watcher:To State POED not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_STOP:
//				printf("NRT_Watcher:On state STOP!\n");
//				if(sysBase->OnSTOP!=NULL)
//					sysBase->OnSTOP(NULL);
//				else
//					printf("NRT_Watcher:To State STOP not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_STOPPED:
//				printf("NRT_Watcher:On state STOPPED!\n");
//				if(sysBase->OnSTOPPED!=NULL)
//					sysBase->OnSTOPPED(NULL);
//				else
//					printf("NRT_Watcher:To State STOPPED not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_ENABLE:
//				printf("NRT_Watcher:On state ENABLE!\n");
//				if(sysBase->OnENABLE!=NULL)
//					sysBase->OnENABLE(NULL);
//				else
//					printf("NRT_Watcher:To State ENABLE not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_ENABLED:
//				printf("NRT_Watcher:On state ENABLED!\n");
//				if(sysBase->OnENABLED!=NULL)
//					sysBase->OnENABLED(NULL);
//				else
//					printf("NRT_Watcher:To State ENABLED not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_HOMING:
//				printf("NRT_Watcher:On state HOMING!\n");
//				if(sysBase->OnHOMING!=NULL)
//					sysBase->OnHOMING(NULL);
//				else
//					printf("NRT_Watcher:To State HOMING not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_HOMED:
//				printf("NRT_Watcher:On state HOMED!\n");
//				if(sysBase->OnHOMED!=NULL)
//					sysBase->OnHOMED(NULL);
//				else
//					printf("NRT_Watcher:To State HOMED not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_H2RING:
//				printf("NRT_Watcher:On state H2RING!\n");
//				if(sysBase->OnH2RING!=NULL)
//					sysBase->OnH2RING(NULL);
//				else
//					printf("NRT_Watcher:To State H2RING not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_RUNNING:
//				printf("NRT_Watcher:On state RUNNING!\n");
//				if(sysBase->OnRUNNING!=NULL)
//					sysBase->OnRUNNING(NULL);
//				else
//					printf("NRT_Watcher:To State RUNNING not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_STSTILL:
//				printf("NRT_Watcher:On state STSTILL!\n");
//				if(sysBase->OnSTSTILL!=NULL)
//					sysBase->OnSTSTILL(NULL);
//				else
//					printf("NRT_Watcher:To State STSTILL not handled!\n");
//				break;
//			case Aris::Control::EServoState::EMSTAT_EMERGE:
//				printf("NRT_Watcher:On state EMERGE!\n");
//				if(sysBase->OnEMERGENCY!=NULL)
//					sysBase->OnEMERGENCY(NULL);
//				else
//					printf("NRT_Watcher:To State EMERGE not handled!\n");
//				break;
//			default:
//				printf("NRT_Watcher:switch default!\n");
//				break;
//			}
//			//sysBase->InternDataHandler();
//			//CSysBase::InternDataHandler();
//		}
//		else if (NRT_IsCusMsg())
//		{
//			sysBase->OnCustomMsg(m_recvData);
//		}
//	}
//
//	return 0;
//};


int Aris::RT_CONTROL::ACTUATION::SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater)
{
	return sysBase->SetOnDataUpdateHandler(p_DataUpdater);
};

/*
int Aris::RT_CONTROL::ACTUATION::SetOnPowerOff(FuncPtrState p_Handler)
{
	return sysBase->SetOnPOWEROFF(p_Handler);
};
int Aris::RT_CONTROL::ACTUATION::SetOnPowerOffed(FuncPtrState p_Handler)
{
	return sysBase->SetOnPOED(p_Handler);
};

int Aris::RT_CONTROL::ACTUATION::SetOnStop(FuncPtrState p_Handler)
{
	return sysBase->SetOnSTOP(p_Handler);
};
int Aris::RT_CONTROL::ACTUATION::SetOnStopped(FuncPtrState p_Handler)
{
	return sysBase->SetOnSTOPPED(p_Handler);
};

int Aris::RT_CONTROL::ACTUATION::SetOnEnable(FuncPtrState p_Handler)
{
	return sysBase->SetOnENABLE(p_Handler);
};
int Aris::RT_CONTROL::ACTUATION::SetOnEnabled(FuncPtrState p_Handler)
{
	return sysBase->SetOnENABLED(p_Handler);
};

int Aris::RT_CONTROL::ACTUATION::SetOnHoming(FuncPtrState p_Handler)
{
	return sysBase->SetOnHOMING(p_Handler);
};
int Aris::RT_CONTROL::ACTUATION::SetOnHomed(FuncPtrState p_Handler)
{
	return sysBase->SetOnHOMED(p_Handler);
};

int Aris::RT_CONTROL::ACTUATION::SetOnHomedToRunning(FuncPtrState p_Handler)
{
	return sysBase->SetOnH2RING(p_Handler);
};
int Aris::RT_CONTROL::ACTUATION::SetOnRunning(FuncPtrState p_Handler)
{
	return sysBase->SetOnRUNNING(p_Handler);
};

int Aris::RT_CONTROL::ACTUATION::SetOnStandStill(FuncPtrState p_Handler)
{
	return sysBase->SetOnSTSTILL(p_Handler);
};
int Aris::RT_CONTROL::ACTUATION::SetOnEmergency(FuncPtrState p_Handler)
{
	return sysBase->SetOnEMERGE(p_Handler);
};

int Aris::RT_CONTROL::ACTUATION::SetOnCustomMessage(FuncPtrCustom p_Handler)
{
	return sysBase->SetOnCustomMessage(p_Handler);
};
*/
	//generate a CONN_DATA,then send buffer


/*

int Aris::RT_CONTROL::ACTUATION::RT_SendData(Aris::RT_CONTROL::RT_MSG &p_data)
{
	int ret;
	ret=sysBase->RT_SendDataRaw(p_data.m_ptrData,RT_MSG_HEADER_LENGTH);
	if(ret < 0)
		return ret;
	if(p_data.GetLength()>0)
	{
		ret=sysBase->RT_SendDataRaw(p_data.GetDataAddress(),p_data.GetLength());
	}

	return ret;
};

int Aris::RT_CONTROL::ACTUATION::RT_ReadData(void* p_ptrData,const int p_dataLength) //DONE
{
	//copy data from buffer
	sysBase->RT_ReadData(p_ptrData,p_dataLength);
	return 0;
};

int Aris::RT_CONTROL::ACTUATION::NRT_SendData(Aris::Core::MSG &p_data)//DONE
{
	//Aris::RT_CONTROL::RT_MSG msg(p_data);
	int ret;
	//printf("sizeof %d\n",sizeof(p_data));
	p_data.SetType((int)sysBase->EInternDataType::EIMT_CUS);
	ret=sysBase->NRT_SendDataRaw(p_data._pData,p_data.GetLength()+MSG_HEADER_LENGTH);
	printf("NRT_SendData is me :%d\n",ret);
	return 0;
};

int Aris::RT_CONTROL::ACTUATION::NRT_SendCommand(Aris::Core::MSG &p_data)
{
	int ret;
	//printf("sizeof %d\n",sizeof(p_data));
	p_data.SetType((int)sysBase->EInternDataType::EIMT_CMD);
	ret=sysBase->NRT_SendDataRaw(p_data._pData,p_data.GetLength()+MSG_HEADER_LENGTH);
	printf("NRT_SendCommand:%d\n",ret);
	return 0;
};

// NRT_RecvData will not be used by user
int Aris::RT_CONTROL::ACTUATION::NRT_RecvData(Aris::Core::MSG &p_data)// TBD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
{
	char Head[MSG_HEADER_LENGTH];
	sysBase->NRT_RecvDataRaw(Head,MSG_HEADER_LENGTH);
	p_data.SetLength((unsigned int)Head[0]);
	p_data.SetType((long long)Head[8]);
	sysBase->NRT_RecvDataRaw(p_data.GetDataAddress(),p_data.GetLength());
	return 0;
};
*/


/*int Aris::RT_CONTROL::ACTUATION::NRT_PostCommandMsg(Aris::Core::MSG &p_data)
{
	int ret;
  	ret=sysBase->NRT_SendDataRaw(p_data._pData,p_data.GetLength()+MSG_HEADER_LENGTH);
 	return 0;
}//ok*/

int Aris::RT_CONTROL::ACTUATION::NRT_PostMsg(Aris::Core::MSG &p_data)
{
	int ret;
	//printf("sizeof %d\n",sizeof(p_data));
	//p_data.SetMsgID((int)Aris::RT_CONTROL::CM_CUS_MESSAGE);
	ret=sysBase->NRT_SendDataRaw(p_data._pData,p_data.GetLength()+MSG_HEADER_LENGTH);
	//printf("NRT_SendData is me :%d\n",ret);
	return 0;
}//ok

/*int Aris::RT_CONTROL::ACTUATION::NRT_GetStateMsg(Aris::Core::MSG p_msg)
{

    printf("nrt_getstatemsg start\n");
	char Head[MSG_HEADER_LENGTH];
	sysBase->NRT_RecvDataRaw(Head,MSG_HEADER_LENGTH);

	p_msg.SetLength((unsigned int)Head[0]);
	printf("length get in function: %d\n",Head[0]);
	p_msg.SetMsgID((long long int)Head[4]);
	printf("ID get in function: %d\n",Head[4]);

 	return sysBase->NRT_RecvDataRaw(p_msg.GetDataAddress(),p_msg.GetLength());
}*/
 void Aris::RT_CONTROL::ACTUATION::GetMachineState(Aris::RT_CONTROL::EMachineState p_state )
{
	p_state=sysBase->m_machineDataCore.machinestate;

}



/*Aris::RT_CONTROL::RT_MSG Aris::RT_CONTROL::ACTUATION::RT_GetCommandMsg()
{
	return sysBase->m_rtDataRecv;
}*/


//not used by users
/*
int Aris::RT_CONTROL::ACTUATION::RT_PostStateMsg()
{
	return sysBase->RT_PostStateMsg();

}
*/
int Aris::RT_CONTROL::ACTUATION::RT_PostMsg(Aris::RT_CONTROL::RT_MSG &p_data)
{
	return sysBase->RT_PostMsg(p_data);

}

/*
void* Aris::RT_CONTROL::ACTUATION::NRT_Watcher(void*)
{
	printf("NRT_Watcher\n");
	while(!Aris::RT_CONTROL::ACTUATION::IsSysStopped())
	{
		printf("NRT_Watcher is watching\n");
		//read it
		Aris::RT_CONTROL::ACTUATION::NRT_RecvData(Aris::RT_CONTROL::ACTUATION::m_recvData);
		printf("DataLength %d\n",((int*)m_recvData.GetDataAddress())[0]);

		switch(m_recvData.GetType())
		{
		case (int)sysBase->EInternDataType::EMIT_STA:
				printf("NRT_Watcher saw a state change.\n");
				//read state change and call functions**********************************************TBD
				memcpy(&sysBase->m_servoStateNRT,m_recvData.GetDataAddress(),m_recvData.GetLength());
				switch(sysBase->m_servoStateNRT)
				{
				case Aris::RT_CONTROL::EServoState::EMSTAT_NONE:
					printf("NRT_Watcher:This is really weird!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF:
					printf("NRT_Watcher:On state POWEROFF!\n");
					if(sysBase->OnPOWEROFF!=NULL)
						sysBase->OnPOWEROFF(NULL);
					else
						printf("NRT_Watcher:To State POWEROFF not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_POED:
					printf("NRT_Watcher:On state POED!\n");
					if(sysBase->OnPOED!=NULL)
						sysBase->OnPOED(NULL);
					else
						printf("NRT_Watcher:To State POED not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_STOP:
					printf("NRT_Watcher:On state STOP!\n");
					if(sysBase->OnSTOP!=NULL)
						sysBase->OnSTOP(NULL);
					else
						printf("NRT_Watcher:To State STOP not handled!\n");
						break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED:
					printf("NRT_Watcher:On state STOPPED!\n");
					if(sysBase->OnSTOPPED!=NULL)
						sysBase->OnSTOPPED(NULL);
					else
						printf("NRT_Watcher:To State STOPPED not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE:
					printf("NRT_Watcher:On state ENABLE!\n");
					if(sysBase->OnENABLE!=NULL)
						sysBase->OnENABLE(NULL);
					else
						printf("NRT_Watcher:To State ENABLE not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED:
					printf("NRT_Watcher:On state ENABLED!\n");
					if(sysBase->OnENABLED!=NULL)
						sysBase->OnENABLED(NULL);
					else
						printf("NRT_Watcher:To State ENABLED not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_HOMING:
					printf("NRT_Watcher:On state HOMING!\n");
					if(sysBase->OnHOMING!=NULL)
						sysBase->OnHOMING(NULL);
					else
						printf("NRT_Watcher:To State HOMING not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_HOMED:
					printf("NRT_Watcher:On state HOMED!\n");
					if(sysBase->OnHOMED!=NULL)
						sysBase->OnHOMED(NULL);
					else
						printf("NRT_Watcher:To State HOMED not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_H2RING:
					printf("NRT_Watcher:On state H2RING!\n");
					if(sysBase->OnH2RING!=NULL)
						sysBase->OnH2RING(NULL);
					else
						printf("NRT_Watcher:To State H2RING not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING:
					printf("NRT_Watcher:On state RUNNING!\n");
					if(sysBase->OnRUNNING!=NULL)
						sysBase->OnRUNNING(NULL);
					else
						printf("NRT_Watcher:To State RUNNING not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL:
					printf("NRT_Watcher:On state STSTILL!\n");
					if(sysBase->OnSTSTILL!=NULL)
						sysBase->OnSTSTILL(NULL);
					else
						printf("NRT_Watcher:To State STSTILL not handled!\n");
					break;
				case Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE:
					printf("NRT_Watcher:On state EMERGE!\n");
					if(sysBase->OnEMERGENCY!=NULL)
						sysBase->OnEMERGENCY(NULL);
					else
						printf("NRT_Watcher:To State EMERGE not handled!\n");
					break;
				default:
					printf("NRT_Watcher:switch default!\n");
					break;

				}
				break;
		case (int)sysBase->EInternDataType::EIMT_CUS:
				printf("NRT_Watcher saw a custom data.\n");
				//call OnCustomDataReceived function************************************************TBD
				//this function need a CONN_DATA parameter
				sysBase->OnCustomMsg(m_recvData);
				break;

		default:
			break;

		}

	}
};
*/

int Aris::RT_CONTROL::ACTUATION::Load_XML_PrintMessage()
{
	return sysBase->Load_XML_PrintMessages();
}
























