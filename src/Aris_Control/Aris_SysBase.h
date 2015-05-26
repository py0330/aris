/*
 * Aris_SysBase.h
 *
 *  Created on: Apr 15, 2015
 *      Author: hex
 */

#ifndef ARIS_SYSBASE_H_
#define ARIS_SYSBASE_H_


#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <native/queue.h>
#include <native/heap.h>
#include <rtdk.h>
#include <ecrt.h>
#include <rtdm/rtipc.h>
#include <pthread.h>

#include <tinyxml2.h>

#include "Aris_Device.h"

#include "Aris_Thread.h"
#include "Aris_Socket.h"


using namespace ElmoMotor;
/*
 *  There two xddp_ports
 *  XDDP_PORT is for some NRT call to communicate with RT
 *  XDDP_PORT_DATA is for dataServer and RT only
 */

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */
#define XDDP_PORT_DATA 1	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

/*
 * Used for declaration of state machine matrix
 */
#define STATE_NUMBER 14
#define CMD_NUMBER 10




/*****************************************************************************
 * Class CSysBase
 *****************************************************************************/

namespace Aris{ namespace RT_CONTROL{class ACTUATION;}}

class CSysBase{
public:

	CSysBase();
	~CSysBase();

private:
		friend class Aris::RT_CONTROL::ACTUATION;

public:
/*enum EInternDataType
{
	EIMT_NONE=-1, //when none msg received
	EIMT_CMD = 0, //internal command is coming, processed by sysbase
	EIMT_CUS = 1, //custom msg is coming, processed by custom function
	//EMIT_STA = 2, // corresponding EMIT_CMD in NRT side
};*/


// function pointers
static FuncPtrWork  trajectoryGenerator;
static FuncPtrInit  initHandler;
static  FuncPtrWork  dataUpdater;
//static FuncPtrState CSysBase::OnPOWEROFF=NULL;
//static FuncPtrState CSysBase::OnPOED=NULL;
//static FuncPtrState CSysBase::OnSTOP=NULL;
//static FuncPtrState CSysBase::OnSTOPPED=NULL;
//static FuncPtrState CSysBase::OnENABLE=NULL;
//static FuncPtrState CSysBase::OnENABLED=NULL;
//static FuncPtrState CSysBase::OnHOMING=NULL;
//static FuncPtrState CSysBase::OnHOMED=NULL;
//static FuncPtrState CSysBase::OnH2RING=NULL;
//static FuncPtrState CSysBase::OnRUNNING=NULL;
//static FuncPtrState CSysBase::OnSTSTILL=NULL;
//static FuncPtrState CSysBase::OnEMERGENCY=NULL;
//static FuncPtrCustom CSysBase::OnCustomMsg=NULL;

//rt-task members*************************************************************//

static Aris::RT_CONTROL::CSysInitParameters  m_sysInitParam;
static CDeviceMaster  m_deviceMaster;
static RT_TASK  m_realtimeCore;
static RTIME  m_timeStart;

static long long int  m_cycleCount;
//static long long int  m_h2rStartTime;
//static long long int  m_h2rStartTimeRUN;

//machine states and messges**************************************************//

//static Aris::RT_CONTROL::EServoCommand  m_motorCommands[ACTUAL_MOTOR_NUMBER];
//static Aris::RT_CONTROL::EServoState  m_motorStates[ACTUAL_MOTOR_NUMBER];
//static Aris::RT_CONTROL::EOperationMode  m_motorModes[ACTUAL_MOTOR_NUMBER];

//static Aris::RT_CONTROL::EServoCommand  m_machineCommand;
//static Aris::RT_CONTROL::EServoState  m_machineState;
//static Aris::RT_CONTROL::EOperationMode  m_machineMode;
//static Aris::RT_CONTROL::EServoCommand  m_rawInternCommand;
  //static bool m_isInTrajPhase;

//static  Aris::RT_CONTROL::EMachineState  m_machineState;

//static Aris::RT_CONTROL::EServoState  m_machineStateRT;
//static Aris::RT_CONTROL::EServoState  m_currentStateRT;
//static Aris::RT_CONTROL::EServoState  m_nextStateRT;
//static Aris::RT_CONTROL::EServoState  m_machineservoStateNRT;

//static CSysBase::EInternCmd  m_currentInternCmdRT;
//static CSysBase::EIntsernCmd  m_currentInternCmdNRT;

//static Aris::RT_CONTROL::EServoState CSysBase::m_stateTransitionMatrix[CMD_NUMBER][STATE_NUMBER];
//static bool  m_isH2Red[ACTUAL_MOTOR_NUMBER];
//static int  m_homePosition[ACTUAL_MOTOR_NUMBER];
//static int  m_h2rStartPosition[ACTUAL_MOTOR_NUMBER];
//static int  m_h2rDeltaPosition=50;

//static bool  m_isCurrentCycleGetData;
//static bool  m_isGetCustomDataCurrentCycle;

//static bool CSysBase::m_isAllComplete;


static Aris::RT_CONTROL::RT_MSG  m_rtDataRecv;
static Aris::RT_CONTROL::RT_MSG  m_rtDataSend;

static char  m_rtDataRecvBuffer[RT_MSG_BUFFER_SIZE];
static char  m_rtDataSendBuffer[RT_MSG_BUFFER_SIZE];

//static bool  m_isDataFromRT;

static CDeviceData  m_standStillFrame;

static CSysPID  m_pid;


//data logging members*******************************************************//
class CLogData
{
public:
    CDeviceData m_feedbackData;
    CDeviceData m_commandData;
    //Aris::RT_CONTROL::EServoState m_servoState;
    Aris::RT_CONTROL::EMachineState m_machineState;
    long long int time;
};
public:
//*****title elements for printing



static char print_SYSTEM_INITIALIZING [PRINT_INFO_BUFFER_SIZE];
static char print_RT_Message_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_Ethercat_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_Log_File_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_Log_Heap_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_RT_Task_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_SYSTEM_INITIALIZED[PRINT_INFO_BUFFER_SIZE];

static char print_SYSTEM_COMMUNICATION_INITIALIZING[PRINT_INFO_BUFFER_SIZE];
static char print_XDDP_RT_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_XDDP_Data_RT_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_XDDP_NRT_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_XDDP_Data_NRT_Initialized[PRINT_INFO_BUFFER_SIZE];
static char print_SYSTEM_COMMUNICATION_INITIALIZED[PRINT_INFO_BUFFER_SIZE];


static char print_TASK_START[PRINT_INFO_BUFFER_SIZE];
static char print_TASK_STOP[PRINT_INFO_BUFFER_SIZE];


static RT_HEAP  m_logHeap;

static  CLogData  m_logDataRC;
static Aris::RT_CONTROL::CMachineData  m_machineDataCore;
//static Aris::RT_CONTROL::CMachineData  m_machineData;
static bool m_isLog;
static long long int  m_logCount;
static void* m_logDataVoid;
static  CLogData*  m_logData;

static pthread_t  m_dataServer;
static pthread_attr_t  m_dataServer_attr;

static pthread_mutex_t m_dataMutex ;

static  CLogData  m_logDataBuffer;

static struct sigaction  m_sa;

static FILE * fp;
static Aris::RT_CONTROL::CMachineData m_logMachineDataBuffer;

//communication members************************************************************//
static int  m_xddp_socket_rt;
static char m_xddp_buf_rt[4096];

static int m_xddp_socket_data_rt;
static char  m_xddp_buf_data_rt[4096];

static char m_xddp_buf[4096];
static char*  m_xddp_devname;
static int  m_xddp_fd;

static char  m_xddp_buf_data[4096];
static char * m_xddp_devname_data;
static int  m_xddp_fd_data;


static int SysInit(Aris::RT_CONTROL::CSysInitParameters p_Param);

static int SysInitCommunicationRT();

static int SysInitCommunication();


static void CatchWarnUponSwitch(int signalNum, siginfo_t *signalInfo, void *context);

static void CatchStopSignal(int signalNum);
static void fail(const char*reason);


static bool IsSysStopped();

static int SysStart();

static int SysStop();

static int SetSysInitializer(FuncPtrInit p_Func);

static int SetTrajectoryGenerator(FuncPtrWork p_TrajectoryGenerator);

static int SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater);


static void PushDatatoMotors( );
static void UploadDatafromMotors();

//static int SetModeP2P();
//static int SetModeCycPos();
//tatic int SetModeCycVel();
//static int SetModeCycTor();

//static Aris::RT_CONTROL::EServoState  NRT_MCMachineState();
static void RealtimeCore(void* arg);
static Aris::RT_CONTROL::EMachineState GetMachineState();
//static int PostStateToNRT(const Aris::RT_CONTROL::EServoState p_state);
///////////////////////////////////msg/////////////
private:
static int InitRT_MSG();
static int RT_SendDataRaw(const void* p_ptrData, const int p_dataLength);
static int RT_RecvDataRaw(void* p_ptrData,const int p_dataLength);
static int NRT_SendDataRaw(const void* p_ptrData, const int p_dataLength);
static int NRT_RecvDataRaw(void* p_ptrData,const unsigned int p_dataLength);
static int RT_PostStateMsg();
static int RT_PostMsg(Aris::RT_CONTROL::RT_MSG &p_data);



//int NRT_UpdateData(); implement in Aris_Control is preferred, because there will include Aris_Socket.h, not here

/*
 * update m_rtDataRecv object,
 * actually just refresh m_rtDataRecvBuffer is all the work here
 */
//static int RT_UpdateData();
////////////////////////////////xml//////////////////
static void PrintInfo(const char* ch);
static int  Load_XML_PrintMessages();
/////////////////log/////////////////////////////////
static int RawLogDataToMachineData(Aris::RT_CONTROL::CMachineData& p_machineData,CLogData p_logData);
static int InitLogFile();
static void *dataServer(void* arg);

};
#endif /* ARIS_SYSBASE_H_ */
