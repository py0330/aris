/*
 * SysBase.h
 *
 *  Created on: Nov 10, 2014
 *      Author: leo
 */

#ifndef SYSBASE_H_
#define SYSBASE_H_

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

#include "GlobalConfiguration.h"
#include "hardware.h"
#include "SysPID.h"

#include "Aris_ControlData.h"
#include "Aris_Thread.h"
#include "Aris_Socket.h"
#include "Aris_Core.h"


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
 * Class CDeviceData
 *****************************************************************************/
class CDeviceData
{
public:
    CElmoMotorData     m_motorData[ACTUAL_MOTOR_NUMBER];
    CAnalogInputsData  m_analogInputsData[ACTUAL_EL3104_NUM];
    CAnalogOutputsData m_analogOutputsData[ACTUAL_EL4002_NUM];
};


/*****************************************************************************
 * Class CDeviceMaster
 *****************************************************************************/
// Operates all devices, including the Ethercat master, the motor drivers, analog/digital IO devices and the IMU
class CDeviceMaster
{
public:
    CDeviceMaster(void);
    ~CDeviceMaster(void);


    int Initialize(const Aris::RT_CONTROL::CSysInitParameters p_InitParam);

    int Read();   //read each device's data
    int Write();  //write each device's data
    int DCSyncTime(uint64_t nanoSecond);

    //check whether all motors have complete their commands, return the number of the motors that haven't finished
    int CheckComplete();
    int DeactiveMaster();

    // Data exchange area
    CDeviceData m_feedbackData;
    CDeviceData m_commandData;

private:
    int m_motorNum;
    int m_analogInputsTerminalNum;
    int m_analogOutputsTerminalNum;

    /*
     * Added for testing convenience
     */

    int m_startMotorID;
    int m_endMotorID;

    static ec_master_t* m_pMaster;
    static ec_master_state_t m_masterState;//={};

    bool m_isMasterActivated;
    bool m_isMasterRequested;

    // Actuations
    CElmoMotor m_motor[ACTUAL_MOTOR_NUMBER];
    int m_HomeOffset[ACTUAL_MOTOR_NUMBER];
    SMotorGeneralSettings m_motorGeneralSettings;

    // Ethercat coupler
    CEthercatCouplerEK1100 m_ethercatCoupler;
    // Analog Inputs
    CAnalogInputsEL3104 m_analogInputs[ACTUAL_EL3104_NUM];
    // Analog Outputs
    CAnalogOutputsEL4002 m_analogOutputs[ACTUAL_EL4002_NUM];

};




/*****************************************************************************
 * Class CSysBase
 *****************************************************************************/

namespace Aris{ namespace RT_CONTROL{class ACTUATION;}}


class CSysBase
{
public:

	CSysBase();
	~CSysBase();

private:
		friend class Aris::RT_CONTROL::ACTUATION;

public:
	/**
	 * \breif 用户自定义的初始化函数
	 *
	 * 该函数的函数指针是给定了CSysInitParameters类型参数的函数指针，这里没有打算替代SysInit的功能，
	 * 只是可能该函数要参考系统的初始化参数，因此将该函数和系统的初始化函数设置了同样的参数，但是该函数并
	 * 不会调用系统的初始化参数设置函数SysInit
	 */
	static int SetSysInitializer(FuncPtrInit p_Func);
	/**
	 * \brief 初始化控制系统。
	 *
	 * 为系统启动做好准备
	 */
	static int SysInit(Aris::RT_CONTROL::CSysInitParameters p_Param);
	/**
	 * \brief 启动控制系统。
	 *
	 * 启动后为正常运行状态，如无意外，应一直正常运行至机器停止。
	 */
	static int SysStart();
	/**
	 * \brief 停止控制系统。
	 *
	 * 将结束控制系统的所有任务，程序退出。
	 */
	static int SysStop();

	/**
	 * \brief 设置轨迹生成函数
	 *
	 *@param p_TrajectoryGenerator是具有指定参数类型的函数指针，在该函数中，使用者需要调用一系列函数
	 */
	static int SetTrajectoryGenerator(FuncPtrWork p_TrajectoryGenerator);
	/**
	 * \brief 设置日志函数
	 *
	 * @param p_Logger 是记录数据的函数，它工作在实时系统里面,
	 * 可以记录TrajectoryGenerator里面发生的事情。
	 */
	static int SetLogger(FuncPtrWork p_Logger);

	/**
	 * \brief 设置数据更新时执行的函数
	 *
	 * 非实时部分执行的函数
	 */
	static int SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater);

	/**
	 * \brief 初始化非实时部分通讯
	 *
	 * 这个函数的调用需要发生在实时部分通讯初始化之后，调用顺序错误将导致该函数的初始化工作失败。
	 */
	static int SysInitCommunication();

	static int SysInitCommunicationRT();


	/**
	 * \brief 系统是否停止
	 *
	 * 返回表示系统是否停止的布尔值
	 * 该函数可以在RT以及NRT下调用
	 */
	static bool IsSysStopped();
	/*
	 * Set mode before initialize
	 * t is said that drive mode could be changed in RUNNING state
	 * need more testing
	 */
	static int SetModeP2P();
	static int SetModeCycPos();
	static int SetModeCycVel();
	static int SetModeCycTor();

	/*************************************************************************
	 *以NRT或者RT开头的函数是非初始化或者终止系统的函数，即这些函数可以在系统正常运行时使用，
	 *并且会和状态机关联。
	 *************************************************************************/
	static int NRT_GetMessageRaw(void* p_MsgBuffer,const unsigned int p_MsgBufSize);
	static int NRT_PostMessageRaw(const void* p_MsgPointer, const int p_MsgLength);
	static int RT_PostMessageRaw(const void* p_MsgPointer,const int p_MsgLength);
	static int RT_GetMessageRaw(void* p_MsgBuffer,const int p_MsgBufSize);
	/*
	 * BROKEN API, need fix
	 * Two switch get messages
	 * 1st: if flag is set,then do not get type again. after get content, clear flag, so after that can be used
	 * 2nd: if flag is not set, get type, and get content, then not clear flag
	 */
	/*
	 * Correctly use:
	 * RT_IsCusMsg()
	 * RT_GetMessageRaw()
	 *
	 * RT_PrePostCustomMessage();
	 * RT_PostMessageRaw();
	 *
	 * NRT_IsCusMsg()
	 * NRT_GetMessageRaw()
	 *
	 * NRT_PrePostCustomMessage()
	 * NRT_PostMessageRaw()
	 */
	// following two function is not finished yet
	static int NRT_GetMessage(void* p_MsgBuffer,const unsigned int p_MsgBufSize);
	static int RT_GetMessage(void* p_MsgBuffer,const int p_MsgBufSize);

	/*
	 * Send a CUS type before do send content
	 * could directly used in skin Aris_Control
	 */
	static int NRT_PrePostCustomMessage();
	static int RT_PrePostCustomMessage();

	static int NRT_MCPowerOff();
	static int NRT_MCHome();
	static int NRT_MCHomeToRunning();
	static int NRT_MCEnable();
	static int NRT_MCStop();
	static int NRT_MCData(Aris::RT_CONTROL::CMachineData& p_machineData);
	static Aris::RT_CONTROL::EServoState  NRT_MCMachineState();

	/*
	 * not ready at this version
	 */
	static int RT_MCAxisFeedPos(int p_AxisId, int p_Pos);
	static int RT_MCAxisFeedVel(int p_AxisId, int p_Vel);
	static int RT_MCAxisFeedTor(int p_AxisId, short p_Tor);

	/*
	 * 辅助API，在实现Aris_Control时使用
	 *
	 */

	static bool NRT_CheckMessage();
	static bool NRT_IsSysMsg();// switch and send a msg
	static Aris::RT_CONTROL::EServoState NRT_GetSysStateFromRT();
	static bool NRT_IsCusMsg();// send a msg


	static bool RT_IsCusMsg();

	//	static bool RT_CheckMessage();
	//	static bool RT_IsSysMsg();
	/*
	 * GetStateNRT
	 * Used in the beginning of
	 * could be omitted
	 */

	/*
	 * Not good as NRT_GetSysStateFromRT
	 */
	//static Aris::EServoState NRT_GetCurrentState();

	static void RT_DoPID();



	static int SetOnPOWEROFF(FuncPtrState p_Handler);
	static int SetOnPOED(FuncPtrState p_Handler);

	static int SetOnSTOP(FuncPtrState p_Handler);
	static int SetOnSTOPPED(FuncPtrState p_Handler);

	static int SetOnENABLE(FuncPtrState p_Handler);
	static int SetOnENABLED(FuncPtrState p_Handler);

	static int SetOnHOMING(FuncPtrState p_Handler);
	static int SetOnHOMED(FuncPtrState p_Handler);

	static int SetOnH2RING(FuncPtrState p_Handler);
	static int SetOnRUNNING(FuncPtrState p_Handler);

	static int SetOnSTSTILL(FuncPtrState p_Handler);
	static int SetOnEMERGE(FuncPtrState p_Handler);

	static int SetOnCustomMessage(FuncPtrCustom p_Handler);

	static FuncPtrWork dataUpdater;


	static FuncPtrState OnPOWEROFF;
	static FuncPtrState OnPOED;
	static FuncPtrState OnSTOP;
	static FuncPtrState OnSTOPPED;
	static FuncPtrState OnENABLE;
	static FuncPtrState OnENABLED;
	static FuncPtrState OnHOMING;
	static FuncPtrState OnHOMED;
	static FuncPtrState OnH2RING;
	static FuncPtrState OnRUNNING;
	static FuncPtrState OnSTSTILL;
	static FuncPtrState OnEMERGENCY;

	static FuncPtrCustom OnCustomMsg;


	/*
	 * Set buffer and correct address of the type and length
	 */
	static int InitRT_MSG();

	/*
	 * pack in m_rtDataSend and send
	 */
	static int RT_SendDataRaw(const void* p_ptrData, const int p_dataLength);

	//static int RT_SendData(const void* p_ptrData, const int p_dataLength);
	/*
	 * just copy from buffer, receiving happens in RT_UpdateData()
	 */
	static int RT_RecvDataRaw(void* p_ptrData,const int p_dataLength);

	//only rt side need this function
	//
	static int RT_ReadData(void* p_ptrData,const int p_dataLength);

	//NRT doesn't need raw because they
	static int NRT_SendDataRaw(const void* p_ptrData, const int p_dataLength);
	static int NRT_RecvDataRaw(void* p_ptrData,const unsigned int p_dataLength);

	//int NRT_UpdateData(); implement in Aris_Control is preferred, because there will include Aris_Socket.h, not here

	/*
	 * update m_rtDataRecv object,
	 * actually just refresh m_rtDataRecvBuffer is all the work here
	 */
	int RT_UpdateData();
	static Aris::RT_CONTROL::RT_MSG m_rtDataRecv;
	static Aris::RT_CONTROL::RT_MSG m_rtDataSend;
private:

	static char m_rtDataRecvBuffer[RT_MSG_BUFFER_SIZE];
	static char m_rtDataSendBuffer[RT_MSG_BUFFER_SIZE];

private:
	/*************************************************************************
	 * Realtime Core Infrastructure
	 *************************************************************************/
	static RT_TASK m_realtimeCore;
	static RTIME m_timeStart;
	static long long int m_cycleCount;

	//static RT_TASK m_realtimeData;
	static void RealtimeData(void *arg);

	long long int m_rtPostTimeMark;


	//static CLogData m_logDataRD;
	static Aris::RT_CONTROL::CMachineData m_machineDataCore;

	static FuncPtrWork trajectoryGenerator;
	static FuncPtrInit initHandler;


	static int DeviceDataToMachineData(const CDeviceData p_dData,Aris::RT_CONTROL::CMachineData &p_mData);
	static int MachineDataToDeviceData(const Aris::RT_CONTROL::CMachineData p_mData,CDeviceData &p_dData);

	/*
	 * \brief 实时控制系统内核实现
	 *
	 * 使用控制系统内核时，主要有两种方式，第一种是使用消息，控制系统的提供了实时与非实时之间通讯的api，
	 * 一共由四个函数，实现了消息的收发。系统自带的MC大部分是通过这些消息实现的。
	 * 另外一种方法时实现自己的消息，上面说的自带的MC消息时系统实现的，但是自定义的消息处理是通过函数指针
	 * 指定的函数决定的。也就是说我们的系统需要建立两种类型的消息处理机制，一种是系统定义的消息，以及对应的处理函数
	 * 另一种时自定义消息，以及对应的消息处理函数。
	 * 每个循环处理的第一个消息将决定该循环接下来处理的消息时系统函数还是自定义函数
	 *
	 *
	 */
	static void RealtimeCore(void *arg);

	static void CatchStopSignal(int signalNum);

	static void fail(const char*reason);

	static Aris::RT_CONTROL::CSysInitParameters m_sysInitParam;



	static OPERATION_MODE m_operationMode;

	// Describe current state of RT-Core state
	enum ECoreState
	{
		CS_UNINIT 		= 0,
		CS_INITIALIZED 	= 1,
		CS_STARTED		= 2,
		CS_STOPPED		= 3,
	};

	static ECoreState m_coreStateRT;

	/*************************************************************************
	 * Data Exchange and Device Control
	 *************************************************************************/
	static CDeviceMaster m_deviceMaster;

	/*************************************************************************
	 * XDDP RT
	 *************************************************************************/

	//implemented in RealtimeCore function
	static int m_xddp_socket_rt;
	static char m_xddp_buf_rt[4096];  // this limit may forces us transfer big data in small pieces

	//implemented in RealtimeData function
	static int m_xddp_socket_data_rt;
	static char m_xddp_buf_data_rt[4096];
	/*************************************************************************
	 * XDDP NRT
	 *************************************************************************/
	static char m_xddp_buf[4096];
	static char *m_xddp_devname;
	static int m_xddp_fd;

	//inited the same place as above nrt
	static char m_xddp_buf_data[4096];
	static char *m_xddp_devname_data;
	static int m_xddp_fd_data;



	/*************************************************************************
	 * Internal Message
	 *************************************************************************/
	static bool m_isCurrentCycleGetData;
	enum EInternDataType
	{
		EIMT_NONE=-1, //when none msg received
		EIMT_CMD = 0, //internal command is coming, processed by sysbase
		EIMT_CUS = 1, //custom msg is coming, processed by custom function
		EMIT_STA = 2, // corresponding EMIT_CMD in NRT side
	};

	static EInternDataType InternDataHandler();//Detect what type of message received
	static EInternDataType m_currentDataTypeFromNRT;

	enum EInternCmd
	{
		EIC_NONE	   	= 0,
		EIC_POWEROFF   	= 1,//MC ok
		EIC_STOP	   	= 2,//MC ok
		EIC_ENABLE     	= 3,//MC ok a command sent under emergency
		EIC_RUNNING    	= 4,//normally not used, but in single motor test it's good
		EIC_HOME       	= 5,//MC ok
		EIC_HOME2RUN	= 6,//MC ok change from HOMED to RUNNING
		EIC_CUS		   	= 7,//change by MsgType this one will not be used in xddp communication
		EIC_SSTILL	  	= 8,
		EIC_EMERGE		= 9,

	};

	static bool m_isAllComplete;

	//static int InternCmdHandler(EInternCmd& p_Cmd);// called in RT-Core, run internal command

	static EInternCmd m_currentInternCmdNRT;

	static int InternCmdGet();

	/*
	 * NRT message handling
	 * Main problem is message caused by MCcmd, this is inconvient
	 */

	//Modified by NRT_CheckMessage
	static bool m_isDataFromRT;
	//Modified by NRT_CheckMessage, referred by NRT_IsSysMsg NRT_IsCusMsg
	static EInternDataType m_currentDataTypeFromRT;

	//Modified by NRT_GetSystemState
	static Aris::RT_CONTROL::EServoState m_servoStateNRT;

	/*
	 * RT custom msg flag;
	 */
	static bool m_isGetCustomDataCurrentCycle;

	/*
	 * System logger
	 * a default log that help us debug
	 */
	/*
	 * Only be used to pack all data from RC to RD
	 */

	class CLogData
	{
	public:
	    CDeviceData m_feedbackData;
	    CDeviceData m_commandData;
	    Aris::RT_CONTROL::EServoState m_servoState;
	    CSysBase::ECoreState m_coreState;
	    long long int time;
	};

	static CLogData m_logDataRC;

	static RT_HEAP m_logHeap;
	static bool m_isLog;
	static long long int m_logCount;
/*	static int SysLogger();*/

	static void* m_logDataVoid;
	static CSysBase::CLogData* m_logData;
	/*
	 * mutex to read it
	 * by a function like foo(Aris::CMchineData &data)
	 */
	static Aris::RT_CONTROL::CMachineData m_machineData;

	static int RawLogDataToMachineData(Aris::RT_CONTROL::CMachineData& p_machineData,CLogData p_logData);

	/*
	 * NRT data prepare thread
	 */
	static pthread_t m_dataServer;
	static pthread_attr_t m_dataServer_attr;

	static pthread_mutex_t m_dataMutex;

	static Aris::RT_CONTROL::CMachineData m_logMachineDataBuffer;

	static void *dataServer(void* arg);

	static CSysBase::CLogData m_logDataBuffer;

	static int InitLogFile();
	static FILE *fp;


	/*
	 * MSW signal handler
	 */
	static struct sigaction m_sa;

	static void CatchWarnUponSwitch(int signalNum, siginfo_t *signalInfo, void *context);

	/*
	 * State Machine
	 *
	 * m_currentInternCmdRT and State m_servoStateRT decide what to do next
	 * first a matrix will decide what state is now , if return error,will not
	 * execute the cmd. If not error, will run the cmd
	 *
	 *
	 */
	// state transition matrix should be big enough to contain all transitions

	static Aris::RT_CONTROL::EServoState m_stateTransitionMatrix[CMD_NUMBER][STATE_NUMBER];
	static int stateMachineInit();
	static Aris::RT_CONTROL::EServoState EvalStateMachine(EInternCmd p_cmd,const Aris::RT_CONTROL::EServoState p_state);
	static EInternCmd m_currentInternCmdRT;
	static Aris::RT_CONTROL::EServoState m_currentStateRT;
	static Aris::RT_CONTROL::EServoState m_nextStateRT;
	static void PrintStateChange(Aris::RT_CONTROL::EServoState p_orig,Aris::RT_CONTROL::EServoState p_aim);
	static int PostStateToNRT(const Aris::RT_CONTROL::EServoState p_state);
	static void DoState(Aris::RT_CONTROL::EServoState& p_state, long long int p_time);
	static int ActionOfState(Aris::RT_CONTROL::EServoState& p_state, long long int p_time);



	/*
	 * HomeToRun cmd is peculiar
	 * Needed to be special treated
	 */

	//set to false in constructor
	static bool m_isH2Red[ACTUAL_MOTOR_NUMBER];
	static void H2RedClear();
	static int m_homePosition[ACTUAL_MOTOR_NUMBER];
	static long long int m_h2rStartTime;
	static long long int m_h2rStartTimeRUN;
	static int m_h2rDeltaPosition;
	static int m_h2rStartPosition[ACTUAL_MOTOR_NUMBER];
	/*
	 *STSTILL cmd is also need special care
	 */
	static CDeviceData m_standStillFrame;

	static CSysPID m_pid;

};





#endif /* SYSBASE_H_ */
