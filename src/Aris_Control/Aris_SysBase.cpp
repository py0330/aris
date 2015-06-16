/*
 * Aris_SysBase.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: hex
 */

#include "Aris_SysBase.h"

/*****************************************************************************
* CSysBase
*****************************************************************************/
// function pointers
FuncPtrWork CSysBase::trajectoryGenerator=NULL;
FuncPtrInit CSysBase::initHandler=NULL;
FuncPtrWork CSysBase::dataUpdater=NULL;
//FuncPtrState CSysBase::OnPOWEROFF=NULL;
//FuncPtrState CSysBase::OnPOED=NULL;
//FuncPtrState CSysBase::OnSTOP=NULL;
//FuncPtrState CSysBase::OnSTOPPED=NULL;
//FuncPtrState CSysBase::OnENABLE=NULL;
//FuncPtrState CSysBase::OnENABLED=NULL;
//FuncPtrState CSysBase::OnHOMING=NULL;
//FuncPtrState CSysBase::OnHOMED=NULL;
//FuncPtrState CSysBase::OnH2RING=NULL;
//FuncPtrState CSysBase::OnRUNNING=NULL;
//FuncPtrState CSysBase::OnSTSTILL=NULL;
//FuncPtrState CSysBase::OnEMERGENCY=NULL;
//FuncPtrCustom CSysBase::OnCustomMsg=NULL;

//rt-task members*************************************************************//
Aris::RT_CONTROL::CSysInitParameters CSysBase::m_sysInitParam;
CDeviceMaster CSysBase::m_deviceMaster;
RT_TASK CSysBase::m_realtimeCore;
RTIME CSysBase::m_timeStart;

long long int CSysBase::m_cycleCount;
//long long int CSysBase::m_h2rStartTime;
//long long int CSysBase::m_h2rStartTimeRUN;

//machine states and messges**************************************************//

//Aris::RT_CONTROL::EServoState CSysBase::m_currentStateRT;
//Aris::RT_CONTROL::EServoState CSysBase::m_nextStateRT;

//Aris::RT_CONTROL::EServoState CSysBase::m_servoStateNRT;
//bool CSysBase::m_isInTrajPhase;
//Aris::RT_CONTROL::EServoCommand  CSysBase::m_motorCommands[ACTUAL_MOTOR_NUMBER];
//Aris::RT_CONTROL::EServoState  CSysBase::m_motorStates[ACTUAL_MOTOR_NUMBER];
//Aris::RT_CONTROL::EOperationMode  CSysBase::m_motorModes[ACTUAL_MOTOR_NUMBER];

//Aris::RT_CONTROL::EServoCommand  CSysBase::m_machineCommand;
//Aris::RT_CONTROL::EServoState  CSysBase::m_machineState;
//Aris::RT_CONTROL::EOperationMode  CSysBase::m_machineMode;
//Aris::RT_CONTROL::EServoCommand  CSysBase::m_rawInternCommand;
//Aris::RT_CONTROL::EMachineState CSysBase::m_machineState;

//CSysBase::EInternCmd CSysBase::m_currentInternCmdRT;
//CSysBase::EInternCmd CSysBase::m_currentInternCmdNRT;

//Aris::RT_CONTROL::EServoState CSysBase::m_stateTransitionMatrix[CMD_NUMBER][STATE_NUMBER];
//bool CSysBase::m_isH2Red[ACTUAL_MOTOR_NUMBER];


//int CSysBase::m_h2rStartPosition[ACTUAL_MOTOR_NUMBER];
//int CSysBase::m_h2rDeltaPosition=50;

//bool CSysBase::m_isCurrentCycleGetData;
//bool CSysBase::m_isGetCustomDataCurrentCycle;

//bool CSysBase::m_isAllComplete;

char CSysBase:: print_SYSTEM_INITIALIZING [PRINT_INFO_BUFFER_SIZE];
char CSysBase::print_RT_Message_Initialized[PRINT_INFO_BUFFER_SIZE];
char CSysBase::print_Ethercat_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_Log_File_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_Log_Heap_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_RT_Task_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_SYSTEM_INITIALIZED[PRINT_INFO_BUFFER_SIZE];

 char CSysBase::print_SYSTEM_COMMUNICATION_INITIALIZING[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_XDDP_RT_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_XDDP_Data_RT_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_XDDP_NRT_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_XDDP_Data_NRT_Initialized[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_SYSTEM_COMMUNICATION_INITIALIZED[PRINT_INFO_BUFFER_SIZE];

 char CSysBase::print_TASK_START[PRINT_INFO_BUFFER_SIZE];
 char CSysBase::print_TASK_STOP[PRINT_INFO_BUFFER_SIZE];



Aris::RT_CONTROL::RT_MSG CSysBase::m_rtDataRecv;
Aris::RT_CONTROL::RT_MSG CSysBase::m_rtDataSend;
char CSysBase::m_rtDataRecvBuffer[RT_MSG_BUFFER_SIZE];
char CSysBase::m_rtDataSendBuffer[RT_MSG_BUFFER_SIZE];

//bool CSysBase::m_isDataFromRT;

CDeviceData CSysBase::m_standStillFrame;

//data logging members*******************************************************//
RT_HEAP CSysBase::m_logHeap;

CSysBase::CLogData CSysBase::m_logDataRC;
Aris::RT_CONTROL::CMachineData CSysBase::m_machineDataCore;
//Aris::RT_CONTROL::CMachineData CSysBase::m_machineData;
bool CSysBase::m_isLog;
long long int CSysBase::m_logCount;
void* CSysBase::m_logDataVoid;
CSysBase::CLogData* CSysBase::m_logData;

pthread_t CSysBase::m_dataServer;
pthread_attr_t CSysBase::m_dataServer_attr;

pthread_mutex_t CSysBase::m_dataMutex=PTHREAD_MUTEX_INITIALIZER;

CSysBase::CLogData CSysBase::m_logDataBuffer;

struct sigaction CSysBase::m_sa;

FILE *CSysBase::fp;
Aris::RT_CONTROL::CMachineData CSysBase::m_logMachineDataBuffer;

//communication members************************************************************//
int CSysBase::m_xddp_socket_rt;
char CSysBase::m_xddp_buf_rt[4096];

int CSysBase::m_xddp_socket_data_rt;
char CSysBase::m_xddp_buf_data_rt[4096];

char CSysBase::m_xddp_buf[4096];
char* CSysBase::m_xddp_devname;
int CSysBase::m_xddp_fd;

char CSysBase::m_xddp_buf_data[4096];
char *CSysBase::m_xddp_devname_data;
int CSysBase::m_xddp_fd_data;

//****************************************system operation*********************************************************//

CSysBase::CSysBase()
{
 //	CSysBase::m_isCurrentCycleGetData=false;
//	CSysBase::m_operationMode=OM_NOMODE;
//	CSysBase::m_currentInternCmdRT=CSysBase::EInternCmd::EIC_NONE;
//	CSysBase::m_currentInternCmdNRT=CSysBase::EInternCmd::EIC_NONE;
//	CSysBase::m_isDataFromRT=false;

	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		CSysBase::m_machineDataCore.motorsCommands[i]=Aris::RT_CONTROL::EServoCommand::EMCMD_NONE;
		CSysBase::m_machineDataCore.isMotorHomed[i]=false;
	}

	CSysBase::m_machineDataCore.machinestate=Aris::RT_CONTROL::EMachineState::CS_UNINITED;

	 //	CSysBase::m_motorStates[0]=m_deviceMaster.m_motors[0].GetMotorState();
	//	CSysBase::m_motorModes[i]=m_deviceMaster.m_motors[i].GetMotorMode();


//	CSysBase::m_machineCommand=Aris::RT_CONTROL::EServoCommand::EMCMD_NONE;
//	CSysBase::m_rawInternCommand=Aris::RT_CONTROL::EServoCommand::EMCMD_NONE;

//	CSysBase::m_machineState=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
//	CSysBase::m_machineMode=Aris::RT_CONTROL::EOperationMode::OM_NOMODE;

	CSysBase::m_isLog=true;
	CSysBase::m_logCount=0;
//	CSysBase::m_isGetCustomDataCurrentCycle=false;
	//CSysBase::m_isInTrajPhase=CSysBase::m_deviceMaster.m_needSetTrajData;

	//CSysBase::m_isCurrentCycleGetData=false;

//	CSysBase::m_servoStateNRT=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;
//	CSysBase::m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;
//	CSysBase::m_nextStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;

//  change it after start
//	CSysBase::m_isLog=true;
//	CSysBase::m_logCount=0;
//	CSysBase::m_isGetCustomDataCurrentCycle=false;
//	CSysBase::H2RedClear();
};

CSysBase::~CSysBase()
{
	if(m_machineDataCore.machinestate==Aris::RT_CONTROL::EMachineState::CS_STOPPED)
	{
		return;
	}
	else
	{
		// do some clean
		rt_printf("析构函数的调用");
		SysStop();
	}
};

int CSysBase::SysInit(Aris::RT_CONTROL::CSysInitParameters p_Param)
{
	int ret=Load_XML_PrintMessages();
	if(ret!=0)
	{
		printf("error Loading xml File!\n");
	    return -1;
	}


	PrintInfo(CSysBase::print_SYSTEM_INITIALIZING);
	//backup sysInitParam
	CSysBase::m_sysInitParam = p_Param;
	//CSysBase::m_sysInitParam.motorNum = p_Param.motorNum;
	//CSysBase::m_sysInitParam.homeMode = p_Param.homeMode;
	//CSysBase::m_sysInitParam.nsPerCyclePeriod = p_Param.nsPerCyclePeriod;

/*	for(int i=0;i<p_Param.motorNum;i++)
	{
		m_homePosition[p_Param.driverIDs[i]]=-p_Param.homeOffsets[i];
	}*/


	if(CSysBase::initHandler!=NULL)
	{
		CSysBase::initHandler(p_Param);
	}

	//else
		//if(m_print_SysInitHandler!=NULL)
		//{
		//	printf("%s\n",m_print_SysInitHandler);
		//}


	//init two RT_CONN_DATA
	InitRT_MSG();
	PrintInfo(CSysBase::print_RT_Message_Initialized);

	InitLogFile();
	PrintInfo(CSysBase::print_Log_File_Initialized);
	//stateMachineInit();

	//int ret;
    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    // Since the priority inversion happens when writing to the RT
    // serial port, we need to assign an specified action to the SIGDEBUG
    // signal, otherwise the program will exit forcely by the system's
    // default signal handler. If this signal is catched, xenomai will
    // temporarily swiches to secondery mode and a warning is generated.
    sigemptyset(&m_sa.sa_mask);
    m_sa.sa_sigaction = CatchWarnUponSwitch;
    m_sa.sa_flags = SA_SIGINFO;
    sigaction(SIGDEBUG, &m_sa, NULL);

    // Catch the Ctrl-C signal if user want to terminate the program
    signal(SIGTERM, CatchStopSignal);
    signal(SIGINT, CatchStopSignal);

    // Initialize the hardwares
     ret = m_deviceMaster.Initialize(p_Param);
    if (ret != 0)
    {
        printf("Fail to initialize devices, please check your settings\n");
        return -1;
    }
	PrintInfo(CSysBase::print_Ethercat_Initialized);



    //Create Log Heap
    if(MAX_LOG_ENTRIES*sizeof(CLogData)<2000000000)
    {
        ret = rt_heap_create(&m_logHeap,"logHeap",
        		MAX_LOG_ENTRIES * sizeof(CLogData),H_MAPPABLE);
        if(ret < 0 )
        {
        	printf("ERROR:create heap failed %d\n",ret);
        }
        else
        {
        	//printf("Heap created.\n");
         	ret = rt_heap_alloc(&m_logHeap,
        			MAX_LOG_ENTRIES * sizeof(CLogData),TM_INFINITE,&m_logDataVoid);
        	if(ret<0)
        	{
        		printf("ERROR: log memory allocating failed.\n");
        	}
        	else
        	{
        		m_logData=(CLogData*)m_logDataVoid;
        		PrintInfo(CSysBase::print_Log_Heap_Initialized);
        	}
        }

    }
    else
    {
    	printf("WARN:Insuffient memory,reduce log data size.");
    }


    // Create RT task
    ret = rt_task_create(&m_realtimeCore,"realtime core",0,TASK_PRIO_CORE,T_FPU);
    if (ret != 0)
    {
        printf("Fail to create real-time task, please check your xenomai settings\n");
        return -2;
    }


    /*
     * Initial setup of m_dataServer
     */
    pthread_attr_init(&m_dataServer_attr);
    //pthread_attr_setschedpolicy(&m_dataServer_attr, SCHED_FIFO);

    m_machineDataCore.machinestate = Aris::RT_CONTROL::CS_INITED;
	PrintInfo(CSysBase::print_SYSTEM_INITIALIZED);
	//rt_printf("good");
    return 0;
};

int CSysBase::SysInitCommunicationRT()
{
	    /*************************************************************************
     * Initialize XDDP in RT
     *************************************************************************/
    struct sockaddr_ipc saddr;
    int ret,len;
    struct timespec ts;
    size_t poolsz;

	/*
	 * Get a datagram socket to bind to the RT endpoint. Each
	 * endpoint is represented by a port number within the XDDP
	 * protocol namespace.
	 */
	m_xddp_socket_rt = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (m_xddp_socket_rt < 0)
	{
		perror("rc socket");
		exit(EXIT_FAILURE);
	}
	//else
		//rt_printf("rc socket ok\n");


	struct timeval tv;

	tv.tv_sec = 0;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors

	ret=rt_dev_setsockopt(m_xddp_socket_rt, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	if (ret)
		fail("setsockopt--");
	//else
		//rt_printf("setsockopt ok\n");

	/*
	 * Set a local 16k pool for the RT endpoint. Memory needed to
	 * convey datagrams will be pulled from this pool, instead of
	 * Xenomai's system pool.
	 */
	poolsz = 16384; /* bytes */
	ret = rt_dev_setsockopt(m_xddp_socket_rt, SOL_XDDP, XDDP_POOLSZ,
			 &poolsz, sizeof(poolsz));
	if (ret)
		fail("setsockopt");
	//else
		//rt_printf("setsockopt ok\n");

	/*
	 * Bind the socket to the port, to setup a proxy to channel
	 * traffic to/from the Linux domain.
	 *
	 * saddr.sipc_port specifies the port number to use.
	 */
	memset(&saddr, 0, sizeof(saddr));
	saddr.sipc_family = AF_RTIPC;
	saddr.sipc_port = XDDP_PORT;
	ret = rt_dev_bind(m_xddp_socket_rt, (struct sockaddr *)&saddr, sizeof(saddr));
	if (ret)
		fail("bind");
	//else
		//rt_printf("bind ok\n");
	PrintInfo(CSysBase::print_XDDP_RT_Initialized);


    /*************************************************************************
     * Initialize XDDP for data in RT
     *************************************************************************/
	/*
	 * Get a datagram socket to bind to the RT endpoint. Each
	 * endpoint is represented by a port number within the XDDP
	 * protocol namespace.
	 */
	m_xddp_socket_data_rt = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (m_xddp_socket_data_rt < 0)
	{
		perror("rd socket");
		exit(EXIT_FAILURE);
	}
	//else
		//rt_printf("rd socket ok\n");

	tv.tv_sec = 0;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors

	ret=rt_dev_setsockopt(m_xddp_socket_data_rt, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	if (ret)
		fail("rd setsockopt--");
	//else
		//rt_printf("rd setsockopt ok\n");

	/*
	 * Set a local 16k pool for the RT endpoint. Memory needed to
	 * convey datagrams will be pulled from this pool, instead of
	 * Xenomai's system pool.
	 */
	poolsz = 16384; /* bytes */
	ret = rt_dev_setsockopt(m_xddp_socket_data_rt, SOL_XDDP, XDDP_POOLSZ,
			 &poolsz, sizeof(poolsz));
	if (ret)
		fail("rd setsockopt");
	//else
		//rt_printf("rd setsockopt ok\n");

	/*
	 * Bind the socket to the port, to setup a proxy to channel
	 * traffic to/from the Linux domain.
	 *
	 * saddr.sipc_port specifies the port number to use.
	 */
	memset(&saddr, 0, sizeof(saddr));
	saddr.sipc_family = AF_RTIPC;
	saddr.sipc_port = XDDP_PORT_DATA;
	ret = rt_dev_bind(m_xddp_socket_data_rt, (struct sockaddr *)&saddr, sizeof(saddr));
	if (ret)
		fail("bind");
	//else
		//rt_printf("bind ok\n");
//

	PrintInfo(CSysBase::print_XDDP_Data_RT_Initialized);


};

int CSysBase::SysInitCommunication()
{
	PrintInfo(CSysBase::print_SYSTEM_COMMUNICATION_INITIALIZING);

	CSysBase::SysInitCommunicationRT();
    //Initialize XDDP NRT
	if (asprintf(&m_xddp_devname, "/dev/rtp%d", XDDP_PORT) < 0)
		printf("Error in asprintf\n");
	//else
		//printf("OK in asprintf NRT.\n");

	m_xddp_fd =open("/dev/rtp0", O_RDWR);
	free(m_xddp_devname);
	if (m_xddp_fd < 0)
		printf("RC Error in open %d\n",m_xddp_fd);
	//else
		//printf("RC OK in open NRT\n");
	//set to nonblock
//	int flags = fcntl(m_xddp_fd, F_GETFL, 0);
//	fcntl(m_xddp_fd, F_SETFL, flags | O_NONBLOCK);

	//set to block
	int flags = fcntl(m_xddp_fd, F_GETFL, 0);
	fcntl(m_xddp_fd, F_SETFL, flags &~ O_NONBLOCK);
	PrintInfo(CSysBase::print_XDDP_NRT_Initialized);

	//Initialize XDDP_DATA_NRT
	if(asprintf(&m_xddp_devname_data,"/dev/rtp%d",XDDP_PORT_DATA)<0)
		printf("RD Error in asprintf\n");
	//else
		//printf("RD OK in asprintf NRT.\n");

	m_xddp_fd_data =open("/dev/rtp1", O_RDWR);
	free(m_xddp_devname_data);
	if (m_xddp_fd_data < 0)
		printf("RC Error in open %d\n",m_xddp_fd_data);
	//else
		//printf("RC OK in open NRT\n");
	//set to nonblock
//	flags = fcntl(m_xddp_fd_data, F_GETFL, 0);
//	fcntl(m_xddp_fd_data, F_SETFL, flags | O_NONBLOCK);

	//set to block
	flags = fcntl(m_xddp_fd, F_GETFL, 0);
	fcntl(m_xddp_fd, F_SETFL, flags &~ O_NONBLOCK);

    pthread_create(&m_dataServer,NULL,&dataServer,NULL);

	PrintInfo(CSysBase::print_XDDP_Data_NRT_Initialized);

	PrintInfo(CSysBase::print_SYSTEM_COMMUNICATION_INITIALIZED);
	m_machineDataCore.machinestate = Aris::RT_CONTROL::CS_COMM_INITED;

	return 0;
};

void CSysBase::CatchStopSignal(int signalNum)
{
	m_machineDataCore.machinestate = Aris::RT_CONTROL::CS_STOPPED;
    rt_printf("Terminate signal catched, program will exit\n");
    SysStop();
};

void CSysBase::CatchWarnUponSwitch(int signalNum, siginfo_t *signalInfo, void *context)
{
    rt_printf("Xenomai switch to secondary mod\n");
};

void CSysBase::fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
};

int CSysBase::SysStart()
{
	m_machineDataCore.machinestate = Aris::RT_CONTROL::CS_RTTASK_STARTED;
    int ret;
	PrintInfo(CSysBase::print_TASK_START);
    ret = rt_task_start(&m_realtimeCore,&RealtimeCore,NULL);
    //ret = rt_task_start(&m_realtimeData,&RealtimeData,NULL);
    //rt_printf("task started\n");


    return 0;
};

int CSysBase::SysStop()
{
	m_machineDataCore.machinestate = Aris::RT_CONTROL::CS_STOPPED;

    //rt_printf("Delete RT Task RealtimeCore\n");
    rt_task_delete(&m_realtimeCore);
    pthread_cancel(m_dataServer);
    fflush(fp);
    fclose(fp);
    m_isLog=false;
    m_deviceMaster.DeactiveMaster();
	PrintInfo(CSysBase::print_TASK_STOP);

    return 0;
};

bool CSysBase::IsSysStopped()
{
	if(m_machineDataCore.machinestate==Aris::RT_CONTROL::CS_STOPPED)
	{
		return true;
	}
	else
	{
		return false;
	}
};

int CSysBase::SetSysInitializer(FuncPtrInit p_Func)
{
	initHandler=p_Func;
	return 0;
};

int CSysBase::SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater)
{
	dataUpdater=p_DataUpdater;
	return 0;
};

int CSysBase::SetTrajectoryGenerator(FuncPtrWork p_TrajectoryGenerator)
{
	trajectoryGenerator=p_TrajectoryGenerator;
	return 0;
};

void CSysBase::PushDatatoMotors()
{
	    for(int i=0;i<m_deviceMaster.m_motorNum;i++)
	  		{
	  	        m_deviceMaster.m_motors[i].SetMotorCommand(m_machineDataCore.motorsCommands[i]);
	  	        m_deviceMaster.m_motors[i].SetMotorMode(m_machineDataCore.motorsModes[i]);
	  	        //rt_printf("mode of driver from command: %d\n",m_machineDataCore.motorsModes[0]);
	  		}

      //  rt_printf("Pushdata:command data machine pos:%d\n",m_machineDataCore.commandData[0].Position);
        m_deviceMaster.MachineDataToDeviceData(m_machineDataCore);
       // rt_printf("Pushdata:command data device pos:%d\n",m_deviceMaster.m_commandData.m_motorData[0].Position);
        m_deviceMaster.DoPID();

}

void CSysBase::UploadDatafromMotors()
{
	for(int i=0;i<m_deviceMaster.m_motorNum;i++)
	{
        m_machineDataCore.motorsStates[i]=m_deviceMaster.m_motors[i].GetMotorState();
        m_machineDataCore.motorsModesDisplay[i]=m_deviceMaster.m_motors[i].GetMotorMode();
        if(m_deviceMaster.m_motors[i].CheckMotorHomed())
        	m_machineDataCore.isMotorHomed[i]=true;
	}
    m_deviceMaster.DeviceDataToMachineData(m_machineDataCore,CSysBase::m_cycleCount);
  //  rt_printf("Download device data pos:%d\n",m_machineDataCore.feedbackData[0].Position);

}

Aris::RT_CONTROL::EMachineState CSysBase::GetMachineState()
{
	return m_machineDataCore.machinestate;
}

//real time core
//1. device master read
//2. do state ( post state and  action of state)
//3. device master write(do command of motors)

void CSysBase::RealtimeCore(void* arg)
{
	//rt_printf("Realtime Core started.\n");

    RTIME timeNow;
    RTIME timePrevious;
    float period;

    //rt_printf("RealtimeData entering loop\n");
    //rt_task_set_mode(0, T_WARNSW, NULL);
    rt_task_set_periodic(NULL, TM_NOW, PERIOD_NS_CORE);

    int ret=0;

	/*************************************************************************
	 * RT-Core Loop
	 *************************************************************************/
	m_cycleCount = 0;
    m_timeStart = rt_timer_read();

    //clear screen
    // rt_printf("SysMonitor\n");

    while (m_machineDataCore.machinestate == Aris::RT_CONTROL::CS_RTTASK_STARTED)
    {
       // if(m_cycleCount%1==0)
        	//rt_printf("m_cycleCount:%d\n",m_cycleCount);
        rt_task_wait_period(NULL);
        timeNow = rt_timer_read();

        // receive ethercat**************************************************************###
        m_deviceMaster.Read();//motors and sensors get data

        UploadDatafromMotors();//motor data to machine data
       // rt_printf("motor 0 mode:%d\n",m_deviceMaster.m_motors[0].GetMotorMode());

        ret=RT_RecvDataRaw(CSysBase::m_rtDataRecvBuffer,RT_MSG_BUFFER_SIZE);

        if(ret>=0)
        {
        	rt_printf("msg id GET IN RT: %d\n",m_rtDataRecv.GetMsgID());
        }

        if(ret<0)
        {
        	CSysBase::m_rtDataRecv.SetMsgID(-100);
        	//rt_printf("msg in RT not get!!\n");

        }

        if(trajectoryGenerator!=NULL)
        	trajectoryGenerator(m_machineDataCore, m_rtDataRecv);

        PushDatatoMotors();
        //machine data to motor data



       // CSysBase::DoState(m_nextStateRT,m_cycleCount);

 //         rt_printf("%lld %lld :: CMD:%d CW:%d SW:%d AP:%d\tTP:%d  M:%d DIFF %d\n",m_cycleCount,m_h2rStartTimeRUN,
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].MotorCmd.command,
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].ControlWord,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].StatusWord,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].Position,
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].Position,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].Mode,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].Position-
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].Position
//        							);

        m_deviceMaster.DCSyncTime(rt_timer_read());
        m_deviceMaster.Write();//motor data write and state machine/mode transition

        timePrevious = timeNow;

//        m_logDataRC.time=m_cycleCount;
//        m_logDataRC.m_machineState=CSysBase::m_machineDataCore.machinestate;
//        //m_logDataRC.m_servoState=CSysBase::m_machineState;
//        memcpy(&m_logDataRC.m_feedbackData,&CSysBase::m_deviceMaster.m_feedbackData,
//        		sizeof(CSysBase::m_deviceMaster.m_feedbackData));
//        memcpy(&m_logDataRC.m_commandData,&CSysBase::m_deviceMaster.m_commandData,
//        		sizeof(CSysBase::m_deviceMaster.m_commandData));

		m_logCount++;
		/*
		 * listening and answering request from xddp_data
		 */
		//ret = rt_dev_recvfrom(m_xddp_socket_data_rt,&flag,sizeof(flag),MSG_DONTWAIT,NULL,0);
		//if(flag==FLAG)

		//100HZ to NRT

		if(m_cycleCount%10==0)
		{

			/*
			 * Make data should do in DataServer
			 */
			//RawLogDataToMachineData(m_machineData,m_logDataRC);

			/*
			 * Sent machineData to NRT,this simplified data can be exposed to others
			 */
//			int * a = new int();
//			delete a;
//			ret = rt_dev_sendto(m_xddp_socket_data_rt,&m_logDataRC,sizeof(m_logDataRC),0,NULL,0);
			ret = rt_dev_sendto(m_xddp_socket_data_rt,&m_machineDataCore,sizeof(m_machineDataCore),0,NULL,0);
			if(ret==-12)
			{
//				rt_printf("WARN:Internal communication buffer 2 is full.%lld\n",m_cycleCount);
			}

			//rt_printf("%d\n",ret);
			//reset flag
			//flag='D';
		}

		m_cycleCount++;
    }
};

//**************************************************msg handling********************************************//

int CSysBase::InitRT_MSG()
{
	//after this function, this address should not be changed any more in RT side
	CSysBase::m_rtDataRecv.m_ptrData=(char *)CSysBase::m_rtDataRecvBuffer;
	CSysBase::m_rtDataSend.m_ptrData=(char *)CSysBase::m_rtDataSendBuffer;

//	CSysBase::m_rtDataRecv.m_ptrDataLength=(int*)CSysBase::m_rtDataRecv.m_ptrData;
//	CSysBase::m_rtDataSend.m_ptrDataLength=(int*)CSysBase::m_rtDataSend.m_ptrData;
//
//	CSysBase::m_rtDataRecv.m_ptrDataType=(int*)(CSysBase::m_rtDataRecv.m_ptrData+sizeof(int));
//	CSysBase::m_rtDataSend.m_ptrDataType=(int*)(CSysBase::m_rtDataSend.m_ptrData+sizeof(int));
	return 0;
}

/*int CSysBase::PostStateToNRT(const Aris::RT_CONTROL::EServoState p_state)
{
	int ret;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EMIT_STA;
//	ret = CSysBase::RT_PostMessageRaw(&type,sizeof(type));
//	ret = CSysBase::RT_PostMessageRaw(&p_state,sizeof(p_state));
	CSysBase::m_rtDataSend.SetType(type);
	CSysBase::m_rtDataSend.SetLength(sizeof(p_state));
	CSysBase::m_rtDataSend.Copy(&p_state,sizeof(p_state));
	//send in two steps *****************************************************TBD!!!!!!!!!!!!!!!!

    // transfer the whole rt msg
	CSysBase::RT_SendDataRaw(CSysBase::m_rtDataSend.m_ptrData,RT_MSG_HEADER_LENGTH);
	CSysBase::RT_SendDataRaw(CSysBase::m_rtDataSend.GetDataAddress(),CSysBase::m_rtDataSend.GetLength());
	return 0;
};*/

int CSysBase::RT_SendDataRaw(const void* p_ptrData, const int p_dataLength)
{
	int ret;
	if(p_ptrData==NULL)
	{
		rt_printf("RT_SendData: NULL ptr is fed\n");
		return -14;//EFAULT 14 bad address
	}

	//send two datagram, one is head, one is data

	ret = rt_dev_sendto(m_xddp_socket_rt,p_ptrData,p_dataLength,0,NULL,0);
	if(ret<0)
	{
		rt_printf("RT_SendDataRaw%d\n",ret);
	}
	return ret;
};

int CSysBase::RT_RecvDataRaw(void* p_ptrData,const int p_dataLength)
{
	int ret;
	if(p_ptrData==NULL)
	{
		rt_printf("RT_RecvData: NULL ptr is fed\n");
		return -14;//EFAULT 14 bad address
	}
	//just a copy function
	//ret = rt_dev_recvfrom(m_xddp_socket_rt,m_rtDataRecvBuffer,RT_CONN_DATA_BUFFER_SIZE,MSG_DONTWAIT,NULL,0);
	ret = rt_dev_recvfrom(m_xddp_socket_rt,p_ptrData,p_dataLength,MSG_DONTWAIT,NULL,0);
	return ret-RT_MSG_HEADER_LENGTH;
};

int CSysBase::NRT_SendDataRaw(const void* p_ptrData, const int p_dataLength)
{
	int ret;
	if(p_ptrData==NULL)
	{
		rt_printf("NRT_SendData: NULL ptr is fed\n");
		return -14;//EFAULT 14 bad address
	}
	//send all in one time
	ret=write(m_xddp_fd,p_ptrData,p_dataLength);
	//return ret-RT_CONN_DATA_HEADSIZE;
	return ret;
};

int CSysBase::NRT_RecvDataRaw(void* p_ptrData,const unsigned int p_dataLength)
{
	int ret;
	if(p_ptrData==NULL)
	{
		rt_printf("NRT_RecvData: NULL ptr is fed\n");
		return -14;//EFAULT 14 bad address
	}
	//just a copy function?
	ret=read(m_xddp_fd,p_ptrData,p_dataLength);
	return ret;
};


int CSysBase::RT_PostStateMsg()
{
/*	int ret;
	ret=RT_SendDataRaw(CSysBase::m_rtDataSend.m_ptrData,RT_MSG_HEADER_LENGTH);
	if(ret < 0)
		return ret;
	if(m_rtDataSend.GetLength()>0)
	{
		ret=RT_SendDataRaw(m_rtDataSend.GetDataAddress(),m_rtDataSend.GetLength());
	}
	return ret;*/
	int ret;
	ret=RT_SendDataRaw(m_rtDataSend.m_ptrData,m_rtDataSend.GetLength()+RT_MSG_HEADER_LENGTH);
	return ret-RT_MSG_HEADER_LENGTH;
}


int CSysBase::RT_PostMsg(Aris::RT_CONTROL::RT_MSG &p_data)
{
	int ret;
	ret=RT_SendDataRaw(p_data.m_ptrData,RT_MSG_HEADER_LENGTH);
	if(ret < 0)
		return ret;
	if(p_data.GetLength()>0)
	{
		ret=RT_SendDataRaw(p_data.GetDataAddress(),p_data.GetLength());
	}
	return ret;

}


//**************************************************XML*********************************************//

int CSysBase::Load_XML_PrintMessages()
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err=doc.LoadFile("/usr/Aris/resource/Aris_Control/Aris_Control.xml");
    if(err!=0)
    	printf("XML Loading err %d\n, check if file Aris_Control.xml installed in directory /usr/Aris/resource/Aris_Control ",err);
    else
    	printf("XML Loading ...\n");
    if(err==0)
    {

        const char * ch;
     	tinyxml2::XMLElement* SysInitTitle;//,SysInitCommnicationTitle,SysTaskTitle,DriverStateTitle;


     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("SysInitBegin");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_SYSTEM_INITIALIZING,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("InitRTMessage");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_RT_Message_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("InitEthercat");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_Ethercat_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("InitLog");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_Log_File_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("InitLogHeap");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_Log_Heap_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("InitRTTask");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_RT_Task_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInit")->FirstChildElement("SysInitFinish");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_SYSTEM_INITIALIZED,ch);


        SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInitCommunication")->FirstChildElement("SysInitCommunicationBegin");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_SYSTEM_COMMUNICATION_INITIALIZING,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInitCommunication")->FirstChildElement("Init_XDDP_RT");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_XDDP_RT_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInitCommunication")->FirstChildElement("Init_XDDP_Data_RT");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_XDDP_Data_RT_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInitCommunication")->FirstChildElement("Init_XDDP_NRT");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_XDDP_NRT_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInitCommunication")->FirstChildElement("Init_XDDP_Data_NRT");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_XDDP_Data_NRT_Initialized,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysInitCommunication")->FirstChildElement("SysInitCommunicationFinish");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_SYSTEM_COMMUNICATION_INITIALIZED,ch);



     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysTask")->FirstChildElement("SysStart");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_TASK_START,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("SysTask")->FirstChildElement("SysStop");
        ch=SysInitTitle->GetText();
        strcpy(CSysBase::print_TASK_STOP,ch);


     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("DriverState")->FirstChildElement("PoweredOff");
        ch=SysInitTitle->GetText();
        strcpy(CElmoMotor::print_poweredoff,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("DriverState")->FirstChildElement("Stop");
        ch=SysInitTitle->GetText();
        strcpy(CElmoMotor::print_stop,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("DriverState")->FirstChildElement("Enabled");
        ch=SysInitTitle->GetText();
        strcpy(CElmoMotor::print_enabled,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("DriverState")->FirstChildElement("Homing");
        ch=SysInitTitle->GetText();
        strcpy(CElmoMotor::print_homing,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("DriverState")->FirstChildElement("Running");
        ch=SysInitTitle->GetText();
        strcpy(CElmoMotor::print_running,ch);

     	SysInitTitle=doc.FirstChildElement("Aris")->FirstChildElement("DriverState")->FirstChildElement("Fault");
        ch=SysInitTitle->GetText();
        strcpy(CElmoMotor::print_fault,ch);



    }
    return err;
}

void CSysBase::PrintInfo(const char* ch)
{
	if(ch!=NULL)
	{
		if(CSysBase::m_machineDataCore.machinestate==Aris::RT_CONTROL::CS_UNINITED)
		{
			printf("%s\n",ch);
		}
		else
		{
			rt_printf("%s\n",ch);
		}
	}

}

//************************************************logging files*************************************************//
int CSysBase::RawLogDataToMachineData(Aris::RT_CONTROL::CMachineData& p_machineData,CLogData p_logData)
{
	p_machineData.motorNum=ACTUAL_MOTOR_NUMBER;
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		p_machineData.commandData[i].Position	=	p_logData.m_commandData.m_motorData[i].Position;
		p_machineData.commandData[i].Velocity	=	p_logData.m_commandData.m_motorData[i].Velocity;
		p_machineData.commandData[i].Torque		=	p_logData.m_commandData.m_motorData[i].Torque;

		p_machineData.feedbackData[i].Position	=	p_logData.m_feedbackData.m_motorData[i].Position;
		p_machineData.feedbackData[i].Velocity	=	p_logData.m_feedbackData.m_motorData[i].Velocity;
		p_machineData.feedbackData[i].Torque  	=	p_logData.m_feedbackData.m_motorData[i].Torque;
	}
	p_machineData.machinestate=p_logData.m_machineState;
	p_machineData.time=p_logData.time;

	return 0;
};

int CSysBase::InitLogFile()
{
 	char LogFile[300];
	char tmpDate[100];
	time_t now;
	struct tm *p;
	time(&now);
	p = localtime(&now);

	strftime(tmpDate,99,"%Y_%m_%d_%H_%M_%a",p);
	sprintf(LogFile,"Log_%s.botlog",tmpDate);
	if((fp=fopen(LogFile,"wb"))!=NULL)
	{
		//printf("Log file opened.\n");
	}

	return 0;
}

void *CSysBase::dataServer(void* arg)
{
	//printf("DataServer starting...\n");
	timespec tv;
	tv.tv_sec=0;
	tv.tv_nsec=5000000;
	/*
	 *read data from RT
	 */
	int ret;
	RTIME timeNow,timePre;
	m_logCount=0;

	while(1)
	{
		timeNow=rt_timer_read();

		do
		{
			//ret = read(m_xddp_fd_data,&m_logDataBuffer,sizeof(m_logDataBuffer));
			ret = read(m_xddp_fd_data,&m_logMachineDataBuffer,sizeof(m_logMachineDataBuffer));
//			printf("%d,%lld\n",ret,m_logDataBuffer.time);
			if(ret>0)
			{
//				memcpy(&m_logData[m_logCount],&m_logDataBuffer,sizeof(m_logDataBuffer));
//				RawLogDataToMachineData(m_logMachineDataBuffer,m_logDataBuffer);
				m_logCount++;
				/*
				 * Write to file
				 */
				//printf("motor state:%d\n",m_logMachineDataBuffer.motorsStates[0]);
				fwrite(&m_logMachineDataBuffer,sizeof(m_logMachineDataBuffer),1,fp);

				if(m_logCount%1000==0)
				{
					printf("TIME:%lld %d POS:%d VEL%d POD:%d Motor State: %d\n",m_cycleCount,
							m_logDataBuffer.m_feedbackData.m_motorData[0].StatusWord,
							m_logDataBuffer.m_feedbackData.m_motorData[0].Position,
							m_logDataBuffer.m_feedbackData.m_motorData[0].Velocity,
							m_logDataBuffer.m_commandData.m_motorData[0].Position,
							m_logMachineDataBuffer.motorsStates[0]);
				}

			}

		}
		while(ret>0);
		/*
		 * Copy data to shared variable with mutex
		 */
	//	pthread_mutex_lock(&m_dataMutex);
	//	memcpy(&m_machineData,&m_logMachineDataBuffer,sizeof(m_logMachineDataBuffer));
	//	pthread_mutex_unlock(&m_dataMutex);

		//Call Updater
	//	if(CSysBase::dataUpdater!=NULL)
	//	{
	//		CSysBase::dataUpdater(m_machineData);
	//	}

		timePre=timeNow;
		/*
		 * jump out of the while loop so data file will be complete
		 */
		if(m_machineDataCore.machinestate==Aris::RT_CONTROL::CS_STOPPED)
		{
			break;
		}
		nanosleep(&tv,NULL);
	}
	return 0;

};







