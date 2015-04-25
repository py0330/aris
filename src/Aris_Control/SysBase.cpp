/*
 * SysBase.cpp
 *
 *  Created on: Nov 10, 2014
 *      Author: leo
 */

#include "SysBase.h"
/*****************************************************************************
 * CDeviceMaster
 *****************************************************************************/

ec_master_t* CDeviceMaster::m_pMaster = NULL;
ec_master_state_t CDeviceMaster::m_masterState = {};

CDeviceMaster::CDeviceMaster(void)
{
    m_motorNum = ACTUAL_MOTOR_NUMBER;
    m_analogInputsTerminalNum = ACTUAL_EL3104_NUM;
    m_analogOutputsTerminalNum = ACTUAL_EL4002_NUM;
    m_isMasterRequested = false;
    m_isMasterActivated = false;
    m_startMotorID=0;
    m_endMotorID=ACTUAL_MOTOR_NUMBER;

    for (int i = 0; i < ACTUAL_MOTOR_NUMBER; i++)
    {
        m_HomeOffset[i] = HEXBOT_HOME_OFFSETS_RESOLVER[i];
    }
}

CDeviceMaster::~CDeviceMaster(void)
{
    if (m_isMasterActivated == true)
    {
        this->DeactiveMaster();
    }
}

// The sequence of initialization:
// 1. Request master ->
// 2. Set slaves' positions ->
// 3. Register slaves' PDO entries (slave initialization) ->
// 4. Activate master ->
// 5. Activate slaves
int CDeviceMaster::Initialize(const Aris::RT_CONTROL::CSysInitParameters p_InitParam)
{
    int returnValue;

    // request master
    if(!m_isMasterRequested)
    {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
            perror("mlockall failed\n");
            return -1;
        }

        m_pMaster = ecrt_request_master(0);
        if(!m_pMaster)
        {
            printf("master request failed!\n");
            return -2;
        }
        m_isMasterRequested=true;
    }

    // set all the Ethercat slave devices' positions
//    m_ethercatCoupler.SetEtherCATPosition(ECAT_POS_COUPLER);
//
//    for (int i = 0; i < m_analogInputsTerminalNum; i++)
//    {
//        m_analogInputs[i].SetEtherCATPosition(ECAT_START_POS_EL3104 + i);
//    }
//    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
//    {
//        m_analogOutputs[i].SetEtherCATPosition(ECAT_START_POS_EL4002 + i);
//    }
    /*************************************************************************
     * Here should use m_startMotorID and m_endMotorID
     *************************************************************************/
    for(int i = 0; i < m_motorNum; i++)
    {
        m_motor[i].SetEtherCATPosition(ECAT_START_POS_ELMODRIVE + i);
    }

    //initialize all the slaves
//    returnValue = m_ethercatCoupler.Initialize(&m_pMaster);
//    if(returnValue != 0)
//    {
//        printf("Fail to initialize the bus coupler\n");
//        return returnValue;
//    }
//
//    for (int i = 0; i < m_analogInputsTerminalNum; i++)
//    {
//        returnValue = m_analogInputs[i].Initialize(&m_pMaster);
//        if(returnValue != 0)
//        {
//            printf("Fail to initialize the analog inputs No.%d\n", i);
//            return returnValue;
//        }
//    }
//    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
//    {
//        returnValue = m_analogOutputs[i].Initialize(&m_pMaster);
//        if(returnValue != 0)
//        {
//            printf("Fail to initialize the analog outputs No.%d\n", i);
//            return returnValue;
//        }
//    }

    // assign general settings of the motor
//    m_motorGeneralSettings.homeMode = HOMING_MODE;
//    m_motorGeneralSettings.homeAccel = HOMING_ACC;
//    m_motorGeneralSettings.homeLowSpeed = HOMING_LO_SPEED;
//    m_motorGeneralSettings.homeHighSpeed = HOMING_HI_SPEED;
//    m_motorGeneralSettings.p2pMaxSpeed = PTP_MAX_SPEED;
//    m_motorGeneralSettings.p2pSpeed = PTP_SPEED;
//    m_motorGeneralSettings.nsPerCyclePeriod = PERIOD_NS_CORE;

    m_motorGeneralSettings.homeMode = p_InitParam.homeMode;
    m_motorGeneralSettings.homeAccel = p_InitParam.homeAccel;
    m_motorGeneralSettings.homeLowSpeed = p_InitParam.homeLowSpeed;
    m_motorGeneralSettings.homeHighSpeed = p_InitParam.homeHighSpeed;
    m_motorGeneralSettings.p2pMaxSpeed = p_InitParam.p2pMaxSpeed;
    m_motorGeneralSettings.p2pSpeed = p_InitParam.p2pSpeed;
    m_motorGeneralSettings.nsPerCyclePeriod = p_InitParam.nsPerCyclePeriod;
    m_motorNum=p_InitParam.motorNum;
    m_startMotorID=p_InitParam.startMotorID;
    m_endMotorID=p_InitParam.endMotorID;
    for (int i = 0; i < p_InitParam.motorNum; i++)
    {
        m_HomeOffset[i] = p_InitParam.homeOffset[i];
    }



    /*************************************************************************
     * Here may not able use m_startMotorID and m_endMotorID
     * Previously EtherCAT position has been set
     * so here need a experiment
     * to avoid uncertainty,we use [0,m_motorNum)
     *************************************************************************/
    for(int i=0;i<m_motorNum;i++)
    {
        printf("m:%d\n",i);
        //returnValue = m_motor[i].Initialize(&m_pMaster,m_HomeOffset[i], m_motorGeneralSettings);
        returnValue = m_motor[i].Initialize(&m_pMaster,p_InitParam.homeOffset[i], m_motorGeneralSettings);
        if(returnValue!=0)
        {
            printf("Fail to initialize the motor driver No.%d\n", i);
            return returnValue;
        }
    }

    // Activate the master
    ecrt_master_activate(m_pMaster);

    printf("Device activated!\n");
    // Activate the slaves
//    m_ethercatCoupler.Activate();
//    for (int i = 0; i < m_analogInputsTerminalNum; i++)
//    {
//        returnValue = m_analogInputs[i].Activate();
//        if (returnValue != 0)
//        {
//            printf("Failed to activate Analog Input!\n");
//            return returnValue;
//        }
//    }
//    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
//    {
//        returnValue = m_analogOutputs[i].Activate();
//        if (returnValue != 0)
//        {
//            printf("Failed to activate Analog Output!\n");
//            return returnValue;
//        }
//    }

    for(int i = m_startMotorID; i < m_endMotorID; i++)
    {
	    returnValue = m_motor[i].Activate();
        if(returnValue != 0)
        {
            printf("Failed to activate Elmo driver!\n");
            return returnValue;
        }
    }
    m_isMasterActivated = true;

    return 0;
}

int CDeviceMaster::Read()
{

    ecrt_master_receive(m_pMaster);

    // Upload data from devices

//    for (int i = 0; i < m_analogInputsTerminalNum; i++)
//    {
//        m_analogInputs[i].Upload();
//        m_analogInputs[i].GetAnalogInputs(&(m_feedbackData.m_analogInputsData[i]));
//    }
//
//    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
//    {
//        m_analogOutputs[i].Upload();
//    }
    for(int i = m_startMotorID; i < m_endMotorID; i++)
    {
        m_motor[i].Upload();
        m_motor[i].GetData(&m_feedbackData.m_motorData[i]);
    }






    return 0;
}

int CDeviceMaster::Write()
{
//    for (int i = 0; i < m_analogInputsTerminalNum; i++)
//    {
//        m_analogInputs[i].Download();
//    }
//    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
//    {
//        m_analogOutputs[i].SetAnalogOutputs(m_commandData.m_analogOutputsData[i]);
//		// copy set analog output to the feedback data
//        m_feedbackData.m_analogOutputsData[i] = m_commandData.m_analogOutputsData[i];
//        m_analogOutputs[i].Download();
//    }
    for(int i = m_startMotorID; i < m_endMotorID; i++)
    {
        m_motor[i].SetData(m_commandData.m_motorData[i]);
        m_motor[i].DoCommand();
        m_motor[i].Download();
    }

    ecrt_master_send(m_pMaster);
    return 0;
}

int CDeviceMaster::CheckComplete()
{
    int numberLeft=0;
    for(int i=m_startMotorID;i<m_endMotorID;i++)
    {
        if(!m_motor[i].m_isCommandComplete)
        {
            numberLeft++;
        }
    }
    return numberLeft;
}

int CDeviceMaster::DeactiveMaster()
{

    ecrt_master_deactivate(m_pMaster);

    m_isMasterActivated = false;
    return 0;
}

int CDeviceMaster::DCSyncTime( uint64_t nanoSecond )
{
    ecrt_master_application_time(m_pMaster, nanoSecond);
    ecrt_master_sync_reference_clock(m_pMaster);
    ecrt_master_sync_slave_clocks(m_pMaster);
    return 0;
}



/*****************************************************************************
 *****************************************************************************
 * CSysBase
 *****************************************************************************
 *****************************************************************************/


RT_TASK CSysBase::m_realtimeCore;
//RT_TASK CSysBase::m_realtimeData;
//RT_QUEUE CSysBase::m_dataQueue;

CSysBase::CLogData CSysBase::m_logDataRC;
//CLogData CSysBase::m_logDataRD;
Aris::RT_CONTROL::CMachineData CSysBase::m_machineDataCore;

FuncPtrWork CSysBase::trajectoryGenerator=NULL;
FuncPtrInit CSysBase::initHandler=NULL;
FuncPtrWork CSysBase::dataUpdater=NULL;

RTIME CSysBase::m_timeStart;
long long int CSysBase::m_cycleCount;
Aris::RT_CONTROL::CSysInitParameters CSysBase::m_sysInitParam;
Aris::RT_CONTROL::EServoState CSysBase::m_currentStateRT;

CSysBase::ECoreState CSysBase::m_coreStateRT;

CDeviceMaster CSysBase::m_deviceMaster;

long long int CSysBase::m_h2rStartTime;
long long int CSysBase::m_h2rStartTimeRUN;





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



bool CSysBase::m_isCurrentCycleGetData;
CSysBase::EInternCmd CSysBase::m_currentInternCmdRT;
CSysBase::EInternCmd CSysBase::m_currentInternCmdNRT;
OPERATION_MODE CSysBase::m_operationMode;
bool CSysBase::m_isAllComplete;

CSysBase::EInternDataType CSysBase::m_currentDataTypeFromNRT;
CSysBase::EInternDataType CSysBase::m_currentDataTypeFromRT;

bool CSysBase::m_isDataFromRT;
Aris::RT_CONTROL::EServoState CSysBase::m_servoStateNRT;

RT_HEAP CSysBase::m_logHeap;
bool CSysBase::m_isLog;
long long int CSysBase::m_logCount;
void* CSysBase::m_logDataVoid;
CSysBase::CLogData* CSysBase::m_logData;
Aris::RT_CONTROL::CMachineData CSysBase::m_machineData;

pthread_t CSysBase::m_dataServer;
pthread_attr_t CSysBase::m_dataServer_attr;

pthread_mutex_t CSysBase::m_dataMutex=PTHREAD_MUTEX_INITIALIZER;

CSysBase::CLogData CSysBase::m_logDataBuffer;

struct sigaction CSysBase::m_sa;

FILE *CSysBase::fp;
Aris::RT_CONTROL::CMachineData CSysBase::m_logMachineDataBuffer;

bool CSysBase::m_isGetCustomDataCurrentCycle;

Aris::RT_CONTROL::EServoState CSysBase::m_stateTransitionMatrix[CMD_NUMBER][STATE_NUMBER];
bool CSysBase::m_isH2Red[ACTUAL_MOTOR_NUMBER];

Aris::RT_CONTROL::EServoState CSysBase::m_nextStateRT;


int CSysBase::m_homePosition[ACTUAL_MOTOR_NUMBER];
int CSysBase::m_h2rStartPosition[ACTUAL_MOTOR_NUMBER];

int CSysBase::m_h2rDeltaPosition=50;

CDeviceData CSysBase::m_standStillFrame;

CSysPID CSysBase::m_pid;



FuncPtrState CSysBase::OnPOWEROFF=NULL;
FuncPtrState CSysBase::OnPOED=NULL;
FuncPtrState CSysBase::OnSTOP=NULL;
FuncPtrState CSysBase::OnSTOPPED=NULL;
FuncPtrState CSysBase::OnENABLE=NULL;
FuncPtrState CSysBase::OnENABLED=NULL;
FuncPtrState CSysBase::OnHOMING=NULL;
FuncPtrState CSysBase::OnHOMED=NULL;
FuncPtrState CSysBase::OnH2RING=NULL;
FuncPtrState CSysBase::OnRUNNING=NULL;
FuncPtrState CSysBase::OnSTSTILL=NULL;
FuncPtrState CSysBase::OnEMERGENCY=NULL;
FuncPtrCustom CSysBase::OnCustomMsg=NULL;

Aris::RT_CONTROL::RT_MSG CSysBase::m_rtDataRecv;
Aris::RT_CONTROL::RT_MSG CSysBase::m_rtDataSend;
char CSysBase::m_rtDataRecvBuffer[RT_MSG_BUFFER_SIZE];
char CSysBase::m_rtDataSendBuffer[RT_MSG_BUFFER_SIZE];

CSysBase::CSysBase()
{
	CSysBase::m_coreStateRT=CSysBase::ECoreState::CS_UNINIT;
	CSysBase::m_isCurrentCycleGetData=false;
	CSysBase::m_operationMode=OM_NOMODE;
	CSysBase::m_currentInternCmdRT=CSysBase::EInternCmd::EIC_NONE;
	CSysBase::m_currentInternCmdNRT=CSysBase::EInternCmd::EIC_NONE;
	CSysBase::m_isDataFromRT=false;
	CSysBase::m_servoStateNRT=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;

	/*
	 * FOR TEST
	 */
	//CSysBase::m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;
	CSysBase::m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	CSysBase::m_nextStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;
	//change it after start
	CSysBase::m_isLog=true;
	CSysBase::m_logCount=0;
	CSysBase::m_isGetCustomDataCurrentCycle=false;
	CSysBase::H2RedClear();

	// may cause error
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		m_homePosition[i]=-HEXBOT_HOME_OFFSETS_RESOLVER[i];
	}

};

void CSysBase::H2RedClear()
{
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		m_isH2Red[i]=false;
	}

};

CSysBase::~CSysBase()
{
	if(m_coreStateRT==CSysBase::ECoreState::CS_STOPPED)
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

void CSysBase::CatchWarnUponSwitch(int signalNum, siginfo_t *signalInfo, void *context)
{
    rt_printf("Xenomai switch to secondary mod\n");
};

int CSysBase::SysInit(Aris::RT_CONTROL::CSysInitParameters p_Param)
{
	//backup sysInitParam
	CSysBase::m_sysInitParam = p_Param;
	CSysBase::m_sysInitParam.endMotorID = p_Param.endMotorID;
	CSysBase::m_sysInitParam.startMotorID = p_Param.startMotorID;
	CSysBase::m_sysInitParam.motorNum = p_Param.motorNum;
	CSysBase::m_sysInitParam.homeMode = p_Param.homeMode;
	CSysBase::m_sysInitParam.nsPerCyclePeriod = p_Param.nsPerCyclePeriod;

	for(int i=0;i<p_Param.motorNum;i++)
	{
		m_homePosition[i]=-p_Param.homeOffset[i];
	}



	if(CSysBase::initHandler!=NULL)
		CSysBase::initHandler(p_Param);
	else
		printf("SysInit:initHandler not set.\n");

	//init two RT_CONN_DATA
	InitRT_MSG();

	InitLogFile();

	stateMachineInit();

	int ret;
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
    printf("request master\n");
    ret = m_deviceMaster.Initialize(p_Param);
    if (ret != 0)
    {
        printf("Fail to initialize devices, please check your settings\n");
        return -1;
    }

    //Create Data queue
//    ret = rt_queue_create(&m_dataQueue,"Data Queue",8192,10,Q_FIFO);
//    if(!ret)
//    {
//    	printf("Queue created\n");
//    }
//    else
//    {
//    	printf("Queue creating failed!\n");
//    }
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
        	printf("Heap created.\n");
        	ret = rt_heap_alloc(&m_logHeap,
        			MAX_LOG_ENTRIES * sizeof(CLogData),TM_INFINITE,&m_logDataVoid);
        	if(ret<0)
        	{
        		printf("ERROR: log memory allocating failed.\n");
        	}
        	else
        	{
        		m_logData=(CLogData*)m_logDataVoid;
        		printf("LogData allocated.\n");
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

    // Create RT task
//    ret = rt_task_create(&m_realtimeData,"realtime data",0,TASK_PRIO_CORE,T_FPU);
//    if (ret != 0)
//    {
//        printf("Fail to create real-time task, please check your xenomai settings\n");
//        return -2;
//    }

    /*
     * Initial setup of m_dataServer
     */
    pthread_attr_init(&m_dataServer_attr);
    //pthread_attr_setschedpolicy(&m_dataServer_attr, SCHED_FIFO);

    m_coreStateRT = CS_INITIALIZED;
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
	else
		rt_printf("rc socket ok\n");


	struct timeval tv;

	tv.tv_sec = 0;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors

	ret=rt_dev_setsockopt(m_xddp_socket_rt, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	if (ret)
		fail("setsockopt--");
	else
		rt_printf("setsockopt ok\n");

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
	else
		rt_printf("setsockopt ok\n");



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
	else
		rt_printf("bind ok\n");

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
	else
		rt_printf("rd socket ok\n");

	tv.tv_sec = 0;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors

	ret=rt_dev_setsockopt(m_xddp_socket_data_rt, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	if (ret)
		fail("rd setsockopt--");
	else
		rt_printf("rd setsockopt ok\n");

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
	else
		rt_printf("rd setsockopt ok\n");

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
	else
		rt_printf("bind ok\n");

};


int CSysBase::SysInitCommunication()
{
	CSysBase::SysInitCommunicationRT();
    //Initialize XDDP NRT
	if (asprintf(&m_xddp_devname, "/dev/rtp%d", XDDP_PORT) < 0)
		printf("Error in asprintf\n");
	else
		printf("OK in asprintf NRT.\n");

	m_xddp_fd =open("/dev/rtp0", O_RDWR);
	free(m_xddp_devname);
	if (m_xddp_fd < 0)
		printf("RC Error in open %d\n",m_xddp_fd);
	else
		printf("RC OK in open NRT\n");
	//set to nonblock
//	int flags = fcntl(m_xddp_fd, F_GETFL, 0);
//	fcntl(m_xddp_fd, F_SETFL, flags | O_NONBLOCK);

	//set to block
	int flags = fcntl(m_xddp_fd, F_GETFL, 0);
	fcntl(m_xddp_fd, F_SETFL, flags &~ O_NONBLOCK);

	//Initialize XDDP_DATA_NRT
	if(asprintf(&m_xddp_devname_data,"/dev/rtp%d",XDDP_PORT_DATA)<0)
		printf("RD Error in asprintf\n");
	else
		printf("RD OK in asprintf NRT.\n");

	m_xddp_fd_data =open("/dev/rtp1", O_RDWR);
	free(m_xddp_devname_data);
	if (m_xddp_fd_data < 0)
		printf("RC Error in open %d\n",m_xddp_fd_data);
	else
		printf("RC OK in open NRT\n");
	//set to nonblock
//	flags = fcntl(m_xddp_fd_data, F_GETFL, 0);
//	fcntl(m_xddp_fd_data, F_SETFL, flags | O_NONBLOCK);

	//set to block
	flags = fcntl(m_xddp_fd, F_GETFL, 0);
	fcntl(m_xddp_fd, F_SETFL, flags &~ O_NONBLOCK);

    pthread_create(&m_dataServer,NULL,&dataServer,NULL);

	return 0;
};


void CSysBase::CatchStopSignal(int signalNum)
{
	m_coreStateRT = CS_STOPPED;
    rt_printf("Terminate signal catched, program will exit\n");
    SysStop();
};


int CSysBase::SysStart()
{
    m_coreStateRT = CS_STARTED;
    int ret;
    rt_printf("starting my_task\n");
    ret = rt_task_start(&m_realtimeCore,&RealtimeCore,NULL);
    //ret = rt_task_start(&m_realtimeData,&RealtimeData,NULL);
    rt_printf("task started\n");


    return 0;
};

int CSysBase::SysStop()
{
	m_coreStateRT = CS_STOPPED;

    rt_printf("Delete RT Task RealtimeCore\n");
    rt_task_delete(&m_realtimeCore);
    pthread_cancel(m_dataServer);
    fflush(fp);
    fclose(fp);
    m_isLog=false;
    m_deviceMaster.DeactiveMaster();
    rt_printf("System will stop\n");

    return 0;
};


bool CSysBase::IsSysStopped()
{
	if(m_coreStateRT==CS_STOPPED)
	{
		return true;
	}
	else
	{
		return false;
	}
};

void CSysBase::fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
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
	CSysBase::trajectoryGenerator=p_TrajectoryGenerator;
	return 0;
};

//int SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater)
//{
//	CSysBase::dataUpdater=p_DataUpdater;
//	return 0;
//};

/*
 * State Handler
 */
int CSysBase::SetOnPOWEROFF(FuncPtrState p_Handler)
{
	CSysBase::OnPOWEROFF=p_Handler;
	return 0;
};
int CSysBase::SetOnPOED(FuncPtrState p_Handler)
{
	CSysBase::OnPOED=p_Handler;
	return 0;
};

int CSysBase::SetOnSTOP(FuncPtrState p_Handler)
{
	CSysBase::OnSTOP=p_Handler;
	return 0;
};
int CSysBase::SetOnSTOPPED(FuncPtrState p_Handler)
{
	CSysBase::OnSTOPPED=p_Handler;
	return 0;
};

int CSysBase::SetOnENABLE(FuncPtrState p_Handler)
{
	CSysBase::OnENABLE=p_Handler;
	return 0;
};
int CSysBase::SetOnENABLED(FuncPtrState p_Handler)
{
	CSysBase::OnENABLED=p_Handler;
	return 0;
};

int CSysBase::SetOnHOMING(FuncPtrState p_Handler)
{
	CSysBase::OnHOMING=p_Handler;
	return 0;
};
int CSysBase::SetOnHOMED(FuncPtrState p_Handler)
{
	CSysBase::OnHOMED=p_Handler;
	return 0;
};

int CSysBase::SetOnH2RING(FuncPtrState p_Handler)
{
	CSysBase::OnH2RING=p_Handler;
	return 0;
};
int CSysBase::SetOnRUNNING(FuncPtrState p_Handler)
{
	CSysBase::OnRUNNING=p_Handler;
	return 0;
};

int CSysBase::SetOnSTSTILL(FuncPtrState p_Handler)
{
	CSysBase::OnSTSTILL=p_Handler;
	return 0;
};
int CSysBase::SetOnEMERGE(FuncPtrState p_Handler)
{
	CSysBase::OnEMERGENCY=p_Handler;
	return 0;
};

int CSysBase::SetOnCustomMessage(FuncPtrCustom p_Handler)
{
	CSysBase::OnCustomMsg=p_Handler;
	return 0;
};


/*
 * An alternative choice of directly writing to bin file
 */
/*int CSysBase::SysLogger()
{
	if(CSysBase::m_coreStateRT==CS_STOPPED)
	{

		 * close the file


		m_isLog=false;
	}
	if(m_isLog)
	{

		 * copy to heap

		//memcpy(&m_logData[m_logCount],&m_logDataRC,sizeof(m_logDataRC));

		 * write to the file


	}

	return 0;
};*/

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
	p_machineData.state=p_logData.m_servoState;
	p_machineData.time=p_logData.time;

	return 0;
};

int CSysBase::InitLogFile()
{
	printf("Init Log File.\n");
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
		printf("Log file opened.\n");
	}

	return 0;
}

void *CSysBase::dataServer(void* arg)
{
	printf("DataServer starting...\n");
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
			ret = read(m_xddp_fd_data,&m_logDataBuffer,sizeof(m_logDataBuffer));
//			printf("%d,%lld\n",ret,m_logDataBuffer.time);
			if(ret>0&&m_logCount<MAX_LOG_ENTRIES)
			{
				memcpy(&m_logData[m_logCount],&m_logDataBuffer,sizeof(m_logDataBuffer));
				RawLogDataToMachineData(m_logMachineDataBuffer,m_logDataBuffer);
				m_logCount++;
				/*
				 * Write to file
				 */
				fwrite(&m_logMachineDataBuffer,sizeof(m_logMachineDataBuffer),1,fp);
				
				if(m_logCount%1000==0)
				{
					printf("TIME:%lld %d POS:%d VEL%d POD:%d\n",m_cycleCount,
							m_logDataBuffer.m_feedbackData.m_motorData[0].StatusWord,
							m_logDataBuffer.m_feedbackData.m_motorData[0].Position,
							m_logDataBuffer.m_feedbackData.m_motorData[0].Velocity,
							m_logDataBuffer.m_commandData.m_motorData[0].Position);
				}

			}

		}
		while(ret>0);
		/*
		 * Copy data to shared variable with mutex
		 */
		pthread_mutex_lock(&m_dataMutex);
		memcpy(&m_machineData,&m_logMachineDataBuffer,sizeof(m_logMachineDataBuffer));
		pthread_mutex_unlock(&m_dataMutex);

		//Call Updater
		if(CSysBase::dataUpdater!=NULL)
		{
			CSysBase::dataUpdater(m_machineData);
		}


//		printf("______________________________________________________________\n");
//
//		printf("Time elapsed %ld.%ldms.\n",
//				(long)((timeNow-timePre)/1000000l),
//				(long)((timeNow-timePre)%1000000l));
		timePre=timeNow;

		/*
		 * jump out of the while loop so data file will be complete
		 */
		if(m_coreStateRT==CS_STOPPED)
		{
			break;
		}
		nanosleep(&tv,NULL);

	}
	return 0;

};




void CSysBase::RealtimeCore(void* arg)
{
	rt_printf("Realtime Core started.\n");

    RTIME timeNow;
    RTIME timePrevious;
    float period;

    rt_printf("RealtimeData entering loop\n");
    //rt_task_set_mode(0, T_WARNSW, NULL);
    rt_task_set_periodic(NULL, TM_NOW, PERIOD_NS_CORE);




    int ret=0;



	/*************************************************************************
	 * RT-Core Loop
	 *************************************************************************/
	m_cycleCount = 0;
    m_timeStart = rt_timer_read();

    //clear screen
    //rt_printf("\e[1;1H\e[2J");
    rt_printf("SysMonitor\n");


    while (m_coreStateRT == CS_STARTED) {

        rt_task_wait_period(NULL);


        timeNow = rt_timer_read();

        // receive ethercat**************************************************************###
        m_deviceMaster.Read();

        /**********************************************************************************************************************/
//        if ((m_cycleCount % 1)==0)
//        {
//            //rt_printf("Current time is %.3f\n", m_cycleCount/1000.0);
//            //rt_printf("Current time actually is %ld.%.6ld ms\n", ((long)(timeNow - timePrevious))/1000000, ((long)(timeNow - timePrevious)) % 1000000);
//
//            /*
//             * Get first message type of the current stage
//             * then swicth to different branch,which means when fake it in Aris_Control api
//             * we need to send a MSG type before actually send any data
//             *
//             * one cmd will be enough to change the state
//             */
//            switch (CSysBase::InternDataHandler())
//            {
//            case CSysBase::EInternDataType::EIMT_CMD:
//            	rt_printf("RT_Core: Get msg cmd\n");
//
//            	CSysBase::InternCmdGet();
//
//            	//CSysBase::InternCmdHandler(CSysBase::m_currentInternCmdRT);
//
//            	CSysBase::m_isGetCustomDataCurrentCycle=false;
//            	break;
//            case CSysBase::EInternDataType::EIMT_CUS:
//            	m_currentInternCmdRT=EInternCmd::EIC_CUS;
//            	CSysBase::m_isGetCustomDataCurrentCycle=true;
//
//            	rt_printf("RT_Core: CUSTOM,flag get!\n");
//
//            	break;
//            case CSysBase::EInternDataType::EIMT_NONE:
//            	//rt_printf("eimt_none\n");
//            	//should not modify it;
//            	m_currentInternCmdRT=EInternCmd::EIC_NONE;
//            	CSysBase::m_isGetCustomDataCurrentCycle=false;
//            	break;
//
//            default:
//            	//rt_printf("nothing\n");
//            	//should not modify it
//            	m_currentInternCmdRT=EInternCmd::EIC_NONE;
//            	CSysBase::m_isGetCustomDataCurrentCycle=false;
//            	break;
//            }
//        }
//
//
        /********************************************************************************************************************************/
//        if(m_cycleCount % 1000 ==0)
//        {
//        	rt_printf("Will send a message %d\n",m_cycleCount);
//        	int msg[3];
//        	msg[0]=sizeof(CSysBase::EInternCmd);
//        	msg[1]=CSysBase::EInternDataType::EMIT_STA;
//        	msg[2]=(int)Aris::Control::EServoState::EMSTAT_ENABLED;
//        	//rt_printf("&msg[2] %ld\n msg+2*sizeof(int): %ld\n 2*sizeof(int): %ld\n msg: %ld\n",&msg[2],msg+(long int)2*sizeof(int),2*sizeof(int),msg);
//        	CSysBase::RT_SendDataRaw(msg,2*sizeof(int));
//        	CSysBase::RT_SendDataRaw(&msg[2],sizeof(int));
//        }
        if(m_cycleCount%1==0)
        {
        	ret=RT_RecvDataRaw(CSysBase::m_rtDataRecvBuffer,RT_MSG_BUFFER_SIZE);
        	if(ret<0)
        	{
        		//*CSysBase::m_rtDataRecv.m_ptrDataType=(int)CSysBase::EInternDataType::EIMT_NONE;
        		CSysBase::m_rtDataRecv.SetType(CSysBase::EInternDataType::EIMT_NONE);
        	}
        	else
        	{
        		//rt_printf("%d!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",ret);
        	}
        	switch(CSysBase::m_rtDataRecv.GetType())
        	{
        	case (int)CSysBase::EInternDataType::EIMT_CMD:
        		//copy the cmd
        		rt_printf("RT_Core:Get cmd!\n");
        		RT_ReadData(&m_currentInternCmdRT,sizeof(m_currentInternCmdRT));
        		CSysBase::m_isGetCustomDataCurrentCycle=false;
        		break;
        	case (int)CSysBase::EInternDataType::EIMT_CUS:
        		m_currentInternCmdRT=EInternCmd::EIC_CUS;
        		CSysBase::m_isGetCustomDataCurrentCycle=true;
        		rt_printf("RT_Core: CUSTOM,flag get!\n");

        		break;
        	case (int)CSysBase::EInternDataType::EIMT_NONE:
        		m_currentInternCmdRT=EInternCmd::EIC_NONE;
        		CSysBase::m_isGetCustomDataCurrentCycle=false;
        		break;
        	default:
        		m_currentInternCmdRT=EInternCmd::EIC_NONE;
        		CSysBase::m_isGetCustomDataCurrentCycle=false;
        		break;
        	}
        }


        //rt_printf("\E[H");
        //rt_printf("SysMonitor\n");


        /*
         * Handle m_currentInternCmdRT m_servoStateRT  and the output of stateMachine
         */

        m_nextStateRT=CSysBase::EvalStateMachine(m_currentInternCmdRT,m_currentStateRT);



        if(m_nextStateRT==Aris::RT_CONTROL::EServoState::EMSTAT_INVALID)
        {
        	//cmd will be ignored;
//        	if(!(((int)m_currentInternCmdRT)==0))
//        		rt_printf("cmd number %d ignored\n",(int)m_currentInternCmdRT);
        	m_nextStateRT=m_currentStateRT;
        }
        else
        {
//        	if(!(((int)m_currentInternCmdRT)==0))
//        		rt_printf("cmd number %d get at state %d sm out:%d \n",
//        				(int)m_currentInternCmdRT,
//        				m_currentStateRT,
//        				m_nextStateRT);
        }
//		if(m_cycleCount%1000==0)
//			rt_printf("HomeCheck: %d\n",m_deviceMaster.CheckComplete());


        CSysBase::DoState(m_nextStateRT,m_cycleCount);

//        rt_printf("%lld %lld :: CMD:%d CW:%d SW:%d AP:%d\tTP:%d  M:%d DIFF %d\n",m_cycleCount,m_h2rStartTimeRUN,
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].MotorCmd.command,
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].ControlWord,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].StatusWord,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].Position,
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].Position,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].Mode,
//        							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].Position-
//        							CSysBase::m_deviceMaster.m_commandData.m_motorData[0].Position
//        							);

        //not here CSysBase::DoPID();

        // send process data*************************************************************###
        m_deviceMaster.DCSyncTime(rt_timer_read());
        m_deviceMaster.Write();
        timePrevious = timeNow;

        /*********************************************************************
         * SysLogger
         *********************************************************************/
//        char flag='D';
//        char FLAG='d';

        m_logDataRC.time=m_cycleCount;
        m_logDataRC.m_coreState=CSysBase::m_coreStateRT;
        m_logDataRC.m_servoState=CSysBase::m_currentStateRT;
        memcpy(&m_logDataRC.m_feedbackData,&CSysBase::m_deviceMaster.m_feedbackData,
        		sizeof(CSysBase::m_deviceMaster.m_feedbackData));
        memcpy(&m_logDataRC.m_commandData,&CSysBase::m_deviceMaster.m_commandData,
        		sizeof(CSysBase::m_deviceMaster.m_commandData));


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
			ret = rt_dev_sendto(m_xddp_socket_data_rt,&m_logDataRC,sizeof(m_logDataRC),0,NULL,0);
			if(ret==-12)
			{
				rt_printf("WARN:Internal communication buffer 2 is full.%lld\n",m_cycleCount);
			}

			//rt_printf("%d\n",ret);
			//reset flag
			//flag='D';
		}

		m_cycleCount++;
    }
};



/*****************************************************************************
 * Functions for transferring messages through xddp
 *****************************************************************************/

int CSysBase::NRT_GetMessageRaw(void* p_MsgBuffer,const unsigned int p_MsgBufSize)
{
	int ret;
	ret = read(m_xddp_fd,p_MsgBuffer,p_MsgBufSize);
	if(ret>0)
		printf("Get:%d\n",(int)ret);

	return ret;
};
int CSysBase::NRT_PostMessageRaw(const void* p_MsgPointer, const int p_MsgLength)
{
	int ret = 0;
	ret = write(m_xddp_fd,p_MsgPointer,p_MsgLength);
	return ret;
};
int CSysBase::RT_PostMessageRaw(const void* p_MsgPointer,const int p_MsgLength)
{
	int ret = 0;
	ret = rt_dev_sendto(m_xddp_socket_rt,p_MsgPointer,p_MsgLength,0,NULL,0);
	if(ret == p_MsgLength)
	{
		rt_printf("Sent %d\n",ret);
		return ret;
	}
	else
	{
		rt_printf("Post Message in RT-Core failed %d.\n",ret);
		return -1;
	}
};
int CSysBase::RT_GetMessageRaw(void* p_MsgBuffer,const int p_MsgBufSize)
{
	int ret = 0;
	ret = rt_dev_recvfrom(m_xddp_socket_rt,p_MsgBuffer,p_MsgBufSize,MSG_DONTWAIT,NULL,0);
	return ret;
};


int CSysBase::NRT_GetMessage(void* p_MsgBuffer,const unsigned int p_MsgBufSize)
{
	int ret = 0;
	if(CSysBase::m_currentDataTypeFromRT==CSysBase::EInternDataType::EIMT_CUS)
	{

		ret = rt_dev_recvfrom(m_xddp_socket_rt,p_MsgBuffer,p_MsgBufSize,MSG_DONTWAIT,NULL,0);
		return ret;
	}
	else
	{
		//do read loop until read a good type -- nah, don't do anything waste time

		return 0;

	}
};
//int CSysBase::RT_GetMessage(void* p_MsgBuffer,const int p_MsgBufSize);

int CSysBase::NRT_PrePostCustomMessage()
{
	int ret = 0;
	CSysBase::EInternDataType type= CSysBase::EInternDataType::EIMT_CUS;
	ret = write(m_xddp_fd,&type,sizeof(type));
	return ret;

};
int CSysBase::RT_PrePostCustomMessage()
{
	int ret = 0;
	CSysBase::EInternDataType type= CSysBase::EInternDataType::EIMT_CUS;
	ret = rt_dev_sendto(m_xddp_socket_rt,&type,sizeof(type),0,NULL,0);
	if(ret == sizeof(type))
	{
		rt_printf("Sent %d\n",ret);
		return ret;
	}
	else
	{
		rt_printf("Post Message in RT-Core failed.\n");
		return -1;
	}

};

CSysBase::EInternDataType CSysBase::InternDataHandler()
{
	//we test it here,then clean the function
	CSysBase::EInternDataType msgType;
	int ret;
	ret=CSysBase::RT_GetMessageRaw(&msgType,sizeof(CSysBase::EInternDataType));
	//rt_printf("Handler %d\n",ret);
	if(ret<0)
	{
		CSysBase::m_isCurrentCycleGetData=false;
		msgType=CSysBase::EInternDataType::EIMT_NONE;
	}
	else
	{
		CSysBase::m_isCurrentCycleGetData=true;
	}
	return msgType;
};

int CSysBase::InternCmdGet()
{
	int ret;
	ret=CSysBase::RT_GetMessageRaw(&CSysBase::m_currentInternCmdRT,sizeof(CSysBase::EInternCmd));
	rt_printf("Get %d\n",ret);
	return ret;
};


int CSysBase::NRT_MCPowerOff()
{
	int ret;
	CSysBase::EInternCmd cmd = CSysBase::EInternCmd::EIC_POWEROFF;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EIMT_CMD;
	ret = CSysBase::NRT_PostMessageRaw(&type,sizeof(type));
	ret = CSysBase::NRT_PostMessageRaw(&cmd,sizeof(cmd));
	return ret;
};
int CSysBase::NRT_MCHome()
{
	int ret;
	CSysBase::EInternCmd cmd = CSysBase::EInternCmd::EIC_HOME;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EIMT_CMD;
	ret = CSysBase::NRT_PostMessageRaw(&type,sizeof(type));
	ret = CSysBase::NRT_PostMessageRaw(&cmd,sizeof(cmd));
	return ret;
};
int CSysBase::NRT_MCHomeToRunning()
{
	int ret;
	CSysBase::EInternCmd cmd = CSysBase::EInternCmd::EIC_HOME2RUN;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EIMT_CMD;
	ret = CSysBase::NRT_PostMessageRaw(&type,sizeof(type));
	ret = CSysBase::NRT_PostMessageRaw(&cmd,sizeof(cmd));
	return ret;
};
int CSysBase::NRT_MCEnable()
{
	int ret;
	CSysBase::EInternCmd cmd = CSysBase::EInternCmd::EIC_ENABLE;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EIMT_CMD;
	ret = CSysBase::NRT_PostMessageRaw(&type,sizeof(type));
	ret = CSysBase::NRT_PostMessageRaw(&cmd,sizeof(cmd));
	return ret;
};
int CSysBase::NRT_MCStop()
{
	int ret;
	CSysBase::EInternCmd cmd = CSysBase::EInternCmd::EIC_STOP;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EIMT_CMD;
	ret = CSysBase::NRT_PostMessageRaw(&type,sizeof(type));
	ret = CSysBase::NRT_PostMessageRaw(&cmd,sizeof(cmd));
	return ret;
};

int CSysBase::NRT_MCData(Aris::RT_CONTROL::CMachineData& p_machineData)
{
//	int ret;
//	CSysBase::EInternCmd cmd = CSysBase::EInternCmd::EIC_ACQDATA;
//	CSysBase::EInternMsgType type = CSysBase::EInternMsgType::EIMT_CMD;
//	ret = CSysBase::NRT_PostMessageRaw(&type,sizeof(type));
//	ret = CSysBase::NRT_PostMessageRaw(&cmd,sizeof(cmd));
//	return ret;
//	int ret;
//	char flag='d';
//	ret = write(m_xddp_fd_data,&flag,sizeof(flag));
//	ret = read(m_xddp_fd_data,&p_machineData,sizeof(p_machineData));
	pthread_mutex_lock(&m_dataMutex);
	memcpy(&p_machineData,&m_machineData,sizeof(m_machineData));
	pthread_mutex_unlock(&m_dataMutex);

	return 0;
};

Aris::RT_CONTROL::EServoState CSysBase::NRT_MCMachineState()
{
	pthread_mutex_lock(&m_dataMutex);
	memcpy(&m_servoStateNRT,&m_machineData.state,sizeof(Aris::RT_CONTROL::EServoState));
	pthread_mutex_unlock(&m_dataMutex);
	return m_servoStateNRT;

};


int CSysBase::RT_MCAxisFeedPos(int p_AxisId, int p_Pos)
{
	if(m_operationMode!=OM_CYCLICPOS)
	{
		rt_printf("ERROR:Invalid feed parameter, mode is not match!\n");
		return -1;
	}
	else
	{
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].Position=p_Pos;
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].MotorCmd.command=CMD_RUNNING;
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].MotorCmd.operationMode=OM_CYCLICPOS;
		return 0;
	}

};
int CSysBase::RT_MCAxisFeedVel(int p_AxisId, int p_Vel)
{
	if(m_operationMode!=OM_CYCLICVEL)
	{
		rt_printf("ERROR:Invalid feed parameter, mode is not match!\n");
		return -1;
	}
	else
	{
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].Velocity=p_Vel;
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].MotorCmd.command=CMD_RUNNING;
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].MotorCmd.operationMode=OM_CYCLICVEL;
		return 0;
	}

};

int CSysBase::RT_MCAxisFeedTor(int p_AxisId, short p_Tor)
{
	if(m_operationMode!=OM_CYCLICTORQ)
	{
		rt_printf("ERROR:Invalid feed parameter, mode is not match!\n");
		return -1;
	}
	else
	{
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].Velocity=p_Tor;
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].MotorCmd.command=CMD_RUNNING;
		CSysBase::m_deviceMaster.m_commandData.m_motorData[p_AxisId].MotorCmd.operationMode=OM_CYCLICTORQ;
		return 0;
	}
};



int CSysBase::SetModeP2P()
{
	m_operationMode=OM_PROFILEPOS;
	return 0;
};
int CSysBase::SetModeCycPos()
{
	m_operationMode=OM_CYCLICPOS;
	return 0;
};
int CSysBase::SetModeCycVel()
{
	m_operationMode=OM_CYCLICVEL;
	return 0;
};
int CSysBase::SetModeCycTor()
{
	m_operationMode=OM_CYCLICTORQ;
	return 0;
};

/*****************************************************************************
 * NRT XDDP 消息处理的辅助API
 *****************************************************************************/

bool CSysBase::NRT_CheckMessage()
{
	int ret;
	// get msg type first
	ret = CSysBase::NRT_GetMessageRaw(&CSysBase::m_currentDataTypeFromRT,sizeof(CSysBase::m_currentDataTypeFromRT));
	if(ret==sizeof(CSysBase::m_currentDataTypeFromRT))
	{
		printf("NRT get msg from RT.\n");

		return true;
	}
	else
	{

		CSysBase::m_currentDataTypeFromRT=CSysBase::EInternDataType::EIMT_NONE;
		return false;
	}
};
bool CSysBase::NRT_IsSysMsg()
{
	if(CSysBase::m_currentDataTypeFromRT==CSysBase::EInternDataType::EMIT_STA)
		return true;
	else
		return false;
};
bool CSysBase::NRT_IsCusMsg()
{
	if(CSysBase::m_currentDataTypeFromRT==CSysBase::EInternDataType::EIMT_CUS)
		return true;
	else
		return false;

};

//// RT maybe not need do this because it will not be flooded by msgs
//// one cycle one messege session is ok
//bool CSysBase::RT_CheckMessage()
//{
//
//};
//bool CSysBase::RT_IsSysMsg()
//{
//
//};
bool CSysBase::RT_IsCusMsg()
{
	return CSysBase::m_isGetCustomDataCurrentCycle;

};



Aris::RT_CONTROL::EServoState CSysBase::NRT_GetSysStateFromRT()
{
	CSysBase::NRT_GetMessageRaw(&CSysBase::m_servoStateNRT,sizeof(CSysBase::m_servoStateNRT));
	return CSysBase::m_servoStateNRT;
};
//Aris::Control::EServoState CSysBase::NRT_GetCurrentState()
//{
//	return CSysBase::m_servoStateNRT;
//};

/*****************************************************************************
 * State Machine
 *****************************************************************************/
int CSysBase::stateMachineInit()
{
	//matrix[cmd][state]=state
	//currently 8 cmd 9 state in all

	/*******************************************************************************************************/
	//EMSTATE_NONE
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_NONE;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_NONE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTATE_INVALID
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_INVALID]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_POWEROFF
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_POED
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_POED;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_POED;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_POED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_STOP
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_STOP]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_STOPPED
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_ENABLE
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_ENABLED
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_HOMING
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMING;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_HOMED this state is auto transferred from HOMING
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_HOMED;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_H2RING;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_HOMED]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_H2RING this state will automaticlly change to EMSTAT_RUNNING
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_H2RING;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_H2RING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_RUNNING
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_STSTILL manually transfered to running
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	/*******************************************************************************************************/
	//EMSTAT_EMERGE
	m_stateTransitionMatrix[EIC_NONE	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE;
	m_stateTransitionMatrix[EIC_POWEROFF][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF;
	m_stateTransitionMatrix[EIC_STOP	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_STOP;
	m_stateTransitionMatrix[EIC_ENABLE	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE;
	m_stateTransitionMatrix[EIC_RUNNING	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;

	m_stateTransitionMatrix[EIC_HOME	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_HOME2RUN][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_CUS		][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_SSTILL	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;
	m_stateTransitionMatrix[EIC_EMERGE	][Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE]=Aris::RT_CONTROL::EServoState::EMSTAT_INVALID;


	return 0;
};

Aris::RT_CONTROL::EServoState CSysBase::EvalStateMachine(EInternCmd p_cmd,const Aris::RT_CONTROL::EServoState p_state)
{
	return m_stateTransitionMatrix[p_cmd][p_state];

};

void CSysBase::PrintStateChange(Aris::RT_CONTROL::EServoState p_orig,Aris::RT_CONTROL::EServoState p_aim)
{
	rt_printf("Aris_Control: state changed from %s to %s.\n",ServoStateName[p_orig],ServoStateName[p_aim]);
};

int CSysBase::PostStateToNRT(const Aris::RT_CONTROL::EServoState p_state)
{
	int ret;
	CSysBase::EInternDataType type = CSysBase::EInternDataType::EMIT_STA;
//	ret = CSysBase::RT_PostMessageRaw(&type,sizeof(type));
//	ret = CSysBase::RT_PostMessageRaw(&p_state,sizeof(p_state));
	CSysBase::m_rtDataSend.SetType(type);
	CSysBase::m_rtDataSend.SetLength(sizeof(p_state));
	CSysBase::m_rtDataSend.Copy(&p_state,sizeof(p_state));
	//send in two steps *****************************************************TBD!!!!!!!!!!!!!!!!
	CSysBase::RT_SendDataRaw(CSysBase::m_rtDataSend.m_ptrData,RT_MSG_HEADER_LENGTH);
	CSysBase::RT_SendDataRaw(CSysBase::m_rtDataSend.GetDataAddress(),CSysBase::m_rtDataSend.GetLength());


	return 0;
};


int CSysBase::DeviceDataToMachineData(const CDeviceData p_dData,Aris::RT_CONTROL::CMachineData &p_mData)
{
	//copy feedback data
	//cannot simply memcpy, because they have different definition
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		p_mData.feedbackData[i].Position = p_dData.m_motorData[i].Position;
		p_mData.feedbackData[i].Velocity = p_dData.m_motorData[i].Velocity;
		p_mData.feedbackData[i].Torque	 = p_dData.m_motorData[i].Torque;

	}
	p_mData.time=CSysBase::m_cycleCount;

	return 0;
};
int CSysBase::MachineDataToDeviceData(const Aris::RT_CONTROL::CMachineData p_mData,CDeviceData &p_dData)
{
	//copy command data
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		p_dData.m_motorData[i].Position = p_mData.commandData[i].Position;
		p_dData.m_motorData[i].Velocity = p_mData.commandData[i].Velocity;
		p_dData.m_motorData[i].Torque   = p_mData.commandData[i].Torque;

	}
	return 0;
};

void CSysBase::DoState(Aris::RT_CONTROL::EServoState& p_state,long long int p_time)
{
	 switch(p_state)
	        {


	 	 	 	 //OK**************************************************************
	        case Aris::RT_CONTROL::EServoState::EMSTAT_NONE:
	        	if(p_state!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				p_state);
	        		m_currentStateRT=p_state;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		//then do nothing,actually no other state will change to EMSTAT_NONE
	        	}
	        	else
	        	{
	        		//then do nothing,actually no other state will change to EMSTAT_NONE
	        	}
	        	break;


	        	//OK**************************************************************
	        case Aris::RT_CONTROL::EServoState::EMSTAT_INVALID:
	        	/*****************************************************************
	        	 * will never reach here in current code
	        	 *****************************************************************/
	        	if(p_state!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				p_state);
	        		m_currentStateRT=p_state;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);


	        		/* Action of the state */

	        	}
	        	else
	        	{

	        	}
	        	break;

	        	//OK**************************************************************
	        case Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF:
	        	if(p_state!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				p_state);
	        		m_currentStateRT=p_state;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		if(CSysBase::m_deviceMaster.CheckComplete()==0)
	        		{
	            		CSysBase::PrintStateChange(m_currentStateRT,
	            				Aris::RT_CONTROL::EServoState::EMSTAT_POED);
	            		m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_POED;
	        		}
	        	}
	        	break;

	        	//OK**************************************************************
	        case Aris::RT_CONTROL::EServoState::EMSTAT_POED:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		/*
	        		 * If the state has an automatic transition
	        		 * do the transition
	        		 */
	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_STOP:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		/*
	        		 * If the state has an automatic transition
	        		 * do the transition
	        		 */
	        		if(CSysBase::m_deviceMaster.CheckComplete()==0)
	        		{
	            		CSysBase::PrintStateChange(m_currentStateRT,
	            				Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED);
	            		CSysBase::PostStateToNRT(Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED);
	            		m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED;
	        		}
	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);


	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		/*
	        		 * If the state has an automatic transition
	        		 * do the transition
	        		 */
	        		if(p_time%500==0)
	        		{
	        			rt_printf("CheckComplete %d\n",CSysBase::m_deviceMaster.CheckComplete());
	        		}
	        		if(CSysBase::m_deviceMaster.CheckComplete()==0)
	        		{
	            		CSysBase::PrintStateChange(m_currentStateRT,
	            				Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED);
	            		CSysBase::PostStateToNRT(Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED);
	            		m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED;
	        		}
	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		/*
	        		 * If the state has an automatic transition
	        		 * do the transition
	        		 */
	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_HOMING:
	        	//rt_printf("%d start %d end\n",CSysBase::m_sysInitParam.startMotorID,CSysBase::m_sysInitParam.endMotorID);
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		rt_printf("%d start %d end\n",CSysBase::m_sysInitParam.startMotorID,CSysBase::m_sysInitParam.endMotorID);
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

    				/*
    				 * Clear flag
    				 */
    				H2RedClear();
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		if(CSysBase::m_deviceMaster.CheckComplete()==0)
	        		{

	            		CSysBase::PrintStateChange(m_currentStateRT,
	            				Aris::RT_CONTROL::EServoState::EMSTAT_HOMED);
	            		CSysBase::PostStateToNRT(Aris::RT_CONTROL::EServoState::EMSTAT_HOMED);
	            		m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_HOMED;

	        			rt_printf("Change to homed %d\n",CSysBase::m_deviceMaster.m_feedbackData.m_motorData[0].StatusWord);
	        		}

	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_HOMED:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_H2RING://TBD########################################################
	        	//rt_printf("????????????????? %d\t%d\n",m_nextStateRT,m_currentStateRT);
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		rt_printf("I give m_h2rStartTime Value!\n");
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Initialize the state
	        		 */
	        		m_h2rStartTime=CSysBase::m_cycleCount;
	        		rt_printf("I give m_h2rStartTime Value!\n");
	        		//direction
	        		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
	        		{
	        			m_h2rStartPosition[i]=CSysBase::m_deviceMaster.m_feedbackData.m_motorData[i].Position;
	        		}

	        		/*
	        		 * Action of the state
	        		 */

	        		ActionOfState(m_currentStateRT,p_time);

	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        		//check if complete
	        		//change to run state,and normally running state not modify the CDeviceMaster.commanddata
	        		//so it will not change the motion
	        		int numberLeft=CSysBase::m_sysInitParam.endMotorID-CSysBase::m_sysInitParam.startMotorID;
	        		//rt_printf("Number Left %d\t%d\n",CSysBase::m_sysInitParam.endMotorID,CSysBase::m_sysInitParam.startMotorID);
	        		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
	        		{
	        			if (m_isH2Red[0])
	        			{
	        				numberLeft--;
	        				rt_printf("Number Left %d\n",numberLeft);
	        			}
	        		}

	        		if(numberLeft==0)
	        		{
	        			/*
	        			 * change state
	        			 */


	        			CSysBase::PrintStateChange(m_currentStateRT,
	        					Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING);
	        			CSysBase::PostStateToNRT(Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING);
	        			m_currentStateRT=Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING;
	        		}
	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING:
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL://TBD############################################################
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		//get a of current motor cmd
	        		memcpy(&m_standStillFrame,&CSysBase::m_deviceMaster.m_commandData,sizeof(CSysBase::m_deviceMaster.m_commandData));
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        	}
	        	break;
	        case Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE://TBD#############################################################
	        	if(m_nextStateRT!=m_currentStateRT)
	        	{
	        		CSysBase::PrintStateChange(m_currentStateRT,
	        				m_nextStateRT);
	        		m_currentStateRT=m_nextStateRT;

	        		/*
	        		 * Post a state to NRT
	        		 */
	        		CSysBase::PostStateToNRT(m_currentStateRT);

	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);
	        	}
	        	else
	        	{
	        		/*
	        		 * Action of the state
	        		 */
	        		ActionOfState(m_currentStateRT,p_time);

	        	}
	        	break;
	        default:
	        	/*
	        	 * Literally can be here
	        	 */
	        	rt_printf("Holy crap! How can you get here?\n");
	        	break;
	        }
};


int CSysBase::ActionOfState(Aris::RT_CONTROL::EServoState& p_state,long long int p_time)
{
	switch(p_state)
	{
	case Aris::RT_CONTROL::EServoState::EMSTAT_POWEROFF:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_POWEROFF;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}


		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_POED:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_POWEROFF;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}

		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_STOP:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_STOP;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}


		break;

	case Aris::RT_CONTROL::EServoState::EMSTAT_STOPPED:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_STOP;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}

		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_ENABLE:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_ENABLE;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}


		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_ENABLED:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_ENABLE;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}
		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_HOMING:
		//rt_printf("%d start %d end\n",CSysBase::m_sysInitParam.startMotorID,CSysBase::m_sysInitParam.endMotorID);
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_RUNNING;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=OM_HOMING;
		}


		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_HOMED:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_RUNNING;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=OM_HOMING;
		}

		break;

	case Aris::RT_CONTROL::EServoState::EMSTAT_H2RING:
		/*
		 * Interesting part TBD
		 */
		/*
		 *Some initialize need to be done in DoState
		 */
		//rt_printf("m_h2rStartTime %lld\n",m_h2rStartTime);
		if(p_time-m_h2rStartTime<100)
		{
			for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
			{
				CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_ENABLE;
				CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=OM_HOMING;
				m_h2rStartPosition[i]=CSysBase::m_deviceMaster.m_feedbackData.m_motorData[i].Position;
				CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position=m_h2rStartPosition[i];
				m_h2rStartTimeRUN=p_time;
			}
		}

		else
		{


			for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
			{
				if(p_time%500==0)
					rt_printf("HOMEOFFSET %d\n",CSysBase::m_homePosition[i]);
				CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_RUNNING;
				CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
				if(CSysBase::m_homePosition[i]-m_deviceMaster.m_feedbackData.m_motorData[i].Position
						>3*m_h2rDeltaPosition)
				{
					CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position=
							m_h2rStartPosition[i]+((int)(p_time-m_h2rStartTimeRUN))*m_h2rDeltaPosition;
//					rt_printf("%lld %lld + SPOS:%d POS:%d\tCMD:%d  T:%d\n",p_time,m_h2rStartTimeRUN,
//							m_h2rStartPosition[i],
//							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[i].Position,
//							CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position,
//							CSysBase::m_homePosition[i]);
					if(m_isH2Red[i])
					{
						CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position=CSysBase::m_homePosition[i];
					}
				}
				else if(CSysBase::m_homePosition[i]-m_deviceMaster.m_feedbackData.m_motorData[i].Position
						<-3*m_h2rDeltaPosition)
				{
					CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position=
							m_h2rStartPosition[i]-((int)(p_time-m_h2rStartTimeRUN))*m_h2rDeltaPosition;
//					rt_printf("%lld %lld- POS:%d\tCMD:%d  T:%d\n",p_time,m_h2rStartTimeRUN,
//							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[i].Position,
//							CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position,
//							CSysBase::m_homePosition[i]);

					if(m_isH2Red[i])
					{
						CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position=CSysBase::m_homePosition[i];
					}
				}
				else
				{

					CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position=CSysBase::m_homePosition[i];
//					rt_printf("%lld = POS:%d\tCMD:%d  T:%d\n",p_time,
//							CSysBase::m_deviceMaster.m_feedbackData.m_motorData[i].Position,
//							CSysBase::m_deviceMaster.m_commandData.m_motorData[i].Position,
//							CSysBase::m_homePosition[i]);
					m_isH2Red[i]=true;
				}


			}



		}
		/*
		 * As this will move axis, DoPID
		 */
		CSysBase::RT_DoPID();
		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_RUNNING:
		for(int i=CSysBase::m_sysInitParam.startMotorID;i<CSysBase::m_sysInitParam.endMotorID;i++)
		{
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.command=CMD_RUNNING;
			CSysBase::m_deviceMaster.m_commandData.m_motorData[i].MotorCmd.operationMode=m_operationMode;
		}
		if(CSysBase::trajectoryGenerator!=NULL)
		{
			CSysBase::DeviceDataToMachineData(m_deviceMaster.m_feedbackData,m_machineDataCore);
			CSysBase::trajectoryGenerator(m_machineDataCore);
			//rt_printf("In RUNNING %d\n",m_machineDataCore.commandData[0].Position);
			CSysBase::MachineDataToDeviceData(m_machineDataCore,m_deviceMaster.m_commandData);
		}

		CSysBase::RT_DoPID();
		break;

	case Aris::RT_CONTROL::EServoState::EMSTAT_STSTILL:
		/*
		 * Best place to STSTILL is in TrajectoryGenerator
		 * but here we will offer a simplified version of STSTILL
		 * which after this state cannot change to running state
		 * only ENABLE, STOP, POWEROFF, etc. can be used
		 *
		 * revision
		 * it can be changed to
		 */
		memcpy(&CSysBase::m_deviceMaster.m_commandData,&m_standStillFrame,sizeof(m_standStillFrame));
		break;
	case Aris::RT_CONTROL::EServoState::EMSTAT_EMERGE:
		/*
		 *  not fully discussed yet
		 */
		break;
	default:
		//rt_printf("%c[2K", 27);
		rt_printf("WARN:Undefined internal command receieved.\n");
		break;
	}
	return 0;
}
void CSysBase::RT_DoPID()
{
	/*
	 * called in running state
	 * velocity loop verified before
	 * torque loop unusable
	 */
	m_pid.DoPID(m_deviceMaster.m_feedbackData.m_motorData,m_deviceMaster.m_commandData.m_motorData);
}


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


//
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

int CSysBase::RT_ReadData(void* p_ptrData,const int p_dataLength)
{
	int ret;
	//simple operation,just copy data from m_rtRecvDataBuffer;
	//of cause first two int size memory is not the target data
	memcpy(p_ptrData,(m_rtDataRecv.m_ptrData+RT_MSG_HEADER_LENGTH),p_dataLength);
	return ret;
};
int CSysBase::RT_UpdateData()
{
	int ret;
	//recv to the buffer
	if(m_rtDataRecvBuffer==NULL)
	{
		rt_printf("RT_UpdateData: RT Data Receiving Buffer Pointer has not set\n");
		return -14;
	}
	//ret = rt_dev_recvfrom(m_xddp_socket_rt,m_rtDataRecvBuffer,RT_CONN_DATA_BUFFER_SIZE,MSG_DONTWAIT,NULL,0);
	ret=RT_RecvDataRaw(m_rtDataRecvBuffer,RT_MSG_BUFFER_SIZE);

	return ret;
};

