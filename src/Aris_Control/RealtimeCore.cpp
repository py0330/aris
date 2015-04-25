#include "RealtimeCore.h"

using namespace ElmoMotor;

struct sigaction CRealtimeCore::m_sa;
int CRealtimeCore::m_cycCount;
RT_TASK CRealtimeCore::m_realtimeTask;
CDeviceMaster CRealtimeCore::m_deviceMaster;
CRealtimeCore::CORE_STATE CRealtimeCore::m_coreState;
RTIME CRealtimeCore::m_timeStart;

CRealtimeCore::CRealtimeCore()
{
    m_coreState = CRealtimeCore::CS_UNINIT;
};

CRealtimeCore::~CRealtimeCore()
{
    if ( m_coreState != CS_STOPPED)
        Stop();
    
};

int CRealtimeCore::Initialize()
{
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
    ret = m_deviceMaster.Initialize();
    if (ret != 0)
    {
        printf("Fail to initialize devices, please check your settings\n");
        return -1;
    }

    // Create RT task
    ret = rt_task_create(&m_realtimeTask,"realtime task",0,TASK_PRIO_CORE,T_FPU);
    if (ret != 0)
    {
        printf("Fail to create real-time task, please check your xenomai settings\n");
        return -2;
    }
    m_coreState = CS_INITIALIZED;
    return 0;
};


int CRealtimeCore::Start()
{
    int ret;
    rt_printf("starting my_task\n");
    ret = rt_task_start(&m_realtimeTask,&RealtimeTaskHandler,NULL);
    rt_printf("task started\n");
    m_coreState = CS_STARTED;
};

int CRealtimeCore::Stop()
{
    rt_printf("Delete RT Task\n");
    rt_task_delete(&m_realtimeTask);

    m_deviceMaster.DeactiveMaster();
    m_coreState = CS_STOPPED;
};

void CRealtimeCore::CatchWarnUponSwitch(int signalNum, siginfo_t *signalInfo, void *context)
{
    rt_printf("Xenomai switch to secondary mod\n");
};
void CRealtimeCore::CatchStopSignal(int signalNum)
{
    rt_printf("Terminate signal catched, program will exit\n");
    Stop();
};

void CRealtimeCore::RealtimeTaskHandler(void *arg)
{
    int ret;

    RTIME timeNow;
    RTIME timePrevious;
    float period;

    rt_printf("Entering Loop\n");
    //rt_task_set_mode(0, T_WARNSW, NULL);
    rt_task_set_periodic(NULL, TM_NOW, PERIOD_NS_CORE);
    m_cycCount = 0;
    m_timeStart = rt_timer_read();   
    while (m_coreState == CS_STARTED) {
	
        rt_task_wait_period(NULL);
        m_cycCount++;
        
        timeNow = rt_timer_read();
        
        // receive ethercat
        m_deviceMaster.Read();
        
        if ((m_cycCount % 10)==0)
        {
            rt_printf("Current time is %.3f\n", m_cycCount/1000.0);
            rt_printf("Current time actually is %ld.%.6ld ms\n", ((long)(timeNow - timePrevious))/1000000, ((long)(timeNow - timePrevious)) % 1000000);
        }
        
        // send process data
        m_deviceMaster.DCSyncTime(rt_timer_read());
        m_deviceMaster.Write();
        timePrevious = timeNow;
    }
};
