#ifndef REALTIME_CORE_H
#define REALTIME_CORE_H

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
#include <rtdk.h>
#include <ecrt.h>

#include "DeviceMaster.h"
#include "GlobalConfiguration.h"

namespace ElmoMotor{

class CRealtimeCore
{
public:
    CRealtimeCore();
    ~CRealtimeCore();

    static int Initialize();
    static int Start();
    static int Stop();
    

    static int GetMessage();
    static int PostMessage();
     
    enum CORE_STATE
    {
        CS_UNINIT      = 0,
        CS_INITIALIZED = 1,
        CS_STARTED     = 2,
        CS_STOPPED     = 3,
    };

    static CORE_STATE m_coreState;
private:

    static RT_TASK m_realtimeTask;
    static CDeviceMaster m_deviceMaster;
    static RTIME m_timeStart;
    static int m_cycCount;
    static struct sigaction m_sa;
     //RT_PIPE msgPipe;

    static void CatchWarnUponSwitch(int signalNum, siginfo_t *signalInfo, void *context);
    static void CatchStopSignal(int signalNum);

    static void RealtimeTaskHandler(void *arg);
};

}

#endif













