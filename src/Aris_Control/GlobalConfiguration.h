#ifndef ELMOMOTORCONFIGURATION_H
#define ELMOMOTORCONFIGURATION_H


/// Output prefix
#define MAIN        "Main:"
#define SERVER      "Server:"


// Task frequency
//#define FREQUENCY_MAIN  25
//#define FREQUENCY_CORE  1000

///////////////////////////////////////////////////////////
// Hardware Global Settings                              //
///////////////////////////////////////////////////////////

// HomeOffsets, for resolver
static const int HEXBOT_HOME_OFFSETS_RESOLVER[18] = 		
{
		-15849882,	 -16354509,	 -16354509,
		-15849882,	 -16354509,	 -16354509,
		-15849882,	 -16354509,	 -16354509,
		-16354509,	 -15849882,	 -16354509,
		-15849882,	 -16354509,	 -16354509,
		-16354509,	 -16354509,  -15849882
};

// HomeOffsets, for encoder
static const int HEXBOT_HOME_OFFSETS_ENCODER[18]={
    -3498069,-3331976,-3498164,
    -3491771,-3331902,-3501923,
    -3498232,-3333834,-3499731,
    -3490937,-3335608,-3494240,
    -3496280,-3339897,-3494764,
    -3500551,-3339162,-3496986};
		
// IO number settings
#define ACTUAL_EL3104_NUM 2
#define ACTUAL_EL4002_NUM 1

// Devices' positions on the Ethercat network
#define ECAT_POS_COUPLER 18
#define ECAT_START_POS_EL3104  (ECAT_POS_COUPLER + 1)
#define ECAT_START_POS_EL4002  (ECAT_POS_COUPLER + 3)
#define ECAT_START_POS_ELMODRIVE  0
// Motor Default Operation Mode

#define DEFAULT_OPERATION_MODE 9

/////////////////////////////////////////////////////////////

#define LOG_INTERVAL 10  //log data every LOG_INTERVAL cycle
#define LOG_MINUTES  20

//changed by xyl 20141120
#define MAX_LOG_ENTRIES  (FREQUENCY_CORE*LOG_MINUTES*60/LOG_INTERVAL)

#define DATA_SIZE sizeof(CLogData)//byte of your data

#define HEAP_SIZE (MAX_LOG_ENTRIES*DATA_SIZE*ACTUAL_MOTOR_NUMBER)

// Task settings
#define ALARM_VALUE     500000 //First shot at now + 500 us
#define ALARM_INTERVAL  250000 //Period is 250 us TM_INFINITE for one shot

#define TASK_PRIO_CORE 99

//socket

#define MAXPENDING 5    /* Max connection requests */
#define SOCKET_BUFFER_SIZE 32
//IPC
#define PIPE_MINOR 0

//output msg
#define OUTPUT 0

#endif // GLOBALCONFIGURATION_H
