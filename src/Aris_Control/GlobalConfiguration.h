#ifndef ELMOMOTORCONFIGURATION_H
#define ELMOMOTORCONFIGURATION_H


/// Output prefix
#define MAIN        "Main:"
#define SERVER      "Server:"


// Task frequency
#define FREQUENCY_MAIN  25
#define FREQUENCY_CORE  1000

// Task time period
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS_MAIN (NSEC_PER_SEC * 1.0 / FREQUENCY_MAIN)
#define PERIOD_NS_CORE (NSEC_PER_SEC * 1.0 / FREQUENCY_CORE)

///////////////////////////////////////////////////////////
// Hardware Global Settings                              //
///////////////////////////////////////////////////////////
// Actuation number settings
#define ACTUAL_MOTOR_NUMBER 18

// HomeOffsets, for resolver
//static const int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
//{   -895506, -852986, -895530,
//    -893893, -852967, -896492,
//    -895547, -853462, -895931,
//    -896141, -854825, -895228,
//    -895048, -855014, -894660,
//    -893680, -853916, -894525};

//-15849882	 -16354509	 -16354509
//-15849882	 -16354509	 -16354509
//-15849882	 -16354509	 -16354509
//-15849882	 -16354509	 -16354509
//-15849882	 -16354509	 -16354509
//-15849882	 -16354509	 -16354509
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

// SDO configuration
#define HOMING_HI_SPEED   2560
#define HOMING_LO_SPEED   (0.1*HOMING_HI_SPEED)
#define HOMING_MODE       17
#define HOMING_ACC        (10*HOMING_HI_SPEED)
#define PTP_SPEED         1792
#define PTP_MAX_SPEED     2560

//MOTOR SPEED LIMIT
#define MAX_MOTOR_SPEED		4960.0/60.0
#define COUNTS_PER_ROUND	4096.0
#define MAX_SPEED			(int)MAX_MOTOR_SPEED*COUNTS_PER_ROUND

#endif // GLOBALCONFIGURATION_H
