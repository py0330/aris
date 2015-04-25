#ifndef DEVICEMASTER_H
#define DEVICEMASTER_H

#include <stdio.h>
#include <sys/mman.h>
#include <native/task.h>
#include <rtdk.h>

#include "ecrt.h"
#include "GlobalConfiguration.h"
#include "hardware.h"

namespace ElmoMotor{

class CDeviceData
{
public:
    CElmoMotorData     m_motorData[ACTUAL_MOTOR_NUMBER];
    CAnalogInputsData  m_analogInputsData[ACTUAL_EL3104_NUM];
    CAnalogOutputsData m_analogOutputsData[ACTUAL_EL4002_NUM];    
};


// Operates all devices, including the Ethercat master, the motor drivers, analog/digital IO devices and the IMU 
class CDeviceMaster
{
public:
    CDeviceMaster(void);
    ~CDeviceMaster(void);

    int Initialize();

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

}

#endif // DEVICEMASTER_H
