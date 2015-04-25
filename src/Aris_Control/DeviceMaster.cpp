#include "DeviceMaster.h"

namespace ElmoMotor{

ec_master_t* CDeviceMaster::m_pMaster = NULL;
ec_master_state_t CDeviceMaster::m_masterState = {};

CDeviceMaster::CDeviceMaster(void)
{
    m_motorNum = ACTUAL_MOTOR_NUMBER;
    m_analogInputsTerminalNum = ACTUAL_EL3104_NUM;
    m_analogOutputsTerminalNum = ACTUAL_EL4002_NUM;
    m_isMasterRequested = false;
    m_isMasterActivated = false;

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
int CDeviceMaster::Initialize()
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
    m_ethercatCoupler.SetEtherCATPosition(ECAT_POS_COUPLER);

    for (int i = 0; i < m_analogInputsTerminalNum; i++)
    {
        m_analogInputs[i].SetEtherCATPosition(ECAT_START_POS_EL3104 + i);
    }
    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
    {
        m_analogOutputs[i].SetEtherCATPosition(ECAT_START_POS_EL4002 + i);
    }
    for(int i = 0; i < m_motorNum; i++)
    {
        m_motor[i].SetEtherCATPosition(ECAT_START_POS_ELMODRIVE + i);
    }

    //initialize all the slaves
    returnValue = m_ethercatCoupler.Initialize(&m_pMaster);
    if(returnValue != 0)
    {
        printf("Fail to initialize the bus coupler\n");
        return returnValue;
    }

    for (int i = 0; i < m_analogInputsTerminalNum; i++)
    {
        returnValue = m_analogInputs[i].Initialize(&m_pMaster);
        if(returnValue != 0)
        {
            printf("Fail to initialize the analog inputs No.%d\n", i);
            return returnValue;
        }
    }
    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
    {
        returnValue = m_analogOutputs[i].Initialize(&m_pMaster);
        if(returnValue != 0)
        {
            printf("Fail to initialize the analog outputs No.%d\n", i);
            return returnValue;
        }
    }

    // assign general settings of the motor
    m_motorGeneralSettings.homeMode = HOMING_MODE;
    m_motorGeneralSettings.homeAccel = HOMING_ACC;
    m_motorGeneralSettings.homeLowSpeed = HOMING_LO_SPEED;
    m_motorGeneralSettings.homeHighSpeed = HOMING_HI_SPEED;
    m_motorGeneralSettings.p2pMaxSpeed = PTP_MAX_SPEED;
    m_motorGeneralSettings.p2pSpeed = PTP_SPEED;
    m_motorGeneralSettings.nsPerCyclePeriod = PERIOD_NS_CORE;
    for(int i=0;i<m_motorNum;i++)
    {
        printf("m:%d\n",i);
        returnValue = m_motor[i].Initialize(&m_pMaster,m_HomeOffset[i], m_motorGeneralSettings);
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
    m_ethercatCoupler.Activate();
    for (int i = 0; i < m_analogInputsTerminalNum; i++)
    {
        returnValue = m_analogInputs[i].Activate();
        if (returnValue != 0)
        {
            printf("Failed to activate Analog Input!\n");
            return returnValue;
        }
    }
    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
    {
        returnValue = m_analogOutputs[i].Activate();
        if (returnValue != 0)
        {
            printf("Failed to activate Analog Output!\n");
            return returnValue;
        }
    }

    for(int i = 0; i < m_motorNum; i++)
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
    
    for (int i = 0; i < m_analogInputsTerminalNum; i++)
    {
        m_analogInputs[i].Upload();
        m_analogInputs[i].GetAnalogInputs(&(m_feedbackData.m_analogInputsData[i]));
    }
    
    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
    {
        m_analogOutputs[i].Upload();
    }
    for(int i = 0; i < m_motorNum; i++)
    {
        m_motor[i].Upload();
        m_motor[i].GetData(&m_feedbackData.m_motorData[i]);
    }
     





    return 0;
}

int CDeviceMaster::Write()
{
    for (int i = 0; i < m_analogInputsTerminalNum; i++)
    {
        m_analogInputs[i].Download();
    }
    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
    {
        m_analogOutputs[i].SetAnalogOutputs(m_commandData.m_analogOutputsData[i]);
		// copy set analog output to the feedback data
        m_feedbackData.m_analogOutputsData[i] = m_commandData.m_analogOutputsData[i];
        m_analogOutputs[i].Download();
    }
    for(int i = 0; i < m_motorNum; i++)
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
    for(int i=0;i<m_motorNum;i++)
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
}

int CDeviceMaster::DCSyncTime( uint64_t nanoSecond )
{
    ecrt_master_application_time(m_pMaster, nanoSecond);
    ecrt_master_sync_reference_clock(m_pMaster);
    ecrt_master_sync_slave_clocks(m_pMaster);
    return 0;
}

}
