#include "ElmoMotorDriver.h"
#include "rtdk.h"

using namespace ElmoMotor;

CElmoMotor::CElmoMotor()
{
    //home
    m_homeOffset=0;
    m_homeSwitchMask=0x01;
    m_homeProcessFlag=0;
    m_standStillProcessFlag=0;
}

CElmoMotor::~CElmoMotor()
{
}

//when using array, this function is responsible for correctly setting the Ethercat configuration
int CElmoMotor::SetEtherCATPosition(
        uint16_t position,
        uint32_t product_code,
        uint32_t vender_id,
        uint16_t alias)
{

    // Set the driver's position in ethercat network
    m_position=position;
    m_product_code=product_code;
    m_vender_id=vender_id;
    m_alias=alias;

    // Register ethercat domain for this motor driver
    m_domainAllRegs[0]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x607A,0x00,&m_offsetElmoTargetPosition};
    m_domainAllRegs[1]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x60FF,0x00,&m_offsetElmoTargetVelocity};
    m_domainAllRegs[2]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6071,0x00,&m_offsetElmoTargetTorque};
    m_domainAllRegs[3]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6072,0x00,&m_offsetElmoMaxTorque};
    m_domainAllRegs[4]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6040,0x00,&m_offsetElmoControlWord};
    m_domainAllRegs[5]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6060,0x00,&m_offsetElmoModeOfOperation};

    m_domainAllRegs[6]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6064,0x00,&m_offsetElmoPositionActualValue};
    m_domainAllRegs[7]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x60fd,0x00,&m_offsetElmoDigitalInputs};
    m_domainAllRegs[8]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x606c,0x00,&m_offsetElmoVelocityActualValue};
    m_domainAllRegs[9]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6041,0x00,&m_offsetElmoStatusWord};
    m_domainAllRegs[10]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6078,0x00,&m_offsetElmoCurrentActualValue};
    m_domainAllRegs[11]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6077,0x00,&m_offsetElmoTorqueActualValue};
    m_domainAllRegs[12]=(ec_pdo_entry_reg_t){m_alias,m_position,m_vender_id,m_product_code,0x6061,0x00,&m_offsetElmoModeOfOperationDisplay};

    // Register the PDO entries for this motor driver
    m_slave_1605_PdoEntries[0] = (ec_pdo_entry_info_t){0x607A,0x00,32};//target position
    m_slave_1605_PdoEntries[1] = (ec_pdo_entry_info_t){0x60FF,0x00,32};//target velocity
    m_slave_1605_PdoEntries[2] = (ec_pdo_entry_info_t){0x6071,0x00,16};//target torque
    m_slave_1605_PdoEntries[3] = (ec_pdo_entry_info_t){0x6072,0x00,16};//max torque
    m_slave_1605_PdoEntries[4] = (ec_pdo_entry_info_t){0x6040,0x00,16};//control word
    m_slave_1605_PdoEntries[5] = (ec_pdo_entry_info_t){0x6060,0x00, 8};//mode of operation

    m_slave_1a03_PdoEntries[0] = (ec_pdo_entry_info_t){0x6064,0x00,32};//position actual value
    m_slave_1a03_PdoEntries[1] = (ec_pdo_entry_info_t){0x60fd,0x00,32};//digital inputs
    m_slave_1a03_PdoEntries[2] = (ec_pdo_entry_info_t){0x606c,0x00,32};//velocity actual value
    m_slave_1a03_PdoEntries[3] = (ec_pdo_entry_info_t){0x6041,0x00,16};//status word

    m_slave_1a0b_PdoEntries[0] = (ec_pdo_entry_info_t){0x6061, 0x00, 8};// mode of operation display

    m_slave_1a13_PdoEntries[0] = (ec_pdo_entry_info_t){0x6077,0x00,16};//torque actual value

    m_slave_1a1f_PdoEntries[0] = (ec_pdo_entry_info_t){0x6078,0x00,16};//current acutal value

    // Register PDOs for the PDO entries just registered
    m_slaveTxPdos[0] = (ec_pdo_info_t){0x1A03,4,m_slave_1a03_PdoEntries+0};
    m_slaveTxPdos[1] = (ec_pdo_info_t){0x1A1F,1,m_slave_1a1f_PdoEntries+0};
    m_slaveTxPdos[2] = (ec_pdo_info_t){0x1A13,1,m_slave_1a13_PdoEntries+0};
    m_slaveTxPdos[3] = (ec_pdo_info_t){0x1A0B,1,m_slave_1a0b_PdoEntries+0};

    m_slaveRxPdos[0] = (ec_pdo_info_t){0x1605,6,m_slave_1605_PdoEntries+0};

    // Assign sync masters for the PDOs just registered
    m_slaveSyncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, 1, m_slaveRxPdos + 0, EC_WD_ENABLE};
    m_slaveSyncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT, 4, m_slaveTxPdos + 0, EC_WD_ENABLE};
    m_slaveSyncs[4] = (ec_sync_info_t){0xff};

    return 0;
}


int CElmoMotor::Initialize(ec_master_t **p_master,int homeOffset, SMotorGeneralSettings& settings)
{
    m_pMaster=*p_master;
    
    printf("2\n");
    m_pDomainAll = ecrt_master_create_domain(m_pMaster);

    if (!m_pDomainAll)
    {
        printf("create domain failed!\n");
        return -3;
    }

    printf("3\n");
    printf("v:%x:%x\n",(int)m_vender_id,m_product_code);

    // Get the slave configuration 
    if (!(m_slaveConfigElmo = ecrt_master_slave_config(
                        m_pMaster, m_alias,m_position,m_vender_id,m_product_code)))
    {
        printf("Failed to get slave 0 configuration.\n");
        return -4;
    }

    // Configure initial parameters by writing SDO
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6098,0,settings.homeMode);     //homing mode
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x609A,0,settings.homeAccel);      //homing acc
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6099,1,settings.homeHighSpeed); //high speed
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6099,2,settings.homeLowSpeed); //low speed which we don't use here
    
//    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6095,1,1);
//    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6095,2,1);
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6096,1,1);   // Ratio of Position/Velocity, Numerator
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6096,2,1);   // Ratio of Position/Velocity, Divisor


    //set ptp max profile speed
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x607F,0,settings.p2pMaxSpeed);
    //set ptp profile speed
    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x6081,0,settings.p2pSpeed);

    ecrt_slave_config_sdo32(m_slaveConfigElmo,0x607C,0,homeOffset);

    //SDO operation initialize list param
    printf("4\n");

    // Configure the slave's PDOs and sync masters
    if (ecrt_slave_config_pdos(m_slaveConfigElmo, 4, m_slaveSyncs))
    {
        // handle error
        printf("Error slave config PDOs!\n");
        return -5;
    }
    printf("5\n");

    // Configure the slave's domain
    if (ecrt_domain_reg_pdo_entry_list(m_pDomainAll, m_domainAllRegs))
    {
        printf("PDO entry registration failed!\n");
        return -6;
    }
    printf("6\n");

    // Configure the slave's discrete clock
    ecrt_slave_config_dc(m_slaveConfigElmo, 0x0300, settings.nsPerCyclePeriod, 4400000, 0, 0);
    printf("OK!\n");

    return 0;
}


int ElmoMotor::CElmoMotor::Activate()
{
    m_pDomainAllPd = ecrt_domain_data(m_pDomainAll);
    if (!m_pDomainAllPd)
    {
        return -10;
    }
    return 0;
}


int CElmoMotor::SetData(const CElmoMotorData& data)
{
    m_motorOrder.ControlWord   = data.ControlWord;
    m_motorOrder.MaxTorque     = data.MaxTorque;
    m_motorOrder.Mode          = data.Mode;

    m_motorOrder.Position      = data.Position;
    m_motorOrder.Torque        = data.Torque;
    m_motorOrder.Velocity      = data.Velocity;

    m_motorOrder.IsCmdComplete = data.IsCmdComplete;
    m_motorOrder.MotorCmd      = data.MotorCmd;
    //status current DigitalInput
    m_motorOrder.StatusWord    = data.StatusWord;
    m_motorOrder.Current       = data.Current;
    m_motorOrder.DigitalInputs = data.DigitalInputs;

    // Admit the PTP request, IsPTPRequest will be automatically
    // reset to false when motor's PTP process started
    if ( data.IsPTPRequested )  
    {
        m_motorOrder.IsPTPRequested = data.IsPTPRequested;
    }
    return 0;
}

int CElmoMotor::GetData(CElmoMotorData *p_data) const
{

    p_data->Current         = m_motorData.Current;
    p_data->DigitalInputs   = m_motorData.DigitalInputs;
    p_data->MaxTorque       = m_motorData.MaxTorque;
    p_data->Mode            = m_motorData.Mode;

    p_data->ControlWord     = m_motorData.ControlWord;
    p_data->StatusWord      = m_motorData.StatusWord;

    p_data->Position        = m_motorData.Position;
    p_data->Torque          = m_motorData.Torque;
    p_data->Velocity        = m_motorData.Velocity;
    p_data->MotorCmd        = m_motorData.MotorCmd;
    p_data->IsCmdComplete   = m_motorData.IsCmdComplete;

    return 0;
}


int CElmoMotor::ReadData(CElmoMotorData *p_data) const
{
    p_data->Position      = EC_READ_S32(m_pDomainAllPd + m_offsetElmoPositionActualValue);
    p_data->Velocity      = EC_READ_S32(m_pDomainAllPd + m_offsetElmoVelocityActualValue);
    p_data->Torque        = EC_READ_S16(m_pDomainAllPd + m_offsetElmoTorqueActualValue);
    p_data->Current       = EC_READ_S16(m_pDomainAllPd + m_offsetElmoCurrentActualValue);
    p_data->StatusWord    = EC_READ_U16(m_pDomainAllPd + m_offsetElmoStatusWord);
    p_data->Mode          = EC_READ_U8(m_pDomainAllPd + m_offsetElmoModeOfOperationDisplay);
    p_data->DigitalInputs = EC_READ_U32(m_pDomainAllPd + m_offsetElmoDigitalInputs);
    return 0;
}

int CElmoMotor::WriteData(const CElmoMotorData& data)
{
    EC_WRITE_S32(m_pDomainAllPd + m_offsetElmoTargetPosition,  data.Position);
    EC_WRITE_S16(m_pDomainAllPd + m_offsetElmoTargetTorque,    data.Torque);
    EC_WRITE_S32(m_pDomainAllPd + m_offsetElmoTargetVelocity,  data.Velocity);
    EC_WRITE_S16(m_pDomainAllPd + m_offsetElmoMaxTorque,       data.MaxTorque);
    EC_WRITE_U16(m_pDomainAllPd + m_offsetElmoControlWord,     data.ControlWord);
    EC_WRITE_U8 (m_pDomainAllPd + m_offsetElmoModeOfOperation, data.Mode);

    return 0;

}

int CElmoMotor::GoHome()
{
    SUB_STATE       currentSubstate = GetSubState();
    OPERATION_MODE  currentMode     = GetCurrentMode();
    MOTOR_STATE     currentState    = GetState();

    if ( currentSubstate == SUB_BUSY)
        return -1;

    if ( currentState != STA_ENABLED )
        return -2;

    m_motorCommand.command = CMD_RUNNING;
    m_motorCommand.operationMode = OM_HOMING;
    m_isHomingRequested = 1;

    m_motorOrder.Mode=6;//homing mode

    if ( m_isHomingRequested && currentState == STA_RUNNING && currentMode == OM_HOMING)
        m_motorOrder.ControlWord = 0x1F;   //require Homing, this will cause motor shifts to SUB_BUSY state
    if ( currentMode == OM_HOMING && m_motorOrder.ControlWord == 0x1F && (currentSubstate == SUB_BUSY || currentSubstate == SUB_FINISHED) )
        m_isHomingRequested = 0;  // when last request being executed, clear request flag
    return 0;

}

int CElmoMotor::SwitchMode(int mode)
{
    m_motorOrder.Mode=mode;
    return 0;
}

void CElmoMotor::Upload()
{
    ecrt_domain_process(m_pDomainAll);
    ReadData(&m_motorData);
}

void CElmoMotor::Download()
{
    WriteData(m_motorOrder);
    ecrt_domain_queue(m_pDomainAll);
}

int CElmoMotor::GetPositionActualValue() const
{
    return m_motorData.Position;
}
int CElmoMotor::GetVelocityActualValue() const
{
    return m_motorData.Velocity;
}
int CElmoMotor::GetTorqueActualValue() const
{
    return m_motorData.Torque;
}
int CElmoMotor::GetCurrentActualValue() const
{
    return m_motorData.Current;
}
int CElmoMotor::GetStatusWord() const
{
    return m_motorData.StatusWord;
}

SUB_STATE CElmoMotor::GetSubState() const
{
    uint motorState = m_motorData.StatusWord;
    OPERATION_MODE currentMode = GetCurrentMode();
    if (currentMode == OM_PROFILEPOS)   // motor in PTP mode
    {
        if ( (motorState & 0x1000) == 0 )   // the ACK bit is not set
            return SUB_READY;
        else if ( (motorState & 0x0400) == 0 ) // the TARGET REACHED bit is not set
            return SUB_BUSY;
        else
            return SUB_FINISHED;
    }
    else if ( currentMode == OM_HOMING )
    {
        if ( motorState & 0x1000 )   // the HOME ATTAINED bit is set
            return SUB_FINISHED;
        else if ( (motorState & 0x0400) == 0 ) // the TARGET REACHED bit is not set
            return SUB_BUSY;
        else
            return SUB_READY;
    }

    return SUB_READY;
}

MOTOR_STATE CElmoMotor::GetState() const
{
    int motorState = ((m_motorData.StatusWord) & 0x000F);
    switch (motorState)
    {
    case 0x0000:
        return STA_POWEREDOFF;
    case 0x0001:
        return STA_STOPPED;
    case 0x0003:
        return STA_ENABLED;
    case 0x0007:
        return STA_RUNNING;
    default:	  // Motor has fault
        return STA_FAULT;
    }
}


OPERATION_MODE CElmoMotor::GetCurrentMode() const
{
    int opMode = (m_motorData.Mode);
        switch (opMode)
        {
        case 1:
            return OM_PROFILEPOS;
        case 6:
            return OM_HOMING;
        case 8:
            return OM_CYCLICPOS;
        case 9:
            return OM_CYCLICVEL;
        case 10:
            return OM_CYCLICTORQ;
        }
        return OM_OTHER;
}

int CElmoMotor::GetDigitalInput() const
{
    return m_motorData.DigitalInputs;
}


void CElmoMotor::SetTargetPosition(int32_t position)
{
    m_motorOrder.Position=position;
}
void CElmoMotor::SetTargetVelocity(int32_t velocity)
{
    m_motorOrder.Velocity=velocity;
}
void CElmoMotor::SetTargetTorque(int16_t torque)
{
    m_motorOrder.Torque=torque;
}
void CElmoMotor::SetControlWord(int controlword)
{
    m_motorOrder.ControlWord=controlword;
}
void CElmoMotor::SetMode(int mode)
{
    m_motorOrder.Mode=mode;
}
void CElmoMotor::SetMaxTorque(int16_t max_torque)
{
    m_motorOrder.MaxTorque=max_torque;
}


int CElmoMotor::CheckState()//CheckState Each Cycle
{
    MOTOR_STATE currentState = GetState();
    if( currentState == STA_FAULT )
    {
        m_motorOrder.ControlWord=0x80;
        return -1;
    }
    else
    {
        return 0;
    }
}


int CElmoMotor::GetHomeSwitchFlag() const //judge if the motor has reached HOME
{
    if(((m_motorData.DigitalInputs) & m_homeSwitchMask) == 0)
            return 0;
        else
            return 1;
}

void CElmoMotor::SetOffset(int offset) // set the offset of the starting position at the beginning
{
    m_homeOffset=offset;
}


void CElmoMotor::Enable()
{

    m_isPositioningRequested = 0;
    m_isHomingRequested = 0;

    m_motorOrder.MotorCmd.command        =   CMD_ENABLE;
    m_motorOrder.MotorCmd.operationMode  =   OM_STANDSTILL;

}
void CElmoMotor::Stop()
{
    m_isPositioningRequested = 0;
    m_isHomingRequested = 0;

    m_motorOrder.MotorCmd.command        =   CMD_STOP;
    m_motorOrder.MotorCmd.operationMode  =   OM_STANDSTILL;

}
void CElmoMotor::Poweroff()
{
    m_isPositioningRequested = 0;
    m_isHomingRequested = 0;

    m_motorOrder.MotorCmd.command        =   CMD_POWEROFF;
    m_motorOrder.MotorCmd.operationMode  =   OM_STANDSTILL;

}

int CElmoMotor::DoCommand()
{
    int rc = 0;
    m_motorData.IsCmdComplete=false;
    m_motorOrder.IsCmdComplete=false;
    m_isCommandComplete = false;
    int stateComplete = 0;
    int opModeComplete = 0;

    MOTOR_STATE currentState = GetState();
    OPERATION_MODE currentMode = GetCurrentMode();
    SUB_STATE currentSubstate = GetSubState();

    // Motor has fault, require motor power off, and report error
    if( currentState == STA_FAULT )
    {
        m_motorOrder.ControlWord = 0x80;  // Reset the motor
        //m_motorCommand.command = CMD_STOP; // Require motor stop
        return -1;
    }


    switch(m_motorOrder.MotorCmd.command)
    {
    case CMD_POWEROFF:
        m_motorOrder.ControlWord = 0x00;  // require power off immediately
        if ( currentState == STA_POWEREDOFF )
        {
            m_motorOrder.ControlWord = 0x00;//the OrderActuationFromAgent is the clone of OrderAgentToActuation
                                            //which we just give cmd,so cw is created here;
            stateComplete = 1;
        }

        break;

    case CMD_STOP:
        m_motorOrder.ControlWord = 0x06;  // require stop
        if ( currentState == STA_STOPPED )
        {
            m_motorOrder.ControlWord = 0x06;
            stateComplete = 1;
        }
        break;

    case CMD_ENABLE:
        if ( currentState == STA_POWEREDOFF)
            m_motorOrder.ControlWord = 0x06;  // require STOPPED state firstly
        else if( currentState == STA_STOPPED || currentState == STA_RUNNING )
            m_motorOrder.ControlWord = 0x07;  // require ENABLE state

        if ( currentState == STA_ENABLED )
        {
            m_motorOrder.ControlWord = 0x07;
            stateComplete = 1;
        }
        break;

    case CMD_RUNNING:
        if ( currentState == STA_POWEREDOFF )
            m_motorOrder.ControlWord = 0x06;  // require STOPPED state firstly
        else if ( currentState == STA_STOPPED )
            m_motorOrder.ControlWord = 0x07;  // require ENABLE state firstly
        else if ( currentState == STA_ENABLED )
            m_motorOrder.ControlWord = 0x0F;  // require RUNNING state

        if ( currentState == STA_RUNNING )
        {
            m_motorOrder.ControlWord=0x0F;
            stateComplete = 1;
        }
    }


    switch(m_motorOrder.MotorCmd.operationMode)
    {
    case OM_CYCLICPOS:
        m_motorOrder.Mode = 8;
        m_motorOrder.Position += m_homeOffset;
        if(m_standStillProcessFlag==1) m_standStillProcessFlag=0;
        break;

    case OM_CYCLICVEL:
        m_motorOrder.Mode = 9;
        if(m_standStillProcessFlag==1) m_standStillProcessFlag=0;
        break;

    case OM_CYCLICTORQ:
        m_motorOrder.Mode = 10;
        m_motorOrder.MaxTorque = 30000; // Max torque is 30% in this case
        if(m_standStillProcessFlag==1) m_standStillProcessFlag=0;
        break;

    case OM_PROFILEPOS:
        m_motorOrder.Mode = 1;
        
        if ( m_motorOrder.IsPTPRequested && currentState == STA_RUNNING && currentMode == OM_PROFILEPOS )
        {
            if ( currentSubstate == SUB_READY ){
                m_motorOrder.ControlWord = 0x3F;      //require PTP Moving, this will cause motor shifts to SUB_BUSY state
            }
            else if ( currentSubstate == SUB_FINISHED ){
                m_motorOrder.ControlWord = 0x2F;	  //require SUB_READY state
            }
            else if (currentSubstate == SUB_BUSY){
                m_motorOrder.IsPTPRequested = false;  // when last request being executed, clear request flag
            }
        }
        if(m_standStillProcessFlag==1) m_standStillProcessFlag=0;

        break;

    case OM_HOMING:
        m_motorOrder.Mode = 6;
        if (currentState == STA_RUNNING && currentMode == OM_HOMING)
            m_motorOrder.ControlWord = 0x1F;

        if(m_standStillProcessFlag==1) m_standStillProcessFlag=0;

        break;

    case OM_STANDSTILL:
        m_motorOrder.Mode = 8;
        if(m_standStillProcessFlag==0)
        {
            m_standStillPosition = m_motorData.Position;
            //printf(ACTUATION"pos for ss:%d\t%d\n",m_motorOrder.Position,m_motorData.Position);
            m_standStillProcessFlag=1;
            m_motorOrder.Position=m_standStillPosition;
        }
        m_motorOrder.Position=m_standStillPosition;

        break;
    }

    if ( (currentMode == m_motorOrder.MotorCmd.operationMode) ||
         (m_motorOrder.MotorCmd.operationMode == OM_STANDSTILL && currentMode == OM_CYCLICPOS))
        opModeComplete = 1;

    if ( opModeComplete && stateComplete )
    {
//        printf("%d\n",m_motorData.StatusWord);
        m_motorOrder.IsCmdComplete = true;
        m_motorData.IsCmdComplete = true;
        m_isCommandComplete=true;
        if(currentMode == OM_HOMING)
        {
            if(!(m_motorData.StatusWord & 1<<12)) // Home is not reached yet
            {
                m_motorData.IsCmdComplete=false;
                m_motorOrder.IsCmdComplete = false;
                m_isCommandComplete=false;
//                printf("not reach\n");
            }
            else
            {
                m_homeProcessFlag=0;
            }
        }
        if(currentMode == OM_PROFILEPOS &&
           ! (m_motorData.StatusWord & 1<<10))  // Target not reached
        {
            m_motorData.IsCmdComplete=false;
            m_motorOrder.IsCmdComplete = false;
            m_isCommandComplete=false;
        }
    }
    else
    {
        m_motorData.IsCmdComplete=false;
        m_motorOrder.IsCmdComplete = false;
        m_isCommandComplete=false;
    }

    return 0;  // No error
}
