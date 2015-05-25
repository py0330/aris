#include "Aris_Device.h"
#include "rtdk.h"

using namespace ElmoMotor;


const Aris::RT_CONTROL::EServoState CElmoMotor::m_stateMachine[Aris::RT_CONTROL::N_MOTOR_STATE][Aris::RT_CONTROL::N_MOTOR_CMD]=

//  CMD_NONE,                                      CMD_POWEROFF,                           CMD_STOP,                       CMD_ENABLE,                       CMD_RUNNING,                    CMD_GOHOME

  {Aris::RT_CONTROL::EMSTAT_NONE      ,Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_ENABLED,Aris::RT_CONTROL::EMSTAT_RUNNING,Aris::RT_CONTROL::EMSTAT_INVALID, //STA_NONE
   Aris::RT_CONTROL::EMSTAT_POWEREDOFF,Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_ENABLED,Aris::RT_CONTROL::EMSTAT_RUNNING,Aris::RT_CONTROL::EMSTAT_HOMING, //STA_POWEREDOFF
   Aris::RT_CONTROL::EMSTAT_STOPPED,   Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_ENABLED,Aris::RT_CONTROL::EMSTAT_RUNNING,Aris::RT_CONTROL::EMSTAT_HOMING, //STA_STOPPED
   Aris::RT_CONTROL::EMSTAT_ENABLED   ,Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_ENABLED,Aris::RT_CONTROL::EMSTAT_RUNNING,Aris::RT_CONTROL::EMSTAT_HOMING, //STA_ENABLED
   Aris::RT_CONTROL::EMSTAT_RUNNING ,  Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_INVALID,Aris::RT_CONTROL::EMSTAT_RUNNING,Aris::RT_CONTROL::EMSTAT_HOMING, //STA_RUNNING
   Aris::RT_CONTROL::EMSTAT_HOMING   , Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_INVALID,Aris::RT_CONTROL::EMSTAT_INVALID,Aris::RT_CONTROL::EMSTAT_HOMING,  //STA_HOMING
   Aris::RT_CONTROL::EMSTAT_FAULT   ,  Aris::RT_CONTROL::EMSTAT_POWEREDOFF, Aris::RT_CONTROL::EMSTAT_STOPPED,Aris::RT_CONTROL::EMSTAT_ENABLED,Aris::RT_CONTROL::EMSTAT_RUNNING,Aris::RT_CONTROL::EMSTAT_HOMING}; //STA_FAULT

 char CElmoMotor::print_poweredoff[PRINT_INFO_BUFFER_SIZE];
 char CElmoMotor::print_stop[PRINT_INFO_BUFFER_SIZE];
 char CElmoMotor::print_enabled[PRINT_INFO_BUFFER_SIZE];
 char CElmoMotor::print_homing[PRINT_INFO_BUFFER_SIZE];
 char CElmoMotor::print_running[PRINT_INFO_BUFFER_SIZE];
 char CElmoMotor::print_fault[PRINT_INFO_BUFFER_SIZE];

CElmoMotor::CElmoMotor()
{
    //home
    m_homeOffset=0;
    m_currentState=Aris::RT_CONTROL::EMSTAT_POWEREDOFF;
    _currentState=Aris::RT_CONTROL::EMSTAT_POWEREDOFF;
    m_motorMode=Aris::RT_CONTROL::OM_NOMODE;
    m_nextState=Aris::RT_CONTROL::EMSTAT_POWEREDOFF;
    m_subState=SUB_READY;
    m_isMotorCMDComplete=false;
    m_isHomingFinished=false;
    m_hasGoneHome=false;
     m_driverID=0;
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
	//rt_printf("set motor positon in ethercat\n");

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

int CElmoMotor::Initialize(ec_master_t **p_master,int homeOffset,int driverID,SMotorGeneralSettings& settings)
{
	m_driverID= driverID;
    m_pMaster=*p_master;
    
   // printf("2\n");
    m_pDomainAll = ecrt_master_create_domain(m_pMaster);

    if (!m_pDomainAll)
    {
        printf("create domain failed!\n");
        return -3;
    }

   // printf("3\n");
    //printf("v:%x:%x\n",(int)m_vender_id,m_product_code);

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
   // printf("4\n");

    // Configure the slave's PDOs and sync masters
    if (ecrt_slave_config_pdos(m_slaveConfigElmo, 4, m_slaveSyncs))
    {
        // handle error
        printf("Error slave config PDOs!\n");
        return -5;
    }
   // printf("5\n");

    // Configure the slave's domain
    if (ecrt_domain_reg_pdo_entry_list(m_pDomainAll, m_domainAllRegs))
    {
        printf("PDO entry registration failed!\n");
        return -6;
    }
   // printf("6\n");

    // Configure the slave's discrete clock
    ecrt_slave_config_dc(m_slaveConfigElmo, 0x0300, settings.nsPerCyclePeriod, 4400000, 0, 0);
    //printf("OK!\n");

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
  //  m_motorCommandData.ControlWord   = data.ControlWord;
    m_motorCommandData.MaxTorque     = data.MaxTorque;
  // m_motorCommandData.Mode          = data.Mode;

    m_motorCommandData.Position      = data.Position;
    m_motorCommandData.Torque        = data.Torque;
    m_motorCommandData.Velocity      = data.Velocity;

    //m_motorCommandData.IsCmdComplete = data.IsCmdComplete;
   // m_motorCommandData.MotorCmd      = data.MotorCmd;
    //status current DigitalInput
    //m_motorCommandData.StatusWord    = data.StatusWord;
    m_motorCommandData.Current       = data.Current;
    m_motorCommandData.DigitalInputs = data.DigitalInputs;

    // Admit the PTP request, IsPTPRequest will be automatically
    // reset to false when motor's PTP process started

    return 0;
}

int CElmoMotor::GetData(CElmoMotorData *p_data) const
{

    p_data->Current         = m_motorFeedbackData.Current;
    p_data->DigitalInputs   = m_motorFeedbackData.DigitalInputs;
    p_data->MaxTorque       = m_motorFeedbackData.MaxTorque;
    p_data->Mode            = m_motorFeedbackData.Mode;

    p_data->ControlWord     = m_motorFeedbackData.ControlWord;
    p_data->StatusWord      = m_motorFeedbackData.StatusWord;

    p_data->Position        = m_motorFeedbackData.Position;
    p_data->Torque          = m_motorFeedbackData.Torque;
    p_data->Velocity        = m_motorFeedbackData.Velocity;
    p_data->MotorCmd        = m_motorFeedbackData.MotorCmd;
   // p_data->IsCmdComplete   = m_motorFeedbackData.IsCmdComplete;

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
//	rt_printf("writing data\n");
    EC_WRITE_S32(m_pDomainAllPd + m_offsetElmoTargetPosition,  data.Position);
    EC_WRITE_S16(m_pDomainAllPd + m_offsetElmoTargetTorque,    data.Torque);
    EC_WRITE_S32(m_pDomainAllPd + m_offsetElmoTargetVelocity,  data.Velocity);
    EC_WRITE_S16(m_pDomainAllPd + m_offsetElmoMaxTorque,       data.MaxTorque);
    EC_WRITE_U16(m_pDomainAllPd + m_offsetElmoControlWord,     data.ControlWord);
    EC_WRITE_U8 (m_pDomainAllPd + m_offsetElmoModeOfOperation, data.Mode);
    return 0;
}

void CElmoMotor::Upload()
{
    ecrt_domain_process(m_pDomainAll);
    ReadData(&m_motorFeedbackData);

  //  rt_printf("reading status word %d\n",m_motorFeedbackData.StatusWord);
   // rt_printf("reading mode of operation %d\n",m_motorFeedbackData.Mode);

}

void CElmoMotor::Download()
{
    WriteData(m_motorCommandData);
  //  rt_printf("writing control word %d\n",m_motorCommandData.ControlWord);
    //rt_printf("writing data\n");
    ecrt_domain_queue(m_pDomainAll);
}

int CElmoMotor::GetDigitalInput() const
{
    return m_motorFeedbackData.DigitalInputs;
}

Aris::RT_CONTROL::EServoState CElmoMotor::GetMotorState() const
{
	return m_currentState;
}

Aris::RT_CONTROL::EOperationMode CElmoMotor::GetMotorMode() const
{
	return m_motorMode;
}

bool CElmoMotor::CheckComplete() const
{
	return m_isMotorCMDComplete;
}

void CElmoMotor::SetTargetPosition(int32_t position)
{
    m_motorCommandData.Position=position;
}

void CElmoMotor::SetTargetVelocity(int32_t velocity)
{
	m_motorCommandData.Velocity=velocity;
}

void CElmoMotor::SetTargetTorque(int16_t torque)
{
	m_motorCommandData.Torque=torque;
}

void CElmoMotor::SetMaxTorque(int16_t max_torque)
{
	m_motorCommandData.MaxTorque=max_torque;
}

int CElmoMotor::CheckState()//CheckState Each Cycle
{
    UpdateMotorState();
    if( m_currentState == Aris::RT_CONTROL::EMSTAT_FAULT )
    {
    	m_motorCommandData.ControlWord=0x80;
        return -1;
    }
    else
    {
        return 0;
    }
}

/*void CElmoMotor::SetOffset(int offset) // set the offset of the starting position at the beginning
{
    m_homeOffset=offset;
}*/

void CElmoMotor::SetMotorCommand(Aris::RT_CONTROL::EServoCommand p_command)
{
	m_motorCommand.command=p_command;
}

void CElmoMotor::SetMotorMode(Aris::RT_CONTROL::EOperationMode p_mode)
{
	m_motorCommand.operationMode=p_mode;
}

int CElmoMotor::DoCommand()
{
    UpdateMotorState();

    if(m_isMotorCMDComplete==false)
    {
    	if(m_currentState==Aris::RT_CONTROL::EMSTAT_FAULT)
    	{
    		m_motorCommandData.ControlWord=0x80;
    		return -1;
    	}

    	    switch(m_nextState)
    	    {
    	   // case Aris::RT_CONTROL::EMSTAT_FAULT:
    	    	// Motor has fault, require motor power off, and report error
     	    //	m_motorCommandData.ControlWord=0x80;
    	    	//m_motorCommandData.Mode=m_motorCommand.operationMode;
    	    	//return -1;

    	    case Aris::RT_CONTROL::EMSTAT_POWEREDOFF:
    	  //  	if(m_currentState==Aris::RT_CONTROL::EMSTAT_FAULT)
    	  //  		m_motorCommandData.ControlWord=0x80;
    	   // 	else
    	   // 	{
        	    	m_motorCommandData.ControlWord = 0x00;  // require power off immediately
        	    	m_motorCommandData.Mode=m_motorCommand.operationMode;
    	   // 	}
    	    	//rt_printf("going poweredoff\n");
    	        break;

    	    case Aris::RT_CONTROL::EMSTAT_STOPPED:
               // rt_printf("going stopped\n");

    	   // 	if(m_currentState==Aris::RT_CONTROL::EMSTAT_FAULT)
    	   // 		m_motorCommandData.ControlWord=0x80;
    	    //	else
    	    //	{
        	    	m_motorCommandData.ControlWord = 0x06;  // require stop
        	    	m_motorCommandData.Mode=m_motorCommand.operationMode;
    	    //	}

    	        break;

    	    case Aris::RT_CONTROL::EMSTAT_ENABLED:
               // rt_printf("going enabled\n");

    	   // 	if(m_currentState==Aris::RT_CONTROL::EMSTAT_FAULT)
    	    //		m_motorCommandData.ControlWord=0x80;
    	    //	else
    	    //	{
    	   	    	m_motorCommandData.Mode=m_motorCommand.operationMode;
    	    	        if ( m_currentState == Aris::RT_CONTROL::EMSTAT_POWEREDOFF)
    	    	        {
    	    	        	m_motorCommandData.ControlWord = 0x06;  // require STOPPED state firstly
    	    	        }
    	    	        else if( m_currentState == Aris::RT_CONTROL::EMSTAT_STOPPED)
    	    	        	m_motorCommandData.ControlWord = 0x07;  // require ENABLE state
    	   // 	}

    	        break;

    	    case Aris::RT_CONTROL::EMSTAT_RUNNING:
                //rt_printf("going running\n");
    	   // 	if(m_currentState==Aris::RT_CONTROL::EMSTAT_FAULT)
    	    //		m_motorCommandData.ControlWord=0x80;
    	    //	else
    	    //	{
        	    	m_motorCommandData.Mode=m_motorCommand.operationMode;
        	        if ( m_currentState == Aris::RT_CONTROL::EMSTAT_POWEREDOFF )
        	        	m_motorCommandData.ControlWord = 0x06;  // require STOPPED state firstly
        	        else if ( m_currentState == Aris::RT_CONTROL::EMSTAT_STOPPED )
        	        	m_motorCommandData.ControlWord = 0x07;  // require ENABLE state firstly
        	        else if ( m_currentState == Aris::RT_CONTROL::EMSTAT_ENABLED)
        	        {
         	        	m_motorCommandData.ControlWord = 0x0F;  // require RUNNING state
        	        }
    	    //	}

    	        break;

    	    case Aris::RT_CONTROL::EMSTAT_HOMING:
    	    	//m_motorCommandData.Mode=m_motorCommand.operationMode;
    	    	if (m_currentState == Aris::RT_CONTROL::EMSTAT_POWEREDOFF)
    	    		m_motorCommandData.ControlWord = 0x06;  // require STOPPED state firstly
    	    	else if(m_currentState==Aris::RT_CONTROL::EMSTAT_STOPPED)
    	    		m_motorCommandData.ControlWord=0x07;
    	    	else if(m_currentState==Aris::RT_CONTROL::EMSTAT_ENABLED)
    	    		m_motorCommandData.ControlWord=0x0F;
    	    	else if(m_currentState==Aris::RT_CONTROL::EMSTAT_RUNNING)
        	    	{
        	    		m_motorCommandData.Mode=6;//homing mode
                         rt_printf("going home\n");
        	    	}

        	    	else if(m_currentState==Aris::RT_CONTROL::EMSTAT_HOMING)
        	    		if(m_subState==SUB_FINISHED)
        	    		{
        	    			m_isHomingFinished=true;

        	    	 	 	rt_printf("homng finished\n");
        	    		}
        	    		else //SUB_READY or SUB_BUSY
        	    		{

        	    			// keep going home
        	    			m_motorCommandData.ControlWord=0x1F;
        	    		}
    	    //	}

    	    	break;
    	    case Aris::RT_CONTROL::EMSTAT_INVALID:
    	    	break;
    	    }
    }

    return 0;  // No error
}

void CElmoMotor::UpdateMotorState()
{
	uint motorState=m_motorFeedbackData.StatusWord;
    int motorState_0x000F = (motorState& 0x000F);
    switch (motorState_0x000F)
    {
    case 0x0000:
    	m_currentState=Aris::RT_CONTROL::EMSTAT_POWEREDOFF;
    	//rt_printf("current state pwoered off\n");
        break;
    case 0x0001:
    	m_currentState=Aris::RT_CONTROL::EMSTAT_STOPPED;
        break;
    case 0x0003:
    	m_currentState=Aris::RT_CONTROL::EMSTAT_ENABLED;
    	break;
    case 0x0007:
    	if(m_motorFeedbackData.Mode==6)
    		m_currentState=Aris::RT_CONTROL::EMSTAT_HOMING;
    	else
    		m_currentState=Aris::RT_CONTROL::EMSTAT_RUNNING;
    	break;
    default:	  // Motor has fault
    	m_currentState=Aris::RT_CONTROL::EMSTAT_FAULT;
    	break;
    }

    if (m_currentState==Aris::RT_CONTROL::EMSTAT_HOMING)
    {
        if ( motorState & 0x1000 )   // the HOME ATTAINED bit is set
        {
            m_subState=SUB_FINISHED;

        }

        else if ( (motorState & 0x0400) == 0 ) // the TARGET REACHED bit is not set
        {
            m_subState=SUB_BUSY;
         }
        else
            m_subState=SUB_READY;
    }


	if(m_currentState!=_currentState)
	{
		//rt_printf("Aris_Device:motor state changed from %s to %s.\n",ServoStateName[_currentState],ServoStateName[m_currentState]);
	    rt_printf("Device Number %d:",m_driverID);
	    switch(m_currentState)
	    {
	    case Aris::RT_CONTROL::EMSTAT_POWEREDOFF:
	    	rt_printf(print_poweredoff);
	    	rt_printf("\n");

	    break;

	    case Aris::RT_CONTROL::EMSTAT_STOPPED:
	    	rt_printf(print_stop);
	    	rt_printf("\n");
	    break;

	    case Aris::RT_CONTROL::EMSTAT_ENABLED:
	    	rt_printf(print_enabled);
	    	rt_printf("\n");
	    break;

	    case Aris::RT_CONTROL::EMSTAT_HOMING:
	    	rt_printf(print_homing);
	    	rt_printf("\n");
	    break;

	    case Aris::RT_CONTROL::EMSTAT_RUNNING:
	    	rt_printf(print_running);
	    	rt_printf("\n");
	    break;

	    case Aris::RT_CONTROL::EMSTAT_FAULT:
	    	rt_printf(print_fault);
	    	rt_printf("\n");
	    break;

	    }
		_currentState=m_currentState;
	}

 //   if(m_hasGoneHome==true&&m_motorCommand.command==Aris::RT_CONTROL::EMCMD_GOHOME)
//	{
//		m_motorCommand.command=Aris::RT_CONTROL::EMCMD_NONE;
		//rt_printf("not go home \n");
	//}

//	if(m_motorCommand.command==Aris::RT_CONTROL::EMCMD_GOHOME)
//	{
//		m_hasGoneHome=true;
//		m_isHomingFinished=false;
	//	rt_printf("go home only this time \n");
//	}

	if(m_isHomingFinished==1&&m_motorCommand.command==Aris::RT_CONTROL::EMCMD_GOHOME)
	{
		m_motorCommand.command=Aris::RT_CONTROL::EMCMD_RUNNING;
	}

	//rt_printf("currentstate:%d\n",m_currentState);
	m_nextState=m_stateMachine[m_currentState][m_motorCommand.command];

  // rt_printf("motor command in motor level: %d ",m_motorCommand.command);
  //  rt_printf("motor state in motor level: %d",m_currentState);
  //  rt_printf("motor sub state in motor level: %d\n",m_subState);

   //  rt_printf("motor next state in motor level: %d\n",m_nextState);


	if(m_nextState==m_currentState&&m_motorCommand.operationMode==m_motorFeedbackData.Mode&&m_currentState!=Aris::RT_CONTROL::EMSTAT_HOMING)
	{
		m_isMotorCMDComplete=true;
 	}

	else if(m_currentState==Aris::RT_CONTROL::EMSTAT_HOMING&&m_isHomingFinished==true)
	{
		m_motorCommandData.Mode=9;//NEED MUST
	    m_isMotorCMDComplete=true;

	}
	else
		m_isMotorCMDComplete=false;
  }

bool CElmoMotor::CheckMotorHomed()
{
	return m_isHomingFinished;
}

void CElmoMotor::ResettoGoHome()
{
	m_hasGoneHome=false;
}


//*****************************PID************************************//
double CSysPID::Kp_PosToTor_1_high = 150.0;
double CSysPID::Kp_PosToTor_1_low = 30;
double CSysPID::Kp_PosToTor_2 = 0.09;
double CSysPID::Ki_PosToTor_1 = 0.0;//0.01
double CSysPID::Ki_PosToTor_2 = 0.000;//0.0001
double CSysPID::Kd_PosToTor_1 = 0.0;
double CSysPID::Kd_PosToTor_2 = 0.0;

//threshold
double CSysPID::Threshhold_Kp_1_up=200000;
double CSysPID::Threshhold_Kp_1_down=100000;

double CSysPID::PositionErrorBefore[ACTUAL_MOTOR_NUMBER];
double CSysPID::PositionErrorD[ACTUAL_MOTOR_NUMBER];
double CSysPID::PositionErrorI[ACTUAL_MOTOR_NUMBER];
double CSysPID::PositionErrorP[ACTUAL_MOTOR_NUMBER];
double CSysPID::VelocityErrorBefore[ACTUAL_MOTOR_NUMBER];
double CSysPID::VelocityErrorD[ACTUAL_MOTOR_NUMBER];
double CSysPID::VelocityErrorI[ACTUAL_MOTOR_NUMBER];
double CSysPID::VelocityErrorP[ACTUAL_MOTOR_NUMBER];

CSysPID::CSysPID()
{
	InitPositionToTorque();
};

CSysPID::~CSysPID()
{

};

void CSysPID::InitPositionToTorque()
{

	memset(&PositionErrorP, 0, sizeof(PositionErrorP));
	memset(&PositionErrorI, 0, sizeof(PositionErrorI));
	memset(&PositionErrorD, 0, sizeof(PositionErrorD));
	memset(&PositionErrorBefore, 0, sizeof(PositionErrorBefore));
	memset(&VelocityErrorP, 0, sizeof(VelocityErrorP));
	memset(&VelocityErrorI, 0, sizeof(VelocityErrorI));
	memset(&VelocityErrorD, 0, sizeof(VelocityErrorD));
	memset(&VelocityErrorBefore, 0, sizeof(VelocityErrorBefore));

};

void CSysPID::DoPID(ElmoMotor::CElmoMotorData* p_Data, ElmoMotor::CElmoMotorData* p_Order,
		int SpeedLimit, int n)
{
	int PositionNow 	= 	0;
	int VelocityNow 	= 	0;
	int TorqueNow 		= 	0;
	int PositionTarget 	=	0;
	int VelocityTarget 	=	0;
	int TorqueTarget	=	0;

	for (int i = 0; i < n; i++)
	{
		PositionNow = p_Data[i].Position;
		VelocityNow = p_Data[i].Velocity;
		TorqueNow = p_Data[i].Torque;

		//*********************Position Loop*************************//

		PositionTarget = p_Order[i].Position;

		PositionErrorD[i] = PositionErrorP[i] - PositionErrorBefore[i];

		PositionErrorP[i] = PositionTarget - PositionNow;

		//first time PositionErrorI will be only the first PositionError
		PositionErrorI[i] += PositionErrorP[i];

		PositionErrorBefore[i] = PositionErrorP[i];

		VelocityTarget = (int) (Kp_PosToTor_1_high * PositionErrorP[i]
				+ Ki_PosToTor_1 * PositionErrorI[i]
				+ Kd_PosToTor_1 * PositionErrorD[i]);
		//OrderAgentToActuation[0].Velocity=(int)(Kp*PositionErrorP+Ki*PositionErrorI+Kd*PositionErrorD);
		if (VelocityTarget > SpeedLimit)
		{
			VelocityTarget=SpeedLimit;
			//printf("%f\t%f\t%f\t%d\n",PositionErrorP,PositionErrorI,PositionErrorD,VelocityTarget);
		}
		else if (VelocityTarget < -SpeedLimit)
		{
			VelocityTarget=-SpeedLimit;
			//printf("%f\t%f\t%f\t%d\n",PositionErrorP,PositionErrorI,PositionErrorD,VelocityTarget);
		}
		//                  //VelocityTarget=VelocityTarget2;

		//*********************Velocity Loop*************************//

		VelocityErrorD[i] = VelocityErrorP[i] - VelocityErrorBefore[i];
		VelocityErrorP[i] = VelocityTarget - VelocityNow;
		VelocityErrorI[i] += VelocityErrorP[i];
		VelocityErrorBefore[i] = VelocityErrorP[i];

		TorqueTarget = (int) (Kp_PosToTor_2 * VelocityErrorP[i] + Ki_PosToTor_2 * VelocityErrorI[i]
				+ Kd_PosToTor_2 * VelocityErrorD[i]);

		//printf("%f\t%f\t%f\t%d\n", VelocityErrorP, VelocityErrorI,
		//		VelocityErrorD, (int) TorqueTarget);

		int TorqueLimit = 2000;
		if (TorqueTarget > TorqueLimit)
		{
			TorqueTarget = TorqueLimit;
		}
		else if (TorqueTarget < -TorqueLimit)
		{
			TorqueTarget = -TorqueLimit;
		}

		p_Order[i].Torque = TorqueTarget;
		p_Order[i].Position = PositionTarget;
		p_Order[i].Velocity = VelocityTarget;


	}
};


ec_master_t* CDeviceMaster::m_pMaster;
ec_master_state_t CDeviceMaster::m_masterState;

CDeviceMaster::CDeviceMaster(void)
{
    m_motorNum = ACTUAL_MOTOR_NUMBER;
    m_analogInputsTerminalNum = ACTUAL_EL3104_NUM;
    m_analogOutputsTerminalNum = ACTUAL_EL4002_NUM;
    m_isMasterRequested = false;
    m_isMasterActivated = false;
    m_needSetTrajData=false;

    //safty mesure
    for (int i = 0; i < ACTUAL_MOTOR_NUMBER; i++)
    {
        m_homeOffsets[i] = HEXBOT_HOME_OFFSETS_RESOLVER[i];
    }

    for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
    {
    	m_driverIDs[i]=i;
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
    m_motorGeneralSettings.homeMode = p_InitParam.homeMode;
    m_motorGeneralSettings.homeAccel = p_InitParam.homeAccel;
    m_motorGeneralSettings.homeLowSpeed = p_InitParam.homeLowSpeed;
    m_motorGeneralSettings.homeHighSpeed = p_InitParam.homeHighSpeed;
    m_motorGeneralSettings.p2pMaxSpeed = p_InitParam.p2pMaxSpeed;
    m_motorGeneralSettings.p2pSpeed = p_InitParam.p2pSpeed;
    m_motorGeneralSettings.nsPerCyclePeriod = p_InitParam.nsPerCyclePeriod;
    m_motorNum=p_InitParam.motorNum;

   // rt_printf("motor number in device init %d\n",m_motorNum);

    for (int i = 0; i < m_motorNum; i++)
    {
       // m_driverIDs[i]=p_InitParam.driverIDs[i];
        m_homeOffsets[i] = p_InitParam.homeOffsets[i];
     }


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
        m_motors[i].SetEtherCATPosition(ECAT_START_POS_ELMODRIVE + i);
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

    for(int i=0;i<m_motorNum;i++)
    {
       // printf("m:%d\n",i);
         returnValue = m_motors[i].Initialize(&m_pMaster,m_homeOffsets[i],i,m_motorGeneralSettings);
        if(returnValue!=0)
        {
            printf("Fail to initialize the motor driver No.%d\n", i);
            return returnValue;
        }
    }

   // Activate the master
    ecrt_master_activate(m_pMaster);

   //  printf("Device activated!\n");
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
	    returnValue = m_motors[i].Activate();
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
        m_analogInputs[i].GetAnalogInputs(&(m_feedbackData.m_analogInputsData[i]));    }

    for (int i = 0; i < m_analogOutputsTerminalNum; i++)
    {
        m_analogOutputs[i].Upload();
    }
    for(int i = 0; i < m_motorNum; i++)
    {
        m_motors[i].Upload();
        m_motors[i].GetData(&m_feedbackData.m_motorData[i]);
    }
    //if (m_motors[0].GetMotorState()==Aris::RT_CONTROL::EMSTAT_RUNNING)
    //	rt_printf(" machine data feedback pos: %d\n", m_feedbackData.m_motorData[0].Position);

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
        //order is important
    //	if(m_motors[i].CheckMotorHomed()==true)

       m_motors[i].SetData(m_commandData.m_motorData[i]);

       m_motors[i].DoCommand();
     //  if (m_motors[0].GetMotorState()==Aris::RT_CONTROL::EMSTAT_RUNNING)
      //   rt_printf(" machine data command pos: %d\n", m_commandData.m_motorData[0].Position);
        m_motors[i].Download();
    }
    ecrt_master_send(m_pMaster);
    return 0;
}

void CDeviceMaster::DoPID()
{
	/*
	 * called in running state
	 * velocity loop verified before
	 * torque loop unusable
	 */
	m_pid.DoPID(m_feedbackData.m_motorData,m_commandData.m_motorData);
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

bool CDeviceMaster::CheckAllMotorsHomed ()
{
	for(int i=0;i<m_motorNum;i++)
	{
		if(m_motors[i].CheckMotorHomed()==false)
			return false;
	}
	return true;
}

int CDeviceMaster::DeviceDataToMachineData( Aris::RT_CONTROL::CMachineData &p_mData,long long int p_cycleCount)
{
	//copy feedback data
	//cannot simply memcpy, because they have different definition
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		//p_mData.feedbackData[i].Position = p_dData.m_motorData[i].Position;
		//p_mData.feedbackData[i].Velocity = p_dData.m_motorData[i].Velocity;
		//p_mData.feedbackData[i].Torque   = p_dData.m_motorData[i].Torque;
		p_mData.feedbackData[i].Position = m_feedbackData.m_motorData[i].Position;
		p_mData.feedbackData[i].Velocity = m_feedbackData.m_motorData[i].Velocity;
		p_mData.feedbackData[i].Torque	 = m_feedbackData.m_motorData[i].Torque;
	}
	p_mData.time=p_cycleCount;

	return 0;
};

int CDeviceMaster::MachineDataToDeviceData(const Aris::RT_CONTROL::CMachineData p_mData)
{
	//copy command data
	for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
	{
		m_commandData.m_motorData[i].Position = p_mData.commandData[i].Position;
		m_commandData.m_motorData[i].Velocity = p_mData.commandData[i].Velocity;
		m_commandData.m_motorData[i].Torque   = p_mData.commandData[i].Torque;
	}
	return 0;
};
