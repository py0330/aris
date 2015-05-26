#ifndef ARIS_MOTOR_H
#define ARIS_MOTOR_H

#include "Aris_ControlData.h"
#include "ecrt.h"
#include "GlobalConfiguration.h"
#include "Aris_Sensor.h"

#include <sys/mman.h>
#include <stdio.h>
#include <cstring>

namespace ElmoMotor
{

/*enum EMOTOR_STATE
{

	STA_INVALID=-1,
    STA_POWEREDOFF = 0,
    STA_STOPPED,
    STA_ENABLED,
    STA_RUNNING,
    STA_HOMING,
    STA_FAULT,
};*/



enum EMOTOR_SUB_STATE
{
    SUB_READY = 0, SUB_BUSY = 1, SUB_FINISHED = 2
};

enum EPOSITIONING_MODE
{
    PM_RELATIVE = 0, PM_ABSOLUTE = 1
};

typedef struct _MotorCommand
{
    Aris::RT_CONTROL::EServoCommand command;
    Aris::RT_CONTROL::EOperationMode operationMode;
} MotorCommand;

// Data package of the motor driver
class CElmoMotorData
{
public:

 short int Mode;       //EC_WRITE_S8
 short int MaxTorque;

 int Position;   //EC_READ_S32
 int Velocity;   //EC_READ_S32
 short int Current;    //EC_READ_S16
 short int Torque;     //EC_READ_S16

 short int ControlWord;//EC_WRITE_U16
 short int StatusWord; //EC_READ_S16

 int DigitalInputs;

 MotorCommand MotorCmd;

 void operator=(const CElmoMotorData &other);

 CElmoMotorData(short int mode=DEFAULT_OPERATION_MODE,
     int position=0,
     int velocity=0,
     short int current=0,
     short int torque=0,
     short int control=0,
     short int status=0,
     short int maxTorque=0,
     int digitalInputs=0) //at first IsCmdComplete should be true, so we can give new command
     :Mode(mode)
     ,Position(position)
     ,Velocity(velocity)
     ,Current(current)
     ,Torque(torque)
     ,ControlWord(control)
     ,StatusWord(status)
     ,MaxTorque(maxTorque)
     ,DigitalInputs(digitalInputs)
 {}

 CElmoMotorData(const CElmoMotorData &other)
     :Mode(other.Mode)
     ,Position(other.Position)
     ,Velocity(other.Velocity)
     ,Current(other.Current)
     ,Torque(other.Torque)
     ,ControlWord(other.ControlWord)
     ,StatusWord(other.StatusWord)
     ,MaxTorque(other.MaxTorque)
     ,DigitalInputs(other.DigitalInputs)

 {}
 ~CElmoMotorData()
 {}

};

struct SMotorGeneralSettings {

 int homeHighSpeed;
 int homeLowSpeed;
 int homeAccel;
 int homeMode;
 int p2pMaxSpeed;
 int p2pSpeed;
 int nsPerCyclePeriod;
 int homeTorqueLimit;
};



class CSysPID
{
public:
	static double Kp_PosToTor_1_high;
	static double Kp_PosToTor_1_mid;
	static double Kp_PosToTor_1_low;
	static double Kp_PosToTor_2;
	static double Ki_PosToTor_1;
	static double Ki_PosToTor_2;
	static double Kd_PosToTor_1;
	static double Kd_PosToTor_2;

	//switch threshold
	static double Threshhold_Kp_1_up;
	static double Threshhold_Kp_1_down;

	static double PositionErrorD[ACTUAL_MOTOR_NUMBER];
	static double PositionErrorP[ACTUAL_MOTOR_NUMBER];
	static double PositionErrorI[ACTUAL_MOTOR_NUMBER];
	static double PositionErrorBefore[ACTUAL_MOTOR_NUMBER];

	static double VelocityErrorD[ACTUAL_MOTOR_NUMBER];
	static double VelocityErrorP[ACTUAL_MOTOR_NUMBER];
	static double VelocityErrorI[ACTUAL_MOTOR_NUMBER];
	static double VelocityErrorBefore[ACTUAL_MOTOR_NUMBER];

	CSysPID();
	~CSysPID();
	/*
	 * will be feed commanddata
	 */
	void DoPID(ElmoMotor::CElmoMotorData* p_Data,ElmoMotor::CElmoMotorData* p_Order,int SpeedLimit=5417643,int n=ACTUAL_MOTOR_NUMBER);
	void InitPositionToTorque();
private:


};


//transfered by pipe or queue
class CElmoMotorDataInfo
{
public:
 CElmoMotorDataInfo(int pType
     ,int pDataLength
     ,int pDataArrayNumber
     ,void* pData)
     :m_dataType(pType)
     ,m_dataLength(pDataLength)
     ,m_dataArrayNumber(pDataArrayNumber)
     ,m_pData(pData) {}

 int m_dataType;
 int m_dataLength;
 int m_dataArrayNumber;
 void* m_pData;
private:

};
/// Used for controlling the Elmo Golden motor drive.
class CElmoMotor
{
public:

    CElmoMotor();
    ~CElmoMotor();

    // assign the driver's position in a Ethercat net according to the network's topology, register domain, pdos and sync masters for this driver
    int SetEtherCATPosition(uint16_t position,
                            uint32_t product_code=0x00030924,
                            uint32_t vender_id=0x0000009a,
                            uint16_t alias=0);

    // set the values of the motor drive's data area
    int SetData(const CElmoMotorData& data);

    // Set value of the motor's target position, in count
    void SetTargetPosition(int32_t position);

    // Set value of the motor's target velocity, in count/s
    void SetTargetVelocity(int32_t velocity);

    // Set value of the motor's target torque, in per thousand of the maximum torque
    void SetTargetTorque(int16_t torque);

    // Set controlword to the driver
  //  void SetControlWord(int controlword);

    // Set the max allowed torque of the motor, in per thousand of the preset maximum torque
    void SetMaxTorque(int16_t max_torque);
    // Set the home offset of the motor
   // void SetOffset(int offset);



    // function to initialize the driver, mush be called after ecrt_request_master(0) which is defined in the EthercatMaster object
    
    int Initialize(ec_master_t **p_master,int homeOffset, int driverID,SMotorGeneralSettings& settings);

    // Activate the slave's cyclic task, return value 0 means successfully activated, otherwise failed
    int Activate();


    // generate control commands that should be sent to the driver via PDO according to the motor's state machine, should be called in every cycle
    int DoCommand();

    // following two functions are the operations on drive parameters and elmo_motor's member MotorData,
    // reading and writing data to the memory are done by private functions ReadData() and WriteData(), which are to be called by Upload()/Download()


    // get the values of this motor drive's data area
    int GetData(CElmoMotorData *p_data) const;

    // Get the value of the driver's digital input
    int GetDigitalInput() const;


    void SetMotorCommand(Aris::RT_CONTROL::EServoCommand p_command );
    void SetMotorMode(Aris::RT_CONTROL::EOperationMode p_mode);

    Aris::RT_CONTROL::EServoState GetMotorState() const;
    Aris::RT_CONTROL::EOperationModeDisplay GetMotorMode() const;

    // Update the motor state
    void UpdateMotorState();

    // Check whether the driver has a fault
    int  CheckState();
    bool CheckMotorHomed();

    bool CheckComplete() const;
    void Upload();//call at the beginning of a cycle, will be called in actuation.read
    void Download();//call at the end of a cycle, will be called int actuation.write
    void ResettoGoHome();
    static const Aris::RT_CONTROL::EServoState m_stateMachine[Aris::RT_CONTROL::N_MOTOR_STATE][Aris::RT_CONTROL::N_MOTOR_CMD];

    static char print_poweredoff[PRINT_INFO_BUFFER_SIZE];
    static char print_stop[PRINT_INFO_BUFFER_SIZE];
    static char print_enabled[PRINT_INFO_BUFFER_SIZE];
    static char print_homing[PRINT_INFO_BUFFER_SIZE];
    static char print_running[PRINT_INFO_BUFFER_SIZE];
    static char print_fault[PRINT_INFO_BUFFER_SIZE];
private:
    // public memebers
	bool m_hasGoneHome;
    CElmoMotorData m_motorFeedbackData;
	CElmoMotorData m_motorCommandData;
    int m_driverID;
    MotorCommand m_motorCommand;
    int m_homeOffset;
    bool m_isHomingFinished;

    bool m_isMotorCMDComplete;
    Aris::RT_CONTROL::EServoState m_currentState;
    Aris::RT_CONTROL::EServoState _currentState;
    Aris::RT_CONTROL::EServoCommand _currentCommand;
    Aris::RT_CONTROL::EServoState m_nextState;

//display
    EMOTOR_SUB_STATE m_subState;
    Aris::RT_CONTROL::EOperationModeDisplay m_motorMode;
   // Aris::RT_CONTROL::EOperationModeDisplay m_motor

    int WriteData(const CElmoMotorData& data);//cyclic tasks
    int ReadData(CElmoMotorData *p_data)  const;//cyclic tasks

    //mater
    ec_master_t* m_pMaster;
    //configuration for multi-domain
    ec_domain_t* m_pDomainAll;// = NULL;
    uint8_t* m_pDomainAllPd;

	
    //slave
    uint16_t m_alias; //= 0;
    uint16_t m_position;
    uint32_t m_vender_id; //= 0x0000009a;
    uint32_t m_product_code; //= 0x00030924;

    ec_slave_config_t* m_slaveConfigElmo;


    //PDO things

    //1A03 +1A1F
    unsigned int m_offsetElmoPositionActualValue;		// 0x1A03 0x6064.0 DINT 32
    unsigned int m_offsetElmoDigitalInputs;				// 0x1A03 0x60FD.0 DINT 32
    unsigned int m_offsetElmoVelocityActualValue;		// 0x1A03 0x606c.0 DINT 32
    unsigned int m_offsetElmoStatusWord;					// 0x1A03 0x6041.0 UINT 16

    unsigned int m_offsetElmoModeOfOperationDisplay;    // 0x1A0B 0x6061.0 SINT  8

    unsigned int m_offsetElmoCurrentActualValue;		// 0x1A1F 0x6078.0 INT 	16
    //1A13
    unsigned int m_offsetElmoTorqueActualValue;		// 0x1A13 0x6077.0 INT  16

	//1605
    unsigned int m_offsetElmoTargetPosition;				// 0x1605 0x607A.0 DINT 32
    unsigned int m_offsetElmoTargetVelocity;				// 0x1605 0x60FF.0 DINT 32
    unsigned int m_offsetElmoTargetTorque;				// 0x1605 0x6071.0 INT 	16
    unsigned int m_offsetElmoMaxTorque;					// 0x1605 0x6072.0 UINT 16
    unsigned int m_offsetElmoControlWord;					// 0x1605 0x6040.0 UINT 16
    unsigned int m_offsetElmoModeOfOperation;			// 0x1605 0x6060.0 USINT 8
	
    ec_pdo_entry_reg_t m_domainAllRegs[14];// = {};				//set it in initialize function
    ec_pdo_entry_info_t m_slave_1605_PdoEntries[6];/* =
	{	
		{0x607A,0x00,32},//target position
		{0x60FF,0x00,32},//target velocity
		{0x6071,0x00,16},//target torque
		{0x6072,0x00,16},//max torque
		{0x6040,0x00,16},//control word
		{0x6060,0x00, 8},//mode of operation
	};*/

    ec_pdo_entry_info_t m_slave_1a03_PdoEntries[4];/*=
	{	
		{0x6064,0x00,32},//position actual value
		{0x60fd,0x00,32},//digital inputs
		{0x606c,0x00,32},//velocity actual value
		{0x6041,0x00,16},//status word
	};*/

    ec_pdo_entry_info_t m_slave_1a0b_PdoEntries[1];

    ec_pdo_entry_info_t m_slave_1a13_PdoEntries[1];

    ec_pdo_entry_info_t m_slave_1a1f_PdoEntries[1];/*=
	{
		{0x6078,0x00,16},//current acutal value
	};*/

    ec_pdo_info_t m_slaveTxPdos[4];/*=
	{
		{0x1A03,4,slave_1a03_pdo_entries+0},
		//{0x1A0A,1,slave_1a0a_pdo_entries+0},
        {0x1A0B,1,slave_1a0b_pdo_entries+0},
		//{0x1A0E,1,slave_1a0e_pdo_entries+0},
		//{0x1A11,1,slave_1a11_pdo_entries+0},
		//{0x1A12,1,slave_1a12_pdo_entries+0},
        {0x1A13,1,slave_1a13_pdo_entries+0},
		{0x1A1F,1,slave_1a1f_pdo_entries+0},
	};*/

    ec_pdo_info_t m_slaveRxPdos[1];/*=
	{	
		{0x1605,6,slave_1605_pdo_entries+0},
		//{0x160b,1,slave_160b_pdo_entries+0},
	};*/


    ec_sync_info_t m_slaveSyncs[5];/*=
    {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, slave_rx_pdos + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 4, slave_tx_pdos + 0, EC_WD_ENABLE},
        {0xff}
    };*/
	//here we have done the pdo part, we can try it in a cpp file

	//SDO things
	//for Homing notice that Homing has its own controlword and statusword
	// cw: bit 4:Home operation start 0:inactive 0->1:start 1:active
	//     bit 8:halt 0:execute the instruction of bit 4 1:stop axle with homing deceleration
	// sw: bit 10:target reached 	0:Halt=0,homing position not reached
	//								  Halt=1,axle decelerates	
	//								1:Halt=0,homing position reached
	//								  Halt=1,velocity of axle is 0
	//	   bit 12:homing attained   0:homing mode not yet completed
	//								1:homing mode carried out successfully
	//	   bit 13:homing error		0:no homing error
	//								1:homing error occurred
	//								  homing mode carried out unsuccessfully	
	//								  error cause found in error code
	ec_sdo_request_t *sdo_607c_0; // 0x607c 0 int32  home offset
	ec_sdo_request_t *sdo_6098_0; // 0x6098 0 int8   homing method
	ec_sdo_request_t *sdo_6099_0; // 0x6099 0 val=2  homing speed number of entries
	ec_sdo_request_t *sdo_6099_1; // 0x6099 1 uint32 homing speed speed during search for switch
	ec_sdo_request_t *sdo_6099_2; // 0x6099 2 uint32 homing speed speed during search for zero
	ec_sdo_request_t *sdo_609a_0; // 0x609a 0 uint32 homing acceleration
	//sdo will r/w outside the rt cyclic task,so we will use  

	// sth went terribly wrong, ec_sdo_request_t is created for r/w SDOs during realtime operation,
	// so above code is useless for us.
	// let sdo be some function calls in the function initalize() 

};



/*****************************************************************************
 * Class CDeviceData
 *****************************************************************************/
class CDeviceData
{
public:
    CElmoMotorData     m_motorData[ACTUAL_MOTOR_NUMBER];
    CAnalogInputsData  m_analogInputsData[ACTUAL_EL3104_NUM];
    CAnalogOutputsData m_analogOutputsData[ACTUAL_EL4002_NUM];
};

/*****************************************************************************
 * Class CDeviceMaster
 *****************************************************************************/
// Operates all devices, including the Ethercat master, the motor drivers, analog/digital IO devices and the IMU


class CDeviceMaster
{
public:
    CDeviceMaster(void);
    ~CDeviceMaster(void);


    int Initialize( const Aris::RT_CONTROL::CSysInitParameters p_InitParam);

    int Read();   //read each device's data
    int Write();  //write each device's data
    int DCSyncTime(uint64_t nanoSecond);

    //check whether all motors have complete their commands, return the number of the motors that haven't finished
    bool CheckAllMotorsHomed();


    int DeactiveMaster();
   // int DeviceDataToMachineData(const CDeviceData p_dData,Aris::RT_CONTROL::CMachineData &p_mData,long long int p_cycleCount);
    //int MachineDataToDeviceData(const Aris::RT_CONTROL::CMachineData p_mData,CDeviceData &p_dData);

    int DeviceDataToMachineData(Aris::RT_CONTROL::CMachineData &p_mData,long long int p_cycleCount);
    int MachineDataToDeviceData(const Aris::RT_CONTROL::CMachineData p_mData);
    void DoPID();


    // Data exchange area
    CDeviceData m_feedbackData;
    CDeviceData m_commandData;
    CDeviceData m_standstillData;

    int m_driverIDs[ACTUAL_MOTOR_NUMBER];  //[0-17]
  //  void SetMotorCommands(Aris::RT_CONTROL::EServoCommand* p_motorCommands,int* p_driverIDs);
    CElmoMotor m_motors[ACTUAL_MOTOR_NUMBER];//CElmoMotor m_motors[ACTUAL_MOTOR_NUM]
    int m_homeOffsets[ACTUAL_MOTOR_NUMBER];
    int m_motorNum;
    bool m_needSetTrajData;




private:
    CSysPID m_pid;

    int m_analogInputsTerminalNum;
    int m_analogOutputsTerminalNum;

    //master
    static ec_master_t* m_pMaster;
    static ec_master_state_t m_masterState;

    bool m_isMasterActivated;
    bool m_isMasterRequested;
  //  bool m_isAllComplete;


    // Actuations Sysinitparameters

    SMotorGeneralSettings m_motorGeneralSettings;

    // Ethercat coupler
    CEthercatCouplerEK1100 m_ethercatCoupler;
    // Analog Inputs
    CAnalogInputsEL3104 m_analogInputs[ACTUAL_EL3104_NUM];
    // Analog Outputs
    CAnalogOutputsEL4002 m_analogOutputs[ACTUAL_EL4002_NUM];

};
}

#endif
