#ifndef ELMOMOTORDRIVER_H
#define ELMOMOTORDRIVER_H

#include "ElmoMotorEnum.h"
#include "ecrt.h"
#include "ElmoMotorData.h"

#include <sys/mman.h>
#include <stdio.h>
#include <cstring>

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_FAULT_BIT 3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 4
#define STATUSWORD_QUICK_STOP_BIT 5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 6
#define STATUSWORD_NO_USED_WARNING_BIT 7
#define STATUSWORD_ELMO_NOT_USED_BIT 8
#define STATUSWORD_REMOTE_BIT 9
#define STATUSWORD_TARGET_REACHED_BIT 10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT 11
#define STATUSWORD_HOME_ATTAINED_BIT 12

namespace ElmoMotor
{
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


    // function to initialize the driver, mush be called after ecrt_request_master(0) which is defined in the EthercatMaster object
    
    int Initialize(ec_master_t **p_master,int homeOffset, SMotorGeneralSettings& settings);

    // Activate the slave's cyclic task, return value 0 means successfully activated, otherwise failed
    int Activate();

    // Home the motor
    int GoHome();

    // switch the driver's operation mode
    int SwitchMode(int mode);

    // generate control commands that should be sent to the driver via PDO according to the motor's state machine, should be called in every cycle
    int DoCommand();

    // following two functions are the operations on drive parameters and elmo_motor's member MotorData,
    // reading and writing data to the memory are done by private functions ReadData() and WriteData(), which are to be called by Upload()/Download()

    // set the values of the motor drive's data area
    int SetData(const CElmoMotorData& data);

    // get the values of this motor drive's data area
    int GetData(CElmoMotorData *p_data) const;

    // Get value of the motor's actual position, in count
    int GetPositionActualValue() const;

    // Get value of the motor's actual velocity, in count/s
    int GetVelocityActualValue() const;

    // Get value of the motor's actual torque, in per thousand of the maximum torque
    int GetTorqueActualValue() const;

    // Get value of the motor's actual current, in per thousand of the maximum current
    int GetCurrentActualValue() const;

    // Get the motor's statusword
    int GetStatusWord() const;

    // Get the value of the driver's digital input
    int GetDigitalInput() const;

    // Get current motor state from the status word: poweroff, stop, enable, running, or fault
    MOTOR_STATE    GetState() const;

    // Get current motor substate: busy, ready, or finished
    SUB_STATE      GetSubState() const;

    // Get current operation mode of the driver: standstill, home, cyclic pos, cyclic vel, cyclic trq, profiled pos, or none
    OPERATION_MODE GetCurrentMode() const;

    // Set value of the motor's target position, in count
    void SetTargetPosition(int32_t position);

    // Set value of the motor's target velocity, in count/s
    void SetTargetVelocity(int32_t velocity);

    // Set value of the motor's target torque, in per thousand of the maximum torque
    void SetTargetTorque(int16_t torque);

    // Set controlword to the driver
    void SetControlWord(int controlword);

    // Set the driver's operation mode
    void SetMode(int mode);

    // Set the max allowed torque of the motor, in per thousand of the preset maximum torque
    void SetMaxTorque(int16_t max_torque);

    // Check whether the driver has a fault
    int  CheckState();

    // Check whether the motor's home position is reached
    int  GetHomeSwitchFlag() const;

    // Set the home offset of the motor
    void SetOffset(int offset);

    // Enable the driver
    void Enable();

    // Stop the driver
    void Stop();

    // Poweroff the driver
    void Poweroff();

	CElmoMotorData m_motorData;
	CElmoMotorData m_motorOrder;

    bool m_isCommandComplete;
    int m_isPositioningRequested;
    int m_isHomingRequested;

    void Upload();//call at the beginning of a cycle, will be called in actuation.read
    void Download();//call at the end of a cycle, will be called int actuation.write

private:
    int WriteData(const CElmoMotorData& data);//cyclic tasks
    int ReadData(CElmoMotorData *p_data)  const;//cyclic tasks

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

    MotorCommand m_motorCommand;
    //home
    int m_homeOffset;
    unsigned int m_homeSwitchMask;
    int m_homeProcessFlag;

    int m_standStillProcessFlag;
    int m_standStillPosition;
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

}

#endif
