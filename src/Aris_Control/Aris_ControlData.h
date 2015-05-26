/*
 * Aris_ControlData.h
 *
 *  Created on: Nov 13, 2014
 *      Author: leo
 */

#ifndef ARIS_CONTROLDATA_H_
#define ARIS_CONTROLDATA_H_
#include "Aris_Core.h"

#define AXIS_NUMBER 18


extern char ServoStateName[7][20];

class CSysBase;

namespace Aris
{
namespace RT_CONTROL
{
class ACTUATION;



enum EServoState
{
    EMSTAT_NONE=0,
	EMSTAT_POWEREDOFF=1,
	EMSTAT_STOPPED=2,
	EMSTAT_ENABLED=3,
	EMSTAT_RUNNING=4,
	EMSTAT_HOMING=5,
	EMSTAT_FAULT,
	EMSTAT_INVALID,
	EMSTAT_EMERGE,
};
const int N_MOTOR_STATE=9;

enum EOperationMode
{
    OM_INVALID     = -1,
    //OM_PROFILEPOS = 1,
    OM_CYCLICPOS  = 8,
    OM_CYCLICVEL  = 9,
    OM_CYCLICTORQ = 10,
  //  OM_OTHER      = 18
};

enum EOperationModeDisplay
{
    OMD_INVALID     = -1,
    OMD_PROFILEPOS = 1,
	OMD_HOMING =6,
    OMD_CYCLICPOS  = 8,
    OMD_CYCLICVEL  = 9,
    OMD_CYCLICTORQ = 10,
    OMD_OTHER      = 18
};

enum EServoCommand
{
	EMCMD_NONE=0,
	EMCMD_POWEROFF=1,
	EMCMD_STOP=2,
	EMCMD_ENABLE=3,
	EMCMD_RUNNING=4,
	EMCMD_GOHOME=5,
};

const int N_MOTOR_CMD=6;

enum EControl_Msg
{
	CM_PUSH_CMD_TO_MOTORS=100,
	CM_GO_TRAJ,
	CM_CUS_MESSAGE,
	CM_NONE,
};



enum EMachineState
{
	CS_UNINITED,
	CS_INITED,
	CS_COMM_INITED,
 	CS_RTTASK_STARTED,
 	CS_STOPPED,
};





/*
 * CSysInitParameters should be used by SetSysInitializer
 */
class CSysInitParameters
{

public:
	CSysInitParameters();
	~CSysInitParameters();
//	CSysInitParameters& operator=(const CSysInitParameters& other);
	int motorNum;
    int homeMode;
    int homeAccel;
    int homeLowSpeed;
    int homeHighSpeed;
    int p2pMaxSpeed;
    int p2pSpeed;
    int nsPerCyclePeriod;
    int homeTorqueLimit;
    int* homeOffsets;
    int* driverIDs;
};


/*
 * CMotorData struct contain one single motor's data, used by CMachineData
 */
class CMotorData
{
public:
	CMotorData& operator=(const CMotorData& other);
	int Position;
	int Velocity;
	int Torque;
};
/*
 * CWorkData struct should be used by TrajectoryGenerator and Logger as Input Data
 *
 * may include sensor data, and other data
 *
 */

class CMachineData
{
public:
	int motorNum;
	EMachineState machinestate;

	long long int time;
	//Motor Data
	CMachineData& operator=(const CMachineData& other);

	bool isMotorHomed[AXIS_NUMBER];
    EServoState motorsStates[AXIS_NUMBER];
    EServoCommand motorsCommands[AXIS_NUMBER];
    EOperationMode motorsModes[AXIS_NUMBER];
    EOperationModeDisplay motorsModesDisplay[AXIS_NUMBER];
	CMotorData feedbackData[AXIS_NUMBER];//currentFeedback,collected after read()
	CMotorData commandData[AXIS_NUMBER];//lastCommand,collected before write()
	//sensor data

};

/*
 * Reimplementation of the XDDP communication after discussion
 * One cycle, only one update of the buffer
 * RT_CONN_DATA class needed
 *
 * RT side: update buffer at cycle begin
 *
 */

/*
 * RT_MSG related settings
 */
#define RT_MSG_BUFFER_SIZE 8192
#define RT_MSG_HEADER_LENGTH MSG_HEADER_LENGTH
#define PRINT_INFO_BUFFER_SIZE 200
class RT_MSG
{
	friend class ::CSysBase;
	friend class Aris::RT_CONTROL::ACTUATION;
public:

	RT_MSG();
	~RT_MSG();

	void operator=(const RT_MSG& other);
	void SetLength(unsigned int dataLength);
	void SetMsgID(int msgID);

	unsigned int GetLength() const;
	int GetMsgID() const;
	char* GetDataAddress() const;

	/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，在拷贝完之后，MSG的长度自动设置为dataLength。
	* \param fromThisMemory    待拷贝的内存地址。
	* \param dataLength        数据长度
	*
	*/
	void Copy(const void * fromThisMemory, const unsigned int dataLength);
	/** \brief 从fromThisMemory指针中拷贝MSG.GetLength()大小的数据。
	* \param fromThisMemory    待拷贝的内存地址。
	*
	*/
	void Copy(const void * fromThisMemory);
	/** \brief 从fromThisMemory指针中拷贝MSG.GetLength()大小的数据到MSG内存中的指定地点。
	* \param fromThisMemory       待拷贝的内存地址。
	* \param dataLength           数据长度
	* \param atThisPositionInMsg  将数据拷贝到MSG.GetDataAddress()[atThisPositionInMsg]处。
	*/
	void CopyAt(const void * fromThisMemory, const unsigned int dataLength, const unsigned int atThisPositionInMsg);
	/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，这些数据添加到自身的尾部，在拷贝完之后，MSG的长度自动增加dataLength。
	* \param fromThisMemory    目标内存地址。
	*
	*/
	void CopyMore(const void * fromThisMemory, const unsigned int dataLength);

	/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
	* \param fromThisMemory    目标内存地址。
	* \param dataLength        数据长度
	*
	*/
	void Paste(void * toThisMemory,const unsigned int dataLength);
	/** \brief 向toThisMemory指针中粘贴MSG.GetLength()长度的数据。
	* \param fromThisMemory    目标内存地址。
	*
	*/
	void Paste(void * toThisMemory);
	/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
	* \param fromThisMemory    目标内存地址。
	* \param dataLength        数据长度
	*
	*/
	void PasteAt(void * toThisMemory,const unsigned int dataLength,const unsigned int atThisPositionInMsg);



	// in RT we only have twos object of this class, and we need to SetBuffer once for each object.
	// two objects: m_dataRecv m_dataSend
	//void SetBuffer(char* p_ptrBuffer);

	//used in send data
	//void SetData(const unsigned int p_dataLength = 0, const int p_dataType = 0, const void *p_src = 0);
	//used in receive data,
	//int GetData();
private:
	void SetType(long long message);
	long long GetType() const;

//	int *m_ptrDataType;// address will be setup in SetBuffer
//	int *m_ptrDataLength;// address will be setup in SetBuffer
	char *m_ptrData;// assign to whatever buffer you want to use.
};

}

}

/*
 * Function pointer with peculiar parameters
 */
typedef int (*FuncPtrWork)(Aris::RT_CONTROL::CMachineData&, Aris::RT_CONTROL::RT_MSG&);
typedef int (*FuncPtrInit)(Aris::RT_CONTROL::CSysInitParameters&);
typedef int (*FuncPtrState)(void*);
typedef int (*FuncPtrCustom)(Aris::Core::MSG&);


#endif /* ARIS_CONTROLDATA_H_ */
