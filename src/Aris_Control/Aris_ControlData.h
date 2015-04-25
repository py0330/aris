/*
 * Aris_ControlData.h
 *
 *  Created on: Nov 13, 2014
 *      Author: leo
 */

#ifndef ARIS_CONTROLDATA_H_
#define ARIS_CONTROLDATA_H_
#include "Aris_Core.h"


#define MAX_AXIS_NUMBER 20


extern char ServoStateName[14][20];

class CSysBase;

namespace Aris
{
namespace RT_CONTROL
{
class ACTUATION;

enum EServoState
{
	EMSTAT_NONE		= 0,// init

	EMSTAT_INVALID	= 1,

	EMSTAT_POWEROFF	= 2,
	EMSTAT_POED		= 3,

	EMSTAT_STOP		= 4,// atfer
	EMSTAT_STOPPED	= 5,

	EMSTAT_ENABLE	= 6,
	EMSTAT_ENABLED	= 7,

	EMSTAT_HOMING	= 8,// a transistion state to HOMED
	EMSTAT_HOMED	= 9,

	EMSTAT_H2RING	= 10,//home to running
	EMSTAT_RUNNING	= 11,// but not sure which mode is

	EMSTAT_STSTILL	= 12,// a transistion state from HOMED to RUNNING

	EMSTAT_EMERGE	= 13,
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
    int homeMode;
    int homeAccel;
    int homeLowSpeed;
    int homeHighSpeed;
    int p2pMaxSpeed;
    int p2pSpeed;
    int nsPerCyclePeriod;
    int motorNum;
    int* homeOffset;
	/*
	 * Following two parameter is convenient for testing
	 */
	int startMotorID;
	int endMotorID;
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
	EServoState state;
	long long int time;
	//Motor Data
	CMachineData& operator=(const CMachineData& other);
	CMotorData feedbackData[MAX_AXIS_NUMBER];//currentFeedback,collected after read()
	CMotorData commandData[MAX_AXIS_NUMBER];//lastCommand,collected before write()

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
typedef int (*FuncPtrWork)(Aris::RT_CONTROL::CMachineData&);
typedef int (*FuncPtrInit)(Aris::RT_CONTROL::CSysInitParameters&);
typedef int (*FuncPtrState)(void*);
typedef int (*FuncPtrCustom)(Aris::Core::MSG&);


#endif /* ARIS_CONTROLDATA_H_ */
