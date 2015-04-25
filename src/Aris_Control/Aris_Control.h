#include "Aris_ControlData.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdk.h>
#include "Aris_Socket.h"

#ifndef ARIS_CONTROL_H_
#define ARIS_CONTROL_H_
/**\file Aris_Control.h
 * \brief 命名空间Control中包括了基于实时系统的控制系统API。
 * 		  包括通讯，电控等内容。
 *
 *具体而言，通讯部分实现了消息队列，实时非实时通信等，电控部分实现了电机控制，传感器读取等。
 *并且提供了一系列自定义函数的接口，使用者可以在控制系统中加入自己的函数，从而方便地结合API灵活地实现目标。
 *
 */


class CSysBase;

namespace Aris
{
namespace RT_CONTROL
{
/**
 * \brief
 */
class ACTUATION
{
public:

	/**
	 * \brief 类CControlSystem的构造函数，初始化工作。
	 */
	ACTUATION();
	/**
	 *\brief 类CControlSystem的析够函数，清理工作。
	 */
	~ACTUATION();


	/**
	 * @name 系统函数
	 */

	///@{

	/**
	 * \brief 设置初始化函数
	 *
	 * 初始化函数将在系统初始化时被自动调用，从而执行函数内所需要的工作。
	 *
	 * @param p_Initializer 是一个FuncPtrInit类型的函数指针，该类行指针指向的函数需要一个CSysInitParameters的指针。
	 * 该函数在被调用时包含的参数就是就是SysInit()函数的参数。
	 */
	static int SetSysInitializer(FuncPtrInit p_Initializer);
	/**
	 * \brief 初始化控制系统。
	 *
	 * 为系统启动做好准备
	 */
	static int SysInit(CSysInitParameters p_Param);
	/**
	 * \brief 启动控制系统。
	 *
	 * 启动后为正常运行状态，如无意外，应一直正常运行至机器停止。
	 */
	static int SysStart();
	/**
	 * \brief 停止控制系统。
	 *
	 * 将结束控制系统的所有任务，程序退出。
	 */
	static int SysStop();

	/**
	 * \brief 设置轨迹生成函数
	 * 轨迹生成函数需要的参数为：反馈数据，命令数据，消息数据，是否当前循环消息，四个参数构成
	 * 或者我们直接调用一个结构体，这个结构体将包含所有的数据
	 *
	 */
	static int SetTrajectoryGenerator(FuncPtrWork p_Generator);
//	/**
//	 * \brief 设置日志函数
//	 *
//	 * @param p_Logger 是记录数据的函数，它工作在实时系统里面
//	 */
//	static int SetLogger(FuncPtrWork p_Logger);

	/**
	 * \brief 设置数据更新时执行的函数
	 *
	 * @param p_DataUpdater 一个FuncPtrWork类型的函数指针，该函数指针在系统更新数据时自动被调用，
	 * 其参数就是当前更新好的数据。
	 *
	 */
	static int SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater);//TBD

	/**
	 * \brief 初始化非实时部分通讯
	 *
	 * 这个函数的调用需要发生在实时部分通讯初始化之后，调用顺序错误将导致该函数的初始化工作失败。
	 */
	static int SysInitCommunication();

	/**
	 * \brief 系统是否停止
	 *
	 * 返回表示系统是否停止的布尔值
	 * 该函数可以在RT以及NRT下调用
	 */
	static bool IsSysStopped();


	/**
	 * \brief 状态变化处理函数
	 *
	 *	在一个周期开始的时候调用
	 */
//	static int NRT_SysStateChangeHandler();
//	static bool NRT_IsCusMsg();
	static bool RT_IsCusMsg();


	///@}





	/**
	 * @name 数据传递函数
	 */
	///@{
	/**
	 * \brief 发送自定义数据标志（非实时环境）
	 *
	 */
	static int NRT_PrePostCustomMessage();
	/**
	 * \brief 发送自定义数据标志（实时环境）
	 */
	static int RT_PrePostCustomMessage();
	/**
	 * \brief 读取数据（非实时）
	 *
	 * @param p_MsgBuffer 指针,指向数据地址
	 * @param p_MsgBufSize 整数，表示数据大小
	 */
	static int NRT_GetMessageRaw(void* p_MsgBuffer,const unsigned int p_MsgBufSize);
	/**
	 * \brief 发送数据（非实时）
	 * @param p_MsgPointer 指针，指向数据地址
	 * @param p_MsgLength 整数，表示数据大小
	 */
	static int NRT_PostMessageRaw(const void* p_MsgPointer, const int p_MsgLength);
	/**
	 * \brief 发送数据（实时）
	 *
	 * @param p_MsgPointer 指针，指向数据地址
	 * @param p_MsgLength 整数，表示数据大小
	 */
	static int RT_PostMessageRaw(const void* p_MsgPointer,const int p_MsgLength);
	/**
	 * \brief 读取数据（实时）
	 * @param p_MsgBuffer 指针,指向数据地址
	 * @param p_MsgBufSize 整数，表示数据大小
	 */
	static int RT_GetMessageRaw(void* p_MsgBuffer,const int p_MsgBufSize);
	///@}


	/**
	 * @name 运动控制函数
	 * 以NRT开始的函数只能在非实时环境下调用
	 * 以RT开始的函数只能在实时环境下调用
	 * 模式设置函数可以在初始化前调用。
	 */
	///@{
	/**
	 * \brief 设置运行模式为P2P
	 */
	static int SetModeP2P();
	/**
	 * \brief 设置运行模式为循环位置模式
	 */
	static int SetModeCycPos();
	/**
	 * \brief 设置运行模式为循环速度模式
	 */
	static int SetModeCycVel();
	/**
	 * \brief 设置运行模式为循环力矩模式
	 */
	static int SetModeCycTor();

	/**
	 *
	 */
	static int NRT_MCPowerOff();
	static int NRT_MCHome();
	static int NRT_MCHomeToRunning();
	static int NRT_MCEnable();
	static int NRT_MCStop();
	//static int NRT_MCData(Aris::Control::CMachineData& p_machineData);

	/**
	 * \brief 返回电机当前状态
	 */
	static EServoState NRT_MCMachineState();
	static int RT_MCAxisFeedPos(int p_AxisId, int p_Pos);
	static int RT_MCAxisFeedVel(int p_AxisId, int p_Vel);
	static int RT_MCAxisFeedTor(int p_AxisId, short p_Tor);

	///@}


	/**
	 * @name 消息回调函数
	 * 当系统收到相应消息时会执行对应的回调函数。
	 */
	///@{
	//all functions in this lists will be called after NRT_RecvData recovery from blocking
	static int SetOnPowerOff(FuncPtrState p_Handler);
	static int SetOnPowerOffed(FuncPtrState p_Handler);

	static int SetOnStop(FuncPtrState p_Handler);
	static int SetOnStopped(FuncPtrState p_Handler);

	static int SetOnEnable(FuncPtrState p_Handler);
	static int SetOnEnabled(FuncPtrState p_Handler);

	static int SetOnHoming(FuncPtrState p_Handler);
	static int SetOnHomed(FuncPtrState p_Handler);

	static int SetOnHomedToRunning(FuncPtrState p_Handler);
	static int SetOnRunning(FuncPtrState p_Handler);

	static int SetOnStandStill(FuncPtrState p_Handler);
	static int SetOnEmergency(FuncPtrState p_Handler);


	static int SetOnCustomMessage(FuncPtrCustom p_Handler);
	///@}



	//generate a CONN_DATA,then send buffer
	static int RT_SendData(const void* p_ptrData, const int p_dataLength);
	static int RT_SendData(Aris::RT_CONTROL::RT_MSG &p_data);

	/*
	 * just copy from buffer, receiving happens in RT_UpdateData()
	 */
	//static int RT_RecvData(void* p_ptrData,const int p_dataLength);

	//only rt side need this function
	//
	static int RT_ReadData(void* p_ptrData,const int p_dataLength);

	static int NRT_SendData(Aris::Core::MSG &p_data);


	static int NRT_SendCommand(Aris::Core::MSG &p_data);


	static Aris::RT_CONTROL::RT_MSG* DataRecv;
	static Aris::RT_CONTROL::RT_MSG* DataSend;

private:
//	static bool NRT_CheckMessage();
//	static bool NRT_IsSysMsg();
//
//	static EServoState NRT_GetSysStateFromRT();


	/*
	 * Only used in a blocking thread
	 * a RT_CONN_DATA format
	 */
	static int NRT_RecvData(Aris::Core::MSG &p_data);
	static CSysBase* sysBase;

	static Aris::Core::MSG m_recvData;

	static void* NRT_Watcher(void*);
	static Aris::Core::THREAD m_threadNRTWatcher;

};

}

}

#endif /* CONTROLSYSTEMAPI_H_ */
