
#ifndef ARIS_CONTROL_H_
#define ARIS_CONTROL_H_

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
#include "Aris_ControlData.h"


/**\file Aris_Control.h
 * \brief The namespace RT_Control includes realtime control API
 *
 *By these functions, you can send and get messages between real-time task and normal Linux thread.
 *Function pointers are provided to enable you implement your own control algorithm in real-time or
 * non-real-time domain.
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
     * \brief Constructor of class ACTUATION
	 */
	ACTUATION();
	/**
     *\brief Destrcutor of class ACTUATION
	 */
	~ACTUATION();


	/**
     * @name Control system functions
	 */

	///@{

	/**
     * \brief Set a custom initializing function
	 *
     * This fucntion will be called when the system is initializing by function SysInit().
     * The parameter passed to the SysInit() will also pass to this custom intializing function.
	 *
     * @param p_Initializer is a pointer, the type is FuncPtrInit.
     *
	 */
	static int SetSysInitializer(FuncPtrInit p_Initializer);
	/**
     * \brief Init control system
	 *
     * Prepare to start the system
	 */
	static int SysInit(CSysInitParameters p_Param);
	/**
     * \brief Start the control system
	 *
     * This function will start the real-time task for the real-time control.
     * The default frequency of the real-time control loop is 1000Hz.
	 */
	static int SysStart();
	/**
     * \brief Stop the control system
	 *
     * Stop the real-time task.
	 */
	static int SysStop();
	/**
     * \brief Set trajectory generator
     *
     * The trajectory generator will be called in the real-time control loop.
     * The real time control algorithm of your own is supposed to be implemented here.
     * This function will assign the address of your custom trajectory generator (a function) to a function pointer.
     * @param p_Generator a function pointer with type FuncPtrWork, it will be called if it is not NULL
	 */
	static int SetTrajectoryGenerator(FuncPtrWork p_Generator);
	/**
     * \brief Set data update function
	 *
     * @param p_DataUpdater a function pointer. This function will run in non real-time domain
     * It will be called when some data is received from the real-time domain if it isn't NULL.
     *
	 *
	 */
	static int SetOnDataUpdateHandler(FuncPtrWork p_DataUpdater);

	/**
     * \brief Initailize communication
	 *
     * This function intialize the communication between real-time and non real-time domain
     * It shoud be called after the real-time system initialized
	 */
	static int SysInitCommunication();

	/**
     * \brief If the system has stopped
	 *
     * Return a bool value to show if the system has stopped.
	 */
	static bool IsSysStopped();

	/**
	 * \brief Post message from non-Real-time to Real-time
	 *
 	 * Normally, the real-time control system would need control commands or customized instructions
 	 * The parameter is a Core MSG type message class defined in Aris_Core. On real-time side, the message
 	 * could be dealt with user-defined functions TrajecotryGenerator.
	 *
	 */
	static int NRT_PostMsg(Aris::Core::MSG &p_data);

	/**
	 * \brief Post message from Real-time to non-Real-time
	 *
  	 * This function is used for transmitting machine information from RT side to NRT side. The function could be
  	 * applied in the customized functions defined by users, such as TrajectoryGenerator.
	 *
	 */
	static int RT_PostMsg(Aris::RT_CONTROL::RT_MSG &p_data);

	/**
	 * \brief Get the machine current status
	 *
  	 * Copy the current machine state to the given parameter of type EMachineState defined in Aris_ControlData
  	 *
	 */
 	static void GetMachineState(Aris::RT_CONTROL::EMachineState p_servostate);
	/**
	 * \brief Load the XML file for user-defined printing messages.
	 *
   	 *  The xml file should be in the current folder and named as PrintInfo.xml. An error would be given if
   	 *  loading the file encounters problems.This function is firstly called during system initialization,
   	 *  and users could call the function again if further modifications of printing messages would be necessary.
	 */
    static int Load_XML_PrintMessage();


private:

	static CSysBase* sysBase;

};

}

}

#endif /* CONTROLSYSTEMAPI_H_ */
