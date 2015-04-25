/*
 * SysPID.h
 *
 *  Created on: Nov 24, 2014
 *      Author: leo
 */

#ifndef SYSPID_H_
#define SYSPID_H_

#include "hardware.h"
#include "GlobalConfiguration.h"

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



#endif /* SYSPID_H_ */
