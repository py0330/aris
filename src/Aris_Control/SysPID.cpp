/*
 * SysPID.cpp

 *
 *  Created on: Nov 24, 2014
 *      Author: leo
 */


#include "SysPID.h"


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








