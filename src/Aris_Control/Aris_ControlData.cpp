/*
 * Aris_ControlData.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: leo
 */

#include "Aris_ControlData.h"
#include "GlobalConfiguration.h"
#include "stdio.h"
#include <string.h>


char ServoStateName[7][20]=
	{
		"EMSTAT_NONE",
		"EMSTAT_POWEREDOFF",
		"EMSTAT_STOPPED",
		"EMSTAT_ENABLED",
		"EMSTAT_RUNNING",
		"EMSTAT_HOMING",
		"EMSTAT_FAULT",
	};

namespace Aris
{
namespace RT_CONTROL
{
CSysInitParameters::CSysInitParameters()
{

//    m_motorGeneralSettings.homeMode = HOMING_MODE;
//    m_motorGeneralSettings.homeAccel = HOMING_ACC;
//    m_motorGeneralSettings.homeLowSpeed = HOMING_LO_SPEED;
//    m_motorGeneralSettings.homeHighSpeed = HOMING_HI_SPEED;
//    m_motorGeneralSettings.p2pMaxSpeed = PTP_MAX_SPEED;
//    m_motorGeneralSettings.p2pSpeed = PTP_SPEED;
//    m_motorGeneralSettings.nsPerCyclePeriod = PERIOD_NS_CORE;

    motorNum 			= ACTUAL_MOTOR_NUMBER;
    homeMode 			= HOMING_MODE;
    homeAccel 			= HOMING_ACC;
    homeLowSpeed 		= HOMING_LO_SPEED;
    homeHighSpeed 		= HOMING_HI_SPEED;
    p2pMaxSpeed 		= PTP_MAX_SPEED;
    p2pSpeed 			= PTP_SPEED;
    nsPerCyclePeriod 	= PERIOD_NS_CORE;
    homeTorqueLimit     = HOMING_TORQUE_LIMIT;
    homeOffsets=NULL;
    driverIDs=NULL;
  /*  for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
    {
    	homeOffsets[i]=HEXBOT_HOME_OFFSETS_RESOLVER[i];
    	driverIDs[i]=i;
    }
*/
};

CSysInitParameters::~CSysInitParameters()
{

};

//CSysInitParameters& CSysInitParameters::operator=(const CSysInitParameters& other)
//{
//	return *this;
//};

CMotorData& CMotorData::operator=(const CMotorData& other)
{
	return *this;
};

CMachineData& CMachineData::operator=(const CMachineData& other)
{
	return *this;
};

RT_MSG::RT_MSG()
{

	RT_MSG::m_ptrData=NULL;
}

RT_MSG::~RT_MSG()
{

	RT_MSG::m_ptrData=NULL;
}

void RT_MSG::operator=(const RT_MSG& other)
{
	memcpy(m_ptrData,other.m_ptrData,MSG_HEADER_LENGTH+other.GetLength());

}

void RT_MSG::SetLength(unsigned int dataLength)
{
	*((unsigned int *)m_ptrData)=dataLength;

}

void RT_MSG::SetMsgID(int msgID)
{
	*((int *)(m_ptrData+4))=msgID;
}

unsigned int RT_MSG::GetLength() const
{
	return *((unsigned int *)m_ptrData);
}

int RT_MSG::GetMsgID() const
{
	return *((int *)(m_ptrData+4));
}

char* RT_MSG::GetDataAddress() const
{
	if(GetLength()>0)
		return &m_ptrData[RT_MSG_HEADER_LENGTH];
	else
		return 0;
}

void RT_MSG::SetType(long long type)
{
	*((long long*)(m_ptrData+8))=type;
}

long long RT_MSG::GetType() const
{
	return *((long long *)(m_ptrData+8));
}

void RT_MSG::Copy(const void * fromThisMemory, const unsigned int dataLength)
{
	SetLength(dataLength);
	if(dataLength > 0)
	{
		memcpy(GetDataAddress(),fromThisMemory,dataLength);
	}
}

void RT_MSG::Copy(const void* fromThisMemory)
{
	if(GetLength()>0)
	{
		memcpy(GetDataAddress(),fromThisMemory,GetLength());
	}
}

void RT_MSG::CopyAt(const void* fromThisMemory,unsigned int dataLength,unsigned int atThisPositionInMsg)
{
	if((dataLength+atThisPositionInMsg)>GetLength())
	{
		SetLength(dataLength+atThisPositionInMsg);
	}

	if(dataLength>0)
	{
		memcpy(&GetDataAddress()[atThisPositionInMsg],fromThisMemory,dataLength);
	}
}

void RT_MSG::CopyMore(const void * fromThisMemory,unsigned int dataLength)
{
	unsigned int pos = GetLength();
	if(dataLength>0)
	{
		SetLength(GetLength()+dataLength);
		memcpy(&GetDataAddress()[pos],fromThisMemory,dataLength);
	}
}

void RT_MSG::Paste(void * toThisMemory, const unsigned int dataLength)
{
	if((dataLength>0)&&(GetLength()>0))
	{
		if(dataLength > GetLength())
		{
			memcpy(toThisMemory,GetDataAddress(),GetLength());
		}
		else
		{
			memcpy(toThisMemory,GetDataAddress(),dataLength);
		}
	}
}

void RT_MSG::Paste(void * toThisMemory)
{
	if(GetLength()>0)
	{
		memcpy(toThisMemory,GetDataAddress(),GetLength());
	}
}

void RT_MSG::PasteAt(void * toThisMemory, const unsigned int dataLength,const unsigned int atThisPositionInMsg)
{
	int actualLength=((atThisPositionInMsg + dataLength)> GetLength() ? (GetLength() - atThisPositionInMsg):dataLength);
	if(actualLength > 0)
	{
		memcpy(toThisMemory,&GetDataAddress()[atThisPositionInMsg],actualLength);
	}
}
}

}
