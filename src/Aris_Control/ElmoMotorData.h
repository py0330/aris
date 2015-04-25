#ifndef ELMOMOTORDATA_H
#define ELMOMOTORDATA_H

#include "ecrt.h"
#include "ElmoMotorEnum.h"

namespace ElmoMotor
{

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
    bool IsPTPRequested;
    bool IsCmdComplete;    
    
    MotorCommand MotorCmd;

    void operator=(const CElmoMotorData &other);

    CElmoMotorData(short int mode=8,
        int position=0,
        int velocity=0,
        short int current=0,
        short int torque=0,
        short int control=0,
        short int status=0,
        short int maxTorque=0,
        int digitalInputs=0,
        bool cmdComplete=true,
        bool isPTPRequested=false
                   ) //at first IsCmdComplete should be true, so we can give new command
        :Mode(mode)
        ,Position(position)
        ,Velocity(velocity)
        ,Current(current)
        ,Torque(torque)
        ,ControlWord(control)
        ,StatusWord(status)
        ,MaxTorque(maxTorque)
        ,DigitalInputs(digitalInputs)
        ,IsCmdComplete(cmdComplete)
        ,IsPTPRequested(isPTPRequested)
 
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
        ,IsCmdComplete(other.IsCmdComplete)
        ,IsPTPRequested(other.IsPTPRequested)
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

}

#endif // ELMO_MOTOR_DATA_H
