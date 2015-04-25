#include "ElmoMotorData.h"

using namespace ElmoMotor;

void CElmoMotorData::operator =(const CElmoMotorData &other)
{
    Mode            =   other.Mode;

    Position        =   other.Position;
    Velocity        =   other.Velocity;
    Torque          =   other.Torque;
    Current         =   other.Current;

    ControlWord     =   other.ControlWord;
    StatusWord      =   other.StatusWord;

    MaxTorque       =   other.MaxTorque;
    DigitalInputs   =   other.DigitalInputs;

    IsCmdComplete   =   other.IsCmdComplete;
    MotorCmd        =   other.MotorCmd;
    IsPTPRequested  =   other.IsPTPRequested;
}
