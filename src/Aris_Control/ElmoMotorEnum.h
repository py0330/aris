#ifndef ELMOMOTORENUM_H
#define ELMOMOTORENUM_H

namespace ElmoMotor
{

    enum MOTOR_STATE
    {
        STA_POWEREDOFF = 0,
        STA_STOPPED    = 1,
        STA_ENABLED    = 3,
        STA_RUNNING    = 7,
        STA_FAULT      = 8
    };

    enum MOTOR_CMD
    {
        CMD_POWEROFF = 0,
        CMD_STOP     = 1,
        CMD_ENABLE   = 2,
        CMD_RUNNING  = 3,
        CMD_PLAIN    = 4 //just write all your data
    };

    enum OPERATION_MODE
    {
        OM_NOMODE     = -1,
        OM_STANDSTILL = 0,
        OM_HOMING     = 6,
        OM_PROFILEPOS = 1,
        OM_CYCLICPOS  = 8,
        OM_CYCLICVEL  = 9,
        OM_CYCLICTORQ = 10,
        OM_OTHER      = 18
    };

    enum SUB_STATE
    {
        SUB_READY = 0, SUB_BUSY = 1, SUB_FINISHED = 2
    };

    enum POSITIONING_MODE
    {
        PM_RELATIVE = 0, PM_ABSOLUTE = 1
    };

    typedef struct _MotorCommand
    {
        MOTOR_CMD command;
        OPERATION_MODE operationMode;
    } MotorCommand;
}


#endif // ELMO_MOTOR_ENUM_H
