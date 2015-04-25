#ifndef ANALOGIODATA_H
#define ANALOGIODATA_H

#include "ecrt.h"

class CAnalogInputsData
{
public:
    int32_t values[4];
    int32_t isZeroingRequest;
};

class CAnalogOutputsData
{
public:
    uint16_t values[2];
};

#endif // ANALOGIODATA_H










