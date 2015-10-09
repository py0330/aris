/*
 * Aris_ControlData.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: leo
 */

#include "Aris_ControlData.h"
#include <cstdio>
#include <cstring>


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
