#ifndef ARIS_CORE_ERROR_HPP_
#define ARIS_CORE_ERROR_HPP_

#include <aris_lib_export.h>

//enum RetStatus {
//	SUCCESS = 0,
//	PARSE_EXCEPTION = -1,
//	PREPARE_EXCEPTION = -2,
//	SERVER_IN_ERROR = -10,
//	SERVER_NOT_STARTED = -11,
//	COMMAND_POOL_IS_FULL = -12,
//	PREPARE_CANCELLED = -40,
//	EXECUTE_CANCELLED = -41,
//
//	SLAVE_AT_INIT = -101,
//	SLAVE_AT_SAFEOP = -102,
//	SLAVE_AT_PREOP = -103,
//	SLAVE_AT_OP = -104,
//	MOTION_NOT_ENABLED = -501,
//	MOTION_POS_BEYOND_MIN = -502,
//	MOTION_POS_BEYOND_MAX = -503,
//	MOTION_POS_NOT_CONTINUOUS = -504,
//	MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER = -505,
//	MOTION_POS_FOLLOWING_ERROR = -506,
//	MOTION_POS_INFINITE = -507,
//	MOTION_VEL_BEYOND_MIN = -527,
//	MOTION_VEL_BEYOND_MAX = -528,
//	MOTION_VEL_NOT_CONTINUOUS = -529,
//	MOTION_VEL_FOLLOWING_ERROR = -530,
//	MOTION_VEL_INFINITE = -531,
//	MOTION_INVALID_MODE = -541,
//
//	PLAN_OVER_TIME = -1001,
//	INVERSE_KINEMATIC_POSITION_FAILED = -1002,
//
//	PROGRAM_EXCEPTION = -2000,
//};

/*
#define ARIS_ERROR_PLAN_PARSE_EXCEPTION         aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_PREPARE_EXCEPTION       aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_SERVER_IN_ERROR         aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_SERVER_NOT_STARTED      aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_COMMAND_POOL_IS_FULL    aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_PREPARE_CANCELLED       aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_EXECUTE_CANCELLED       aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}

#define ARIS_ERROR_PLAN_SLAVE_AT_INIT           aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_SLAVE_AT_SAFEOP         aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_SLAVE_AT_PREOP          aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_SLAVE_AT_OP             aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_MOTION_NOT_ENABLED      aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_MOTION_POS_BEYOND_MIN   aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}
#define ARIS_ERROR_PLAN_MOTION_POS_BEYOND_MAX   aris::core::LogLvl::kError, -1, {"some error:%d", "一些错误：%d"}


*/

#define ARIS_ERROR_PLAN_TOOL_WOBJ_PART_SET_TWICE      aris::core::LogLvl::kError, -2001, {"PART %s /'s pose has been set twice", "杆件 %s 的位姿被设置了两次"}
#define ARIS_ERROR_PLAN_TOOL_WOBJ_PART_UNRWCOGNIZED   aris::core::LogLvl::kError, -2002, {"PART unrecognized", "杆件不连接末端，不应被设置"}



#endif // ARIS_CORE_DATA_STRUCTURE_HPP_