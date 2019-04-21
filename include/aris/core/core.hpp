#ifndef ARIS_CORE_H_
#define ARIS_CORE_H_

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>
#include <aris/core/msg.hpp>
#include <aris/core/log.hpp>
#include <aris/core/expression_calculator.hpp>
#include <aris/core/socket.hpp>
#include <aris/core/pipe.hpp>
#include <aris/core/command.hpp>

#endif // ARIS_H_

// to do list 2019-04-14:
// - improve socket on linux
// - improve check motions of control server
// - improve manual_plan, remove margin of 0.005
// - improve robot generation
// - improve moveAbsolute2
// - check control pdos config successfully and throw exception           half check
// - make better registration                                             check
// - find bugs of Qianchao                                                check
// - make dynamic calibration as a plan                                   


// to do list 2019-04-21:
// - use extern template to reduce compile time