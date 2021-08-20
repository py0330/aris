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
#include <aris/core/reflection.hpp>
#include <aris/core/serialization.hpp>
#include <aris/core/data_structure.hpp>
#include <aris/core/signal_slots.hpp>

#endif // ARIS_H_

// to do list 2019-04-14:
// - use keepalive for socket                                             check
// - make better registration                                             check
// - check motion enable state                                            check
// - check control pdos config successfully and throw exception           half check
// - remove margin(0.005) of manual plan
// - fix moveAbsolute2 bug on linux                                       check
// - generate 6-axis from DH                                              check
// - make dynamic calibration as a plan                                   check
// - use extern template to reduce compile time                           

// to do list 2019-05-07:
// - svd decomposition
// - check ethercat link                                                  check
// - scan slave info to motion                                            check
// - generate ur from DH
// - 7-axis inverse solver                                                check
// - generate stewart from fix points
// - judge mechanism is 7-axis 6-axis ur stewart
// - use pre-compiled header(stdafx)
// - exclusive plan option                                                not active
// - check scan result when controller start                              check

// to do list 2019-05-14:
// - make pdo of curent/targetpos/... can be customized                   use toq instead of cur
// - fix server stastic data bug: when lose some frame, it corrupt        check
// - check motion respectively                                            check
// - make startWebSock and runCmdLine available                           check
// - make callbacks before and after plans                                check
// - object insert & remove child node                                    not active
// - make pdo init value to zero                                          check

// to do list 2019-06-03:
// - make xml support chinese                                             check
// - check if xenomai can be started from sub-thread                      use runCmdLine to work
// - scan from esi file                                                   check
// - make virtual axis                                                    check
// - add tool coordinate easy                                             not active

// to do list 2019-06-17:
// - make SubRefPool                                                      check
// - make ChildRefPool                                                    check

// to do list 2019-06-24:
// - check of require ENABLE MODE8\9\10                                   check
// - config sdo
// 
// to do list 2019-07-24
// - make interface of server                                             not active                                     
// - check pdo instead of corrupt                                         
// - make motionPool() available before start controller                  not active
// - make new parsing system, which parse if/else/while ...
//
// to do list 2021-08-20
// - reflect functions
// - svd...                                       
