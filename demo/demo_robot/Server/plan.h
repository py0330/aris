#ifndef PLAN_H
#define PLAN_H
#include <aris.h>

namespace robot
{

    auto basicParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

    auto testParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    auto testGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

}


#endif
