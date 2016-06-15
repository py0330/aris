#ifndef PLAN_H
#define PLAN_H
#include <aris.h>

namespace robot
{

    //for enable disable and home
    auto basicParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

    //for basic gait command parser
    auto basicGaitParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

    //the robot can recover to a default pose by using 'rc' command when it has found home. the pose can be changed in the RecoverParam
    struct RecoverParam final : public aris::server::GaitParamBase
    {
        int total_count_{ 10000 };//the total time(ms)
        double target_pin_[4]{0.52822, 0.52822, 0.543628, 0.543628};//the length of axis
        double target_pee_[4]{0.435,0,0,0};//the default pose
    };
    auto recoverParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    auto recoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

    //the robot can move to a profile pose by using 'mvpee' command
    struct MovePeeParam final: public aris::server::GaitParamBase
    {
        double target_y_{0.435},   target_z_{0},  target_a_{0}, target_c_{0};
        int total_count_{10000};
    };
    auto movePeeParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    auto movePeeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

    //the motions of robot can move a profile length by using 'mvpin' command
    struct MovePinParam final: public aris::server::GaitParamBase
    {
        double length_{0};
        double velocity_{0};
        int total_count_{10000};
    };
    auto movePinParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    auto movePinGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

    //the robot can move sin in one dimention
    struct SinPeeParam final: public aris::server::GaitParamBase
    {
        bool active_dimention_[4]{false,false,false,false};//y, z, a, c
        double amplitude_{0};
        double frequence_{0};
        int total_count_{10000};
    };
    auto sinPeeParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    auto sinPeeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

    //the motion of robot can move sin
    struct SinPinParam final: public aris::server::GaitParamBase
    {
        double amplitude_{0};
        double frequence_{0};
        int total_count_{10000};
    };
    auto sinPinParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    auto sinPinGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
}


#endif
