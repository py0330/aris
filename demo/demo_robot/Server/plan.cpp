#include "plan.h"
#define Motion_Num 18

namespace plan
{
    auto basicParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        aris::server::BasicFunctionParam param;

        for (auto &i : params)
        {
            if (i.first == "all")
            {
                std::fill_n(param.active_motor_, 18, true);
            }
            else if (i.first == "first")
            {
                std::fill_n(param.active_motor_, 18, false);
                std::fill_n(param.active_motor_ + 0, 3, true);
                std::fill_n(param.active_motor_ + 6, 3, true);
                std::fill_n(param.active_motor_ + 12, 3, true);
            }
            else if (i.first == "second")
            {
                std::fill_n(param.active_motor_, 18, false);
                std::fill_n(param.active_motor_ + 3, 3, true);
                std::fill_n(param.active_motor_ + 9, 3, true);
                std::fill_n(param.active_motor_ + 15, 3, true);
            }
            else if (i.first == "motion_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + 18, false);
                param.active_motor_[cs.model().motionAtAbs(id).absID()] = true;
            }
            else if (i.first == "physical_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + 18, false);
                param.active_motor_[cs.model().motionAtPhy(id).absID()] = true;
            }
            else if (i.first == "slave_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + 18, false);
                if (cs.model().motionAtSla(id).absID() >= cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\", this slave is not motion");
                param.active_motor_[cs.model().motionAtSla(id).absID()] = true;
            }
            else if (i.first == "leg")
            {
                auto leg_id = std::stoi(i.second);
                if (leg_id<0 || leg_id>5)throw std::runtime_error("invalid param in parseRecover func");

                std::fill_n(param.active_motor_, 18, false);
                std::fill_n(param.active_motor_ + leg_id * 3, 3, true);
            }
        }

        msg_out.copyStruct(param);
    }

    auto testParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        aris::server::GaitParamBase param;

        for (auto &i : params)
        {
            if (i.first == "all")
            {
                std::fill_n(param.active_motor_, 18, true);
            }
            else if (i.first == "motion_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + 18, false);
                param.active_motor_[cs.model().motionAtAbs(id).absID()] = true;
            }
            else if (i.first == "physical_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + 18, false);
                param.active_motor_[cs.model().motionAtPhy(id).absID()] = true;
            }
            else if (i.first == "slave_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + 18, false);
                if (cs.model().motionAtSla(id).absID() >= cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\", this slave is not motion");
                param.active_motor_[cs.model().motionAtSla(id).absID()] = true;
            }
        }

        msg_out.copyStruct(param);
    }
    auto testGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &param = static_cast<const aris::server::GaitParamBase&>(param_in);

    static double begin_pin[Motion_Num];
    if(param.count_==0)
     {
        for(int i=0;i<Motion_Num;i++){
            std::size_t slaID=model.motionPool().at(i).slaID();
            begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().slavePool().at(slaID).rxData()).feedback_pos;
        }
    }

    double pin[Motion_Num];
    std::copy(begin_pin,begin_pin+Motion_Num,pin);

    //pin[1] = 0.002*std::sin(param.count/1000.0*2*PI)+begin_pin[1];
    for(int i=0;i<Motion_Num;i++)
    {
        pin[i] = 0.02*param.count_/1000.0+begin_pin[i];
        model.motionPool().at(i).setMotPos(pin[i]);
    }


    return 10000 - param.count_ -1;
}

}
