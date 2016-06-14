#include "plan.h"
#define Motion_Num 4

namespace robot
{
    auto basicParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        aris::server::BasicFunctionParam param;

        for (auto &i : params)
        {
            if (i.first == "all")
            {
                std::fill_n(param.active_motor_, Motion_Num, true);
            }
            else if (i.first == "motion_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                param.active_motor_[cs.model().motionAtAbs(id).absID()] = true;
            }
            else if (i.first == "physical_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                param.active_motor_[cs.model().motionAtPhy(id).absID()] = true;
            }
            else if (i.first == "slave_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                if (cs.model().motionAtSla(id).absID() >= cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\", this slave is not motion");
                param.active_motor_[cs.model().motionAtSla(id).absID()] = true;
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
                std::fill_n(param.active_motor_, Motion_Num, true);
            }
            else if (i.first == "motion_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                param.active_motor_[cs.model().motionAtAbs(id).absID()] = true;
            }
            else if (i.first == "physical_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                param.active_motor_[cs.model().motionAtPhy(id).absID()] = true;
            }
            else if (i.first == "slave_id")
            {
                std::size_t id{ std::stoul(i.second) };
                if (id < 0 || id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
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
                std::size_t slaID=model.motionAtAbs(i).slaID();
                begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
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
