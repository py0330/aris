#include "plan.h"
#include "kinematic.h"
#define Motion_Num 3

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
    auto basicGaitParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
           msg_out.copyStruct(aris::server::GaitParamBase());
    }

    auto recoverParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        msg_out.copyStruct(robot::RecoverParam());
    }
    auto recoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
    {
        auto &robot=static_cast<robot::Robot &>(model);
        auto &param = static_cast<const robot::RecoverParam&>(param_in);

        static double begin_pin[Motion_Num];
        if(param.count_==0)
         {
            for(int i=0;i<Motion_Num;i++){
                std::size_t slaID=model.motionAtAbs(i).slaID();
                begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
            }
        }

        int leftCount = 0;
        int rightCount = param.total_count_ ;
        double s = -(PI / 2)*cos(PI * (param.count_ - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

        double pin[Motion_Num];
        for(int i=0;i<Motion_Num;++i)pin[i] = begin_pin[i] * (cos(s) + 1) / 2 + param.target_pin_[i] * (1 - cos(s)) / 2;

        robot.setPin(pin);

       if(param.total_count_- param.count_ - 1==0)
       {
           robot.setPee(param.target_pee_);
       }
       return param.total_count_- param.count_ - 1;
    }

    auto sinPeeParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        robot::SinPeeParam param;

        for (auto &i : params)
        {
            if (i.first == "totalCount")
            {
                    param.total_count_ = std::stoi(i.second);
            }
            else if (i.first == "active")
            {
                std::fill_n(param.active_dimention_,4,false);
                if(i.second == "y")
                    param.active_dimention_[0]=true;
                else if(i.second == "z")
                    param.active_dimention_[1]=true;
                else if(i.second == "a")
                    param.active_dimention_[2]=true;
                else if(i.second == "c")
                    param.active_dimention_[3]=true;
                else
                {
                    throw std::runtime_error("invalid param in 'active', the param must be 'y', 'z', 'a', or 'c'. ");
                }
            }
            else if (i.first == "amplitude")
            {
               param.amplitude_ = std::stod(i.second);
            }
            else if (i.first == "frequence")
            {
               param.frequence_= std::stod(i.second);
            }
            else
            {
                throw std::runtime_error("unknown param in move parse func");
            }
        }

        msg_out.copyStruct(param);
    }
    auto sinPeeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
    {
        auto &robot=static_cast<robot::Robot&>(model);
        auto &param=static_cast<const robot::SinPeeParam&>(param_in);

        static double begin_pee[4];
        if(param.count_==0)
         {
            std::copy(robot.getpee(),robot.getpee()+4,begin_pee);
        }

        double pee[4];

        double relative=param.amplitude_*std::sin(2*PI*param.frequence_*param.count_/1000.0);

        for(int i=0;i<4;i++)
        {
            if(param.active_dimention_[i])
                pee[i] = begin_pee[i]+relative;
            else
                pee[i] = begin_pee[i];
        }

        robot.setPee(pee);
        return param.total_count_- param.count_ - 1;
    }

    auto sinPinParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        robot::SinPinParam param;

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
            else if (i.first == "totalCount")
            {
                param.total_count_ = std::stoi(i.second);
            }
            else if (i.first == "amplitude")
            {
               param.amplitude_ = std::stod(i.second);
            }
            else if (i.first == "frequence")
            {
               param.frequence_= std::stod(i.second);
            }
            else
            {
                throw std::runtime_error("unknown param in move parse func");
            }
        }

        msg_out.copyStruct(param);
    }
    auto sinPinGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
    {
        auto &robot=static_cast<robot::Robot&>(model);
        auto &param=static_cast<const robot::SinPinParam&>(param_in);

        static double begin_pin[Motion_Num];
        if(param.count_==0)
         {
            for(int i=0;i<Motion_Num;i++){
                std::size_t slaID=model.motionAtAbs(i).slaID();
                begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
            }
        }

        double pin[Motion_Num];
        double relative=param.amplitude_*std::sin(2*PI*param.frequence_*param.count_/1000.0);
        for(int i=0;i<Motion_Num;i++)
        {
            if(param.active_motor_[i])
                pin[i] = relative+begin_pin[i];
            else
                pin[i] = begin_pin[i];
        }

        robot.setPin(pin);

        return param.total_count_ - param.count_ -1;
    }

    auto moveParse(const aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        robot::MoveParam param;

        if(params.find("x")!=params.end())
        {
            param.total_count_=std::stoi(params.find("totalCount")->second);
            param.target_x_=std::stod(params.find("y")->second);
            param.target_y_=std::stod(params.find("y")->second);
            param.target_z_=std::stod(params.find("z")->second);
            param.mode_=1;
        }
        else if(params.find("length")!=params.end())
        {
            param.total_count_=std::stoi(params.find("totalCount")->second);
            param.length_=std::stod(params.find("length")->second);
            if (params.find("all")!=params.end())
            {
                std::fill_n(param.active_motor_, Motion_Num, true);
            }
            else if (params.find("motion_id")!=params.end())
            {
                std::size_t id{ std::stoul(params.find("motion_id")->second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param 'motion_id'.");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                param.active_motor_[cs.model().motionAtAbs(id).absID()] = true;
            }
            else if (params.find("physical_id")!=params.end())
            {
                std::size_t id{ std::stoul(params.find("physical_id")->second) };
                if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param 'physical_id'.");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                param.active_motor_[cs.model().motionAtPhy(id).absID()] = true;
            }
            else if (params.find("slave_id")!=params.end())
            {
                std::size_t id{ std::stoul(params.find("slave_id")->second) };
                if (id < 0 || id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param 'slave_id'.");
                std::fill(param.active_motor_, param.active_motor_ + Motion_Num, false);
                if (cs.model().motionAtSla(id).absID() >= cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param 'slave_id', this slave is not motion");
                param.active_motor_[cs.model().motionAtSla(id).absID()] = true;
            }

            if(params.find("force")->second == "no")
            {
                param.if_check_pos_max_=true;
                param.if_check_pos_min_=true;
            }
            else if(params.find("force")->second == "yes")
            {
                param.if_check_pos_max_=false;
                param.if_check_pos_min_=false;
            }
            else
            {
                throw std::runtime_error("invalid param in 'force', the param must be 'yes' or 'no'. ");
            }

            param.mode_=2;
        }
        else if(params.find("L1")!=params.end())
        {
            param.total_count_=std::stoi(params.find("totalCount")->second);
            param.l1_=std::stod(params.find("L1")->second);
            param.l2_=std::stod(params.find("L2")->second);
            param.l3_=std::stod(params.find("L3")->second);
            param.mode_=3;
        }
        else
        {
            throw std::runtime_error("unknown param in move parse func");
        }
        msg_out.copyStruct(param);
    }
    auto moveGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
    {
        auto &robot=static_cast<robot::Robot&>(model);
        auto &param=static_cast<const robot::MoveParam&>(param_in);
        static double begin_pin[Motion_Num];
        static double begin_pee[3];
        double pin[Motion_Num];

        int leftCount = 0;
        int rightCount = param.total_count_ ;
        double s = -(PI / 2)*cos(PI * (param.count_ - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

        switch (param.mode_) {
        case 1:
            if(param.count_==0)
             {
                std::copy(robot.getpee(),robot.getpee()+3,begin_pee);
            }

            double pee[3];
            pee[0] = begin_pee[0] * (cos(s) + 1) / 2 + param.target_x_ * (1 - cos(s)) / 2;
            pee[1] = begin_pee[1] * (cos(s) + 1) / 2 + param.target_y_ * (1 - cos(s)) / 2;
            pee[2] = begin_pee[2] * (cos(s) + 1) / 2 + param.target_z_ * (1 - cos(s)) / 2;
            robot.setPee(pee);
            break;
        case 2:
            if(param.count_==0)
             {
                for(int i=0;i<Motion_Num;i++){
                    std::size_t slaID=model.motionAtAbs(i).slaID();
                    begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
                }
            }

            for(int i=0;i<Motion_Num;i++)
                pin[i] = begin_pin[i] + param.length_ * (1 - cos(s)) / 2;
            robot.setPin(pin);
            break;
        case 3:
            if(param.count_==0)
             {
                for(int i=0;i<Motion_Num;i++){
                    std::size_t slaID=model.motionAtAbs(i).slaID();
                    begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
                }
            }

            pin[0] = begin_pin[0]  + param.l1_* (1 - cos(s)) / 2;
            pin[1] = begin_pin[1]  + param.l2_* (1 - cos(s)) / 2;
            pin[2] = begin_pin[2]  + param.l3_* (1 - cos(s)) / 2;
            robot.setPin(pin);
            break;
        default:
            break;
        }
        return param.total_count_- param.count_ - 1;
    }

}
