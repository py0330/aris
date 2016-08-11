#include "plan.h"
#include "kinematic.h"
#include "newslave.h"
#define Motion_Num 3
#define Dimention 3

namespace robot
{
    auto basicParse(aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
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
            else if (cmd=="en" && i.first== "mode")
            {
                for(int count=0;count<Motion_Num;count++)
                {
                    std::size_t slaID=cs.model().motionAtAbs(count).slaID();
                    auto &tx_motion_data = static_cast<aris::control::TxMotionData&>(cs.controller().txDataPool().at(slaID));
                    tx_motion_data.mode=static_cast<std::uint8_t>(std::stoul(i.second));
                }
            }
            else
            {
                throw std::runtime_error("invalid param.");
            }
        }

        msg_out.copyStruct(param);
    }

    auto basicGaitParse(aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        msg_out.copyStruct(aris::server::GaitParamBase());
    }

    auto logParse(aris::server::ControlServer &cs,const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        aris::server::GaitParamBase logparam;
        for (auto &param : params)
        {
            if(param.first == "switch")
            {
                if(param.second == "on")
                {
                    cs.controller().dataLogger().prepair(params.find("name")->second);//must be in Nrt
                    cs.controller().dataLogger().start();
                }
                else if(param.second =="off")
                {
                    cs.controller().dataLogger().stop();
                }
                else
                    throw std::runtime_error("unknown param in log parse func");
            }
        }
        msg_out.copyStruct(logparam);
    }

    auto helpParse(aris::server::ControlServer &cs,const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        aris::server::GaitParamBase helpparam;

        std::function<bool(aris::core::ParamBase &, std::string paramName, bool isfull)> findParamHelp = [&findParamHelp](aris::core::ParamBase &param, std::string paramName, bool isfull)->bool
        {
            static bool isExist = false;
            if (param.name() == paramName)
            {
                std::cout << param.help(isfull, 0);
                isExist = true;
            }
            else
            {
                for (auto &subparam : param)
                {
                    findParamHelp(subparam, paramName,isfull);
                }
            }
            return isExist;

        };

        std::string commandName{};
        std::string paramName={};
        bool isfull=false;
        for (auto &param : params)
        {
            if(param.first == "command")
            {
                commandName=param.second;
            }
            else if(param.first == "param")
            {
                paramName=param.second;
            }
            else if(param.first == "full")
            {
                if(param.second == "yes")
                {
                    isfull=true;
                }
                else if(param.second =="no")
                {
                    isfull=false;
                }
                else
                    throw std::runtime_error("unknown param in 'full' param, it must be 'yes' or 'no'.");
            }
            else
                throw std::runtime_error("unknown param in 'help' command.");
        }

        if(commandName=="")
        {
            if(paramName=="")
                std::cout<<cs.widgetRoot().commandParser().help();
            else
                throw std::runtime_error("need a command name to look for the param.");
        }
        else
        {
            auto command = cs.widgetRoot().commandParser().commandPool().findByName(commandName);
            if(command != cs.widgetRoot().commandParser().commandPool().end())
            {
                if(paramName=="")
                    std::cout<<command->help(isfull,0);
                else
                {
                    for (auto &param : *command)
                    {
                        if(!findParamHelp(param, paramName,isfull))
                            throw std::runtime_error("invalid param name.");
                    }
                }

            }
            else
                throw std::runtime_error("invalid command name.");
        }
        msg_out.copyStruct(helpparam);
    }

    auto recoverParse(aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
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
        for(int i=0;i<Motion_Num;++i)
            pin[i] = begin_pin[i] * (cos(s) + 1) / 2 + param.target_pin_[i] * (1 - cos(s)) / 2;

        robot.setPin(pin);

       if(param.total_count_- param.count_ - 1==0)
       {
           robot.setPee(param.target_pee_);
       }
       return param.total_count_- param.count_ - 1;
    }

    auto moveParse(aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
    {
        robot::MoveParam param;

        if(params.find("x")!=params.end())
        {
            param.total_count_=std::stoi(params.find("totalCount")->second);
            param.target_x_=std::stod(params.find("x")->second);
            param.target_y_=std::stod(params.find("y")->second);
            param.target_A_=std::stod(params.find("a")->second);
            param.mode_=1;
        }
        else if(params.find("round")!=params.end())
        {
            param.total_count_=std::stoi(params.find("totalCount")->second);
            param.round_=std::stod(params.find("round")->second);
            param.kp_=std::stod(params.find("kp")->second);
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
        else if(params.find("a1")!=params.end())
        {
            param.total_count_=std::stoi(params.find("totalCount")->second);
            param.a1_=std::stod(params.find("a1")->second);
            param.a2_=std::stod(params.find("a2")->second);
            param.a3_=std::stod(params.find("a3")->second);
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
        static double begin_pee[Dimention];
        double pin[Motion_Num];
        double vin[Motion_Num];

        int leftCount = 0;
        int rightCount = param.total_count_ ;
        double s = -(PI / 2)*cos(PI * (param.count_ - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

        switch (param.mode_) {
        case 1:
            if(param.count_==0)
             {
                std::copy(robot.getPee(),robot.getPee()+Dimention,begin_pee);
            }

            double pee[Dimention];
            pee[0] = begin_pee[0] * (cos(s) + 1) / 2 + param.target_x_ * (1 - cos(s)) / 2;
            pee[1] = begin_pee[1] * (cos(s) + 1) / 2 + param.target_y_ * (1 - cos(s)) / 2;
            pee[2] = begin_pee[2] * (cos(s) + 1) / 2 + param.target_A_ * (1 - cos(s)) / 2;
            robot.setPee(pee);
            break;
        case 2:
        {
            if(param.count_==0)
             {
                for(int i=0;i<Motion_Num;i++){
                    std::size_t slaID=model.motionAtAbs(i).slaID();
                    begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
                }
            }


            for(int i=0;i<Motion_Num;i++)
            {
                //position mode
                pin[i] = begin_pin[i] + param.round_ * (1 - cos(s)) / 2;

                std::size_t slaID=model.motionAtAbs(i).slaID();
                auto &rx_motion_data=static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID));
                //velocity mode
                if(rx_motion_data.mode==9)
                {
                    double der_s=(PI*PI/2/(rightCount - leftCount)*1000)*std::sin(PI * (param.count_ - leftCount + 1) / (rightCount - leftCount));
                    vin[i] = (param.round_ *std::sin(s))*der_s/2;

                    //offset
                    double vel_offset = param.kp_*(pin[i]-rx_motion_data.feedback_pos);
                    vin[i] =vin[i]+vel_offset;
                }

            }

            //record
            auto &rx_record_data = static_cast<robot::RxRecordData&>(param.cs_->controller().rxDataPool().at(3));
            rx_record_data.target_pos_=pin[0];

            robot.setPin(pin);
            robot.setVin(vin);


            break;
        }
        case 3:
            if(param.count_==0)
             {
                for(int i=0;i<Motion_Num;i++){
                    std::size_t slaID=model.motionAtAbs(i).slaID();
                    begin_pin[i] = static_cast<aris::control::RxMotionData&>(param.cs_->controller().rxDataPool().at(slaID)).feedback_pos;
                }
            }

            pin[0] = begin_pin[0]  + param.a1_* (1 - cos(s)) / 2;
            pin[1] = begin_pin[1]  + param.a2_* (1 - cos(s)) / 2;
            pin[2] = begin_pin[2]  + param.a3_* (1 - cos(s)) / 2;
            robot.setPin(pin);
            break;
        default:
            break;
        }
        return param.total_count_- param.count_ - 1;
    }

}
