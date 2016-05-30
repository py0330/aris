
#include <aris_control.h>
#include <iostream>

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif
#define PI 3.141592653

aris::control::Controller controller;
aris::control::Pipe<std::uint8_t> my_pipe;//pipe for sent command
static std::string command_in("idle");//command input from terminal
static std::uint8_t command_rec{aris::control::Motion::IDLE};
static std::uint8_t command{aris::control::Motion::IDLE};
static int cmd_count{0};
static bool cmd_success{false};

void tg()
{
    my_pipe.recvInRT(command_rec);

    if(command != command_rec){
        cmd_count=0;
        command=command_rec;
        cmd_success=false;
    }
    else{
        cmd_count++;
    }


    auto &txmotiondata=static_cast<aris::control::TxMotionData&>(controller.slavePool().at(0).txData());
    auto &rxmotiondata=static_cast<aris::control::RxMotionData&>(controller.slavePool().at(0).rxData());
    //auto &txmotiondata=dynamic_cast<aris::control::Motion&>(controller.slavePool().at(0)).txData();
    //auto &rxmotiondata=dynamic_cast<aris::control::Motion&>(controller.slavePool().at(0)).rxData();
    //auto &txmotiondata=static_cast<aris::control::TxMotionData&>(controller.txDataPool().at(0));
    //auto &rxmotiondata=static_cast<aris::control::RxMotionData&>(controller.rxDataPool().at(0));

    switch(command)
    {
    case aris::control::Motion::Cmd::IDLE:
        break;
    case aris::control::Motion::Cmd::ENABLE:
        txmotiondata.cmd=aris::control::Motion::Cmd::ENABLE;
        if(rxmotiondata.ret==0 && cmd_count !=0 && !cmd_success){
            rt_printf("command finished, cmd_count: %d\n", cmd_count);
            cmd_success=true;
        }
        if(cmd_count%1000==0 && !cmd_success)
            rt_printf("executing command: %d\n", cmd_count);
        if(cmd_success){
            txmotiondata.cmd=aris::control::Motion::Cmd::RUN;
            txmotiondata.target_pos=rxmotiondata.feedback_pos;
        }
        break;
    case aris::control::Motion::Cmd::DISABLE:
        txmotiondata.cmd=aris::control::Motion::Cmd::DISABLE;
        if(rxmotiondata.ret==0 && cmd_count !=0 && !cmd_success){
            rt_printf("command finished, cmd_count: %d\n", cmd_count);
            cmd_success=true;
        }
        if(cmd_count%1000==0 && !cmd_success)
            rt_printf("executing command: %d\n", cmd_count);
        break;
    case aris::control::Motion::Cmd::HOME:
        txmotiondata.cmd=aris::control::Motion::Cmd::HOME;
        if(rxmotiondata.ret==0 && cmd_count !=0 && !cmd_success){
            rt_printf("command finished, cmd_count: %d\n", cmd_count);
            cmd_success=true;
        }
        if(cmd_count%1000==0 && !cmd_success)
            rt_printf("executing command: %d\n", cmd_count);
        if(cmd_success){
            txmotiondata.cmd=aris::control::Motion::Cmd::RUN;
            txmotiondata.target_pos=rxmotiondata.feedback_pos;
        }
        break;
    case aris::control::Motion::Cmd::RUN:
        txmotiondata.cmd=aris::control::Motion::Cmd::RUN;
        txmotiondata.target_pos=0.020*std::sin(cmd_count/10000.0*2*PI);
        //txmotiondata.target_pos=0.002*cmd_count/1000.0;
        if(cmd_count%5000==0)
            rt_printf("executing command: %d\n", cmd_count);
        break;
    default:
        break;
    }
}

int main()
{

#ifdef WIN32
    controller.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
    controller.loadXml("/usr/aris/resource/Robot_III.xml");
#endif

    //for(auto &slave: mycontroller.slavePool())
        //std::cout<<slave.type()<<std::endl;

    controller.setControlStrategy(tg);

    controller.start();

    while (std::cin>>command_in)
    {
        if(command_in=="ds")
        {
            my_pipe.sendToRT(aris::control::Motion::Cmd::DISABLE);
        }
        else if(command_in=="en")
        {
            my_pipe.sendToRT(aris::control::Motion::Cmd::ENABLE);
        }
        else if(command_in=="hm")
        {
            my_pipe.sendToRT(aris::control::Motion::Cmd::HOME);
        }
        else if(command_in=="run")
        {
            my_pipe.sendToRT(aris::control::Motion::Cmd::RUN);
        }
        else if(command_in=="exit")
        {
            controller.stop();
            return 0;
        }
        else
        {
            std::cout<<"unrecoginize command, you can type 'ds', 'en', 'hm', 'run'. now the cmd is 'IDLE'."<<std::endl;
            my_pipe.sendToRT(aris::control::Motion::Cmd::IDLE);
        }
    }

    controller.stop();

    return 0;
}
