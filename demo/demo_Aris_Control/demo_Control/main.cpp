
#include <aris.h>
#include <iostream>

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

#ifdef WIN32
#define rt_printf printf
#endif


#define SlaveNumber 18
enum { MAX_MOTOR_NUM = 100 };
//for enable, disable, and home
struct BasicFunctionParam
{
    std::uint8_t cmd_type;
    bool active_motor[MAX_MOTOR_NUM];

    BasicFunctionParam() {
        cmd_type=aris::control::Motion::IDLE;
        std::fill(active_motor, active_motor + MAX_MOTOR_NUM, true);
    }
};



aris::control::Controller controller;
aris::core::CommandParser parser;
static bool is_running=true;

aris::control::Pipe<aris::core::Msg> msg_pipe;
char cmd_char[aris::core::MsgRT::RT_MSG_SIZE];

static std::string command_in("idle");//command input from terminal
static std::int32_t cmd_count{0};
static bool cmd_success{false};



void tg();
BasicFunctionParam decode(const std::string input);
int main()
{
#ifdef WIN32
    controller.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
    controller.loadXml("/usr/aris/resource/Robot_III.xml");

    tinyxml2::XMLDocument doc;
    doc.LoadFile("/usr/aris/resource/Robot_III.xml");
    parser.loadXml(doc);
#endif

    controller.setControlStrategy(tg);

    controller.start();

    while (std::getline(std::cin,command_in) && is_running)
    {
        BasicFunctionParam param=decode(command_in);

        if(param.cmd_type<100){
            aris::core::Msg cmd_msg;
            cmd_msg.copyStruct(param);
            if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function");
            cmd_msg.setMsgID(0);
            msg_pipe.sendToRT(cmd_msg);
        }
        if(param.cmd_type==110){
            controller.stop();
            is_running=false;
            return 0;
        }
    }

    controller.stop();

    return 0;
}

BasicFunctionParam decode(const std::string input)
{
    std::string cmd;
    std::map<std::string, std::string> params;
    try
    {
        parser.parse(input, cmd, params);

        std::cout <<"\n"<< cmd << std::endl;
        int paramPrintLength;
        if (params.empty())
        {
            paramPrintLength = 2;
        }
        else
        {
            paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
            {
                return a.first.length() < b.first.length();
            })->first.length() + 2;
        }

        for (auto &i : params)
        {
            std::cout << std::string(paramPrintLength - i.first.length(), ' ') << i.first << " : " << i.second << std::endl;
        }

        std::cout << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl << std::endl;
    }

    BasicFunctionParam bfParam;
    if(cmd=="ds")
    {
        bfParam.cmd_type=aris::control::Motion::Cmd::DISABLE;
    }
    else if(cmd=="en")
    {
        bfParam.cmd_type=aris::control::Motion::Cmd::ENABLE;
    }
    else if(cmd=="hm")
    {
        bfParam.cmd_type=aris::control::Motion::Cmd::HOME;
    }
    else if(cmd=="test")
    {
        bfParam.cmd_type=aris::control::Motion::Cmd::RUN;
    }
    else if(cmd=="exit")
    {
        bfParam.cmd_type=110;
    }
    else
        bfParam.cmd_type=120;


    std::fill_n(bfParam.active_motor, SlaveNumber, false);
    for (auto &i : params){
        if(i.first !="slave_motor")
            std::cout<<"the first param must be 'c', it means the slave id."<<std::endl;
        else{
            if(stoi(i.second)>SlaveNumber-1 || stoi(i.second)<0){
                throw std::runtime_error("the second param is invalid");
            }
            else
                bfParam.active_motor[stoi(i.second)] = true;
        }
    }
    return bfParam;
}

void tg()
{
    BasicFunctionParam *param;
    if (msg_pipe.recvInRT(aris::core::MsgRT::instance()[0]) > 0)
    {
        aris::core::MsgRT::instance()[0].paste(cmd_char);
        param = reinterpret_cast<BasicFunctionParam *>(cmd_char);
        cmd_count=0;
        cmd_success=false;
    }

    for(std::size_t i=0;i<SlaveNumber && param->active_motor[i];i++){
        auto &txmotiondata=static_cast<aris::control::TxMotionData&>(controller.slavePool().at(i).txData());
        auto &rxmotiondata=static_cast<aris::control::RxMotionData&>(controller.slavePool().at(i).rxData());
        if(param->cmd_type !=aris::control::Motion::Cmd::RUN)
        {
            if(rxmotiondata.ret==0 && cmd_count !=0 && !cmd_success){
                rt_printf("command finished, cmd_count: %d\n", cmd_count);
                cmd_success=true;
            }
            if(cmd_count%1000==0 && !cmd_success)
                rt_printf("executing command: %d\n", cmd_count);
        }
        switch(param->cmd_type)
        {
        case aris::control::Motion::Cmd::ENABLE:
            txmotiondata.cmd=aris::control::Motion::Cmd::ENABLE;
            if(cmd_success){
                txmotiondata.cmd=aris::control::Motion::Cmd::RUN;
                txmotiondata.target_pos=rxmotiondata.feedback_pos;
            }
            break;
        case aris::control::Motion::Cmd::DISABLE:
            txmotiondata.cmd=aris::control::Motion::Cmd::DISABLE;
            break;
        case aris::control::Motion::Cmd::HOME:
            txmotiondata.cmd=aris::control::Motion::Cmd::HOME;
            if(cmd_success){
                txmotiondata.cmd=aris::control::Motion::Cmd::RUN;
                txmotiondata.target_pos=rxmotiondata.feedback_pos;
            }
            break;
        case aris::control::Motion::Cmd::RUN:
            txmotiondata.cmd=aris::control::Motion::Cmd::RUN;
            static double begin;
            if(cmd_count==0)
                begin=rxmotiondata.feedback_pos;
            txmotiondata.target_pos=0.002*std::sin(cmd_count/1000.0*2*PI)+begin;
            //txmotiondata.vel_offset= 0.002*2*PI/1000.0*std::cos(cmd_count/1000.0*2*PI);
            //txmotiondata.target_pos=0.002*cmd_count/1000.0+begin;
            if(cmd_count%5000==0)
                rt_printf("executing command: %d\n", cmd_count);
            break;
        default:
            break;
        }

    }
    cmd_count++;
}
