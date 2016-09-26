#include <iostream>
#include <algorithm>
#include <condition_variable>
#include <future>

#include <aris.h>
#include "plan.h"
#include "kinematic.h"
#include "newslave.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
    auto &cs = aris::server::ControlServer::instance();

    cs.resetModel(new robot::Robot);
    //if no special robot model, you can use the default model

    cs.controller().registerChildType<robot::Record>();

    cs.loadXml("/usr/aris/robot/resource/robot.xml");

    cs.addCmd("en", robot::basicParse, nullptr);
    cs.addCmd("ds", robot::basicParse, nullptr);
    cs.addCmd("hm", robot::basicParse, nullptr);
    //if no special command parser, we can use the default parser for command 'en' , 'ds' , 'hm'
    //cs.addCmd("en", nullptr, nullptr);
    cs.addCmd("log",robot::logParse,nullptr);
    cs.addCmd("help",robot::helpParse,nullptr);

    //some default command
    cs.addCmd("rc",robot::recoverParse,robot::recoverGait);//you need to change the default pose in the RecoverParam
    cs.addCmd("mv",robot::moveParse,robot::moveGait);

    //log
    //cs.controller().dataLogger().prepair("");
    //cs.controller().dataLogger().start();

    std::promise<void> exit_ready;
    auto fut = exit_ready.get_future();
    cs.setOnExit([&exit_ready]()
    {
        //cs.controller().dataLogger().stop();
        exit_ready.set_value();
    });


    // Set socket connection callback function //
    auto &cmdSock = *cs.widgetRoot().findType<aris::core::Socket>("command_socket");
    cmdSock.setOnReceivedConnection([](aris::core::Socket *sock, const char *remote_ip, int remote_port)
    {
        aris::core::log(std::string("received connection, the server_socket_ip_ is: ") + remote_ip);
        return 0;
    });
    cmdSock.setOnReceivedRequest([&cs](aris::core::Socket *sock, aris::core::Msg &msg)
    {
        try
        {
            if (msg.size() == 0)
            {
                throw std::runtime_error("received data but it's an empty message \\0");
            }
            else if (msg.data()[msg.size() - 1] != 0)
            {
                throw std::runtime_error("received data but it's not a string, because last char is nots \\0");
            }
            else
            {
                cs.executeCmd(msg.data());
            }
        }
        catch (std::exception &e)
        {
            return aris::core::Msg(e.what());
        }

        return aris::core::Msg();

    });
    cmdSock.setOnLoseConnection([&cs](aris::core::Socket *sock)
    {
        aris::core::log("lost connection");
        while (true)
        {
            try
            {
                sock->startServer();
                break;
            }
            catch (aris::core::Socket::StartServerError &e)
            {
                std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        aris::core::log("restart server socket successful");

        return 0;
    });
    cmdSock.startServer();

    while (fut.wait_for(std::chrono::microseconds(10)) == std::future_status::timeout)
    {
        aris::core::Msg msg;
        cs.widgetRoot().msgPipe().recvMsg(msg);

        if (!msg.empty())
        {
            std::cout << msg.data() <<std::endl;
        }
    }
    return 0;
}
