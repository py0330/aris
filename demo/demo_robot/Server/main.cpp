#include <iostream>
#include <algorithm>
#include <condition_variable>

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

    cs.createModel<robot::Robot>();
    cs.createController<aris::control::Controller>();
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

    cs.open();
    cs.controller().dataLogger().prepair("");
    cs.controller().dataLogger().start();

    /*
    cs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile("/usr/aris/robot/resource/robot.xml");
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        cs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();
    */

    std::condition_variable cv;
    std::mutex mu;
    std::unique_lock<std::mutex> lck(mu);

    cs.setOnExit([&cv, &mu, &cs]()
    {
        cs.controller().dataLogger().stop();
        std::unique_lock<std::mutex> lck(mu);
        cv.notify_one();
    });
    cv.wait(lck);

    return 0;
}
