#include <iostream>
#include <algorithm>
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
    cs.createSensorRoot<aris::sensor::SensorRoot>();
    //if no special robot model, you can use the default model
    //cs.createModel<aris::dynamic::Model>();

    //register new slave or new sensor
    cs.controller().registerChildType<robot::EsgImu, false, false, false, false>();
    cs.sensorRoot().registerChildType<aris::sensor::Imu,false,false,false,false>();

    cs.loadXml("/usr/aris/demo/resource/robot_motion.xml");
    //cs.loadXml("/usr/aris/resource/Robot_III.xml");

    cs.addCmd("en", robot::basicParse, nullptr);
    cs.addCmd("ds", robot::basicParse, nullptr);
    cs.addCmd("hm", robot::basicParse, nullptr);
    //if no special command parser, we can use the default parser for command 'en' , 'ds' , 'hm'
    //cs.addCmd("en", nullptr, nullptr);

    //some default command
    cs.addCmd("rc",robot::recoverParse,robot::recoverGait);//you need to change the default pose in the RecoverParam
    cs.addCmd("mvpee",robot::movePeeParse,robot::movePeeGait);//you should make sure the robot has always arrived to the target pose before using 'mvpee' command
    cs.addCmd("mvpin",robot::movePinParse,robot::movePinGait);
    cs.addCmd("sinpee",robot::sinPeeParse,robot::sinPeeGait);
    cs.addCmd("sinpin",robot::sinPinParse,robot::sinPinGait);

    cs.open();

    cs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile("/usr/aris/demo/resource/robot_motion.xml");
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        cs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();



    return 0;
}
