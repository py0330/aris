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
    //cs.createModel<aris::dynamic::Model>();
    cs.createController<aris::control::Controller>();
    cs.createSensorRoot<aris::sensor::SensorRoot>();

    cs.controller().registerChildType<robot::EsgImu, false, false, false, false>();
    cs.sensorRoot().registerChildType<aris::sensor::Imu,false,false,false,false>();

    cs.loadXml("/usr/aris/demo/resource/robot_motion.xml");
    //cs.loadXml("/usr/aris/resource/Robot_III.xml");

    cs.addCmd("en", robot::basicParse, nullptr);
    //cs.addCmd("en", nullptr, nullptr);
    cs.addCmd("ds", robot::basicParse, nullptr);
    cs.addCmd("hm", robot::basicParse, nullptr);
    cs.addCmd("test",robot::testParse,robot::testGait);

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
