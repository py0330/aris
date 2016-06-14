#include <iostream>
#include <algorithm>
#include <aris.h>
#include "plan.h"
#include "kinematic.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
    auto &cs = aris::server::ControlServer::instance();

    cs.createModel<aris::dynamic::Model>();
    cs.createController<aris::control::Controller>();
    cs.createSensorRoot<aris::sensor::SensorRoot>();

    cs.sensorRoot().registerChildType<aris::sensor::Imu,false,false,false,false>();

    cs.loadXml("/usr/aris/resource/Robot_III.xml");

    cs.addCmd("en", plan::basicParse, nullptr);
    cs.addCmd("ds", plan::basicParse, nullptr);
    cs.addCmd("hm", plan::basicParse, nullptr);
    cs.addCmd("test",plan::testParse,plan::testGait);

    cs.open();

    cs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile("/home/vincent/qt_workspace/build/build_new_aris/Robot_III.xml");
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        cs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();



    return 0;
}
