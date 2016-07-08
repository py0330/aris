#include <iostream>
#include <algorithm>
#include <aris.h>
#include "..\Server\plan.h"
#include "..\Server\kinematic.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
	try
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.LoadFile("C:\\aris\\robot\\resource\\robot_motion.xml");

		auto &cs = aris::server::ControlServer::instance();
		cs.createModel<robot::Robot>();
		cs.createController<aris::control::Controller>();
		cs.createSensorRoot<aris::sensor::SensorRoot>();
		cs.loadXml(xml_doc);

		const double begin_pee[3]{1,1,0};
		static_cast<robot::Robot&>(cs.model()).setPee(begin_pee);
		robot::MoveParam param;
		param.total_count_ = 10000;
		param.target_x_ = 1.5;
		param.target_y_ = 1;
		param.target_A_ = PI/4;
		param.mode_ = 1;

		aris::dynamic::PlanParamBase p;
		cs.model().saveDynEle("before");
		cs.model().simKin(robot::moveGait, param);
		cs.model().loadDynEle("before");
		cs.model().saveAdams("C:\\aris\\robot\\resource\\test.cmd");
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}


	std::cout << "test_control_server finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}
