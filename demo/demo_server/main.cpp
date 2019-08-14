#include <iostream>
#include <aris.hpp>

using namespace aris::dynamic;
using namespace aris::robot;

int main(int argc, char *argv[])
{
	double robot_pm[16];
	std::string robot_name = argc < 2 ? "rokae_xb4" : argv[1];
	auto port = argc < 3 ? 5866 : std::stoi(argv[2]);
	aris::dynamic::s_pq2pm(argc < 4 ? nullptr : aris::core::Calculator().calculateExpression(argv[3]).data(), robot_pm);

	auto&cs = aris::server::ControlServer::instance();
	cs.setName(robot_name);
	if (robot_name == "ur5")
	{
		cs.resetController(createControllerUr5().release());
		cs.resetModel(createModelUr5(robot_pm).release());
		cs.resetPlanRoot(createPlanRootUr5().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
	}
	else if (robot_name == "rokae_xb4")
	{
		cs.resetController(createControllerRokaeXB4().release());
		cs.resetModel(aris::robot::createModelRokaeXB4(robot_pm).release());
		cs.resetPlanRoot(createPlanRootRokaeXB4().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.interfaceRoot().loadXmlStr(aris::robot::createRokaeXB4Interface());
	}
	else if (robot_name == "servo_press")
	{
		cs.resetController(createControllerServoPress().release());
		cs.resetModel(aris::robot::createModelServoPress(robot_pm).release());
		cs.resetPlanRoot(createPlanRootServoPress().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
	}
	else if (robot_name == "stewart")
	{
		cs.resetController(createControllerStewart().release());
		cs.resetModel(aris::robot::createModelStewart(robot_pm).release());
		cs.resetPlanRoot(createPlanRootStewart().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.interfaceRoot().loadXmlStr(aris::robot::createRokaeXB4Interface());

		// init model pos //
		//cs.model().generalMotionPool()[0].setMpe(std::array<double, 6>{0, 0, 0.5, 0, 0, 0}.data(), "313");
		//cs.model().solverPool()[0].kinPos();

		//cs.saveXmlFile("C:\\Users\\py033\\Desktop\\stewart.xml");
		
		//cs.loadXmlFile(ARIS_INSTALL_PATH + std::string("/resource/demo_server/stewart.xml"));
	}
	else
	{
		std::cout << "unknown robot:" << robot_name << std::endl;
		return -1;
	}
	std::cout << "this server robot   :" << robot_name << std::endl;
	std::cout << "this server port    :" << std::to_string(port) << std::endl;
	std::cout << "this server position:" << std::endl;
	dsp(4, 4, robot_pm);

	////////////////////////////////////////////////////////////////////////////////////
	//aris::dynamic::SevenAxisParam param;

	//param.d1 = 0.3705;
	//param.d3 = 0.330;
	//param.d5 = 0.320;
	//param.tool0_pe[2] = 0.2205;

	//auto m = aris::dynamic::createModelSevenAxis(param);
	//cs.resetModel(m.release());
	//dynamic_cast<aris::control::EthercatMotion&>(cs.controller().slaveAtAbs(1)).setMinPos(-0.1);
	//dynamic_cast<aris::control::EthercatMotion&>(cs.controller().slaveAtAbs(1)).setMaxPos(0.1);
	////////////////////////////////////////////////////////////////////////////////////

	cs.planRoot().planPool().add<aris::plan::UniversalPlan>("tt", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &t)->void
	{
		auto ct = cs.currentCollectTarget();

		t.ret = std::vector<std::pair<std::string, std::any>>();

		if (ct)
		{
			std::cout << "current plan:" << ct->plan->name() << std::endl;
		}
		else
		{
			std::cout << "no current plan" << std::endl;
		}
		
		//t.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}, [&](const aris::plan::PlanTarget &param)->int
	{
		param.controller->motionAtAbs(0).setTargetPos(param.count*0.002);
		return 100 - param.count;
	}, [&](aris::plan::PlanTarget &)->void
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}, "<Command name=\"tt\"/>");
	

	// make log file has enough space
	cs.planRoot().planPool().add<aris::plan::RemoveFile>("remove_file");
	cs.start();

	//for (int i = 0; i < 1000; ++i)
	//{
	//	cs.executeCmd(aris::core::Msg("en --limit_time=1"));
	//}


	try
	{
		cs.executeCmd(aris::core::Msg("rmFi --filePath=/home/kaanh/log --memo=20000"));
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	cs.planRoot().planPool().add<aris::plan::MoveSeries>("move_series");

	/*
	auto ec_ptr = std::make_unique<aris::control::EthercatController>();
	ec_ptr->setEsiDirs({
		std::filesystem::path("C:\\Users\\py033\\Desktop\\esi_dirs"),
		std::filesystem::path("C:\\Users\\py033\\Desktop\\esi_dirs\\Beckhoff AX5xxx")
		});

	auto str = ec_ptr->xmlString();
	std::cout << str << std::endl;
	ec_ptr->loadXmlStr(str);
	std::cout << ec_ptr->xmlString() << std::endl;

	ec_ptr->updateDeviceList();

	std::cout << ec_ptr->getDeviceList() << std::endl;
	std::cout << ec_ptr->getPdoList(0x000002E1, 0x00, 0x29001) << std::endl;
	std::cout << ec_ptr->getPdoList(0x0000009A, 0x00030924, 0x000103f4) << std::endl;
	*/

	// interaction //
	cs.interfacePool().add<aris::server::WebInterface>("", "5866");
	cs.open();
	cs.runCmdLine();
	
	return 0;
}