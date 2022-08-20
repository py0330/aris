#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>




int main(int argc, char *argv[]){
	aris::core::setLanguage(1);

	auto& cs = aris::server::ControlServer::instance();

	cs.resetMaster(aris::control::createDefaultEthercatMaster(4, 0, 0).release());

	cs.resetController(
		aris::control::createDefaultEthercatController(4, 0, 0, dynamic_cast<aris::control::EthercatMaster&>(cs.master())
		).release()
	);
	for (int i = 0; i < 4; ++i) {
		cs.controller().motorPool()[i].setMaxPos(3.14);
		cs.controller().motorPool()[i].setMinPos(-3.14);
		cs.controller().motorPool()[i].setMaxVel(3.14);
		cs.controller().motorPool()[i].setMinVel(-3.14);
		cs.controller().motorPool()[i].setMaxAcc(100);
		cs.controller().motorPool()[i].setMinAcc(-100);
	}


	aris::dynamic::ScaraParam scara_param;
	scara_param.a = 1.0;
	scara_param.b = 1.0;
	scara_param.pitch = 0.0;
	auto& scara = aris::dynamic::createModelScara(scara_param);

	cs.resetModel(scara.release());

	cs.resetPlanRoot(aris::plan::createDefaultPlanRoot().release());

	

	

	cs.resetTransferModelController(new aris::server::ScaraTransferModelController(0.16));
	std::cout << aris::core::toXmlString(cs) << std::endl;
	//cs.resetModel(new aris::dynamic::Model);
	//cs.resetPlanRoot(new aris::plan::PlanRoot);
	//cs.resetModel(new aris::dynamic::Model);


	




	try
	{
		cs.interfacePool().add<aris::server::HttpInterface>("http", "8001", "C:/Users/py033/WebstormProjects/RobotControllerHMI/client/build");
		cs.interfacePool().add<aris::server::ProgramWebInterface>();
		
		cs.init();
		cs.open();
		cs.start();

		// 读取数据 //
		//double data[6];
		//cs.controller().ftSensorPool()[0].getFtData(data);

		//std::cout << aris::core::toXmlString(cs) << std::endl;

		cs.runCmdLine();
		//aris::core::toXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
		//aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	

	

	return 0;
}