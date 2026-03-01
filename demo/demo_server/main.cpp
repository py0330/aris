#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>

int main(int argc, char *argv[]){
	auto& cs = aris::server::ControlServer::instance();

	cs.resetMaster(aris::robot::rokae::xb4::createMaster().release());
	cs.resetController(aris::robot::rokae::xb4::createController().release());
	cs.resetModel(aris::robot::rokae::xb4::createModel().release());
	cs.resetPlanRoot(aris::robot::rokae::xb4::createPlanRoot().release());

	std::cout << aris::core::toXmlString(cs) << std::endl;
	std::cout << aris::core::logExeDirectory() << std::endl;

	auto str = aris::core::toXmlString(cs);
	aris::core::fromXmlString(cs,str);

	try{
		cs.interfacePool().add<aris::server::ProgramWebInterface>();
		
		cs.init();
		cs.open();
		cs.start();

		// 读取数据 //
		double data[6];
		cs.controller().ftSensorPool()[0].getFtData(data);

		cs.executeCmd("sl --count=3000", nullptr, 0);
		cs.executeCmd("sl --count=3000", nullptr, 1);

		cs.runCmdLine();
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	return 0;
}