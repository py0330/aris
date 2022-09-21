#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>




int main(int argc, char *argv[]){
	aris::core::setLanguage(1);

	auto& cs = aris::server::ControlServer::instance();

	cs.resetMaster(aris::control::createDefaultEthercatMaster(6, 0, 0).release());

	cs.resetController(
		aris::control::createDefaultEthercatController(6, 0, 0, dynamic_cast<aris::control::EthercatMaster&>(cs.master())
		).release()
	);
	for (int i = 0; i < 6; ++i) {
		cs.controller().motorPool()[i].setMaxPos(3.14);
		cs.controller().motorPool()[i].setMinPos(-3.14);
		cs.controller().motorPool()[i].setMaxVel(3.14);
		cs.controller().motorPool()[i].setMinVel(-3.14);
		cs.controller().motorPool()[i].setMaxAcc(100);
		cs.controller().motorPool()[i].setMinAcc(-100);
	}


	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 1;

	puma_param.base2ref_pe[0] = 1;
	puma_param.base2ref_pe[1] = 2;
	puma_param.base2ref_pe[2] = 3;
	// 安装方式
	// 0, 正常安装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
	// 1，顶部吊装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
	// 2，侧装向上，零位时末端法兰盘朝向：地面 z 轴，零位时末端1轴朝向：地面 x 轴
	// 3，侧装向下，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
	auto puma = aris::dynamic::createModelPuma(puma_param);

	puma->forwardKinematics();
	double pe[6];
	puma->getInputPos(pe);
	aris::dynamic::dsp(1, 6, pe);

	cs.resetModel(puma.release());

	cs.resetPlanRoot(aris::plan::createDefaultPlanRoot().release());
	std::cout << aris::core::toXmlString(cs) << std::endl;

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