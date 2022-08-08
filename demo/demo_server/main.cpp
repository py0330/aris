#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>

#include <aris/dynamic/puma_5axis.hpp>

auto createModelRokaeXB4_5(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>{
	aris::dynamic::PumaParam param;
	param.d1 = 0.3295;
	param.a1 = 0.04;
	param.a2 = 0.275;
	param.d3 = 0.0;
	param.a3 = 0.025;
	param.d4 = 0.28;

	param.tool0_pe[2] = 0.078;

	param.iv_vec =
	{
		{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
		{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
		{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
		{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
		{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
		{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
	};

	param.mot_frc_vec =
	{
		{ 9.34994758321915, 7.80825641041495, 0.00000000000000 },
		{ 11.64080253106441, 13.26518528472506, 3.55567932576820 },
		{ 4.77014054273075, 7.85644357492508, 0.34445460269183 },
		{ 3.63141668516122, 3.35461524886318, 0.14824771620542 },
		{ 2.58310846982020, 1.41963212641879, 0.04855267273770 },
		{ 1.78373986219597, 0.31920640440152, 0.03381545544099 },
	};

	return aris::dynamic::createModelPuma5(param);
}


int main(int argc, char *argv[]){
	aris::core::setLanguage(1);

	auto& cs = aris::server::ControlServer::instance();
	try {
		aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\kaanh.xml");
	}
	catch (std::runtime_error&e) {
		std::cout << e.what() << std::endl;
	}
	
	
	auto &m = dynamic_cast<aris::dynamic::MultiModel&>(cs.model());
	
	double pos[3]{ 0,0,0 };
	double axis[3]{ 0,0,1 };
	m.subModels().push_back(aris::dynamic::createExternalAxisModel(pos, axis, false, false).release());

	auto t = m.getMotionTypes();
	


	//aris::core::toXmlFile(cs, "C:\\Users\\py033\\Desktop\\kaanh(5).xml");

	cs.resetMaster(aris::robot::rokae::xb4::createMaster().release());
	cs.resetController(aris::robot::rokae::xb4::createController().release());
	cs.resetModel(aris::robot::rokae::xb4::createModel().release());
	cs.resetPlanRoot(aris::robot::rokae::xb4::createPlanRoot().release());

	std::cout << aris::core::toXmlString(cs) << std::endl;

	

	//cs.resetModel(new aris::dynamic::Model);
	//cs.resetPlanRoot(new aris::plan::PlanRoot);
	//cs.resetModel(new aris::dynamic::Model);

	auto str = aris::core::toXmlString(cs);

	aris::core::fromXmlString(cs,str);

	aris::dynamic::ScaraParam scara_param;

	scara_param.a = 1.0;
	scara_param.b = 1.0;

	auto &scara = aris::dynamic::createModelScara(scara_param);

	double input[4]{ 0.2,-0.1,0.23,0.3 };
	scara->setInputPos(input);
	scara->forwardKinematics();
	double pm[4];
	scara->getOutputPos(pm);
	aris::dynamic::dsp(1, 4, pm);
	
	double output2[4]{ 1.9751,   0.2985,   0.2300, - 1.0708 };
	scara->setOutputPos(output2);
	scara->inverseKinematics();
	scara->getInputPos(input);

	aris::dynamic::dsp(1, 4, input);


	try
	{
		cs.interfacePool().add<aris::server::HttpInterface>("http", "8001", "C:/Users/py033/WebstormProjects/RobotControllerHMI/client/build");
		cs.interfacePool().add<aris::server::ProgramWebInterface>();
		
		cs.init();
		cs.open();
		cs.start();

		// 读取数据 //
		double data[6];
		cs.controller().ftSensorPool()[0].getFtData(data);

		std::cout << aris::core::toXmlString(cs) << std::endl;

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