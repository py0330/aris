#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>

#include <aris/dynamic/puma_5axis.hpp>

using namespace aris::dynamic;
using namespace aris::robot;

//系统传递函数H(s)=1/(ms)
void PIDcalOne(double m, double ts, double *KP)
{
	double T = ts / 3.0;
	KP[0] = m / T;
}
//系统传递函数H(s)=1/(ms+h)
void PIDcalTeo(double m, double h, double ts, double overshoot, double *KP, double *KI)
{
	double temp = std::log(overshoot);
	double kesi = 1 / sqrt(1 + aris::PI * aris::PI / temp / temp);
	double omega = 4 / kesi / ts;

	KI[0] = omega * omega * m;
	KP[0] = 2 * kesi * omega * m - h;
}
auto f(aris::dynamic::Model *m, double *A)
{
	auto &s = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool()[1]);
	s.kinPos();
	s.kinVel();
	s.cptGeneralInverseDynamicMatrix();
	s.cptJacobiWrtEE();

	// J_inv
	double U[36], tau[6], J_inv[36], tau2[6];
	aris::Size p[6], rank;
	s_householder_utp(6, 6, s.Jf(), U, tau, p, rank, 1e-7);
	s_householder_utp2pinv(6, 6, rank, U, tau, p, J_inv, tau2, 1e-7);

	// M = (M + I) * J_inv 
	double M[36], tem[36];
	s_mc(6, 6, s.M(), s.nM(), M, 6);
	for (int i = 0; i < 6; ++i)M[at(i, i, 6)] += m->motionPool()[i].frcCoe()[2];
	s_mm(6, 6, 6, M, J_inv, tem);
	s_mm(6, 6, 6, J_inv, T(6), tem, 6, A, 6);
}

auto createModelRokaeXB4_5(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
{
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


int main(int argc, char *argv[])
{
	double robot_pm[16];
	std::string robot_name = argc < 2 ? "rokae_xb4" : argv[1];
	auto port = argc < 3 ? 5866 : std::stoi(argv[2]);
	aris::dynamic::s_pq2pm(argc < 4 ? nullptr : std::any_cast<const aris::core::Matrix&>(aris::core::Calculator().calculateExpression(argv[3]).second).data(), robot_pm);

	auto&cs = aris::server::ControlServer::instance();
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
		cs.resetController(aris::robot::createControllerStewart().release());
		cs.resetModel(aris::robot::createModelStewart(robot_pm).release());
		cs.resetPlanRoot(aris::robot::createPlanRootStewart().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
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

	//std::fstream file1;
	//file1.open("C:\\Users\\py033\\Desktop\\seri0.xml");
	//std::string file1_xml_str((std::istreambuf_iterator<char>(file1)), (std::istreambuf_iterator<char>()));
	//aris::core::fromXmlString(cs, file1_xml_str);
	//file1.close();

	//std::fstream file2;
	//file2.open("C:\\Users\\py033\\Desktop\\seri2.xml", std::ios::out | std::ios::trunc);
	//file2 << aris::core::toXmlString(cs);
	//file2.close();

	//std::fstream file3;
	//file3.open("C:\\Users\\py033\\Desktop\\seri2.xml");
	//std::string file3_xml_str((std::istreambuf_iterator<char>(file3)), (std::istreambuf_iterator<char>()));
	//aris::core::fromXmlString(cs, file3_xml_str);
	//file3.close();

	//std::fstream file4;
	//file4.open("C:\\Users\\py033\\Desktop\\seri4.xml", std::ios::out | std::ios::trunc);
	//file4 << aris::core::toXmlString(cs);
	//file4.close();

	//std::cout << aris::core::toJsonString(cs) << std::endl;

	//auto str = aris::core::toJsonString(cs);
	//std::cout << aris::core::toJsonString(cs) << std::endl;
	//cs.resetController(nullptr);
	//cs.resetModel(nullptr);

	//aris::core::fromJsonString(cs, str);

	//std::cout << aris::core::toJsonString(cs) << std::endl;

	//auto &m = cs.model();
	//m.init();
	//
	//auto &c = m.calculator();

	//double mp[6]{ 0,0,0,0,1.57,0 };
	//double mv[6]{ 0.001,0.02,0.01,0.04,0.01,0.02 };
	//double ma[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	//for (auto &mot : cs.model().motionPool())
	//{
	//	mot.setMp(mp[mot.id()]);
	//	mot.setMv(mv[mot.id()]);
	//	mot.setMa(ma[mot.id()]);
	//}

	//double pq[7]{0.1370, 0.345, 0.279968, 0, -1, 0, 0};
	//m.generalMotionPool()[0].setMpq(pq);
	//m.solverPool()[0].kinPos();
	//
	////std::cout << m.xmlString() << std::endl;

	//auto &s = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m.solverPool()[1]);
	//s.kinPos();
	//s.kinVel();
	//s.cptGeneralInverseDynamicMatrix();
	//s.cptJacobiWrtEE();
	//
	//// J_inv
	//double U[36], tau[6], J_inv[36], tau2[6];
	//aris::Size p[6], rank;
	//s_householder_utp(6, 6, s.Jf(), U, tau, p, rank, 1e-7);
	//s_householder_utp2pinv(6, 6, rank, U, tau, p, J_inv, tau2, 1e-7);

	//// M = (M + I) * J_inv 
	//double M[36], A[36], tem[36];
	//s_mc(6, 6, s.M(), s.nM(), M, 6);
	//for (int i = 0; i < 6; ++i)M[at(i, i, 6)] += m.motionPool()[i].frcCoe()[2];
	//s_mm(6, 6, 6, M, J_inv, tem);
	//s_mm(6, 6, 6, J_inv, T(6), tem, 6, A, 6);

	//dsp(6, 6, A);

	//f(&cs.model(), M);
	//dsp(6, 6, M);

	//// cout torque 
	//double mf[6];
	//for (int i = 0; i < 6; ++i)tem[i] = m.motionPool()[i].mf();
	//s_mm(6, 1, 6, J_inv, T(6), tem, 1, mf, 1);
	////dsp(1, 6, mf);

	//// h = -M * c + h
	//double h[6];
	//s_vc(6, s.h(), tem);
	//s_mm(6, 1, 6, J_inv, T(6), tem, 1, h, 1);
	//s_mms(6, 1, 6, A, s.cf(), h);
	//double ee_as[6];
	//m.generalMotionPool()[0].getMas(ee_as);
	//s_mma(6, 1, 6, A, ee_as, h);
	////dsp(1, 6, h);

	//// 
	//double max_value[6]{ 0,0,0,0,0,0 };
	//double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
	//for (int i = 0; i < 6; ++i)
	//{
	//	//s_nv(6, f2c_index[i], A + i * 6);
	//	
	//	for (int j = 0; j < 6; ++j)
	//	{
	//		max_value[j] = std::max(max_value[j], std::abs(A[at(i, j, 6)]));
	//	}
	//}

	//dsp(1, 6, max_value);

	//double kpp[6];
	//double kpv[6], kiv[6];
	//for (int i = 0; i < 6; ++i)
	//{
	//	PIDcalOne(max_value[i], 0.2, kpp + i);
	//	PIDcalTeo(max_value[i], 0, 0.25, 0.0433, kpv + i, kiv + i);
	//}

	//dsp(1, 6, kpp);
	//dsp(1, 6, kpv);
	//dsp(1, 6, kiv);

	//double ft[6]{ 0,0,0,0,15,0 };
	//double JoinTau[6];
	//s_mm(6, 1, 6, s.Jf(), T(6), ft, 1, JoinTau, 1);

	//dsp(1, 6, JoinTau);


	//for (int i = 0; i < 6; ++i)
	//{
	//	JoinTau[i] *= f2c_index[i];
	//}

	//dsp(1, 6, JoinTau);


	//for (int i = 0; i < 6; ++i)
	//{
	//	max_value[i] *= f2c_index[i];
	//}
	//
	//dsp(1, 6, max_value);


	////////////////////////////////////////////////////////////////////////////////////
	//aris::dynamic::SevenAxisParam param;

	//param.d1 = 0.3705;
	//param.d3 = 0.330;
	//param.d5 = 0.320;
	//param.tool0_pe[2] = 0.2205;

	//auto m = aris::dynamic::createModelSevenAxis(param);
	//cs.resetModel(m.release());
	//dynamic_cast<aris::control::EthercatMotor&>(cs.controller().slaveAtAbs(1)).setMinPos(-0.1);
	//dynamic_cast<aris::control::EthercatMotor&>(cs.controller().slaveAtAbs(1)).setMaxPos(0.1);
	////////////////////////////////////////////////////////////////////////////////////

	cs.model().init();


	// make log file has enough space
	cs.planRoot().planPool().add<aris::plan::RemoveFile>("remove_file");
	cs.planRoot().planPool().add<aris::plan::MoveSeries>("move_series");
	cs.planRoot().planPool().add<aris::server::GetInfo>();

	// interaction //
	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5868", aris::core::Socket::TCP);
	cs.interfacePool().add<aris::server::HttpInterface>("", "8001", "D:/UI_DarkColor_English-0103_panbo/UI_DarkColor_English-0103_panbo/www");


	for (auto &m : cs.controller().slavePool()) dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);

	auto model5 = createModelRokaeXB4_5(robot_pm);

	model5->init();
	cs.model().init();

	model5->motionPool()[5].setMp(0.5);

	model5->generalMotionPool()[0].setMpe(std::array<double, 6>{0.3, 0.4, 0.6, 0.1, 0.2, 0.0}.data(), "123");
	model5->solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)
	{
		std::cout << model5->motionPool()[i].mp() << std::endl;
	}
	std::cout  << std::endl;
	model5->solverPool()[1].kinPos();
	model5->generalMotionPool()[0].updMpm();
	aris::dynamic::dsp(4, 4, *model5->generalMotionPool()[0].mpm());




	cs.model().generalMotionPool()[0].setMpe(std::array<double, 6>{0.3, 0.4, 0.6, 0.1, 0.2, 0.0}.data(), "123");
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)
	{
		std::cout << cs.model().motionPool()[i].mp() << std::endl;
	}
	cs.model().solverPool()[1].kinPos();
	cs.model().generalMotionPool()[0].updMpm();
	aris::dynamic::dsp(4, 4, *cs.model().generalMotionPool()[0].mpm());

	try
	{
		//aris::core::toXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
		//aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	
	cs.init();
	cs.open();
	cs.runCmdLine();
	
	return 0;
}