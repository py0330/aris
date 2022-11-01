#include "test_dynamic_model_solver_universal.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_single_body_universal(){
	std::cout << "test single body:" << std::endl;

	aris::dynamic::Model m;
	auto &p = m.partPool().add<aris::dynamic::Part>();
	auto &s = m.solverPool().add<aris::dynamic::UniversalSolver>();
	m.init();

	p.setPe(std::array<double, 6>{0.1, 0.2, 0.3, 0.000423769269879415, 1.38980987554835, 1.79253453841257}.data(), "313");
	p.setVs(std::array<double, 6>{-0.244517963270725, 1.25737650310373, -0.874318412470487, -0.244517963270725, 1.25737650310373, -0.874318412470487}.data());
	p.setAs(std::array<double, 6>{0.0, -0.192390604845803, 0.136512424183815, 0.904633672502324, -1.24440604199266, 1.45568007018557}.data());

	s.kinPos();
	s.kinVel();
	s.dynAccAndFce();

	if (!s_is_equal(6, p.as(), std::array<double, 6>{0.2318970967746941, -9.2746063132688601, 0.6907262413433608, 0.0, 0.0, 0.0}.data(), 1e-10))std::cout << s.id() << "::dynAccAndFce() failed in single body" << std::endl;
}
void test_servo_press_universal(){
	std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

	// 设置重力 //
	const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
	model->environment().setGravity(gravity);

	// 添加变量 //
	model->calculator().addVariable("PI", "Number", aris::core::Matrix(aris::PI));

	// add part //
	auto &p1 = model->partPool().add<Part>("L1");

	//p1.setPe(std::array<double, 6>{0.4, 0.5, 0.6, 0.7, 0.8, 0.9}.data(), "321");

	// add joint //
	const double j1_pos[3]{ 0.13, -0.14, 0.85 };
	const double j1_axis[6]{ 0.0, 0.0, 1.0 };

	auto &j1 = model->addPrismaticJoint(p1, model->ground(), j1_pos, j1_axis);

	// add actuation //
	auto &m1 = model->addMotion(j1);

	// add ee general motion //
	double pq_ee_i[]{ 0.13, 0.54, 0.85, 0, 0, 0, 1 };
	double pm_ee_i[16];
	double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	s_pq2pm(pq_ee_i, pm_ee_i);

	auto &makI = p1.addMarker("ee_makI", pm_ee_i);
	auto &makJ = model->ground().addMarker("ee_makJ", pm_ee_j);
	auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ);

	// add solver
	auto &universal_solver = model->solverPool().add<UniversalSolver>();

	m1.activate(false);
	

	auto &m = *model;
	m.init();

	//p1.setPe(std::array<double, 6>{0.4, 0.5, 0.6, 0.7, 0.8, 0.9}.data(), "321");
	//ee.setMpq(std::array<double, 7>{0.1, 0.2, 0.3, 0, 0, 0, 1}.data());
	//ee.setMvs(std::array<double, 6>{0, 0, 0.2, 0, 0, 0}.data());
	//ee.setMas(std::array<double, 6>{0, 0, 0.7, 0, 0, 0}.data());
	//if (universal_solver.kinPos())std::cout << "failed1" << std::endl;

	//p1.setPe(std::array<double, 6>{0.4, 0.5, 0.6, 0.7, 0.8, 0.9}.data(), "321");
	//ee.setMpq(std::array<double, 7>{0.0, 0.0, 0.7, 0, 0, 0, 1}.data());
	//ee.setMvs(std::array<double, 6>{0, 0, 0.2, 0, 0, 0}.data());
	//ee.setMas(std::array<double, 6>{0, 0, 0.7, 0, 0, 0}.data());
	//if (universal_solver.kinPos())std::cout << "failed2" << std::endl;
	ee.setMpq(std::array<double, 7>{0.13, 0.54, 0.2, 0, 0, 0, 1}.data());
	ee.setMvs(std::array<double, 6>{0, 0, 0.2, 0, 0, 0}.data());
	ee.setMas(std::array<double, 6>{0, 0, 0.7, 0, 0, 0}.data());
	if (universal_solver.kinPos())std::cout << "failed3" << std::endl;
	universal_solver.kinVel();

	ee.updP();

	double cmI[36], cmJ[36];
	ee.cptGlbCm(cmI, cmJ);

	universal_solver.dynAccAndFce();

	double fs[6];
	double cm1[36], cm2[36];


	j1.cptGlbCm(cm1, cm2);
	s_mm(6, 1, j1.dim(), cm1, j1.cf(), fs);

	ee.cptGlbCm(cm1, cm2);
	s_mma(6, 1, ee.dim(), cm1, ee.cf(), fs);

	dsp(1, 6, fs);
}
void test_float_5_bar_universal()
{
	std::cout << "test float 5 bar:" << std::endl;

	const double joint1_position[3]{ 0.7 , 0.8 , 0.0 };
	const double joint1_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint2_position[3]{ 1.0 , 0.8 , 0.0 };
	const double joint2_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint3_position[3]{ 1.1 , 1.1 , 0.0 };
	const double joint3_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint4_position[3]{ 0.75 , 1.2 , 0.0 };
	const double joint4_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint5_position[3]{ 0.6 , 1.0 , 0.0 };
	const double joint5_axis[3]{ 0.0 , 0.0 , 1.0 };

	aris::dynamic::Model m;
	auto &p1 = m.partPool().add<aris::dynamic::Part>("p1", std::array<double, 10>{2.0, 0.7, 0.8, 0.1, 4.0, 7.0, 4.8, 0.01, 0.03, 0.02}.data());
	auto &p2 = m.partPool().add<aris::dynamic::Part>("p2", std::array<double, 10>{1.8, 1.1, 0.6, 0.2, 5.0, 6.1, 3.8, 0.1, 0.03, 0.02}.data());
	auto &p3 = m.partPool().add<aris::dynamic::Part>("p3", std::array<double, 10>{0.5, 0.5, -0.4, 1.2, 6.1, 5.2, 2.8, 0.01, 0.3, 0.02}.data());
	auto &p4 = m.partPool().add<aris::dynamic::Part>("p4", std::array<double, 10>{1.6, 0.9, 0.3, -0.3, 4.0, 4.3, 3.2, 0.01, 0.03, 0.2}.data());
	auto &p5 = m.partPool().add<aris::dynamic::Part>("p5", std::array<double, 10>{1.7, 2.1, 1.1, 0.8, 7.0, 3.4, 5.8, 0.01, 0.03, 0.02}.data());
	auto &j1 = m.addRevoluteJoint(p2, p1, joint1_position, joint1_axis);
	auto &j2 = m.addRevoluteJoint(p3, p2, joint2_position, joint2_axis);
	auto &j3 = m.addRevoluteJoint(p4, p3, joint3_position, joint3_axis);
	auto &j4 = m.addRevoluteJoint(p5, p4, joint4_position, joint4_axis);
	auto &j5 = m.addRevoluteJoint(p1, p5, joint5_position, joint5_axis);
	auto &m1 = m.addMotion(j1);
	auto &s = m.solverPool().add<aris::dynamic::UniversalSolver>();
	
	//auto &adams = m.simulatorPool().add<AdamsSimulator>();

	m.init();
	m1.setMp(0.0);
	m1.setMv(0.1);
	m1.setMa(0.2);

	p1.setVs(std::array<double, 6>{0.00000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000}.data());
	p2.setVs(std::array<double, 6>{0.08000000000000, -0.07000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.10000000000000}.data());
	p3.setVs(std::array<double, 6>{0.01714285714286, 0.00857142857143, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.02142857142857}.data());
	p4.setVs(std::array<double, 6>{0.06428571428571, -0.03857142857143, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.06428571428571}.data());
	p5.setVs(std::array<double, 6>{0.06428571428571, -0.03857142857143, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.06428571428571}.data());

	s.dynAccAndFce();

	if (!s_is_equal(6, p1.vs(), std::array<double, 6>{0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p2.vs(), std::array<double, 6>{0.08000000000000,   -0.07000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.10000000000000}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p3.vs(), std::array<double, 6>{0.01714285714286,   0.00857142857143,  0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02142857142857}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p4.vs(), std::array<double, 6>{0.06428571428571,   -0.03857142857143,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.06428571428571}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p5.vs(), std::array<double, 6>{0.06428571428571,   -0.03857142857143,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.06428571428571}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;

	if (!s_is_equal(6, p1.as(), std::array<double, 6>{-0.08859089478486, - 9.75558073147311, - 0.00018296503713,   0.00166526381296,   0.00049171676773, - 0.09405578038763}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p2.as(), std::array<double, 6>{0.07140910521514, - 9.89558073147311, - 0.00018296503713,   0.00166526381296,   0.00049171676773,   0.10594421961237}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p3.as(), std::array<double, 6>{-0.05136868452535, - 9.74505492286892, - 0.00018296503713,   0.00166526381296,   0.00049171676773, - 0.04458158899181}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p4.as(), std::array<double, 6>{0.03368654217674, - 9.82845708834653, - 0.00018296503713,   0.00166526381296,   0.00049171676773,   0.03148902526426}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p5.as(), std::array<double, 6>{0.05329075431848, - 9.84070972093511, - 0.00018296503713,   0.00166526381296,   0.00049171676773,   0.04782586871571}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;

	p1.addMarker("origin");
	p2.addMarker("origin");
	p3.addMarker("origin");
	p4.addMarker("origin");
	p5.addMarker("origin");
	m.init();
	//adams.saveAdams("C:\\Users\\py033\\Desktop\\test.cmd");
}


void test_model_solver_universal()
{
	std::cout << std::endl << "-----------------test model solver universal---------------------" << std::endl;
	//test_single_body_universal();
	test_float_5_bar_universal();
	test_servo_press_universal();

	std::cout << "-----------------test model solver universal------------" << std::endl << std::endl;
}

