#include "test_dynamic_model_solver_puma.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

const double j_pos[6][3]{
	{ 0.0,     0.0,     0.176 },
	{ 0.04,   -0.0465,  0.3295, },
	{ 0.06,    0.0508,  0.6045 },
	{ -0.1233, 0.1,     0.6295, },// 珞石的需要把这里的0.1去掉
	{ 0.32,   -0.03235, 0.6295, },
	{ 0.32,   0.1,     0.6295, },// 珞石的需要把这里的0.1去掉
};
const double j_axis[6][3]{
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, -1.0, 0.0 },
	{ 1.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 1.0 }
};

const double pe_ee_i[6]{ 0.315, 0.05, 0.63, 0.1, 0.03, 0.15 };
const double pe_ee_j[6]{ 0.013, -0.15, 0.1, 0.01, 0.02, 0.2 };

auto createPumaModel(const double (*j_pos)[3], const double (*j_axis)[3], const double *pe_ee_i, const double *pe_ee_j)->std::unique_ptr<aris::dynamic::Model>
{
	std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

	// 设置重力 //
	const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
	model->environment().setGravity(gravity);

	// add part //
	auto &p1 = model->partPool().add<Part>("L1");
	auto &p2 = model->partPool().add<Part>("L2");
	auto &p3 = model->partPool().add<Part>("L3");
	auto &p4 = model->partPool().add<Part>("L4");
	auto &p5 = model->partPool().add<Part>("L5");
	auto &p6 = model->partPool().add<Part>("L6");

	// add joint //
	auto &j1 = model->addRevoluteJoint(p1, model->ground(), j_pos[0], j_axis[0]);
	auto &j2 = model->addRevoluteJoint(p1, p2, j_pos[1], j_axis[1]);
	auto &j3 = model->addRevoluteJoint(p3, p2, j_pos[2], j_axis[2]);
	auto &j4 = model->addRevoluteJoint(p3, p4, j_pos[3], j_axis[3]);
	auto &j5 = model->addRevoluteJoint(p5, p4, j_pos[4], j_axis[4]);
	auto &j6 = model->addRevoluteJoint(p5, p6, j_pos[5], j_axis[5]);

	// add actuation //
	auto &m1 = model->addMotion(j1);
	auto &m2 = model->addMotion(j2);
	auto &m3 = model->addMotion(j3);
	auto &m4 = model->addMotion(j4);
	auto &m5 = model->addMotion(j5);
	auto &m6 = model->addMotion(j6);

	// add ee general motion //
	double pm_ee_i[16], pm_ee_j[16];
	s_pe2pm(pe_ee_i, pm_ee_i);
	s_pe2pm(pe_ee_j, pm_ee_j);
	
	auto &makI = p6.addMarker("ee_makI", pm_ee_i);
	auto &makJ = model->ground().addMarker("ee_makJ", pm_ee_j);
	auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

	// add solver
	auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
	auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
	auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto &forward_dynamic = model->solverPool().add<ForwardDynamicSolver>();

	inverse_kinematic.setWhichRoot(8);

	model->init();

	return model;
}
// 验证是否正解有解
void test_puma_forward_solver(){
	auto m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);

	const double input_series[5]{ aris::PI * 0 / 2.5, aris::PI * 1 / 2.5, aris::PI * 2 / 2.5, aris::PI * 3 / 2.5, aris::PI * 4 / 2.5 };
	for (int i = 0; i < 5 * 5 * 5 * 5 * 5 * 5; ++i){
		double q[6]{
			input_series[(i / 1) % 5],
			input_series[(i / 5) % 5],
			input_series[(i / 25) % 5],
			input_series[(i / 125) % 5],
			input_series[(i / 625) % 5],
			input_series[(i / 3125) % 5],
		};

		m->setInputPos(q);

		if (m->forwardKinematics())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;
	}
}
void test_puma_inverse_solver(){
	auto m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	auto &ee = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool().at(0));
	
	double ee_pm[16];
	aris::dynamic::s_pe2pm(std::array<double, 7>{0.32, 0.01, 0.62, 0.1, 0.3, 0.2}.data(), ee_pm);

	const double input_series[5]{ aris::PI * 0 / 2.5, aris::PI * 1 / 2.5, aris::PI * 2 / 2.5, aris::PI * 3 / 2.5, aris::PI * 4 / 2.5 };
	for (int i = 0; i < 5 * 5 * 5 * 5 * 5 * 5; ++i){
		double q[6]{
			input_series[(i / 1) % 5],
			input_series[(i / 5) % 5],
			input_series[(i / 25) % 5],
			input_series[(i / 125) % 5],
			input_series[(i / 625) % 5],
			input_series[(i / 3125) % 5],
		};
		m->setInputPos(q);
		if (m->forwardKinematics())
			std::cout << __FILE__ << __LINE__ << "failed" << std::endl;

		double result[6];
		m->getInputPos(result);
		if (!s_is_equal(6, result, q, 1e-9)){
			std::cout << __FILE__ << __LINE__ << " failed inverse kinematics"  << std::endl;
			dsp(1, 6, q);
			dsp(1, 6, result);
		}

		double j_pos[6][3], j_axis[6][3], pe_ee_i[6], pe_ee_j[6];
		for (int i = 0; i < 6; ++i){
			s_vc(3, *m->motionPool().at(i).makI()->pm() + 3, 4, j_pos[i], 1);
			s_vc(3, *m->motionPool().at(i).makI()->pm() + 2, 4, j_axis[i], 1);
		}
		ee.makI()->getPe(pe_ee_i);
		ee.makJ()->getPe(pe_ee_j);
		auto new_m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
		auto &new_inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(new_m->solverPool().at(0));
		auto &new_ee = dynamic_cast<aris::dynamic::GeneralMotion&>(new_m->generalMotionPool().at(0));

		for (int i = 0; i < 8; ++i){
			new_ee.setMpm(ee_pm);
			new_inv.setWhichRoot(i);
			if (new_m->inverseKinematics())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;;

			if (new_m->forwardKinematics())std::cout << "forward failed" << std::endl;
			new_ee.updP();
			if (!s_is_equal(16, ee_pm, *new_ee.mpm(), 1e-9))
			{
				std::cout << __FILE__ << __LINE__ << " failed root:" << i << std::endl;
				dsp(4, 4, *ee.mpm());
			}
		}
	}
}
void test_puma_vel() {
	auto m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);

	// 计算正解 //
	double input_pos[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	double input_vel[6]{ 0.11,-0.22,0.33,-0.44,0.5,-0.66 };
	double input_acc[6]{ 0.15,0.25,0.35,0.45,0.55,0.65 };
	double output_pm[16], output_vs[6], output_as[6];

	m->setInputPos(input_pos);
	m->setInputVel(input_vel);
	m->setInputAcc(input_acc);
	
	if (m->forwardKinematics())std::cout << "failed" << std::endl;
	if (m->forwardKinematicsVel())std::cout << "failed" << std::endl;
	if (m->forwardKinematicsAcc())std::cout << "failed" << std::endl;

	m->getOutputPos(output_pm);
	m->getOutputVel(output_vs);
	m->getOutputAcc(output_as);

	// 将输入都设置为零，方便后续验证 //
	double zeros[6]{ 0,0,0,0,0,0 };
	m->setInputPos(zeros);
	m->setInputVel(zeros);
	m->setInputAcc(zeros);

	// 计算反解，看看是否能够计算正确 //
	if (m->inverseKinematics())std::cout << "failed" << std::endl;
	if (m->inverseKinematicsVel())std::cout << "failed" << std::endl;
	if (m->inverseKinematicsAcc())std::cout << "failed" << std::endl;

	double result[6];
	m->getInputPos(result);
	if (!s_is_equal(6, result, input_pos, 1e-9))std::cout << "failed" << std::endl;
	m->getInputVel(result);
	if (!s_is_equal(6, result, input_vel, 1e-9))std::cout << "failed" << std::endl;
	m->getInputAcc(result);
	if (!s_is_equal(6, result, input_acc, 1e-9))std::cout << "failed" << std::endl;

	// 验证雅可比 //
	auto &inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));

	inv.cptJacobi();
	s_mm(6, 1, 6, inv.Ji(), output_vs, result);
	if (!s_is_equal(6, result, input_vel, 1e-9))std::cout << "failed" << std::endl;

	s_vc(6, inv.ci(), result);
	s_mma(6, 1, 6, inv.Ji(), output_as, result);
	if (!s_is_equal(6, result, input_acc, 1e-9))std::cout << "failed" << std::endl;

	fwd.cptJacobi();
	s_mm(6, 1, 6, fwd.Jf(), input_vel, result);
	if (!s_is_equal(6, result, output_vs, 1e-9))std::cout << "failed" << std::endl;

	s_vc(6, fwd.cf(), result);
	s_mma(6, 1, 6, fwd.Jf(), input_acc, result);
	if (!s_is_equal(6, result, output_as, 1e-9))std::cout << "failed" << std::endl;
}

void test_model_solver_puma(){
	std::cout << std::endl << "-----------------test model solver puma---------------------" << std::endl;

	test_puma_forward_solver();
	test_puma_inverse_solver();
	test_puma_vel();

	std::cout << "-----------------test model solver puma finished------------" << std::endl << std::endl;
}

