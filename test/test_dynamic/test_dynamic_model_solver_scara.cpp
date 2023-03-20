#include "test_dynamic_model_solver_scara.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>
#include <aris/core/serialization.hpp>

#include<type_traits>

using namespace aris::dynamic;

#define EE_SIZE 4

const double j_pos[EE_SIZE][3]{
	{ 0.0,     0.0,     0.3  },
	{ 0.3,     0.0,     0.3, },
	{ 0.3,     0.2,     0.4  },
	{ 0.3,     0.2,     0.5, },
};
const double j_axis[EE_SIZE][3]{
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 0.0, 1.0 },
};

const double pe_ee_i[6]{ 0.5,0,0,0,0,0 };
const double pe_ee_j[6]{ 0.0,0,0,0,0,0 };

auto createScaraModel(const double (*j_pos)[3], const double (*j_axis)[3], const double *pe_ee_i, const double *pe_ee_j)->std::unique_ptr<aris::dynamic::Model>
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

	// add joint //
	auto &j1 = model->addRevoluteJoint(p1, model->ground(), j_pos[0], j_axis[0]);
	auto &j2 = model->addRevoluteJoint(p1, p2, j_pos[1], j_axis[1]);
	auto &j3 = model->addPrismaticJoint(p3, p2, j_pos[2], j_axis[2]);
	auto &j4 = model->addScrewJoint(p4, p3, j_pos[3], j_axis[3], 0.0);

	// add actuation //
	auto &m1 = model->addMotion(j1);
	auto &m2 = model->addMotion(j2);
	auto &m3 = model->addMotion(j3);
	auto &m4 = model->addMotion(j4);

	// add ee general motion //
	double pm_ee_i[16], pm_ee_j[16];
	s_pe2pm(pe_ee_i, pm_ee_i);
	s_pe2pm(pe_ee_j, pm_ee_j);
	
	auto &makI = p4.addMarker("ee_makI", pm_ee_i);
	auto &makJ = model->ground().addMarker("ee_makJ", pm_ee_j);
	auto &ee = model->generalMotionPool().add<aris::dynamic::XyztMotion>("ee", &makI, &makJ, false);

	// add solver
	auto& inverse_kinematic = model->solverPool().add<aris::dynamic::ScaraInverseKinematicSolver>();
	auto& forward_kinematic = model->solverPool().add<aris::dynamic::ScaraForwardKinematicSolver>();
	auto& inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic = model->solverPool().add<ForwardDynamicSolver>();

	model->init();

	return model;
}
void test_scara_forward_solver(){
	auto m = createScaraModel(j_pos, j_axis, pe_ee_i, pe_ee_j);

	const double input_series[5]{ aris::PI * 0 / 2.5, aris::PI * 1 / 2.5, aris::PI * 2 / 2.5, aris::PI * 3 / 2.5, aris::PI * 4 / 2.5 };
	for (int i = 0; i < 5 * 5 * 5 * 5; ++i){
		double q[4]{
			input_series[(i / 1) % 5],
			input_series[(i / 5) % 5],
			input_series[(i / 25) % 5],
			input_series[(i / 125) % 5],
		};

		m->setInputPos(q);

		if (m->forwardKinematics())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;
	}
}
void test_scara_inverse_solver(){
	auto m = createScaraModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	auto &ee = m->generalMotionPool().at(0);
	
	double ee_p[4]{ 0.32, -0.28, 0.62, 0.15 };

	const double input_series[5]{ aris::PI * 0 / 2.5, aris::PI * 1 / 2.5, aris::PI * 2 / 2.5, aris::PI * 3 / 2.5, aris::PI * 4 / 2.5 };
	for (int i = 0; i < 5 * 5 * 5 * 5; ++i){
		double q[4]{
			input_series[(i / 1) % 5],
			input_series[(i / 5) % 5],
			input_series[(i / 25) % 5],
			input_series[(i / 125) % 5],
		};
		m->setInputPos(q);
		if (m->forwardKinematics())
			std::cout << __FILE__ << __LINE__ << "failed" << std::endl;

		double j_pos[4][3], j_axis[4][3], pe_ee_i[6], pe_ee_j[6];
		for (int i = 0; i < 4; ++i){
			s_vc(3, *m->motionPool().at(i).makI()->pm() + 3, 4, j_pos[i], 1);
			s_vc(3, *m->motionPool().at(i).makI()->pm() + 2, 4, j_axis[i], 1);
		}
		ee.makI()->getPe(pe_ee_i);
		ee.makJ()->getPe(pe_ee_j);
		auto new_m = createScaraModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
		auto &new_inv = dynamic_cast<aris::dynamic::ScaraInverseKinematicSolver&>(new_m->solverPool().at(0));
		auto &new_ee = dynamic_cast<aris::dynamic::XyztMotion&>(new_m->generalMotionPool().at(0));

		for (int i = 0; i < 2; ++i){
			new_inv.setWhichRoot(i);
			
			new_m->setOutputPos(ee_p);
			if (new_m->inverseKinematics())
				std::cout << __FILE__ << __LINE__ << "failed" << std::endl;;

			if (new_m->forwardKinematics())
				std::cout << "forward failed" << std::endl;

			if (!s_is_equal(new_ee.pSize(), ee_p, new_ee.p(), 1e-9)) {
				std::cout << __FILE__ << __LINE__ << " failed root:" << i << std::endl;
				dsp(1, new_ee.pSize(), new_ee.p());
				dsp(1, ee.pSize(), ee_p);
			}
		}
	}
}
void test_scara_vel() {
	auto m = createScaraModel(j_pos, j_axis, pe_ee_i, pe_ee_j);

	// 计算正解 //
	double input_pos[EE_SIZE]{ 0.1,-0.2,0.3,0.4 };
	double input_vel[EE_SIZE]{ -0.11,0.22,0.33,0.44 };
	double input_acc[EE_SIZE]{ 0.15,0.35,0.55,0.75 };
	double output_p[EE_SIZE], output_v[EE_SIZE], output_a[EE_SIZE];

	m->setInputPos(input_pos);
	m->setInputVel(input_vel);
	m->setInputAcc(input_acc);
	
	if (m->forwardKinematics())std::cout << "failed" << std::endl;
	if (m->forwardKinematicsVel())std::cout << "failed" << std::endl;
	if (m->forwardKinematicsAcc())std::cout << "failed" << std::endl;

	m->getOutputPos(output_p);
	m->getOutputVel(output_v);
	m->getOutputAcc(output_a);

	// 将输入都设置为零，方便后续验证 //
	double zeros[6]{ 0,0,0,0,0,0 };
	m->setInputPos(zeros);
	m->setInputVel(zeros);
	m->setInputAcc(zeros);

	dynamic_cast<aris::dynamic::ScaraInverseKinematicSolver&>(m->solverPool()[0]).setWhichRoot(2);

	// 计算反解，看看是否能够计算正确 //
	if (m->inverseKinematics())std::cout << "failed" << std::endl;
	if (m->inverseKinematicsVel())std::cout << "failed" << std::endl;
	if (m->inverseKinematicsAcc())std::cout << "failed" << std::endl;

	double result[6];
	m->getInputPos(result);
	if (!s_is_equal(EE_SIZE, result, input_pos, 1e-9))std::cout << "failed" << std::endl;
	m->getInputVel(result);
	if (!s_is_equal(EE_SIZE, result, input_vel, 1e-9))std::cout << "failed" << std::endl;
	m->getInputAcc(result);
	if (!s_is_equal(EE_SIZE, result, input_acc, 1e-9))std::cout << "failed" << std::endl;

	// 验证雅可比 //
	auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));

	inv.cptJacobi();


	s_mm(inv.mJi(), 1, inv.nJi(), inv.Ji(), output_v, result);
	if (!s_is_equal(EE_SIZE, result, input_vel, 1e-9))std::cout << "failed" << std::endl;

	s_vc(6, inv.ci(), result);
	s_mma(inv.mJi(), 1, inv.nJi(), inv.Ji(), output_a, result);
	if (!s_is_equal(EE_SIZE, result, input_acc, 1e-9))std::cout << "failed" << std::endl;

	fwd.cptJacobi();
	s_mm(fwd.mJf(), 1, fwd.nJf(), fwd.Jf(), input_vel, result);
	if (!s_is_equal(EE_SIZE, result, output_v, 1e-9))std::cout << "failed" << std::endl;
	
	s_vc(EE_SIZE, fwd.cf(), result);
	s_mma(fwd.mJf(), 1, fwd.nJf(), fwd.Jf(), input_acc, result);
	if (!s_is_equal(EE_SIZE, result, output_a, 1e-9))std::cout << "failed" << std::endl;
}

void test_model_solver_scara(){
	std::cout << std::endl << "-----------------test model solver scara---------------------" << std::endl;

	test_scara_forward_solver();
	test_scara_inverse_solver();
	test_scara_vel();

	auto m = createScaraModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	auto& ee = m->generalMotionPool().at(0);

	double input[4]{ 0.0, -0.0, 0.62, 0.15 }, output[4];

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -1.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -2.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -3.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -4.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -5.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -6.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -7.0;

	m->setInputPos(input);
	m->forwardKinematics();
	m->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);
	input[3] = -7.0;

	std::cout << "-----------------test model solver scara finished------------" << std::endl << std::endl;
}

