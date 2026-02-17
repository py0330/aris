#include "test_dynamic_model_solver_ur.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>
#include <aris/core/core.hpp>

#include<type_traits>

using namespace aris::dynamic;

const double j_pos[6][3]{
	{ 0.0,  0.0, 0.176 },
	{ 0.0,  0.0, 0.176,},
	{ 0.56, 0.0, 0.176 },
	{ 0.90, 0.0, 0.176,},// 珞石的需要把这里的0.1去掉
	{ 0.90, 0.1, 0.3,  },
	{ 0.90, 0.1, 0.3,  },// 珞石的需要把这里的0.1去掉
};
const double j_axis[6][3]{
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 1.0, 0.0 }
};

const double pe_ee_i[6]{ 1.0, 0.15, 0.13, 0.0, 0.0, 0.0 };
const double pe_ee_j[6]{ 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

auto createUrModel(const double (*j_pos)[3], const double (*j_axis)[3], const double *pe_ee_i, const double *pe_ee_j)->std::unique_ptr<aris::dynamic::Model>{
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
	auto &j2 = model->addRevoluteJoint(p2, p1, j_pos[1], j_axis[1]);
	auto &j3 = model->addRevoluteJoint(p3, p2, j_pos[2], j_axis[2]);
	auto &j4 = model->addRevoluteJoint(p4, p3, j_pos[3], j_axis[3]);
	auto &j5 = model->addRevoluteJoint(p5, p4, j_pos[4], j_axis[4]);
	auto &j6 = model->addRevoluteJoint(p6, p5, j_pos[5], j_axis[5]);

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
	auto &inverse_kinematic = model->solverPool().add<aris::dynamic::UrInverseKinematicSolver>();
	auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

	inverse_kinematic.setWhichRoot(8);

	model->init();

	return model;
}
void test_ur_forward_solver(){
	auto m = createUrModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));

	const double input_series[5]{ aris::PI * 0 / 2.5, aris::PI * 1 / 2.5, aris::PI * 2 / 2.5, aris::PI * 3 / 2.5, aris::PI * 4 / 2.5 };
	for (int i = 0; i < 5 * 5 * 5 * 5 * 5 * 5; ++i)
	{
		double q[6]
		{
			input_series[(i / 1) % 5],
			input_series[(i / 5) % 5],
			input_series[(i / 25) % 5],
			input_series[(i / 125) % 5],
			input_series[(i / 625) % 5],
			input_series[(i / 3125) % 5],
		};

		for (int i = 0; i < 6; ++i)m->motionPool().at(i).setMp(q[i]);

		if (fwd.kinPos())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;
	}
}
void test_ur_inverse_solver(){
	auto m = createUrModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool().at(0));
	ee.setPosType(aris::dynamic::PosType::PM);
	m->init();

	double ee_pm[16];
	aris::dynamic::s_pe2pm(std::array<double, 7>{0.4, 0.2, 0.4, 0.0, 0.0, 0.0}.data(), ee_pm);

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
		if (!s_is_equal(6, result, q, 1e-9)) {
			std::cout << __FILE__ << __LINE__ << " failed inverse kinematics" << std::endl;
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

		auto new_m = createUrModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
		auto &new_inv = dynamic_cast<aris::dynamic::UrInverseKinematicSolver&>(new_m->solverPool().at(0));
		auto &new_ee = dynamic_cast<aris::dynamic::GeneralMotion&>(new_m->generalMotionPool().at(0));
		new_ee.setPosType(aris::dynamic::PosType::PM);
		new_m->init();

		for (int i = 0; i < 8; ++i){
			new_ee.setMpm(ee_pm);
			new_inv.setWhichRoot(i);
			if (new_m->inverseKinematics()) {
				new_m->inverseKinematics();
				std::cout << __FILE__ << __LINE__ << "failed" << std::endl;
			}
				

			if (new_m->forwardKinematics())std::cout << "forward failed" << std::endl;
			new_ee.updP();

			double mpm[16];
			new_ee.getMpm(mpm);
			if (!s_is_equal(16, ee_pm, mpm, 1e-9)){
				std::cout << __FILE__ << __LINE__ << " failed root:" << i << std::endl;
				dsp(4, 4, mpm);
			}
		}
	}
}
void test_ur_vel() {
	//auto m = createUrModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	UrParam param;
	param.H1 = 0.089159 ;
	param.W1 = 0.13585 - 0.1197 + 0.093 ;
	param.L1 = 0.425 ;
	param.L2 = 0.39225 ;
	param.H2 = -0.09465 ;
	param.W2 = 0.0823 ;
	auto m = aris::dynamic::createModelUr(param);

	m->setInputPos(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
	m->forwardKinematics();
	double loc_pm[16];
	m->getOutputPos(loc_pm);


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
	auto &inv = dynamic_cast<aris::dynamic::UrInverseKinematicSolver&>(m->solverPool().at(0));
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
void test_ur_calib_dh() {
	UrParam param;
	param.H1 = 0.089159;
	param.W1 = 0.13585 - 0.1197 + 0.093;
	param.L1 = 0.425;
	param.L2 = 0.39225;
	param.H2 = -0.09465;
	param.W2 = 0.0823;
	auto m = aris::dynamic::createModelUr(param);
	dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool()[0]).setPosType(aris::dynamic::PosType::PE123);

	// 待标定的真实值
	double joint_error[6]{ 0.01,0.03,0.015,0.02,0.001,-0.02 };
	double dh_with_error[6]{ 0.089159 + 0.01,  0.13585 - 0.1197 + 0.093 + 0.02, 0.425 + 0.025, 0.39225 + 0.02,  -0.09465-0.01, 0.0287 };

	// make joint data //
	const int n = 10;
	double joint_pos_seen[n][6]{
		{0.1,0.2,0.1,0.1,0.2,0.3},
		{0.2,0.4,-0.3,0.5,0.2,0.3},
		{0.3,0.6,0.3,0.8,0.2,0.3},
		{0.3,0.8,-0.3,0.1,0.2,0.3},
		{0.2,0.9,0.3,0.1,0.5,0.3},
		{0.1,0.9,-0.3,0.1,0.8,0.3},
		{0.1,0.8,0.3,0.1,0.2,0.3},
		{0.2,0.7,-0.3,0.1,0.2,0.6},
		{0.3,0.6,0.3,0.1,0.2,0.9},
	};

	// make 视觉数据 //
	double pq_obj_in_eye[n][7];
	{
		// make fake robot //
		UrParam param;
		param.H1 = 0.089159;
		param.W1 = 0.13585 - 0.1197 + 0.093;
		param.L1 = 0.425;
		param.L2 = 0.39225;
		param.H2 = -0.09465;
		param.W2 = 0.0823;
		param.H1 = dh_with_error[0];
		param.W1 = dh_with_error[1];
		param.L1 = dh_with_error[2];
		param.L2 = dh_with_error[3];
		param.H2 = dh_with_error[4];
		param.W2 = dh_with_error[5];

		auto m = aris::dynamic::createModelUr(param);
		dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool()[0]).setPosType(aris::dynamic::PosType::PE123);

		for (int i = 0; i < 6; ++i) {
			m->motionPool()[i].setMpOffset(joint_error[i]);
		}

		m->init();
		
		// eye in tool //
		const double pe_eye_in_tool[6]{ 0.1,0.2,0.3,0.111,0.221,0.832 };
		double pq_eye_in_tool[7];
		s_pe2pq(pe_eye_in_tool, pq_eye_in_tool, "321");

		// obj in base //
		const double pe_obj_in_base[6]{ 1.23,2.2,4.3,0.511,0.321,0.932 };
		double pq_obj_in_base[7];
		s_pe2pq(pe_obj_in_base, pq_obj_in_base, "321");

		// make data
		double pe_tool_in_base[n][6];
		
		// make joint data //
		for (int i = 0; i < n; ++i) {
			if (m->forwardKinematics(joint_pos_seen[i], pe_tool_in_base[i], 0))
				std::cout << "failed" << std::endl;
		}

		double pq_tool_in_base[n][7];
		// make eye data //
		for (int i = 0; i < n; ++i) {
			s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "123");
			
			double pq_eye_in_base[7];
			s_pq_dot_pq(pq_tool_in_base[i], pq_eye_in_tool, pq_eye_in_base);

			double pm_eye_in_base[16];
			s_pq2pm(pq_eye_in_base, pm_eye_in_base);

			s_inv_pq2pq(pm_eye_in_base, pq_obj_in_base, pq_obj_in_eye[i]);
		}
	}


	CalibUrParam ur_param;
	ur_param.dh_init[0] = param.H1;
	ur_param.dh_init[1] = param.W1;
	ur_param.dh_init[2] = param.L1;
	ur_param.dh_init[3] = param.L2;
	ur_param.dh_init[4] = param.H2;
	ur_param.dh_init[5] = param.W2;

	ur_param.mp_offset_init[0] = 0.0;
	ur_param.mp_offset_init[1] = 0.0;
	ur_param.mp_offset_init[2] = 0.0;
	ur_param.mp_offset_init[3] = 0.0;
	ur_param.mp_offset_init[4] = 0.0;
	ur_param.mp_offset_init[5] = 0.0;

	ur_param.n = n;
	ur_param.pq_obj_in_eye = *pq_obj_in_eye;
	ur_param.joint_pos = *joint_pos_seen;

	aris::dynamic::calibUrParamBy3DEye(ur_param);

	dsp(1, 6, ur_param.mp_offset_result);
	dsp(1, 6, ur_param.dh_result);
	dsp(1, 7, ur_param.eye_pq);
	std::cout << ur_param.avg_obj_pos_err << "  " << ur_param.avg_obj_quad_err << std::endl;



}


void test_model_solver_ur()
{
	std::cout << std::endl << "-----------------test model solver ur-----------------------" << std::endl;


	test_ur_calib_dh();
	test_ur_forward_solver();
	test_ur_inverse_solver();
	test_ur_vel();

	//auto m = createUrModel(j_pos, j_axis, pe_ee_i, pe_ee_j);

	//auto &inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
	//auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
	//auto &ee = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool().at(0));

	//double ee_pm[16];
	//aris::dynamic::s_pe2pm(std::array<double, 7>{0.32, 0.01, 0.62, 0.6, 0.3, 0.2}.data(), ee_pm);
	//ee.setMpm(ee_pm);
	//double vs[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	//ee.setMvs(vs);
	//double as[6]{ -0.01,0.02,-0.03,0.04,-0.05,0.06 };
	//ee.setMas(as);

	//inv.setWhichRoot(1);

	//if (inv.kinPos())std::cout << "failed" << std::endl;
	//inv.kinVel();
	//inv.dynAccAndFce();

	//double tem[6];


	//inv.cptJacobi();
	//for (auto &mot : m->motionPool())std::cout << mot.mv() << "  ";	std::cout << std::endl;
	//s_mm(6, 1, 6, inv.Ji(), ee.mvs(), tem);
	//dsp(1, 6, tem);

	//for (auto &mot : m->motionPool())std::cout << mot.ma() << "  "; std::cout << std::endl;
	//s_vc(6, inv.ci(), tem);
	//s_mma(6, 1, 6, inv.Ji(), ee.mas(), tem);
	//dsp(1, 6, tem);

	//double mv[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	//for (int i = 0; i < 6; ++i)m->motionPool()[i].setMv(mv[i]);
	//fwd.kinVel();
	//double ma[6]{ 0.01,-0.02,0.03,-0.04,0.05,-0.06 };
	//for (int i = 0; i < 6; ++i)m->motionPool()[i].setMa(ma[i]);
	//fwd.dynAccAndFce();

	//fwd.cptJacobi();
	//dsp(1, 6, ee.mvs());
	//s_mm(6, 1, 6, fwd.Jf(), mv, tem);
	//dsp(1, 6, tem);

	//dsp(1, 6, ee.mas());
	//s_vc(6, fwd.cf(), tem);
	//s_mma(6, 1, 6, fwd.Jf(), ma, tem);
	//dsp(1, 6, tem);
	////double tem2[6];
	////

	////dsp(1, 6, tem2);

	//////////////////////////////////////////////////测试建模/////////////////////////////////////////////////
	//aris::dynamic::PumaParam param;
	//param.d1 = 0.3295;
	//param.a1 = 0.04;
	//param.a2 = 0.275;
	//param.d3 = 0.0;
	//param.a3 = 0.025;
	//param.d4 = 0.28;

	//param.tool0_pe[2] = 0.078;

	//param.iv_vec =
	//{
	//	{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
	//	{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
	//	{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
	//	{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
	//	{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
	//	{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
	//};

	//param.mot_frc_vec =
	//{
	//	{9.34994758321915, 7.80825641041495, 0.00000000000000},
	//	{11.64080253106441, 13.26518528472506, 3.55567932576820},
	//	{4.77014054273075, 7.85644357492508, 0.34445460269183},
	//	{3.63141668516122, 3.35461524886318, 0.14824771620542},
	//	{2.58310846982020, 1.41963212641879, 0.04855267273770},
	//	{1.78373986219597, 0.31920640440152, 0.03381545544099},
	//};
	//auto m1 = aris::dynamic::createModelPuma(param);

	//dynamic_cast<aris::dynamic::GeneralMotion&>(m1->generalMotionPool().at(0)).setMpe(std::array<double, 6>{0.38453, 0, 0.6294, 0.0001, 0 + aris::PI/2, 0}.data(), "321");
	//m1->solverPool().at(0).kinPos();

	std::cout << "-----------------test model solver ur finished------------" << std::endl << std::endl;
}

