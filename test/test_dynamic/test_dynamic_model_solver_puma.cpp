#include "test_dynamic_model_solver_puma.h"
#include <iostream>
#include <aris_dynamic.h>

#include<type_traits>

using namespace aris::dynamic;

const double j_pos[6][3]
{
	{ 0.0, 0.0, 0.176 },
	{ 0.04, -0.0465, 0.3295, },
	{ 0.04, 0.0508, 0.6045 },
	{ -0.1233, 0.1, 0.6295, },// 珞石的需要把这里的0.1去掉
	{ 0.32,-0.03235, 0.6295, },
	{ 0.383,0.1, 0.6295, },// 珞石的需要把这里的0.1去掉
};
const double j_axis[6][3]
{
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, -1.0, 0.0 },
	{ 1.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0 },
	{ -1.0, 0.0, 0.0 }
};

const double pe_ee_i[6]{ 0.315, 0.05, 0.63, 0.1, 0.03, 0.15 };
const double pe_ee_j[6]{ 0.013, -0.15, 0.1, 0.01, 0.02, 0.2 };

auto createPumaModel(const double (*j_pos)[3], const double (*j_axis)[3], const double *pe_ee_i, const double *pe_ee_j)->std::unique_ptr<aris::dynamic::Model>
{
	std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

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
	
	auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
	auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
	auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

	// add solver
	auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
	auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

	inverse_kinematic.allocateMemory();
	forward_kinematic.allocateMemory();

	return model;
}
void test_puma_forward_solver()
{
	auto m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
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

		if (!fwd.kinPos())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;
	}
}
void test_puma_inverse_solver()
{
	auto m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
	
	auto &inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
	auto &ee = m->generalMotionPool().at(0);
	
	double ee_pm[16];
	aris::dynamic::s_pe2pm(std::array<double, 7>{0.32, 0.01, 0.62, 0.1, 0.3, 0.2}.data(), ee_pm);

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

		if (!fwd.kinPos())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;

		double j_pos[6][3], j_axis[6][3], pe_ee_i[6], pe_ee_j[6];
		for (int i = 0; i < 6; ++i)
		{
			s_vc(3, *m->motionPool().at(i).makI().pm() + 3, 4, j_pos[i], 1);
			s_vc(3, *m->motionPool().at(i).makI().pm() + 2, 4, j_axis[i], 1);
		}
		ee.makI().getPe(pe_ee_i);
		ee.makJ().getPe(pe_ee_j);
		auto new_m = createPumaModel(j_pos, j_axis, pe_ee_i, pe_ee_j);
		auto &new_inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(new_m->solverPool().at(0));
		auto &new_fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(new_m->solverPool().at(1));
		auto &new_ee = new_m->generalMotionPool().at(0);

		for (int i = 0; i < 8; ++i)
		{
			new_ee.setMpm(ee_pm);
			new_inv.setWhichRoot(i);
			if (!new_inv.kinPos())std::cout << __FILE__ << __LINE__ << "failed" << std::endl;;

			if (!new_fwd.kinPos())std::cout << "forward failed" << std::endl;
			new_ee.updMpm();
			if (!s_is_equal(16, ee_pm, *new_ee.mpm(), 1e-9))
			{
				std::cout << __FILE__ << __LINE__ << " failed root:" << i << std::endl;
				dsp(4, 4, *ee.mpm());
			}
		}
	}
}

void test_model_solver_puma()
{
	std::cout << std::endl << "-----------------test model solver puma---------------------" << std::endl;

	test_puma_forward_solver();
	test_puma_inverse_solver();

	std::cout << "-----------------test model solver puma finished------------" << std::endl << std::endl;
}

