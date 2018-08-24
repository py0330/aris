#include "test_dynamic_model_solver_puma.h"
#include <iostream>
#include <aris_dynamic.h>

#include<type_traits>

using namespace aris::dynamic;

auto createPumaModel()->std::unique_ptr<aris::dynamic::Model>
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
	const double j1_pos[3]{ 0.0, 0.0, 0.176 };
	const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
	const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
	const double j4_pos[3]{ -0.1233, 0.1, 0.6295, };// 把0.1去掉
	const double j5_pos[3]{ 0.32,-0.03235, 0.6295, };
	const double j6_pos[3]{ 0.383,0.1, 0.6295, };// 把0.1去掉

	const double j1_axis[6]{ 0.0, 0.0, 1.0 };
	const double j2_axis[6]{ 0.0, 1.0, 0.0 };
	const double j3_axis[6]{ 0.0, -1.0, 0.0 };
	const double j4_axis[6]{ 1.0, 0.0, 0.0 };
	const double j5_axis[6]{ 0.0, 1.0, 0.0 };
	const double j6_axis[6]{ -1.0, 0.0, 0.0 };

	auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
	auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
	auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
	auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
	auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
	auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

	// add actuation //
	auto &m1 = model->addMotion(j1);
	auto &m2 = model->addMotion(j2);
	auto &m3 = model->addMotion(j3);
	auto &m4 = model->addMotion(j4);
	auto &m5 = model->addMotion(j5);
	auto &m6 = model->addMotion(j6);

	// add ee general motion //
	const double pm_ee[16]{ 1,0,0,0.32,0,1,0,0.1,0,0,1,0.6295,0,0,0,1 };// 把0.1去掉
	double pm_ee_j[16];
	s_pe2pm(std::array<double, 6>{0.1, 0.02, 0.03, 0.02, 0.01, 0.03}.data(), pm_ee_j);
	s_eye(4, pm_ee_j);
	auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee);
	auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
	auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

	// add solver
	auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
	auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
	auto &inverse_dynamic = model->solverPool().add<InverseKinematicSolver>();

	return model;
}
void test_puma_inverse_solver(aris::dynamic::Model *m, double* ee_pm)
{
	auto &inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
	inv.allocateMemory();
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
	fwd.allocateMemory();
	auto &gen = m->solverPool().at(2);
	gen.allocateMemory();
	
	fwd.plotRelation();
	


	auto &ee = m->generalMotionPool().at(0);
	ee.setMpm(ee_pm);

	dsp(4, 4, ee_pm);

	if (!gen.kinPos())std::cout << __FILE__ << __LINE__ << " failed" << std::endl;

	for (auto &m : m->motionPool())
	{
		m.updMp();
		std::cout << m.mp() << "   ";
	}
	std::cout << std::endl;
	std::cout << std::endl;

	for (int i = 0; i < 8; ++i)
	{
		for (auto &mot : m->motionPool())mot.setMp(0.5);
		if (!fwd.kinPos())std::cout << "forward failed" << std::endl;
		ee.updMpm();
		//dsp(4, 4, *ee.mpm());
		//for (auto &m : m->motionPool())std::cout << m.mp() << "   ";
		//std::cout << std::endl;
		
		//for (auto &mot : m->motionPool())mot.setMp(0.3);
		//if (!fwd.kinPos())std::cout << "forward failed" << std::endl;
		//ee.updMpm();
		//dsp(4, 4, *ee.mpm());
		//for (auto &mot : m->motionPool())std::cout << mot.mp() << "   ";
		//std::cout << std::endl;

		ee.setMpm(ee_pm);
		inv.setWhichRoot(i);
		inv.kinPos();

		for (auto &mot : m->motionPool())std::cout << mot.mp() << "   ";
		std::cout << std::endl;

		if (!fwd.kinPos())std::cout << "forward failed" << std::endl;
		ee.updMpm();
		// dsp(4, 4, *ee.mpm());
		//if(!s_is_equal(16, ee_pm, *ee.mpm(), 1e-10))
		//	std::cout << __FILE__ << __LINE__ << " failed root:"<<i << std::endl;

		if (!s_is_equal(16, ee_pm, *ee.mpm(), 1e-10))
		{
			//std::cout << __FILE__ << __LINE__ << " failed root:" << i << std::endl;
			std::cout << " failed root:" << i << std::endl;
			dsp(4, 4, *ee.mpm());
		}

	}
	

}



void test_model_solver_puma()
{
	std::cout << std::endl << "-----------------test model solver puma---------------------" << std::endl;

	auto m = createPumaModel();
	auto &ee = m->generalMotionPool().at(0);

	double pm[16];
	aris::dynamic::s_pe2pm(std::array<double, 7>{0.32, 0.1, 0.6295, 0, 0, 0}.data(), pm);
	test_puma_inverse_solver(m.get(), pm);

	
	std::cout << "-----------------test model solver puma finished------------" << std::endl << std::endl;
}

