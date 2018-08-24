#include "test_robot_rokae.h"
#include <iostream>
#include <aris_robot.h>

#include<type_traits>

using namespace aris::dynamic;

void test_rokae_solver()
{
	auto m = aris::robot::createRokaeModel();


	auto &inv = m->solverPool().add<aris::dynamic::InverseKinematicSolver>();

	inv.allocateMemory();

	auto &ee = m->generalMotionPool().at(0);

	ee.setMpq(std::array<double, 7>{0.3,0.01,0.6,0,0,0,1}.data());

	if (!inv.kinPos())std::cout << "failed" << std::endl;



	for (auto &m : m->motionPool())
	{
		m.updMp();
		std::cout << m.mp() << "   ";
	}

	std::cout << std::endl;

	auto &s = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
	s.allocateMemory();
	s.setWhichRoot(4);
	s.kinPos();


}

void test_robot_rokae()
{
	std::cout << std::endl << "-----------------test robot rokae---------------------" << std::endl;
	test_rokae_solver();

	std::cout << "-----------------test robot rokae finished------------" << std::endl << std::endl;
}

