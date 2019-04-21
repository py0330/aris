#include "test_robot_rokae.h"
#include <iostream>
#include <array>
#include <aris/robot/robot.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_rokae_solver()
{
	auto m = aris::robot::createModelRokaeXB4();
	auto &inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
	auto &ee = m->generalMotionPool().at(0);

	ee.setMpq(std::array<double, 7>{0.32,0.0,0.6295,0,0,0,1}.data());

	for (int i = 0; i < 8; ++i)
	{
		inv.setWhichRoot(i);
		if (!inv.kinPos())std::cout << "failed" << std::endl;

		for (auto &m : m->motionPool())
		{
			m.updMp();
			std::cout << m.mp() << "   ";
		}
		std::cout << std::endl;
	}
}

void test_robot_rokae()
{
	std::cout << std::endl << "-----------------test robot rokae---------------------" << std::endl;
	test_rokae_solver();

	std::cout << "-----------------test robot rokae finished------------" << std::endl << std::endl;
}

