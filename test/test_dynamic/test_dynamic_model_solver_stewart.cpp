#include "test_dynamic_model_solver_stewart.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_stewart_inverse_solver()
{
	auto m = aris::dynamic::createModelStewart();
	auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));

	auto &adams = m->simulatorPool().add<aris::dynamic::AdamsSimulator>();
	adams.saveAdams("C:\\Users\\py033\\Desktop\\test1.cmd");

	m->generalMotionPool()[0].setMpe(std::array<double, 6>{0,1,0.2,aris::PI/4,aris::PI/5,aris::PI/7}.data(), "313");

	inv.kinPos();

	for (int i = 0; i < 6; ++i)std::cout << m->motionPool()[i].mp() << "  ";
	std::cout << std::endl;

	adams.saveAdams("C:\\Users\\py033\\Desktop\\test2.cmd");
}

void test_model_solver_stewart()
{
	std::cout << std::endl << "-----------------test model solver stewart---------------------" << std::endl;

	test_stewart_inverse_solver();

	std::cout << "-----------------test model solver stewart finished------------" << std::endl << std::endl;
}

