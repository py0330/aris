#include "test_dynamic_model_solver_seven_axis.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_seven_axis_forward_solver()
{

}
void test_seven_axis_inverse_solver()
{
	aris::dynamic::SevenAxisParam param;

	param.d1 = 0.3705;
	param.d3 = 0.330;
	param.d5 = 0.320;
	param.tool0_pe[2] = 0.2205;

	auto m = aris::dynamic::createModelSevenAxis(param);

	double pe[6]{ 0.45 , 0.12 , 0.32 , 0 , 0 , 0 };
	m->generalMotionPool()[0].setMpe(pe, "321");

	dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver&>(m->solverPool()[0]).setWhichRoot(8);
	dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver&>(m->solverPool()[0]).setAxisAngle(0.0);
	m->solverPool()[0].kinPos();

	m->solverPool()[1].kinPos();

	double result[6];
	m->generalMotionPool()[0].updMpm();
	m->generalMotionPool()[0].getMpe(result, "321");
	dsp(1, 6, result);
}

void test_model_solver_seven_axis()
{
	std::cout << std::endl << "-----------------test model solver seven_axis---------------------" << std::endl;

	test_seven_axis_inverse_solver();

	std::cout << "-----------------test model solver seven_axis finished------------" << std::endl << std::endl;
}

