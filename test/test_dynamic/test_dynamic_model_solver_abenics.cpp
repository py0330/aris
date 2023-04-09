#include "test_dynamic_model_solver_scara.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>
#include <aris/core/serialization.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_abenics_forward_solver() {

	aris::dynamic::AbenicsParam p;
	auto m = aris::dynamic::createModelAbenics(p);

	// 输出是 123 的欧拉角 //
	double output[3]{0.1,0.2,0.3}, input[6];
	m->setOutputPos(output);
	m->inverseKinematics();
	m->getInputPos(input);

	dsp(1, m->inputPosSize(), input);
}

void test_model_solver_abenics(){
	std::cout << std::endl << "-----------------test model solver scara---------------------" << std::endl;

	test_abenics_forward_solver();

	std::cout << "-----------------test model solver scara finished------------" << std::endl << std::endl;
}

