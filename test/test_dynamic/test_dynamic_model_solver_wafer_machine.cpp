#include "test_dynamic_model_solver_wafer_machine.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

// 验证是否正解有解
void test_wafer_machine_forward_solver(){
	aris::dynamic::WaferMachineParam param;
	param.a = 0.1;
	param.b = 0.3;
	param.c = 0.3;
	auto m = aris::dynamic::createModelWaferMachine(param);

	double input[3]{ 1.58, 0.0, 0.3 }, output[3];
	m->forwardKinematics(input, output, 0);

	m->setInputPos(input);
	m->forwardKinematics();

	aris::dynamic::dsp(1, 3, output);

	m->inverseKinematics(output, input, 0);
	aris::dynamic::dsp(1, 3, input);

	output[1] = 0.1;
	m->setOutputPos(output);
	m->inverseKinematics(output, input, 0);
	aris::dynamic::dsp(1, 3, input);

	output[1] = -0.1;
	m->setOutputPos(output);
	m->inverseKinematics(output, input, 0);
	aris::dynamic::dsp(1, 3, input);

	m->forwardKinematics(input, output, 0);
	aris::dynamic::dsp(1, 3, output);
}
void test_wafer_machine_inverse_solver(){
	aris::dynamic::WaferMachineParam param;
	param.a = 0.1;
	param.b = 0.3;
	param.c = 0.3;
	auto m = aris::dynamic::createModelWaferMachineTwoArms(param);

	double input[4]{ 0.1,0.2,0.3,0.4 };
	m->setInputPos(input);

	m->forwardKinematics();
	m->inverseKinematics();

	m->getInputPos(input);

	aris::dynamic::dsp(1, 4, input);
}

void test_model_solver_wafer_machine(){
	std::cout << std::endl << "-----------------test model solver puma---------------------" << std::endl;

	test_wafer_machine_forward_solver();
	test_wafer_machine_inverse_solver();



	std::cout << "-----------------test model solver puma finished------------" << std::endl << std::endl;
}

