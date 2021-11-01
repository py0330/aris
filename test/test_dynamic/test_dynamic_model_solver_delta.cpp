#include "test_dynamic_model_solver_delta.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_model_solver_delta()
{

	////////////////////////////////////////////////测试建模/////////////////////////////////////////////////
	aris::dynamic::DeltaParam param;
	param.a = 0.5;
	param.b = 0.2;
	param.c = 0.1;
	param.d = 0.7;
	param.e = 0.1;

	auto model = aris::dynamic::createModelDelta(param);
	auto &m1 = *model;

	double pos[4]{ -0.1, 0.1, -0.45, 0.3 };
	m1.setOutputPos(pos);
	if(m1.inverseKinematics())std::cout << "error!" << std::endl;

	double input[4];
	m1.getInputPos(input);
	dsp(1, 4, input);

	double input2[4]{ -0.1, 0.1, -0.4, 0.2 };
	m1.setInputPos(input2);
	m1.forwardKinematics();

	double output2[4];
	m1.getOutputPos(output2);
	dsp(1, 4, output2);

	m1.setOutputPos(output2);
	m1.inverseKinematics();

	m1.getInputPos(input);
	dsp(1, 4, input);



	////////////////////////////////////////////////测试建模/////////////////////////////////////////////////
	{
		aris::dynamic::ScaraParam param;
		param.a = 0.3;
		param.b = 0.2;


		auto model = aris::dynamic::createModelScara(param);
		auto &m1 = *model;

		double pos[4]{ -0.2, 0.2, -0.45, 0.3 };
		m1.setOutputPos(pos);
		if (m1.inverseKinematics())std::cout << "error!" << std::endl;

		auto &solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m1.solverPool()[1]);
		solver.cptJacobi();



		solver.cptJacobiWrtEE();
		std::cout << solver.mJf() << std::endl;
		std::cout << solver.nJf() << std::endl;
		aris::dynamic::dsp(solver.mJf(), solver.nJf(), solver.Jf());



		//double input[4];
		//m1.getInputPos(input);
		//dsp(1, 4, input);

		//double input2[4]{ -0.1, 0.1, -0.4, 0.2 };
		//m1.setInputPos(input2);
		//m1.forwardKinematics();

		//double output2[4];
		//m1.getOutputPos(output2);
		//dsp(1, 4, output2);

		//m1.setOutputPos(output2);
		//m1.inverseKinematics();

		//m1.getInputPos(input);
		//dsp(1, 4, input);

	}










	std::cout << "-----------------test model solver delta finished------------" << std::endl << std::endl;
}

