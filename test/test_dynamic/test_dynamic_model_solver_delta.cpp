#include "test_dynamic_model_solver_delta.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include "common_function.h"

#include <aris.hpp>

#include<type_traits>

using namespace aris::dynamic;


void test_model_solver_delta_reduced()
{

	////////////////////////////////////////////////测试建模/////////////////////////////////////////////////
	aris::dynamic::DeltaParam param;
	param.a = 0.5;
	param.b = 0.2;
	param.c = 0.1;
	param.d = 0.7;
	param.e = 0.1;

	auto model = aris::dynamic::createModelDelta(param);
	auto& m1 = *model;

	double pos[4]{ -0.1, 0.1, -0.45, 0.3 };
	m1.setOutputPos(pos);
	if (m1.inverseKinematics())std::cout << "error!" << std::endl;

	double input[4];
	m1.getInputPos(input);
	//dsp(1, 4, input);

	double input2[4]{ -0.1, 0.1, -0.4, 0.2 };
	m1.setInputPos(input2);
	m1.forwardKinematics();

	double output2[4];
	m1.getOutputPos(output2);
	//dsp(1, 4, output2);

	m1.setOutputPos(output2);
	m1.inverseKinematics();

	m1.getInputPos(input);
	//dsp(1, 4, input);



	////////////////////////////////////////////////测试建模/////////////////////////////////////////////////
	{
		aris::dynamic::ScaraParam param;
		param.a = 0.3;
		param.b = 0.2;


		auto model = aris::dynamic::createModelScara(param);
		auto& m1 = *model;

		double pos[4]{ -0.2, 0.2, -0.45, 0.3 };
		m1.setOutputPos(pos);
		if (m1.inverseKinematics())std::cout << "error!" << std::endl;

		auto& solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m1.solverPool()[1]);
		solver.cptJacobi();



		//solver.cptJacobiWrtEE();
		//std::cout << solver.mJf() << std::endl;
		//std::cout << solver.nJf() << std::endl;
		//aris::dynamic::dsp(solver.mJf(), solver.nJf(), solver.Jf());



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


	double input_below[5]{ 0,0,0,0 }, input_upper[4]{ 2 * aris::PI,2 * aris::PI, 2 * aris::PI, 2 * aris::PI, };
	test_model_kinematics_pos(*model, 5, input_below, input_upper);






	std::cout << "-----------------test model solver delta finished------------" << std::endl << std::endl;
}
void test_model_solver_delta_full() {
	////////////////////////////////////////////////测试建模/////////////////////////////////////////////////
	double h = 0.1;
	
	aris::dynamic::DeltaFullParam param;
	//double ratio = 1.0;
	//param.ax1 = 0.55 * ratio;
	//param.ay1 = 0.05 * ratio;
	//param.az1 = 0.01 * ratio;
	//param.b1 = 0.2 * ratio;
	//param.c1 = (h+0.01) * ratio;
	//param.d1 = 0.7 * ratio;
	//param.ex1 = 0.1 * ratio;
	//param.ey1 = 0.05 * ratio;

	//param.ax2 = 0.49 * ratio;
	//param.b2 = 0.21 * ratio;
	//param.c2 = h * ratio;
	//param.d2 = 0.69 * ratio;
	//param.ex2 = 0.09 * ratio;

	//param.ax3 = 0.5 * ratio;
	//param.b3 = 0.23 * ratio;
	//param.c3 = h * ratio;
	//param.d3 = 0.71 * ratio;
	//param.ex3 = 0.12 * ratio;

	//param.ax1 = 0.5 * ratio;
	//param.ay1 = 0.0 * ratio;
	//param.az1 = 0.0 * ratio;
	//param.b1 = 0.2 * ratio;
	//param.c1 = (h+0.0) * ratio;
	//param.d1 = 0.7 * ratio;
	//param.ex1 = 0.1 * ratio;
	//param.ey1 = 0.05* ratio;

	//param.ax2 = 0.5 * ratio;
	//param.b2 = 0.2 * ratio;
	//param.c2 = h * ratio;
	//param.d2 = 0.7 * ratio;
	//param.ex2 = 0.1* ratio;

	//param.ax3 = 0.5 * ratio;
	//param.b3 = 0.2 * ratio;
	//param.c3 = h * ratio;
	//param.d3 = 0.7 * ratio;
	//param.ex3 = 0.1* ratio;

	param.ax1 = 0.11;
	param.ay1 = 0.0;
	param.az1 = 0.0;
	param.b1 = 0.26;
	param.c1 = 0.0;
	param.d1 = 0.7;
	param.ex1 = 0.06;
	param.ey1 = 0.0;
	param.ez1 = 0.0;
	param.theta1 = 0.0;
	param.f1 = 0.0;

	param.ax2 = 0.11;
	param.ay2 = 0.0;
	param.az2 = 0.0;
	param.b2 = 0.26;
	param.c2 = 0.0;
	param.d2 = 0.7;
	param.ex2 = 0.06;
	param.ey2 = 0.0;
	param.ez2 = 0.0;
	param.theta2 = aris::PI*2/3;
	param.f2 = 0.0;

	param.ax3 = 0.11;
	param.ay3 = 0.0;
	param.az3 = 0.0;
	param.b3 = 0.26;
	param.c3 = 0.0;
	param.d3 = 0.7;
	param.ex3 = 0.06;
	param.ey3 = 0.0;
	param.ez3 = 0.0;
	param.theta3 = -aris::PI * 2 / 3;
	param.f3 = 0.0;

	auto model = aris::dynamic::createModelDelta(param);

	double input[4]{0.1,0.2,0.3,0.4};
	model->setInputPos(input);
	model->forwardKinematics();
	double p[4];
	model->getOutputPos(p);
	dsp(1, 4, p);

	model->inverseKinematics();
	model->getInputPos(p);
	dsp(1, 4, p);

	//model->inverseKinematics();
	//model->getInputPos(p);
	//dsp(1, 4, p);

	auto &adams = model->simulatorPool().add<aris::dynamic::AdamsSimulator>();
	model->init();
	adams.saveAdams("C:\\Users\\py033\\Desktop\\test.cmd");

	double input_below[5]{ 0,0,0,0 }, input_upper[4]{ 2 * aris::PI,2 * aris::PI, 2 * aris::PI, 2 * aris::PI, };
	test_model_kinematics_pos(*model, 30, input_below, input_upper);

	std::cout << "-----------------test model solver delta finished------------" << std::endl << std::endl;
}
void test_model_solver_delta()
{

	test_model_solver_delta_full();




	std::cout << "-----------------test model solver delta finished------------" << std::endl << std::endl;
}

