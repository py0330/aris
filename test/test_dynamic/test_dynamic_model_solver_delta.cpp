#include "test_dynamic_model_solver_delta.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

auto test_model_kinematics_pos(aris::dynamic::ModelBase &m, int linspace_num, const double *range_below, const double *range_upper, double error = 1e-10)->int {
	// lin space //
	std::vector<std::vector<double>> input_series(m.inputPosSize(), std::vector<double>(linspace_num, 0.0));
	for (int i = 0; i < m.inputPosSize(); ++i) {
		for (int j = 0; j < linspace_num; ++j) {
			double ratio = (double)j / (linspace_num - 1);
			input_series[i][j] = (1 - ratio) * range_below[i] + ratio * range_upper[i];
		}
	}

	std::vector<double> input(m.inputPosSize()), output(m.outputPosSize()), output_compare(m.outputPosSize());
	for (int i = 0; i < std::pow(linspace_num, (int)m.inputPosSize()); ++i) {
		for (int j = 0; j < m.inputPosSize(); ++j) {
			input[j] = input_series[j][(i / (int)std::pow(linspace_num, j)) % linspace_num];
		}
		
		// 通过正解设置初值，考虑到反解可能会有多解，因此这里不去比较直接反解回来的输入 //
		m.setInputPos(input.data());
		if (m.forwardKinematics()) {
			std::cout << __FILE__ << __LINE__ << " failed forward kinematics: perhaps outside the workspace" << std::endl;
			aris::dynamic::dsp(1, m.inputPosSize(), input.data());
			continue;
		}
			
		m.getOutputPos(output.data());


		// 得到反解的值 //
		if (m.inverseKinematics()) {
			std::cout << __FILE__ << __LINE__ << " failed inverse kinematics" << std::endl;
			continue;
		}
			

		// 再次正解，理应得到同样的反解值 //
		if (m.forwardKinematics()) {
			std::cout << __FILE__ << __LINE__ << " failed forward kinematics again" << std::endl;
			continue;
		}
			

		// 比较两次的末端位置
		m.getOutputPos(output_compare.data());
		if (!aris::dynamic::s_is_finite(m.outputPosSize(), output.data()) 
			|| !aris::dynamic::s_is_equal(m.outputPosSize(), output.data(), output_compare.data(), error)) 
		{
			m.setInputPos(input.data());
			m.forwardKinematics();
			
			std::cout << __FILE__ << __LINE__ << " failed inverse & forward kinematics mismatch" << std::endl;
			aris::dynamic::dsp(1, m.inputPosSize(), input.data());
			aris::dynamic::dsp(1, m.outputPosSize(), output.data());
			aris::dynamic::dsp(1, m.outputPosSize(), output_compare.data());
			continue;
		}
			
	}

	return 0;

}



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
		auto &m1 = *model;

		double pos[4]{ -0.2, 0.2, -0.45, 0.3 };
		m1.setOutputPos(pos);
		if (m1.inverseKinematics())std::cout << "error!" << std::endl;

		auto &solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m1.solverPool()[1]);
		solver.cptJacobi();



		solver.cptJacobiWrtEE();
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

