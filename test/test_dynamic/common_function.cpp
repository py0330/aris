#include "common_function.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>




auto test_model_kinematics_pos(aris::dynamic::ModelBase& m, int linspace_num, const double* range_below, const double* range_upper, double error)->int {
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
		//if (i == 2520)
		//	std::cout << "check" << std::endl;
		//else
		//	continue;

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
