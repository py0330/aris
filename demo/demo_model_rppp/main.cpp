#include <iostream>

#include "aris.hpp"

int main() {
	// 构造 rppp 模型（R + P + P + P：绕 z 旋转 + 三个平动）//
	aris::dynamic::RpppParam param;

	// tool0 偏移（"321" 欧拉角为 [rz, ry, rx]，仅 xyz 与 rz 非零）//
	param.tool0_pe[0] = 0.1;
	param.tool0_pe[1] = -0.2;
	param.tool0_pe[2] = 0.3;
	param.tool0_pe[3] = 0.4;
	param.tool0_pe_type = "321";

	auto m = aris::dynamic::createModelRppp(param);

	//////////////////// 反解 ////////////////////
	// 末端目标位姿 [x, y, z, rz] //
	double target[4]{ 0.61, 0.85, 0.3, 0.2 };
	m->setOutputPos(target);

	if (m->inverseKinematics())
		std::cout << "inverse kinematics failed" << std::endl;

	// 获取关节位置 //
	double input[4];
	m->getInputPos(input);

	std::cout << "input:  ";
	aris::dynamic::dsp(1, 4, input);

	//////////////////// 正解 ////////////////////
	m->setInputPos(input);
	if (m->forwardKinematics())
		std::cout << "forward kinematics failed" << std::endl;

	double output[4];
	m->getOutputPos(output);

	std::cout << "output: ";
	aris::dynamic::dsp(1, 4, output);

	std::cout << "demo_model_rppp finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}
