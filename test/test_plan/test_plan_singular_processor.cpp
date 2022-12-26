﻿#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

auto test_singular_processor_1()->void {
	aris::plan::TrajectoryGenerator tg;

	const int PE_SIZE = 3;
	const int EE_NUM = 1;
	const int A_NUM = 0;

	//  INIT TG //
	tg.setEeTypes({ aris::dynamic::EEType::PE321 });
	double init_pe[EE_NUM * 6]{ 0.45, 0, - 0.75,   0, - aris::PI/2,   aris::PI };
	double init_vel[EE_NUM * 2 + A_NUM]{ 1,1 };
	tg.insertLinePos(1, init_pe, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[PE_SIZE][6 * EE_NUM]{
		{ 0.45, 0.1, -0.75,   0, -aris::PI / 2,   aris::PI},
		{ 0.45, 0.0, -0.75,   0, -aris::PI / 2,   aris::PI},
		{ 0.45, 0.0, -0.65,   0, -aris::PI / 2,   aris::PI},
	};

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
	};
	double accs[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
	};
	double jerks[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 1.0, 10.0 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
	};
	double zones[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.2, 0.2 },
		{ 0.0, 0.0 },
		{ 0.1, 0.1 },
	};

	for (int i = 0; i < PE_SIZE; ++i) {
		tg.insertLinePos(i + 10, pes[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}

	double out_pe[7 * EE_NUM + A_NUM];

	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 1;

	puma_param.base2ref_pe[0] = 1;
	puma_param.base2ref_pe[1] = 2;
	puma_param.base2ref_pe[2] = 3;
	// 安装方式
	// 0, 正常安装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
	// 1，顶部吊装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
	// 2，侧装向上，零位时末端法兰盘朝向：地面 z 轴，零位时末端1轴朝向：地面 x 轴
	// 3，侧装向下，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
	auto puma = aris::dynamic::createModelPuma(puma_param);

	puma->forwardKinematics();
	double input[6];
	puma->getOutputPos(input);
	aris::dynamic::dsp(1, 6, input);


	SingularProcessor sp;

	sp.setMaxVels({ 3.14,3.14,3.14,3.14,3.14,3.14 });
	sp.setMaxAccs({ 3.14,3.14,3.14,3.14,3.14,3.14 });
	sp.setModel(*puma);
	sp.setTrajectoryGenerator(tg);

	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{};
	while (sp.setModelPosAndMoveDt()) {
		m++;
		vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		v_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		a_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		
		puma->getInputPos(vec.data() + (6 * EE_NUM + A_NUM) * (m - 1));
	}

	aris::dynamic::dlmwrite(m, (6 * EE_NUM + A_NUM), vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), v_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\vpes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), a_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\apes.txt");

}
void test_singular_processor(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	test_singular_processor_1();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

