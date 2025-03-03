#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;


auto test_time_optimal_processor_1()->void {
	// 构造 TG //
	aris::plan::TrajectoryGenerator tg;

	const int PE_SIZE = 6;
	const int EE_NUM = 1;
	const int A_NUM = 0;

	//  INIT TG //
	tg.setEeTypes({ aris::dynamic::EEType::PE321 });
	double init_pe[EE_NUM * 6]{ 0.45, 0, 0.75,   aris::PI, 1.0,   aris::PI };
	double init_vel[EE_NUM * 2 + A_NUM]{ 1,1 };
	tg.insertLinePos(1, init_pe, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[PE_SIZE][6 * EE_NUM]{
		{ 0.45, 0.4, 0.75,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.75,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.65,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.65,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
		{ 0.45, 0.0, 0.75,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
		{ 0.45, 0.1, 0.75,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
	};

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 10, 10 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 1000, 1000 },
		{ 1000, 1000 },
		{ 1000, 1000 },
	};
	double accs[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 10, 10 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 10000, 10000 },
		{ 10000, 10000 },
		{ 10000, 10000 },
	};
	double jerks[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 100, 100 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 10000000, 1000000 },
		{ 10000000, 1000000 },
		{ 10000000, 1000000 },
	};
	double zones[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.2, 0.2 },
		{ 0.0, 0.0 },
		{ 0.1, 0.1 },
		{ 0.0, 0.0 },
		{ 0.0, 0.0 },
		{ 0.0, 0.0 },
	};

	for (int i = 0; i < PE_SIZE; ++i) {
		tg.insertLinePos(i + 10, pes[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}

	double out_pe[7 * EE_NUM + A_NUM];

	// 构造模型 //
	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 0;
	auto puma = aris::dynamic::createModelPuma(puma_param);

	puma->setOutputPos(init_pe);
	puma->inverseKinematics();

	dynamic_cast<aris::dynamic::GeneralMotion&>(puma->generalMotionPool()[0]).setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
	double input_init[6]{ 0,0,0,0,0,0 };
	puma->setInputPos(input_init);
	puma->forwardKinematics();
	double pm[16];
	puma->getOutputPos(pm);
	aris::dynamic::dsp(1, 16, pm);


	//  这里处理 //
	aris::plan::SingularProcessor sp;

	// 最大速度、加速度 //
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> max_jerks{ 314, 314, 314, 314, 314, 314 };

	// 设置模型等参数 //
	sp.setModel(*puma);
	sp.setMaxVels(max_vels.data());
	sp.setMaxAccs(max_accs.data());
	sp.setMaxJerks(max_jerks.data());
	sp.setTrajectoryGenerator(tg);
	//sp.setInverseKinematicMethod([](aris::dynamic::ModelBase &model, const double *output) {
	//	model
	//
	//
	//	});

	sp.init();

	// 设置速度百分比 //
	//sp.setDs(0.5);

	// 打印数据 //
	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{};
	while (auto ret = sp.setModelPosAndMoveDt()) {
		static auto last_ret = -1;
		if (ret != last_ret) {
			std::cout << "cmd:" << ret << std::endl;
			last_ret = ret;
		}
			
		
		m++;

		if (m > 3000 && m < 6000)
			sp.setTargetDs(0.0);
		else
			sp.setTargetDs(1.0);

		if (m == 12793) {
			std::cout << "debug" << std::endl;
		}



		vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		v_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		a_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		
		puma->getInputPos(vec.data() + (6 * EE_NUM + A_NUM) * (m - 1));
	}

	aris::dynamic::dlmwrite(m, (6 * EE_NUM + A_NUM), vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), v_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\vpes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), a_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\apes.txt");

}



void test_time_optimal(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	test_time_optimal_processor_1();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

