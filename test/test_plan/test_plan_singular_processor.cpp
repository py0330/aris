#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

auto test_s_smooth3_a_range() {
	auto p_max = 0.55;
	auto p_min = -10.0;
	auto v_max = 10.0;
	auto v_min = -10.0;
	auto a_max = 10.0;
	auto a_min = -10.0;
	auto j_max = 15.0;
	auto j_min = -15.0;
	auto dt = 1e-3;

	auto p = 0.5;
	auto v = 1.5;
	auto a = 0.1;

	double a_upper, a_below;

	auto ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_upper - (-22.1686981556106)) > 1e-10 || std::abs(a_below - (-22.1686981556106)) > 1e-10)
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;

	p_max = 0.62;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_upper - (-8.58149320825049)) > 1e-10 || std::abs(a_below - (-10)) > 1e-10)
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;

	p_max = 1.5;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_upper - (3.28669092032092)) > 1e-10 || std::abs(a_below - (-10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 4.2;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (8.56369443603907)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 5.59;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (10)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}


	p_max = 0.61175125-1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (-10.000000008889)) >1e-10 || std::abs(a_below - (-10.000000008889)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 0.61175125+1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (-9.99999866232704)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 0.634829342120835-1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (-7.42370228620955)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 0.634829342120835+1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (-7.42370227272833)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 3.32663362334213-1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (7.40870227932057)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 3.32663362334213+1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (7.40870227961728)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 5.58288087962963-1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (9.99999999990828)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_max = 5.58288087962963+1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if(std::abs(a_upper - (10)) >1e-10 || std::abs(a_below - (-10)) >1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	v = 5.0;
	p_max = 5.59;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_upper - (4.69225187829757)) > 1e-10 || std::abs(a_below - (-10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}


	v = -1.5;
	p_max = 10;
	j_min = -5;
	p_min = 1.0 - 0.62;

	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (8.58149320825049)) > 1e-10 || std::abs(a_upper - a_max) > 1e-10)
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;

	p_min = 1.0 - 1.5;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-3.28669092032092)) > 1e-10 || std::abs(a_upper - a_max) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 4.2;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-8.56369443603907)) > 1e-10 || std::abs(a_upper - a_max) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 5.59;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-10)) > 1e-10 || std::abs(a_upper - a_max) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 0.61175125 + 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (10.000000008889)) > 1e-10 || std::abs(a_upper - (10.000000008889)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 0.61175125 - 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (9.99999866232704)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 0.634829342120835 + 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (7.42370228620955)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 0.634829342120835 - 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (7.42370227272833)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 3.32663362334213 + 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-7.40870227932057)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 3.32663362334213 - 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-7.40870227961728)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 5.58288087962963 + 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-9.99999999990828)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	p_min = 1.0 - 5.58288087962963 - 1e-10;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-10)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}

	v = -5.0;
	p_min = 1.0 - 5.59;
	ret = s_smooth3_a_range(p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min, dt, p, v, a, a_upper, a_below);
	if (std::abs(a_below - (-4.69225187829757)) > 1e-10 || std::abs(a_upper - (10)) > 1e-10) {
		std::cout << "\"s_smooth3_a_range\" failed" << std::endl;
	}
}



auto test_singular_processor_1()->void {
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

auto test_singular_processor_2() -> void {
	// 构造 TG //
	aris::plan::TimeOptimalTrajectoryGenerator tg;

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
	aris::plan::LookAheadProcessor sp;

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

	sp.lookAhead(0.0);

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


struct TestSmoothParam {
	int n;
	int dim;
	double dt;
	std::function<void(double, double*)> f;
	std::vector<double> max_p, min_p, max_dp, min_dp, max_d2p, min_d2p, max_d3p, min_d3p;
	double s_begin;
	double ds_begin;
	double max_permit_jerk_times_{ 6 };
	double min_s_end{ 5 };
};
struct TestSmoothResult {
	std::vector<double> poss;
	std::vector<double> max_p, max_dp, max_d2p, max_d3p, min_p, min_dp, min_d2p, min_d3p;
	double s_end;
};
auto test_smooth_func(TestSmoothParam param) -> TestSmoothResult{
	TestSmoothResult result;

	auto dim = param.dim;
	auto n = param.n;
	auto dt = param.dt;
	auto f = param.f;

	result.max_p.resize(dim, std::numeric_limits<double>::min());
	result.min_p.resize(dim, std::numeric_limits<double>::max());
	result.max_dp.resize(dim, std::numeric_limits<double>::min());
	result.min_dp.resize(dim, std::numeric_limits<double>::max());
	result.max_d2p.resize(dim, std::numeric_limits<double>::min());
	result.min_d2p.resize(dim, std::numeric_limits<double>::max());
	result.max_d3p.resize(dim, std::numeric_limits<double>::min());
	result.min_d3p.resize(dim, std::numeric_limits<double>::max());

	std::vector<double> p0(dim), p1(dim), p2(dim), p3(dim);
	double s0 = dt * 0, s1 = dt * 0.1, s2 = dt * 0.2, s3 = dt * 0.3;

	f(s0, p0.data());
	f(s1, p1.data());
	f(s2, p2.data());
	f(s3, p3.data());

	result.poss.clear();
	for (int i = 0; i < dim; ++i)
		result.poss.push_back(p0[i]);
	for (int i = 0; i < dim; ++i)
		result.poss.push_back(p1[i]);
	for (int i = 0; i < dim; ++i)
		result.poss.push_back(p2[i]);
	for (int i = 0; i < dim; ++i)
		result.poss.push_back(p3[i]);

	for (int i = 4; i < n; ++i) {
		SmoothParam p{
			dt,
			dim,
			param.min_p.data(), param.max_p.data(), 
			param.min_dp.data(), param.max_dp.data(), 
			param.min_d2p.data(), param.max_d2p.data(), 
			param.min_d3p.data(), param.max_d3p.data(),
			0.005, 1.0, -1000, 1000, -100000, 1000000,
			(s1 - s0) / dt, (s2 - s1) / dt, (s3 - s2) / dt,
			p0.data(), p1.data(), p2.data(), p3.data(),
			1.0
		};
		SmoothRet ret;
		s_smooth_curve3(p, ret);

		//if (ret.state) {
		//	std::cout << "count:" << i << "  ret:" << ret.state <<"  ds:"<<ret.next_ds << std::endl;
		//}
		
		if (i > 2000 && i < 2050) 
		{
			static double last_ds = 0;

			
			std::cout << "count:" << i << "  ret:" << ret.state << "  ds:" << ret.next_ds << "  d2s:" << (ret.next_ds - last_ds)/1e-3 << std::endl;
			last_ds = ret.next_ds;
		}
		//if (i > 1585 && i < 2000)
		//{
		//	std::cout << "count:" << i << "  ret:" << ret.state << std::endl;
		//	std::cout << "ds:" << ret.next_ds << std::endl;
		//}


		std::swap(p0, p1);
		std::swap(p1, p2);
		std::swap(p2, p3);

		std::swap(s0, s1);
		std::swap(s1, s2);
		std::swap(s2, s3);

		s3 = s2 + ret.next_ds * dt;
		f(s3, p3.data());

		for (int i = 0; i < dim; ++i) {
			result.poss.push_back(p3[i]);
			
			double dp0 = (p1[i] - p0[i]) / dt;
			double dp1 = (p2[i] - p1[i]) / dt;
			double dp2 = (p3[i] - p2[i]) / dt;

			double d2p0 = (dp1 - dp0) / dt;
			double d2p1 = (dp2 - dp1) / dt;

			double d3p0 = (d2p1 - d2p0) / dt;

			result.max_p[i] = std::max(result.max_p[i], p3[i]);
			result.min_p[i] = std::min(result.min_p[i], p3[i]);
			result.max_dp[i] = std::max(result.max_dp[i], dp2);
			result.min_dp[i] = std::min(result.min_dp[i], dp2);
			result.max_d2p[i] = std::max(result.max_d2p[i], d2p1);
			result.min_d2p[i] = std::min(result.min_d2p[i], d2p1);
			result.max_d3p[i] = std::max(result.max_d3p[i], d3p0);
			result.min_d3p[i] = std::min(result.min_d3p[i], d3p0);
		}
	}

	for (int i = 0; i < param.dim; ++i) {
		if (result.max_d3p[i] > param.max_d3p[i] * param.max_permit_jerk_times_)
			std::cout << "error" << std::endl;
		if (result.min_d3p[i] < param.min_d3p[i] * param.max_permit_jerk_times_)
			std::cout << "error" << std::endl;
	}
	


	result.s_end = s3;

	return result;
}

template<int a, int b>
auto sin_func(double s, double* p) ->int{
	double d_a = a;
	double d_b = b;
	// 
	// p   = a * sin(b*s)
	// dp  = a*b * cos(b*s)
	// d2p = -a*b*b* sin(b*s)
	// d3p = -a*b*b*b*cos(b*s)
	p[0] = d_a * std::sin(d_b * s - aris::PI / 2);


	return 0;
}

// 
auto test_smooth_cond_pos_violate() -> void {
	TestSmoothParam param;
	TestSmoothResult result;
	
	//param = TestSmoothParam {6000, 1, 0.001, sin_func<1,2>,
	//	{0.5},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//	0.0, 0.1
	//};
	//result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{1.5},{-0.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);

	param = TestSmoothParam{ 6000, 1, 0.001, sin_func<-1,2>,
	{1.5},{-0.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	0.0, 0.1
	};
	result = test_smooth_func(param);

	param = TestSmoothParam{ 6000, 1, 0.001, sin_func<-1,2>,
	{0.5},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	0.0, 0.1
	};
	result = test_smooth_func(param);

	param = TestSmoothParam{ 6000, 1, 0.01, sin_func<-1,2>,
	{0.99},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	0.0, 0.1
	};
	result = test_smooth_func(param);

	param = TestSmoothParam{ 6000, 1, 0.01, sin_func<1,2>,
	{1.5},{-0.99},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	0.0, 0.1
	};
	result = test_smooth_func(param);

	int n = 6000;

	
	

	std::cout << "s end:"<< result.s_end << std::endl;
	std::cout << "max p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_p.data());
	std::cout << "min p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_p.data());
	std::cout << "max dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_dp.data());
	std::cout << "min dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_dp.data());
	std::cout << "max d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d2p.data());
	std::cout << "min d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d2p.data());
	std::cout << "max d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d3p.data());
	std::cout << "min d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d3p.data());


	aris::dynamic::dlmwrite(n, 1, result.poss.data(), "C:\\Users\\py033\\Desktop\\test_data\\poss.txt");
}

auto test_smooth_cond_vel_violate() -> void {
	TestSmoothParam param;
	TestSmoothResult result;

	param = TestSmoothParam{ 3000, 1, 0.001, sin_func<-1,2>,
	{100},{-100},{1.5},{-1.5},{3.0},{-3.0},{10.0},{-10.0},
	0.0, 0.1
	};
	result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.001, sin_func<-1,2>,
	//{0.5},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.01, sin_func<-1,2>,
	//{0.99},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.01, sin_func<1,2>,
	//{1.5},{-0.99},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);



	//TestSmoothParam param1 = TestSmoothParam{ 6000, 1, 0.01, sin_func<1,2>,
	//	{100},{-100},{1.5},{-1.5},{5.0},{-5.0},{10.0},{-10.0},
	//	0.0, 0.1
	//};
	//TestSmoothParam param2 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{100},{-100},{1.5},{-1.5},{10.0},{-10.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//TestSmoothParam param3 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{100},{-1.5},{1.0},{-1.5},{10.0},{-10.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//TestSmoothParam param4 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{100},{-1.5},{5.0},{-5},{5.0},{-5.0},{50.0},{-40.0},
	//0.0, 0.1
	//};
	//auto& param = param3;
	//auto result = test_smooth_func(param);
	int n = 3000;



	std::cout << "s end:" << result.s_end << std::endl;
	std::cout << "max p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_p.data());
	std::cout << "min p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_p.data());
	std::cout << "max dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_dp.data());
	std::cout << "min dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_dp.data());
	std::cout << "max d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d2p.data());
	std::cout << "min d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d2p.data());
	std::cout << "max d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d3p.data());
	std::cout << "min d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d3p.data());


	aris::dynamic::dlmwrite(n, 1, result.poss.data(), "C:\\Users\\py033\\Desktop\\test_data\\poss.txt");
}

auto test_smooth_cond_acc_violate() -> void {

	TestSmoothParam param1 = TestSmoothParam{ 6000, 1, 0.01, sin_func<1,2>,
		{100},{-100},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
		0.0, 0.1
	};
	TestSmoothParam param2 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	{100},{-100},{1.5},{-1.5},{10.0},{-10.0},{10.0},{-10.0},
	0.0, 0.1
	};
	TestSmoothParam param3 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	{100},{-1.5},{1.0},{-1.5},{10.0},{-10.0},{10.0},{-10.0},
	0.0, 0.1
	};
	TestSmoothParam param4 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	{100},{-1.5},{5.0},{-5},{5.0},{-5.0},{50.0},{-40.0},
	0.0, 0.1
	};

	int n = 6000;

	auto& param = param3;
	auto result = test_smooth_func(param);

	std::cout << "s end:" << result.s_end << std::endl;
	std::cout << "max p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_p.data());
	std::cout << "min p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_p.data());
	std::cout << "max dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_dp.data());
	std::cout << "min dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_dp.data());
	std::cout << "max d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d2p.data());
	std::cout << "min d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d2p.data());
	std::cout << "max d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d3p.data());
	std::cout << "min d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d3p.data());


	aris::dynamic::dlmwrite(n, 1, result.poss.data(), "C:\\Users\\py033\\Desktop\\test_data\\poss.txt");
}

auto test_smooth_cond_discontinue() -> void {
	TestSmoothParam param;
	TestSmoothResult result;

	param = TestSmoothParam{ 6000, 1, 0.001,[](double s, double* p)->int {
		if (s < 0.5) {
			return sin_func<1, 2>(s, p);
		}
		else {
			return sin_func<2, 2>(s, p);
		}
		
		},
	{100},{-100},{1.5},{-1.5},{3.0},{-3.0},{10.0},{-10.0},
	0.0, 0.1
	};
	result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.001, sin_func<-1,2>,
	//{0.5},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.01, sin_func<-1,2>,
	//{0.99},{-1.5},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);

	//param = TestSmoothParam{ 6000, 1, 0.01, sin_func<1,2>,
	//{1.5},{-0.99},{3.0},{-3.0},{3.0},{-3.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//result = test_smooth_func(param);



	//TestSmoothParam param1 = TestSmoothParam{ 6000, 1, 0.01, sin_func<1,2>,
	//	{100},{-100},{1.5},{-1.5},{5.0},{-5.0},{10.0},{-10.0},
	//	0.0, 0.1
	//};
	//TestSmoothParam param2 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{100},{-100},{1.5},{-1.5},{10.0},{-10.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//TestSmoothParam param3 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{100},{-1.5},{1.0},{-1.5},{10.0},{-10.0},{10.0},{-10.0},
	//0.0, 0.1
	//};
	//TestSmoothParam param4 = TestSmoothParam{ 6000, 1, 0.001, sin_func<1,2>,
	//{100},{-1.5},{5.0},{-5},{5.0},{-5.0},{50.0},{-40.0},
	//0.0, 0.1
	//};
	//auto& param = param3;
	//auto result = test_smooth_func(param);
	int n = 6000;



	std::cout << "s end:" << result.s_end << std::endl;
	std::cout << "max p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_p.data());
	std::cout << "min p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_p.data());
	std::cout << "max dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_dp.data());
	std::cout << "min dp:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_dp.data());
	std::cout << "max d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d2p.data());
	std::cout << "min d2p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d2p.data());
	std::cout << "max d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.max_d3p.data());
	std::cout << "min d3p:" << std::endl;
	aris::dynamic::dsp(1, param.dim, result.min_d3p.data());


	aris::dynamic::dlmwrite(n, 1, result.poss.data(), "C:\\Users\\py033\\Desktop\\test_data\\poss.txt");
}

void test_singular_processor(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;


	test_singular_processor_2();
	//test_s_smooth3_a_range();

	//test_smooth_cond_vel_violate();
	//test_smooth_cond_discontinue();
	//for(int i =0; i <1000; ++i)
	//test_singular_processor_1();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

