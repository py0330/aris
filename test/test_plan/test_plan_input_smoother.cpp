#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

// 
auto test_input_smoother_sin()->void {
	// 设置 InputSmoother //
	aris::plan::InputSmoother sp;

	const int input_size = 1;
	sp.setInputSize(input_size);

	// 最大速度、加速度 //
	std::vector<double> max_poss{ 314, 314, 314, 314, 314, 314 };
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> min_vels{ -3.14, -3.14, -3.14, -3.14, -3.14, -3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> min_accs{ -31.4, -31.4, -31.4, -31.4, -31.4, -31.4 };

	sp.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	sp.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	sp.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	sp.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));
	
	sp.setInputGenerator([](double* p)->std::int64_t {
		static int count_{ 0 };

		if (count_ == 0) {
			p[0] = -1;
		}
		else {
			p[0] = std::sin(count_ * 0.001) * 10;
		}

		count_++;

		if ((count_ % 1000) == 0)
			return 0;


		return 1;
	});
	
	sp.allocateMemory();
	double input_init[1]{ -1 };
	sp.init(input_init);
	
	// 设置 AsyncGenerator //
	aris::plan::AsyncGenerator ge;
	ge.setCacheSize(1000);
	ge.setDt(1e-3);
	ge.setInputSize(input_size);
	ge.setInputGenerator([&sp](double* p)->std::int64_t {
		return sp.getNextInput(p);
		});
	ge.allocateMemory();
	ge.init();

	// 设置 SpeedRegulator //
	aris::plan::SpeedRegulator sr;
	sr.setInputSize(input_size);
	sr.setDt(0.001);
	sr.setInputGenerator([&ge](double* p)->std::int64_t {
		return ge.getNextInput(p);
		});
	sr.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	sr.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	sr.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	sr.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));
	sr.allocateMemory();
	sr.init(1.0);
	
	// 打印数据 //
	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{}, ee_pos[16], input_pos[6];
	double s = 0;

	int stop_count = 10;
	std::int64_t ret;

	while ((ret = sr.getNextInput(input_pos)) || stop_count > 0) {
		if (ret < 0) {
			std::cout << "failed:" << ret << std::endl;
			break;
		}

		static auto last_ret = -1;
		if (ret != last_ret) {
			//std::cout << "cmd:" << ret << std::endl;
			last_ret = ret;
		}
		
		m++;
		if(m%100 == 0)
			std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));


		if (ret == 0) {
			stop_count--;
			std::cout << "stopped " << stop_count << ":" << m << std::endl;

			//std::this_thread::sleep_for(std::chrono::seconds(10));
		}
		//if(m > 1000 && m < 2000)
		//	sr.setTargetSpeedRatio(0.0);
		//else if(m > 3000)
		//	sr.setTargetSpeedRatio(1.0);

		//std::cout << "m:" << m << std::endl;

		vec.resize(m * input_size, 0.0);
		std::copy_n(input_pos, input_size, vec.data() + input_size * (m - 1));
	}

	aris::dynamic::dlmwrite(m, input_size, vec.data(), "/Mac/Home/Documents/MATLAB/test/data.txt");
}

// 
auto test_input_smoother_cos() -> void {
	// 设置 InputSmoother //
	aris::plan::InputSmoother sp;

	const int input_size = 1;
	sp.setInputSize(input_size);

	// 最大速度、加速度 //
	std::vector<double> max_poss{ 314, 314, 314, 314, 314, 314 };
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> min_vels{ -3.14, -3.14, -3.14, -3.14, -3.14, -3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> min_accs{ -31.4, -31.4, -31.4, -31.4, -31.4, -31.4 };

	sp.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	sp.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	sp.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	sp.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));

	sp.setInputGenerator([](double* p)->std::int64_t {
		static int count_{ 0 };

		//if (count_ == 0) {
		//	p[0] = -1;
		//}
		//else {
			p[0] = std::cos(count_ * 0.001) * 100;
		//}

		count_++;

		if (count_ > 1000)
			return 0;


		return 1;
		});

	sp.allocateMemory();
	double input_init[1]{ 100 };
	sp.init(input_init);

	// 设置 AsyncGenerator //
	aris::plan::AsyncGenerator ge;
	ge.setCacheSize(1000);
	ge.setDt(1e-3);
	ge.setInputSize(input_size);
	ge.setInputGenerator([&sp](double* p)->std::int64_t {
		return sp.getNextInput(p);
		});
	ge.allocateMemory();
	ge.init();

	// 设置 SpeedRegulator //
	aris::plan::SpeedRegulator sr;
	sr.setInputSize(input_size);
	sr.setDt(0.001);
	sr.setInputGenerator([&ge](double* p)->std::int64_t {
		return ge.getNextInput(p);
		});
	sr.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	sr.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	sr.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	sr.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));
	sr.allocateMemory();
	sr.init(1.0);

	// 打印数据 //
	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{}, ee_pos[16], input_pos[6];
	double s = 0;
	while (auto ret = sr.getNextInput(input_pos)) {
		if (ret < 0) {
			std::cout << "failed:" << ret << std::endl;
			break;
		}

		static auto last_ret = -1;
		if (ret != last_ret) {
			std::cout << "cmd:" << ret << std::endl;
			last_ret = ret;
		}

		m++;
		if (m % 100 == 0)
			std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));

		//if(m > 1000 && m < 2000)
		//	sr.setTargetSpeedRatio(0.0);
		//else if(m > 3000)
		//	sr.setTargetSpeedRatio(1.0);

		std::cout << "m:" << m << std::endl;

		vec.resize(m * input_size, 0.0);
		std::copy_n(input_pos, input_size, vec.data() + input_size * (m - 1));
	}

	aris::dynamic::dlmwrite(m, input_size, vec.data(), "/Mac/Home/Documents/MATLAB/test/data.txt");
}

auto test_input_smoother_2() -> void {
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
	double input_init[6]{ -1,0,0,0,0,0 };
	puma->getInputPos(input_init);


	// 设置 InputSmoother //
	aris::plan::InputSmoother sp;

	const int input_size = 6;
	sp.setInputSize(input_size);
	sp.allocateMemory();

	// 最大速度、加速度 //
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> min_vels{ -3.14, -3.14, -3.14, -3.14, -3.14, -3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> min_accs{ -31.4, -31.4, -31.4, -31.4, -31.4, -31.4 };

	sp.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	sp.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	sp.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	sp.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));

	// 设置反解 //
	sp.setInputGenerator([&puma, &tg](double* p)->std::int64_t {
		double output[16];
		auto ret = tg.getEePosAndMoveDt(output);
		puma->setOutputPos(output);
		puma->inverseKinematics();
		puma->getInputPos(p);
		return ret;
	});
	sp.init(input_init);

	// 打印数据 //
	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{}, ee_pos[16], input_pos[6];
	double s = 0;
	while (auto ret = sp.getNextInput(input_pos)) {
		static auto last_ret = -1;
		if (ret != last_ret) {
			std::cout << "cmd:" << ret << std::endl;
			last_ret = ret;
		}

		m++;

		vec.resize(m * (input_size * EE_NUM + A_NUM), 0.0);
		std::copy_n(input_pos, input_size, vec.data() + (input_size * EE_NUM + A_NUM) * (m - 1));
	}

	aris::dynamic::dlmwrite(m, (input_size * EE_NUM + A_NUM), vec.data(), "/Mac/Home/Documents/MATLAB/test/data.txt");
}

void test_input_smoother(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	test_input_smoother_sin();
	//test_input_smoother_cos();
	//test_input_smoother_2();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

