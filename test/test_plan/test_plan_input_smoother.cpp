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





auto test_input_smoother_7axis()->int {

	// ============================================================================
	// 仿真参数 //
	const bool seg_debug_flag = false; // 是否逐段打印 //
	const int cmd_cnt_max = 3; // 模拟指令执行次数 // 
	const int step_cnt_max = 10000; // 单次执行最大步数 //

	// 规划器参数 //
	const double vel = 1000;
	const double ome = 1000;
	const double acc = 100;
	const double jerk = 1000;
	const double zone = 0.01;

	const double joint_vel = 1.5;
	const double joint_acc = 5.0;
	// ============================================================================
	const int JOINT_NUM = 7;
	const int EE_DIM = 6;
	const double DT = 0.004; // 控制周期 // 
	const double pi = 3.141592653589793;


	auto arm_id = 1;// 0:left 1:right
	double init_arm_angle = 0;

    // 解析 xml
    aris::dynamic::MultiModel dualArm;
    aris::core::fromXmlFile(dualArm, ARIS_INSTALL_PATH + std::string("/resource/test_plan/dual_arm.xml"));

	auto& arm = dynamic_cast<aris::dynamic::Model&>(dualArm.subModels().at(arm_id));
	auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(arm.generalMotionPool().at(0));
	auto& arm_angle = dynamic_cast<aris::dynamic::Motion&>(arm.generalMotionPool().at(1));

	// 真机数据 //
	double joints1[JOINT_NUM]{ 149.982, -43.4585, -30.0078, -74.5814, 137.039, 49.1006, 55.3223 };
	double ee1[EE_DIM]{ 0.203427, -0.420417, 0.263479, 117.546, 42.114, 222.485 };

	double joints2[JOINT_NUM]{ 98.0283, -59.5074, -30.0078, -53.0331, 100.313, 68.8685, 2.34352 };
	double ee2[EE_DIM]{ -0.202153, -0.476769, 0.346417, 95.5752, -19.3522, 279.021 };

	// 仿真时，人为给定初始状态 //
	for (int i = 0; i < JOINT_NUM; i++) {
		joints1[i] *= pi / 180.0;
		joints2[i] *= pi / 180.0;
	}
	for (int i = 0; i < 3; i++) {
		ee1[i + 3] *= pi / 180.0;
		ee2[i + 3] *= pi / 180.0;
	}

	double arm_joint1[1]{ joints1[2] }; // 臂角等于第三个关节角 //
	double arm_joint2[1]{ joints2[2] };

	dualArm.init();

	arm.setInputPos(joints1);
	arm_angle.setP(arm_joint1);
	arm.forwardKinematics();

	double current_ee_pos[EE_DIM]{ 0.0 };
	ee.getMpe(current_ee_pos, "321");
	arm_angle.getP(&init_arm_angle);

	aris::plan::TrajectoryGenerator tg;
	aris::plan::InputSmoother sp;

	// 模拟多次执行指令 //
	for (int cmd_cnt = 0; cmd_cnt < cmd_cnt_max; cmd_cnt++) {
		// 两点间往返 //
		static bool target_flag = true;
		double target_ee[EE_DIM]{ 0.0 };
		if (target_flag) {
			target_flag = false;
			for (int i = 0; i < EE_DIM; i++) {
				target_ee[i] = ee2[i];
			}
		}
		else {
			target_flag = true;
			for (int i = 0; i < EE_DIM; i++) {
				target_ee[i] = ee1[i];
			}
		}

		// 设置 TrajectoryGenerator //
		static bool local_flag{ true };
		static int index = 1;
		const double vel_lim[2]{ vel, ome };
		const double acc_lim[2]{ acc, acc };
		const double jerk_lim[2]{ jerk, jerk };
		const double zone_lim[2]{ zone, zone };

		if (local_flag) {
			tg.setEeTypes({ aris::dynamic::EEType::PE321 });
			tg.setDt(DT);
			std::cout << "tg dt:" << tg.dt() << std::endl;
			ee.getMpe(current_ee_pos, "321");
			tg.insertLinePos(index++, current_ee_pos, vel_lim, acc_lim, jerk_lim, zone_lim);
		}
		tg.insertLinePos(index++, target_ee, vel_lim, acc_lim, jerk_lim, zone_lim);


		if (local_flag) {
			local_flag = false; // 前面也有用到，但只在这里翻转即可 //

			// 设置 InputSmoother //
			sp.setInputSize(JOINT_NUM);
			sp.setDt(DT);

			// 最大速度、加速度 //
			std::vector<double> max_vels(JOINT_NUM), min_vels(JOINT_NUM), max_accs(JOINT_NUM), min_accs(JOINT_NUM);
			for (int i = 0; i < JOINT_NUM; i++) {
				max_vels[i] = joint_vel;
				min_vels[i] = -joint_vel;
				max_accs[i] = joint_acc;
				min_accs[i] = -joint_acc;
			}
			sp.setMaxVel(aris::core::Matrix(JOINT_NUM, 1, max_vels.data()));
			sp.setMinVel(aris::core::Matrix(JOINT_NUM, 1, min_vels.data()));
			sp.setMaxAcc(aris::core::Matrix(JOINT_NUM, 1, max_accs.data()));
			sp.setMinAcc(aris::core::Matrix(JOINT_NUM, 1, min_accs.data()));
			sp.allocateMemory(); //这个函数一定要放在这个位置才行！

			// 设置反解 //
			sp.setInputGenerator([&ee, &arm, &arm_angle, &tg, &init_arm_angle](double* p)->std::int64_t {
				double output[6];
				auto ret = tg.getEePosAndMoveDt(output); // output 是pe321
				ee.setMpe(output, "321");

				static int count_{ 0 };
				if (count_++ < 20) {
					std::cout << "end pos:" << count_ << "\t";
					aris::dynamic::dsp(1, 6, output);
					std::cout << tg.currentDs() << std::endl;
				}


				arm_angle.setP(&init_arm_angle);

				if (arm.inverseKinematics()) {
					// std::cout << "++++++++ IK ERROR +++++++" << std::endl;
				};
				arm.getInputPos(p);
				return ret;
				});
		}

		// 数据输出文件初始化 //
		std::string filename = "C:/Mac/Home/Desktop/test_data/data" + std::to_string(cmd_cnt) + ".csv";
		std::ofstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error: Could not open motion_data.csv" << std::endl;
			return -1;
		}
		file.precision(15);
		file.setf(std::ios::fixed);


		// 模拟实时循环 //
		for (int step_cnt = 0; step_cnt < step_cnt_max; step_cnt++) {
			double joint_ref[JOINT_NUM]{ 0.0 };
			auto ret = sp.getNextInput(joint_ref);  //获取下一个关节路径点


			// 输出关节角度 //
			file << step_cnt << ",";
			for (int i = 0; i < JOINT_NUM; i++) {
				file << joint_ref[i] << ",";
			}
			file << std::endl;

			if (ret <= 0) {
				std::cout << "  CMD[" << cmd_cnt << "] " << "Finished at CNT[" << step_cnt << "], RET[" << ret << "]" << std::endl;
				break;
			}
		}

		file.close();
		tg.clearUsedPos();
		// tg.clearAllPos(); // 不能直接清除所有的点！
		std::cout << "========================================" << std::endl;
	}
	std::cout << "Test Finished. " << std::endl;
	return 0;
}

void test_input_smoother(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	//test_input_smoother_sin();
	//test_input_smoother_cos();
	//test_input_smoother_2();

	test_input_smoother_7axis();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}


