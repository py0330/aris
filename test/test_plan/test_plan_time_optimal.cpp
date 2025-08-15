#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

auto test_s_cpt_d3u_lr()->void {
	int p_size = 2;
	double p_min[]{ -1.5,-2.5 };
	double p_max[]{ 1.5,2.5 };
	double dp_min[]{ -4, -4 };
	double dp_max[]{ 3, 3  };
	double d2p_min[]{ -2,-2 };
	double d2p_max[]{ 3, 3 };
	double d3p_min[]{ -8.0, -25 };
	double d3p_max[]{ 10.0, 20.0 };

	double p0[]{ 0, 0 };
	double p1[]{ 0, 0 };
	double p2[]{ 0, 0 };
	double p3[]{ 0, 0 };

	double s_diff = 1e-2;
	double u0{ 0 }, u1{ 0.0099 }, u2{ 0.0198 };

	double d3u_ds3_3_L, d3u_ds3_3_R;

	s_cpt_d3u_lr(p_size, p0, p1, p2, p3,
		p_min, p_max, dp_min, dp_max,
		d2p_min, d2p_max, d3p_min, d3p_max,
		s_diff, u0, u1, u2, d3u_ds3_3_L, d3u_ds3_3_R);

	
	
	double du_ds_1 = (u1 - u0) / s_diff;
	double du_ds_2 = (u2 - u1) / s_diff;

	double d2u_ds2_2 = (du_ds_2 - du_ds_1) / s_diff;

	double d2u_ds2_3_L = d2u_ds2_2 + d3u_ds3_3_L * s_diff;
	double du_ds_3_L = du_ds_2 + d2u_ds2_3_L * s_diff;
	double u3_L = u2 + du_ds_3_L * s_diff;

	double d2u_ds2_3_R = d2u_ds2_2 + d3u_ds3_3_R * s_diff;
	double du_ds_3_R = du_ds_2 + d2u_ds2_3_R * s_diff;
	double u3_R = u2 + du_ds_3_R * s_diff;

	std::cout << "L:" << d3u_ds3_3_L << "    R:" << d3u_ds3_3_R << std::endl;
	std::cout << "u3R:" << u3_L << "    R:" << u3_R << std::endl;

	double s0 = 0;
	double s1 = 1 * s_diff;
	double s2 = 2 * s_diff;
	double s3 = 3 * s_diff;
	double M_L[4 * 4]{
		u0* u0* u0, u0* u0, u0, 1,
		u1* u1* u1, u1* u1, u1, 1,
		u2* u2* u2, u2* u2, u2, 1,
		u3_L* u3_L* u3_L, u3_L* u3_L, u3_L, 1,
	};

	double U[16];
	aris::Size p[4], rank;
	aris::dynamic::s_householder_up(4, 4, M_L, U, p, rank);

	double k[4];
	double u3 = 0.0099*3;
	u3 = (u3 + u2) / 2;

	double b[4]{s0,s1,s2,s3};
	aris::dynamic::s_householder_up_sov(4, 4, 1, rank, U, p, b, k);

	aris::dynamic::dsp(1, 4, k);
	
	std::cout << 3 * k[0] * u2 * u2 + 2 * k[1] * u2 + k[2] << std::endl;
	std::cout << 3 * k[0] * u3 * u3 + 2 * k[1] * u3 + k[2] << std::endl;


	double M_R[4 * 4]{
	u0 * u0 * u0, u0 * u0, u0, 1,
	u1 * u1 * u1, u1 * u1, u1, 1,
	u2 * u2 * u2, u2 * u2, u2, 1,
	u3_R * u3_R * u3_R, u3_R * u3_R, u3_R, 1,
	};
	double b2[4]{ s0,s1,s2,s3 };
	aris::dynamic::s_householder_up(4, 4, M_R, U, p, rank);
	aris::dynamic::s_householder_up_sov(4, 4, 1, rank, U, p, b2, k);

	aris::dynamic::dsp(1, 4, k);
	u3 = 0.0475;
	std::cout << k[0] * u3 * u3*u3 + k[1] * u3*u3 + k[2]*u3+k[3] << std::endl;
	std::cout << 3 * k[0] * u3 * u3 + 2 * k[1] * u3 + k[2] << std::endl;

}

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
	puma->getInputPos(input_init);
	//puma->setInputPos(input_init);
	//puma->forwardKinematics();
	//double pm[16];
	//puma->getOutputPos(pm);
	//aris::dynamic::dsp(1, 16, pm);

	// 设置 lookahead //
	aris::plan::InputSmoother sp;

	// 最大速度、加速度 //
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> max_jerks{ 314, 314, 314, 314, 314, 314 };

	// 设置模型等参数 //
	sp.setModel(*puma);
	sp.setTrajectoryGenerator(tg);
	sp.setVelLimits(max_vels.data());
	sp.setAccLimits(max_accs.data());
	sp.setJerkLimits(max_jerks.data());


	sp.init(input_init);


	//sp.lookAheadOneStep();


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
		
		//std::cout << "s:" << s << std::endl;

		//s += 0.001;
		m++;

		//if (m > 3000 && m < 6000)
		//	sp.setTargetDs(0.0);
		//else
		//	sp.setTargetDs(1.0);

		//if (m == 12793) {
		//	std::cout << "debug" << std::endl;
		//}


		vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		std::copy_n(input_pos, 6, vec.data() + (6 * EE_NUM + A_NUM) * (m - 1));
		
		//v_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		//a_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		
		//puma->setOutputPos(ee_pos);
		//puma->inverseKinematics();
		//puma->getInputPos(vec.data() + (6 * EE_NUM + A_NUM) * (m - 1));
	}

	aris::dynamic::dlmwrite(m, (6 * EE_NUM + A_NUM), vec.data(), "/Mac/Home/Documents/MATLAB/test/data.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), v_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\vpes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), a_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\apes.txt");

}



void test_time_optimal(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	double x[6]{ 0.0080000000000000002, 0.012000000000000000, 0.015999898477761662, 0.020093382747408947, 0.024474922860291064 };
	double s[6]{ 0.0000000000000000, 0.0010000000000000000, 0.002, 0.003, 0.004, 0.005};


	aris::Size range_num;
	double mem[20];
	aris::dynamic::s_interp_scurve_u5_range(x, s, -5, 5, -50, 50, -500, 500, range_num, mem);



	test_time_optimal_processor_1();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

