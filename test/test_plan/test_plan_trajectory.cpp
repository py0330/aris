#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

auto test_trajectory_1()->void {
	aris::plan::TrajectoryGenerator tg;

	const int PE_SIZE = 3;
	const int EE_NUM = 2;
	const int A_NUM = 2;

	//  INIT TG //
	tg.setEeTypes({ aris::dynamic::EEType::X,aris::dynamic::EEType::X,aris::dynamic::EEType::PQ, aris::dynamic::EEType::PQ, });
	double init_pe[EE_NUM * 6]{ 0,0,0,0,0,0, 0,0,0,0,0,0 };
	double init_pq[EE_NUM * 7 + A_NUM]{ 0,0 };
	double init_vel[EE_NUM * 2 + A_NUM]{ 1,1,1,1,1,1 };
	for (int j = 0; j < EE_NUM; ++j) {
		aris::dynamic::s_pe2pq(init_pe + 6 * j, init_pq + 7 * j + A_NUM, "321");
	}
	tg.insertLinePos(1, init_pq, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[PE_SIZE][6 * EE_NUM]{
		{ 0.10, 0.22, 0.35, 0.20, 0.23, 0.85,  -0.10, -0.22, -0.35, 0.20, 0.23, 0.85},
		{ 0.15, 0.28, 0.38, 0.23, 0.53, 1.85,  0.10, 0.3, 0.35, 0.20, 0.23, 0.85},
		{-0.25, 0.12, 0.49, 0.85, 0.13, 3.85,  0.10, 0.22, 0.35, 0.20, 0.23, 0.85},
	};
	double pqs[PE_SIZE][7 * EE_NUM + A_NUM]{
		{ 0.10, 0.22},
		{ 0.21, 0.08},
		{-0.14, 0.12},
	};
	for (int i = 0; i < PE_SIZE; ++i) {
		for (int j = 0; j < EE_NUM; ++j) {
			aris::dynamic::s_pe2pq((double*)pes[i] + 6 * j, (double*)pqs[i] + 7 * j + A_NUM, "321");
		}
	}

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.1, 0.1, 1.0, 1.0, 1.0, 1.0 },
		{ 0.1, 0.1, 1.0, 1.0, 1.0, 1.0 },
		{ 0.1, 0.1, 1.0, 1.0, 1.0, 1.0 },
	};
	double accs[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.1, 0.1, 5.0, 1.0, 5.0, 1.0 },
		{ 0.1, 0.1, 5.0, 5.0, 5.0, 5.0 },
		{ 0.1, 0.1, 5.0, 5.0, 5.0, 5.0 },
	};
	double jerks[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 1.0, 10.0, 10.0, 10.0, 10.0, 10.0 },
		{ 1.0, 10.0, 10.0, 10.0, 10.0, 10.0 },
		{ 1.0, 10.0, 10.0, 10.0, 10.0, 10.0 },
	};
	double zones[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 },
		{ 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 },
		{ 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 },
	};

	for (int i = 0; i < PE_SIZE; ++i) {
		tg.insertLinePos(i + 10, pqs[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}

	tg.insertInitPos(29, pqs[2]);

	double out_pe[7 * EE_NUM + A_NUM];
	while (tg.getEePosAndMoveDt(out_pe));

	tg.insertInitPos(30, pqs[0]);
	tg.insertCirclePos(50, pqs[0], pqs[1], vels[0], accs[0], jerks[0], zones[0]);
	tg.insertLinePos(100, init_pq, init_vel, init_vel, init_vel, init_vel);
	tg.insertCirclePos(101, pqs[0], pqs[1], vels[0], accs[0], jerks[0], zones[0]);
	tg.insertCirclePos(102, pqs[2], pqs[2], vels[0], accs[0], jerks[0], zones[0]);
	tg.insertLinePos(103, pqs[0], vels[0], accs[0], jerks[0], zones[0]);
	tg.insertLinePos(104, pqs[1], vels[0], accs[0], jerks[0], zones[0]);
	tg.insertLinePos(105, pqs[2], vels[0], accs[0], jerks[0], zones[0]);
	//tg.insertInitPos(31, pqs[2]);
	//tg.insertCirclePos(51, pqs[0], pqs[1], vels[0], accs[0], jerks[0], init_pe);

	//double pes2[2][6 * EE_NUM]{
	//	{ 0.10, 0.22, 0.35, 0.20, 0.23, 1.85,  -0.10, -0.22, -0.35, 0.20, 0.23, 0.85},
	//	{ 0.10, 0.22, 0.35, 0.20, 0.23, 3.85,  -0.10, -0.22, -0.35, 0.20, 0.23, 0.85},
	//};
	//double pqs2[2][7 * EE_NUM + A_NUM]{
	//	{ 0.10, 0.22},
	//	{ 0.21, 0.08},
	//};
	//for (int i = 0; i < 2; ++i) {
	//	for (int j = 0; j < EE_NUM; ++j) {
	//		aris::dynamic::s_pe2pq((double*)pes2[i] + 6 * j, (double*)pqs2[i] + 7 * j + A_NUM, "321");
	//	}
	//}
	//tg.insertLinePos(103, pqs2[0], vels[0], accs[0], jerks[0], zones[0]);
	//tg.insertLinePos(104, pqs2[1], vels[0], accs[0], jerks[0], zones[0]);

	//tg.insertCirclePos(100, init_pq, init_vel, init_vel, init_vel, init_vel);

	//for (int i = 0; i < 3 * PE_SIZE; ++i) {
	//	tg.insertLinePos(i + 30, pqs[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	//}



	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{};
	while (tg.getEePosAndMoveDt(out_pe, out_vel, out_acc)) {
		//if (m > 2597)
		//	std::cout << "debug" << std::endl;

		if (m > 10000)
			tg.setTargetDs(0.1);
		if (m > 15000)
			tg.setTargetDs(1.0);

		m++;
		vec.resize(m * (7 * EE_NUM + A_NUM), 0.0);
		v_vec.resize(m * (7 * EE_NUM + A_NUM), 0.0);
		a_vec.resize(m * (7 * EE_NUM + A_NUM), 0.0);
		aris::dynamic::s_vc((7 * EE_NUM + A_NUM), out_pe, vec.data() + (7 * EE_NUM + A_NUM) * (m - 1));
		aris::dynamic::s_vc((7 * EE_NUM + A_NUM), out_vel, v_vec.data() + (7 * EE_NUM + A_NUM) * (m - 1));
		aris::dynamic::s_vc((7 * EE_NUM + A_NUM), out_acc, a_vec.data() + (7 * EE_NUM + A_NUM) * (m - 1));

		std::fill_n(out_vel, 16, 0.0);
		std::fill_n(out_acc, 16, 0.0);
	}

	aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
	aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), v_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\vpes.txt");
	aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), a_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\apes.txt");

}
auto test_trajectory_2()->void {
	aris::plan::TrajectoryGenerator tg;

	const int PE_SIZE = 6;
	const int EE_NUM = 1;


	//RobotTarget p12 = { -43.718148,501.616394,440.393091,252.502610,-0.005295,179.999803,0,0 }
	//RobotTarget p3 = { -1.912937,501.617792,440.392672,252.495922,-0.005124,179.999365,0,0 }
	//RobotTarget p14 = {41.743427,501.615302,440.391891,252.495996,-0.006445,179.999351,0,0}
	//RobotTarget p15 = { 75.256973,501.616184,440.386031,252.490224,-0.007444,179.999036,0,0 }
	//RobotTarget p16 = { 109.685175,501.619073,440.382413,252.483438,-0.007086,179.998499,0,0 }
	//RobotTarget p17 = { 134.201286,501.621105,440.382326,252.481454,-0.006547,179.997985,0,0 }
	//Speed v100 = {10,1000.000000,180.000000}
	//Zone z10 = {10.000000,10.000000,0.000000}

	//  INIT TG //
	//double init_pe_raw[EE_NUM * 6]{ -43.718148,501.616394,440.393091,252.502610,-0.005295,179.999803 };
	double pes_raw[PE_SIZE][6 * EE_NUM]{
		{ -43.718148,501.616394,440.393091,252.502610,-0.005295,179.999803 },
		{  -1.912937,501.617792,440.392672,240.495922,-0.005124,179.999365},
		{ 41.743427,501.615302,440.391891,230.495996,-0.006445,179.999351},
		{ 75.256973,501.616184,440.386031,220.490224,-0.007444,179.999036},
		{109.685175,501.619073,440.382413,210.483438,-0.007086,179.998499},
		{ 134.201286,501.621105,440.382326,220.481454,-0.006547,179.997985 }
	};
	double pes[PE_SIZE][6 * EE_NUM];
	double speed[EE_NUM * 2]{ 1.0, aris::PI };
	double acc[EE_NUM * 2]{ 2.5, 0.2 };
	double jerk[EE_NUM * 2]{ 10.0, 10};
	double zone[EE_NUM * 2]{ 0.01, 10* aris::PI / 180.0 };

	for (int i = 0; i < PE_SIZE; ++i) {
		aris::dynamic::s_vc(3, 0.001, pes_raw[i], pes[i]);
		aris::dynamic::s_vc(3, aris::PI/180.0, pes_raw[i] + 3, pes[i] + 3);
	}

	std::vector<double> vec, v_vec, a_vec;
	double out_pe[6];
	auto move_and_copy_data = [&]() ->int {
		auto ret = tg.getEePosAndMoveDt(out_pe);
		vec.resize(vec.size() + 6, 0.0);
		aris::dynamic::s_vc(6, out_pe, vec.data() + vec.size() - 6);
		return ret;
	};



	tg.setEeTypes({ aris::dynamic::EEType::PE321 });
	
	
	

	tg.insertLinePos(1, pes[0], speed, acc, jerk, zone);
	//tg.getEePosAndMoveDt(out_pe);
	move_and_copy_data();

	tg.insertLinePos(2, pes[1], speed, acc, jerk, zone);
	tg.insertLinePos(3, pes[2], speed, acc, jerk, zone);
	tg.insertLinePos(4, pes[3], speed, acc, jerk, zone);
	tg.insertLinePos(5, pes[4], speed, acc, jerk, zone);
	tg.insertLinePos(6, pes[5], speed, acc, jerk, zone);
	while (move_and_copy_data() <= 1)
//		aris::dynamic::dsp(1, 6, out_pe)
		;
	while (move_and_copy_data() <= 2)
//		aris::dynamic::dsp(1, 6, out_pe)
		;

	tg.insertLinePos(7, pes[0], speed, acc, jerk, zone);
	while (move_and_copy_data() <= 3)
//		aris::dynamic::dsp(1, 6, out_pe)
		;

	tg.insertLinePos(8, pes[1], speed, acc, jerk, zone);
	tg.insertLinePos(9, pes[2], speed, acc, jerk, zone);

	tg.insertLinePos(10, pes[3], speed, acc, jerk, zone);

	while (move_and_copy_data() <= 4)
//		aris::dynamic::dsp(1, 6, out_pe)
		;
	while (move_and_copy_data() <= 5)
//		aris::dynamic::dsp(1, 6, out_pe)
		;
	while (move_and_copy_data() <= 6)
//		aris::dynamic::dsp(1, 6, out_pe)
		;

	while (move_and_copy_data());

	aris::dynamic::dlmwrite(vec.size()/6, 6, vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
}
void test_trajectory(){
	std::cout << std::endl << "-----------------test trajectory---------------------" << std::endl;

	test_trajectory_2();
	test_trajectory_1();
	
	std::cout << "-----------------test trajectory finished------------" << std::endl << std::endl;
}

