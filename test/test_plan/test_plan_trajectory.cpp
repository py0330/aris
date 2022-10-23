#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;



void test_trajectory(){
	std::cout << std::endl << "-----------------test trajectory---------------------" << std::endl;

	aris::plan::TrajectoryGenerator tg;

	const int PE_SIZE = 3;
	const int EE_NUM = 2;
	const int A_NUM = 2;

	//  INIT TG //
	tg.setEeTypes({ aris::dynamic::EEType::X,aris::dynamic::EEType::X,aris::dynamic::EEType::PQ, aris::dynamic::EEType::PQ, });
	double init_pe[EE_NUM * 6]{ 0,0,0,0,0,0, 0,0,0,0,0,0 };
	double init_pq[EE_NUM * 7 + A_NUM]{ 0,0 };
	double init_vel[EE_NUM*2 + A_NUM]{ 1,1,1,1,1,1 };
	for (int j = 0; j < EE_NUM; ++j) {
		aris::dynamic::s_pe2pq(init_pe + 6 * j, init_pq + 7 * j + A_NUM, "321");
	}
	tg.insertLinePos(1, init_pq, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[PE_SIZE][6* EE_NUM]{
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
		tg.insertLinePos(i + 10, pqs[i% PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
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

	
	std::cout << "-----------------test trajectory finished------------" << std::endl << std::endl;
}

