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

	//  INIT TG //
	tg.setEeTypes({ aris::dynamic::EEType::PQ, aris::dynamic::EEType::PQ });
	double init_pe[EE_NUM * 6]{ 0,0,0,0,0,0, 0,0,0,0,0,0 };
	double init_pq[EE_NUM * 7];
	double init_vel[4]{ 0,0,0,0 };
	for (int j = 0; j < EE_NUM; ++j) {
		aris::dynamic::s_pe2pq(init_pe + 7 * j, init_pq + 7 * j, "321");
	}
	tg.insertPos(1, aris::plan::TrajectoryGenerator::Node::MoveType::ResetInitPos, init_pq, init_pq, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[PE_SIZE][6* EE_NUM]{
		{ 0.10, 0.22, 0.35, 0.20, 0.23, 0.85,  -0.10, -0.22, -0.35, 0.20, 0.23, 0.85},
		{ 0.15, 0.28, 0.38, 0.23, 0.53, 1.85,  0.10, 0.22, 0.35, 0.20, 0.23, 0.85},
		{-0.25, 0.12, 0.49, 0.85, 0.13, 3.85,  0.10, 0.22, 0.35, 0.20, 0.23, 0.85},
	};
	double pqs[PE_SIZE][7 * EE_NUM];
	for (int i = 0; i < PE_SIZE; ++i) {
		for (int j = 0; j < EE_NUM; ++j) {
			aris::dynamic::s_pe2pq((double*)pes[i] + 6 * j, (double*)pqs[i] + 7 * j, "321");
		}
	}

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[PE_SIZE][2 * EE_NUM]{
		{ 1.0, 1.0, 1.0, 1.0 },
		{ 1.0, 1.0, 1.0, 1.0 },
		{ 1.0, 1.0, 1.0, 1.0 },
	};
	double accs[PE_SIZE][2 * EE_NUM]{
		{ 5.0, 5.0, 5.0, 5.0 },
		{ 5.0, 5.0, 5.0, 5.0 },
		{ 5.0, 5.0, 5.0, 5.0 },
	};
	double jerks[PE_SIZE][2 * EE_NUM]{
		{ 10.0, 10.0, 10.0, 10.0 },
		{ 10.0, 10.0, 10.0, 10.0 },
		{ 10.0, 10.0, 10.0, 10.0 },
	};
	double zones[PE_SIZE][2 * EE_NUM]{
		{ 0.2, 0.2, 0.2, 0.2 },
		{ 0.2, 0.2, 0.2, 0.2 },
		{ 0.2, 0.2, 0.2, 0.2 },
	};


	
	
	for (int i = 0; i < 3*PE_SIZE; ++i) {
		tg.insertPos(i + 10, aris::plan::TrajectoryGenerator::Node::MoveType::Line, 
			pqs[i% PE_SIZE], pqs[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}

	tg.insertPos(1, aris::plan::TrajectoryGenerator::Node::MoveType::ResetInitPos, init_pq, init_pq, init_vel, init_vel, init_vel, init_vel);


	for (int i = 0; i < 3 * PE_SIZE; ++i) {
		tg.insertPos(i + 30, aris::plan::TrajectoryGenerator::Node::MoveType::Line,
			pqs[i % PE_SIZE], pqs[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}

	double out_pe[14];
	
	std::vector<double> vec;
	int m = 0;
	while (tg.getEePosAndMoveDt(out_pe)) {
		m++;
		vec.resize(m * 14, 0.0);
		aris::dynamic::s_vc(14, out_pe, vec.data() + 14 * (m - 1));
	}
	
	aris::dynamic::dlmwrite(m, 14, vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");



	
	std::cout << "-----------------test trajectory finished------------" << std::endl << std::endl;
}

