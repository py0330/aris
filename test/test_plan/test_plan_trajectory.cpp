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
		//if (m > 29970)
		//	std::cout << "debug" << std::endl;

		//if (m > 10000)
		//	tg.setTargetDs(0.1);
		//if (m > 15000)
		//	tg.setTargetDs(1.0);

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
	//double pes_raw[PE_SIZE][6 * EE_NUM]{
	//	{ -43.718148,501.616394,440.393091,252.502610,-0.005295,179.999803 },
	//	{  -1.912937,501.617792,440.392672,240.495922,-0.005124,179.999365},
	//	{ 41.743427,501.615302,440.391891,230.495996,-0.006445,179.999351},
	//	{ 75.256973,501.616184,440.386031,220.490224,-0.007444,179.999036},
	//	{109.685175,501.619073,440.382413,210.483438,-0.007086,179.998499},
	//	{ 134.201286,501.621105,440.382326,220.481454,-0.006547,179.997985 }
	//};

	double pes_raw[PE_SIZE][6 * EE_NUM]{
		{ -80.971772,501.599143,440.288021,252.579374,-1.845274,174.240407 },
		{  -80.971772,501.599143,440.288021,253.012596,-4.388165,166.036425},
		{ -80.971772,501.599143,440.288021,253.571205,-6.178004,160.041630},
		{ -80.971772,501.599143,440.288021,252.836917,-3.560856,168.715727},
		{-80.971772,501.599143,440.288021,252.635497,2.323645,187.466237},
		{ -80.971772,501.599143,440.288021,253.444152,5.802127,198.860889 }
	};

	//double pes_raw[PE_SIZE][6 * EE_NUM]{
	//	{ -80.971772,501.599143,440.288021,0,-0,0 },
	//	{  -80.971772,501.599143,440.288021,0,-0,10},
	//	{ -80.971772,501.599143,440.288021,0,-0,20},
	//	{ -80.971772,501.599143,440.288021,0,-0,30},
	//	{-80.971772,501.599143,440.288021,0,0,40},
	//	{ -80.971772,501.599143,440.288021,0,0,50 }
	//};

	// 构造规划器 //
	aris::plan::TrajectoryGenerator tg;
	tg.setEeTypes({ aris::dynamic::EEType::PE321 });
	
	// 构造数据 //
	double pes[PE_SIZE][6 * EE_NUM];
	double speed[EE_NUM * 2]{ 1.0, aris::PI };
	double acc[EE_NUM * 2]{ 2.5, 2.5 };
	double jerk[EE_NUM * 2]{ 10.0, 10 };
	double zone[EE_NUM * 2]{ 0.01, 10 * aris::PI / 180.0 };

	for (int i = 0; i < PE_SIZE; ++i) {
		aris::dynamic::s_vc(3, 0.001, pes_raw[i], pes[i]);
		aris::dynamic::s_vc(3, aris::PI / 180.0, pes_raw[i] + 3, pes[i] + 3);
	}

	// 运行得到数据，便于后续打印 //
	std::vector<double> vec, v_vec, a_vec;
	double out_pe[6];
	auto move_and_copy_data = [&]() ->int {
		auto ret = tg.getEePosAndMoveDt(out_pe);
		vec.resize(vec.size() + 6, 0.0);
		aris::dynamic::s_vc(6, out_pe, vec.data() + vec.size() - 6);
		return ret;
	};

	// 插入点位
	tg.insertLinePos(1, pes[0], speed, acc, jerk, zone);
	move_and_copy_data(); // 运行一次 //

	tg.insertLinePos(2, pes[1], speed, acc, jerk, zone);
	tg.insertLinePos(3, pes[2], speed, acc, jerk, zone);
	tg.insertLinePos(4, pes[3], speed, acc, jerk, zone);
	tg.insertLinePos(5, pes[4], speed, acc, jerk, zone);
	tg.insertLinePos(6, pes[5], speed, acc, jerk, zone);
	while (move_and_copy_data() <= 1) {
	
	
	}
		
		;
	while (move_and_copy_data() <= 3) {
	}

	tg.insertLinePos(7, pes[0], speed, acc, jerk, zone);
	while (move_and_copy_data() <= 3)
//		aris::dynamic::dsp(1, 6, out_pe)
		;

	tg.insertLinePos(8, pes[1], speed, acc, jerk, zone);
	tg.insertLinePos(9, pes[2], speed, acc, jerk, zone);

	tg.insertLinePos(10, pes[3], speed, acc, jerk, zone);

	while (move_and_copy_data() <= 4)
		;
	while (move_and_copy_data() <= 5)
		;
	while (move_and_copy_data() <= 6)
		;

	while (move_and_copy_data())
		;

	// 打印结果 //
	aris::dynamic::dlmwrite(vec.size()/6, 6, vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
}
auto test_trajectory_3()->void {
	aris::plan::TrajectoryGenerator tg;

	const int POINT_SIZE = 3;
	
	const int A_NUM = 2;
	const int XYZT_NUM = 1;
	const int PQ_NUM = 2;
	

	const int P_SIZE = PQ_NUM * 7 + 4 * XYZT_NUM + A_NUM;
	const int V_SIZE = PQ_NUM * 2 + 2 * XYZT_NUM + A_NUM;

	//  INIT TG //
	tg.setEeTypes({ 
		aris::dynamic::EEType::X,
		aris::dynamic::EEType::X,
		aris::dynamic::EEType::XYZT,
		aris::dynamic::EEType::PQ, 
		aris::dynamic::EEType::PQ,
		});
	double init_pe[PQ_NUM * 6]{ 0,0,0,0,0,0, 0,0,0,0,0,0 };
	double init_pq[P_SIZE]{ 0,0,0,0,0,0 };
	double init_vel[V_SIZE]{ 1,1,1,1,1,1,1,1 };
	for (int j = 0; j < PQ_NUM; ++j) {
		aris::dynamic::s_pe2pq(init_pe + 6 * j, init_pq + 7 * j + A_NUM + XYZT_NUM * 4, "321");
	}
	tg.insertLinePos(1, init_pq, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[POINT_SIZE][6 * PQ_NUM]{
		{ 0.10, 0.22, 0.35, 0.20, 0.23, 0.85,  -0.10, -0.22, -0.35, 0.20, 0.23, 0.85},
		{ 0.15, 0.28, 0.38, 0.23, 0.53, 1.85,  0.10, 0.3, 0.35, 0.20, 0.23, 0.85},
		{-0.25, 0.12, 0.49, 0.85, 0.13, 3.85,  0.10, 0.22, 0.35, 0.20, 0.23, 0.85},
	};
	double pqs[POINT_SIZE][P_SIZE]{
		{ 0.10, 0.22, 0.1, 0.2, 0.3, 0.4},
		{ 0.21, 0.08, 0.8, 0.3 ,0.3, 0.7},
		{-0.14, 0.12, 0.4, 0.4, 0.2, 0.6},
	};
	for (int i = 0; i < POINT_SIZE; ++i) {
		for (int j = 0; j < PQ_NUM; ++j) {
			aris::dynamic::s_pe2pq((double*)pes[i] + 6 * j, (double*)pqs[i] + 7 * j + A_NUM + XYZT_NUM*4, "321");
		}
	}

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[POINT_SIZE][V_SIZE]{
		{ 0.1, 0.1, 0.8, 0.6, 1.0, 1.0, 1.0, 1.0 },
		{ 0.1, 0.1, 0.8, 0.6, 1.0, 1.0, 1.0, 1.0 },
		{ 0.1, 0.1, 0.8, 0.6, 1.0, 1.0, 1.0, 1.0 },
	};
	double accs[POINT_SIZE][V_SIZE]{
		{ 0.1, 0.1, 5.0, 10.0, 5.0, 1.0, 5.0, 1.0 },
		{ 0.1, 0.1, 5.0, 10.0, 5.0, 5.0, 5.0, 5.0 },
		{ 0.1, 0.1, 5.0, 10.0, 5.0, 5.0, 5.0, 5.0 },
	};
	double jerks[POINT_SIZE][V_SIZE]{
		{ 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 },
		{ 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 },
		{ 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 },
	};
	double zones[POINT_SIZE][V_SIZE]{
		{ 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 },
		{ 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 },
		{ 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 },
	};

	for (int i = 0; i < POINT_SIZE; ++i) {
		tg.insertLinePos(i + 10, pqs[i % POINT_SIZE], vels[i % POINT_SIZE], accs[i % POINT_SIZE], jerks[i % POINT_SIZE], zones[i % POINT_SIZE]);
	}

	tg.insertInitPos(29, pqs[2]);

	double out_pe[P_SIZE];
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

	//for (int i = 0; i < 3 * POINT_SIZE; ++i) {
	//	tg.insertLinePos(i + 30, pqs[i % POINT_SIZE], vels[i % POINT_SIZE], accs[i % POINT_SIZE], jerks[i % POINT_SIZE], zones[i % POINT_SIZE]);
	//}



	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[P_SIZE]{}, out_acc[P_SIZE]{};
	while (tg.getEePosAndMoveDt(out_pe, out_vel, out_acc)) {
		//if (m > 29970)
		//	std::cout << "debug" << std::endl;

		//if (m > 10000)
		//	tg.setTargetDs(0.1);
		//if (m > 15000)
		//	tg.setTargetDs(1.0);

		m++;
		vec.resize(m * P_SIZE, 0.0);
		v_vec.resize(m * P_SIZE, 0.0);
		a_vec.resize(m * P_SIZE, 0.0);
		aris::dynamic::s_vc(P_SIZE, out_pe, vec.data() + (P_SIZE) * (m - 1));
		aris::dynamic::s_vc(P_SIZE, out_vel, v_vec.data() + (P_SIZE) * (m - 1));
		aris::dynamic::s_vc(P_SIZE, out_acc, a_vec.data() + (P_SIZE) * (m - 1));

		std::fill_n(out_vel, P_SIZE, 0.0);
		std::fill_n(out_acc, P_SIZE, 0.0);
	}

	aris::dynamic::dlmwrite(m, (P_SIZE), vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
	aris::dynamic::dlmwrite(m, (P_SIZE), v_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\vpes.txt");
	aris::dynamic::dlmwrite(m, (P_SIZE), a_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\apes.txt");

}
auto test_trajectory_4()->void {
	const int PE_SIZE = 28;
	const int EE_NUM = 1;

	std::cout << "-----------------test trajectory start---------------" << std::endl;

	int insert_pe_id = 0;
	std::vector<int> prepare_id, cur_exe_id, cur_exe_count, cmd_type;
	auto read_insert_id = [&]()
	{
		std::cout << std::endl << "-----------------read insert id start---------------------" << std::endl;
		std::fstream insert_file;


		prepare_id.resize(PE_SIZE); cur_exe_id.resize(PE_SIZE); cur_exe_count.resize(PE_SIZE); cmd_type.resize(PE_SIZE);
		insert_file.open("C:/Users/py033/Desktop/test_aris_plan/matlab/insert_id_data.txt", std::ios::in);
		if (!insert_file.is_open())
		{
			std::cout << "打开失败" << std::endl;
		}
		else
		{
			std::string a, b, c, d;
			insert_file >> a >> b >> c >> d;
			std::cout << a << "\t" << b << "\t" << c << "\t" << d << std::endl;
			for (size_t i = 0; i < PE_SIZE; i++)
			{
				insert_file >> prepare_id[i] >> cur_exe_id[i] >> cur_exe_count[i] >> cmd_type[i];
				std::cout << prepare_id[i] << "\t" << cur_exe_id[i] << "\t" << cur_exe_count[i] << "\t" << cmd_type[i] << std::endl;
			}
		}

		std::cout << std::endl << "-----------------read insert id finish---------------------" << std::endl;
	};

	read_insert_id();


	// 目标点
	double pes_raw[PE_SIZE][6 * EE_NUM]{
		{-28.557000,18.248000,-14.542000,82.251000,48.042000,3.974000},
		{-28.641000,18.030000,-13.328000,83.013000,44.276000,4.639000},
		{-28.752000,17.746000,-12.039000,83.797000,40.287000,5.290000},
		{-28.818000,17.541000,-10.604000,84.384000,35.789000,5.922000},
		{-28.922000,17.176000,-9.152000,85.023000,31.255000,6.534000},
		{-28.984000,16.776000,-7.686000,85.466000,26.683000,7.140000},
		{-29.001000,16.225000,-6.501000,85.697000,23.161000,7.731000},
		{-29.070000,15.411000,-5.597000,86.073000,20.747000,8.348000},
		{-29.146000,14.578000,-4.558000,86.495000,17.774000,9.034000},
		{-29.254000,13.599000,-3.538000,87.064000,14.883000,9.760000},
		{-29.356000,12.495000,-2.797000,87.631000,13.176000,10.487000},
		{-29.371000,11.553000,-2.048000,87.831000,11.214000,11.182000},
		{-29.340000,10.693000,-1.318000,87.795000,9.014000,11.809000},
		{-29.298000,9.823000,-0.728000,87.682000,7.291000,12.374000},
		{-29.231000,9.008000,-0.243000,87.406000,5.969000,12.902000},
		{-29.184000,8.114000,0.144000,87.246000,5.021000,13.373000},
		{-29.154000,7.167000,0.467000,87.167000,4.235000,13.787000},
		{-29.152000,6.090000,0.772000,87.330000,3.484000,14.200000},
		{-29.143000,5.264000,1.021000,87.420000,2.915000,14.563000},
		{-29.110000,4.584000,1.209000,87.215000,2.450000,14.832000},
		{-29.106000,3.587000,1.285000,87.215000,2.450000,14.832000},
		{-29.096000,2.604000,1.461000,87.215000,2.450000,14.832000},
		{-29.079000,1.642000,1.733000,87.215000,2.450000,14.832000},
		{-29.055000,0.713000,2.100000,87.215000,2.450000,14.832000},
		{-29.025000,-0.175000,2.557000,87.215000,2.450000,14.832000},
		{-28.988000,-1.012000,3.101000,87.215000,2.450000,14.832000},
		{-28.949000,-1.737000,3.678000,87.215000,2.450000,14.832000},
		{-28.906000,-2.406000,4.319000,87.215000,2.450000,14.832000}
	};
	double pes[PE_SIZE][6 * EE_NUM];
	for (int i = 0; i < PE_SIZE; ++i) {
		aris::dynamic::s_vc(3, 0.001, pes_raw[i], pes[i]);
		aris::dynamic::s_vc(3, aris::PI / 180.0, pes_raw[i] + 3, pes[i] + 3);
	}

	// 构造规划器 //
	aris::plan::TrajectoryGenerator tg;
	tg.setEeTypes({ aris::dynamic::EEType::PE321 });



	// 构造数据 //
	double speed[EE_NUM * 2]{ 0.1, aris::PI };
	double acc[EE_NUM * 2]{ 2 * 2.5, aris::PI * 2 * 2.5 };
	double jerk[EE_NUM * 2]{ 10.0 * acc[0], 10 * acc[1] };
	double zone[EE_NUM * 2]{ 0.02, 0.02 };

	// 显示数据
	std::cout << "vel:" << speed[0] << "\t" << speed[1] << std::endl;
	std::cout << "acc:" << acc[0] << "\t" << acc[1] << std::endl;
	std::cout << "jerk:" << jerk[0] << "\t" << jerk[1] << std::endl;
	std::cout << "zone:" << zone[0] << "\t" << zone[1] << std::endl;
	for (size_t i = 0; i < PE_SIZE; i++)
	{
		aris::dynamic::dsp(1, 6, pes[i]);
	}


	// 运行得到数据，便于后续打印 //
	std::vector<double> vec;
	double out_pe[6];
	int cmd_id{ 0 }, cmd_count{ 0 }, last_ret{ 0 }, ret{ 0 };


	// 回调函数
	auto move_and_copy_data = [&]() ->int
	{

		ret = tg.getEePosAndMoveDt(out_pe);

		if (ret != last_ret) // 执行完一条，count清零
		{
			cmd_count = 0;
		}

		last_ret = ret;
		cmd_count++;

		static int total_count = 0;
		total_count++;

		if (total_count > 3230)
			std::cout << "debug" << std::endl;

		if (insert_pe_id < PE_SIZE)
		{

			if (ret == cur_exe_id[insert_pe_id] && cmd_count == cur_exe_count[insert_pe_id])
			{
				if (cmd_id >= 10)
					std::cout << "debug" << std::endl;
				cmd_id++;
				tg.insertLinePos(cmd_id, pes[insert_pe_id], speed, acc, jerk, zone);
				std::cout << "cmd_id=" << cmd_id << "\t" << "insert_pe_id=" << insert_pe_id << "\t" << "cur_exe_id=" << cur_exe_id[insert_pe_id] << "\t" << "count=" << cmd_count << std::endl;
				insert_pe_id++;
			}
		}


		vec.resize(vec.size() + 6, 0.0);
		aris::dynamic::s_vc(6, out_pe, vec.data() + vec.size() - 6);

		return ret;
	};

	{
		// 插入初始点位
		double init_pe[6];
		double init_pes_raw[6] = { -29.091000, 17.520000, -15.596000, 83.478000 ,51.402000, 3.401000 };

		aris::dynamic::s_vc(3, 0.001, init_pes_raw, init_pe);
		aris::dynamic::s_vc(3, aris::PI / 180.0, init_pes_raw + 3, init_pe + 3);
		tg.insertInitPos(10000, init_pe);

		// 插入第一个目标点
		tg.setTargetDs(30 / 100.0);
		cmd_id = 1;
		tg.insertLinePos(cmd_id, pes[insert_pe_id], speed, acc, jerk, zone);
		insert_pe_id++;

		while (move_and_copy_data()) {}
	}

	// 打印结果 //
	aris::dynamic::dlmwrite(vec.size() / 6, 6, vec.data(), "C:/Users/py033/Desktop/test_aris_plan/matlab/pes.txt");


	std::cout << "-----------------test trajectory finished------------" << std::endl;
}

void test_trajectory(){
	std::cout << std::endl << "-----------------test trajectory---------------------" << std::endl;
	
	test_trajectory_1();
	test_trajectory_2();
	test_trajectory_3();
	test_trajectory_4();
	
	std::cout << "-----------------test trajectory finished------------" << std::endl << std::endl;
}

