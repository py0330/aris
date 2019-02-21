#include <iostream>
#include <aris.h>

using namespace aris::dynamic;
using namespace aris::robot;

const double PI = 3.14159265358979;

Model rbt;

void build_model() 
{
	// 设置重力 //
	const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
	rbt.environment().setGravity(gravity);

	// add parts //
	// iv : inertia vector : m cx cy cz Ixx Iyy Izz Ixy Ixz Iyz
	// pm : 4x4 pose matrix
	// pe : 6x1 position and euler angle
	double iv[10], pm[16];

	const double p1_pe[6]{ 0.0, 0.00193, 0.089159 - 0.02561, 0, 0, 0 };
	const double p1_iv[10]{ 3.7, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p1_pe, pm, "321");
	s_iv2iv(pm, p1_iv, iv);
	auto &p1 = rbt.partPool().add<Part>("L1", iv);

	const double p2_pe[6]{ 0.2125, -0.024201 + 0.13585, 0.089159, 0.0, 0.0, 0.0 };
	const double p2_iv[10]{ 8.393, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p2_pe, pm, "321");
	s_iv2iv(pm, p2_iv, iv);
	auto &p2 = rbt.partPool().add<Part>("L2", iv);

	const double p3_pe[6]{ 0.425 + 0.110949, 0.13585 - 0.1197, 0.089159 + 0.01634, 0, 0, 0 };
	const double p3_iv[10]{ 2.275, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p3_pe, pm, "321");
	s_iv2iv(pm, p3_iv, iv);
	auto &p3 = rbt.partPool().add<Part>("L3", iv);

	const double p4_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197, 0.089159, 0, 0, 0 };
	const double p4_iv[10]{ 1.219, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p4_pe, pm, "321");
	s_iv2iv(pm, p4_iv, iv);
	auto &p4 = rbt.partPool().add<Part>("L4", iv);

	const double p5_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159, 0, 0, 0 };
	const double p5_iv[10]{ 1.219, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p5_pe, pm, "321");
	s_iv2iv(pm, p5_iv, iv);
	auto &p5 = rbt.partPool().add<Part>("L5", iv);

	const double p6_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, 0, 0 };
	const double p6_iv[10]{ 0.1879, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p6_pe, pm, "321");
	s_iv2iv(pm, p6_iv, iv);
	auto &p6 = rbt.partPool().add<Part>("L6", iv);

	// add joints //
	const double j1_pos[6]{ 0.0, 0.0, 0.089159 }, j1_axis[6]{ 0.0, 0.0, 1.0 };
	const double j2_pos[6]{ 0.0, 0.13585, 0.089159 }, j2_axis[6]{ 0.0, 1.0, 0.0 };
	const double j3_pos[6]{ 0.425, 0.13585 - 0.1197, 0.089159 }, j3_axis[6]{ 0.0, 1.0, 0.0 };
	const double j4_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197, 0.089159 }, j4_axis[6]{ 0.0, 1.0, 0.0 };
	const double j5_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 }, j5_axis[6]{ 0.0, 0.0, 1.0 };
	const double j6_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465 }, j6_axis[6]{ 0.0, 1.0, 0.0 };

	auto &j1 = rbt.addRevoluteJoint(p1, rbt.ground(), j1_pos, j1_axis);
	auto &j2 = rbt.addRevoluteJoint(p2, p1, j2_pos, j2_axis);
	auto &j3 = rbt.addRevoluteJoint(p3, p2, j3_pos, j3_axis);
	auto &j4 = rbt.addRevoluteJoint(p4, p3, j4_pos, j4_axis);
	auto &j5 = rbt.addRevoluteJoint(p5, p4, j5_pos, j5_axis);
	auto &j6 = rbt.addRevoluteJoint(p6, p5, j6_pos, j6_axis);

	// add actuation //
	auto &m1 = rbt.addMotion(j1);
	auto &m2 = rbt.addMotion(j2);
	auto &m3 = rbt.addMotion(j3);
	auto &m4 = rbt.addMotion(j4);
	auto &m5 = rbt.addMotion(j5);
	auto &m6 = rbt.addMotion(j6);

	// add end effector //
	double ee_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093 + 0.0823, 0.089159 - 0.09465, PI, PI / 2, 0 };
	rbt.addGeneralMotionByPe(p6, rbt.ground(), ee_pe, "321");

	// add solvers //
	auto &forward_kinematic = rbt.solverPool().add<ForwardKinematicSolver>();
	auto &inverse_kinematic = rbt.solverPool().add<Ur5InverseKinematicSolver>();
	auto &inverse_dynamic = rbt.solverPool().add<InverseDynamicSolver>();

	forward_kinematic.allocateMemory();
	inverse_kinematic.allocateMemory();
	inverse_dynamic.allocateMemory();
}

int main()
{
	//std::cout << std::endl << "-----------------test model ur---------------------" << std::endl;
	
	/*try
	{
		build_model();
		
		// 定时器 //
		auto begin_time = std::chrono::high_resolution_clock::now();

		// 动力学计算轨迹 //
		double actuation_force[51][6];
		for (int i = 0; i <= 50; ++i)
		{
			double t = i*0.1;
			
			double q[6]{ 0.1 + 0.1*sin(t),0.2 + 0.2*sin(t * 2),0.3 + 0.3*sin(t * 3),0.4 + 0.4*sin(t * 4),0.5 + 0.5*sin(t * 5),0.6 + 0.6*sin(t * 6) };
			double dq[6]{ 0.1*cos(t), 0.2 * 2 * cos(t * 2), 0.3 * 3 * cos(t * 3), 0.4 * 4 * cos(t * 4), 0.5 * 5 * cos(t * 5), 0.6 * 6 * cos(t * 6) };
			double ddq[6]{ -0.1*sin(t), -0.2 * 2 * 2 * sin(t * 2), -0.3 * 3 * 3 * sin(t * 3), -0.4 * 4 * 4 * sin(t * 4), -0.5 * 5 * 5 * sin(t * 5), -0.6 * 6 * 6 * sin(t * 6) };

			// 分别设置电机的位置、速度、加速度， mp的意思为motion position...
			for (aris::Size i = 0; i < 6; ++i)
			{
				rbt.motionPool().at(i).setMp(q[i]);
				rbt.motionPool().at(i).setMv(dq[i]);
				rbt.motionPool().at(i).setMa(ddq[i]);
			}

			// 分别用正解求解器求解, 0号求解器是运动学正解通用求解器，1号为我们重载的反解求解器，2号为逆动力学求解器
			rbt.solverPool().at(0).kinPos();
			rbt.solverPool().at(0).kinVel();
			rbt.solverPool().at(2).dynAccAndFce();

			for (aris::Size j = 0; j < 6; ++j)
			{
				actuation_force[i][j] = rbt.motionPool().at(j).mf();
			}
		}

		auto time_elapsed = std::chrono::high_resolution_clock::now() - begin_time;

		// 打印结果 //
		for (int i = 0; i <= 50; ++i)
		{
			for (aris::Size j = 0; j < 6; ++j)
			{
				std::cout << std::setfill(' ') << std::setw(15) << actuation_force[i][j] << std::setw(15);
			}
			std::cout << std::endl;
		}

		// 我的机器大概需要0.05 ms/cycle, 但是事实上位置正解花了0.04ms，这是因为默认的求解器是使用迭代来求的，可以像重载反解求解器一样重载它 //
		std::cout << "time elapsed " << time_elapsed.count() <<"(ns) for 51 cycles, each cycle consumes "<< time_elapsed.count()/51.0<<"(ns)" << std::endl;



		// 以下展示使用反解求解器求解的过程
		// 根据位置欧拉角来计算反解
		double ee_pe[6]{ 0.6,0.4,0.2,0.1,0.2,0.3 };
		rbt.generalMotionPool().at(0).setMpe(ee_pe, "321");


		rbt.solverPool().at(1).kinPos();

		std::cout << "inverse kinematic result:" << std::endl;
		for (aris::Size j = 0; j < 6; ++j)
		{
			std::cout << std::setfill(' ') << std::setw(15) << rbt.motionPool().at(j).mp() << std::setw(15);
		}
		std::cout << std::endl;


		// 根据位置四元数来计算反解
		double ee_pq[7]{ 0.6,0.4,0.2,0.1,0.2,0.3,sqrt(1-0.1*0.1-0.2*0.2-0.3*0.3) };
		rbt.generalMotionPool().at(0).setMpq(ee_pq);

		rbt.solverPool().at(1).kinPos();

		std::cout << "inverse kinematic result:" << std::endl;
		for (aris::Size j = 0; j < 6; ++j)
		{
			std::cout << std::setfill(' ') << std::setw(15) << rbt.motionPool().at(j).mp() << std::setw(15);
		}
		std::cout << std::endl;
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
	*/

	
	auto r = aris::robot::createModelUr5();

	r->saveXmlFile("C:\\Users\\py033\\Desktop\\ur5.xml");

	for (auto &p : r->partPool())
	{
		dsp(4, 4, *dynamic_cast<FileGeometry&>(p.geometryPool().front()).prtPm());
	}
	

	//-0.99500416527803   0.00000000000000   0.09983341664683   0.83228026169050
	//	0.09983341664683   0.00000000000000   0.99500416527803   0.10890468768786
	//	0.00000000000000   1.00000000000000 - 0.00000000000000 - 0.00549100000000
	//	0.00000000000000   0.00000000000000   0.00000000000000   1.00000000000000

	// add solvers //
	auto &forward_kinematic = r->solverPool().add<ForwardKinematicSolver>();
	auto &inverse_kinematic = r->solverPool().add<aris::dynamic::Ur5InverseKinematicSolver>();

	auto &sim = r->simulatorPool().add<aris::dynamic::Simulator>("sim");
	auto &result = r->simResultPool().add<aris::dynamic::SimResult>("result");

	auto &ee = r->generalMotionPool().at(0);

	result.allocateMemory();
	inverse_kinematic.allocateMemory();
	forward_kinematic.allocateMemory();

	ee.setMpe(std::array<double, 6>{0.7, 0.1, 0.2, 0, PI + 0.2 , -PI / 2.0 + 0.1}.data(), "321");
	inverse_kinematic.kinPos();

	std::cout << std::setprecision(16);
	dsp(4, 4, *ee.mpm());

	for (auto &m : r->motionPool()) 
	{
		m.updMp();
		std::cout << m.mp() << std::endl;
	}




	double pin[6]{ -0.22007211565796,
		- 1.39951770491866,
		2.01242257417054,
		- 2.18370119604678,
		-1.57079632679490,
		- 1.79086844245286 };


	for (aris::Size i = 0; i<6;++i)
	{
		r->motionPool().at(i).setMp(pin[i]);
	}
	forward_kinematic.kinPos();
	ee.updMpm();
	dsp(4, 4, *ee.mpm());



	sim.simulate([&](const PlanParam &param)->int
	{
		param.model_->setTime(param.count_*0.001);

		aris::Size t0;
		double p, v, a;

		aris::Size c = param.count_;

		aris::plan::moveAbsolute(c, 0.0, 1.0, 1 / 1e3, 10.0 / 1e6, 10.0 / 1e6, p, v, a, t0);

		if (c < t0)
		{
			aris::plan::moveAbsolute(c, 0.0, 1.0, 1 / 1e3, 10.0 / 1e6, 10.0 / 1e6, p, v, a, t0);
			param.model_->generalMotionPool().at(0).setMpe(std::array<double, 6>{0.5 + 0.2*p, 0, 0.35, 0, 0, PI}.data(), "321");
		}

		inverse_kinematic.kinPos();

		return t0 - param.count_;
	}, nullptr, result);


	auto &s = r->simulatorPool().add<aris::dynamic::AdamsSimulator>();
	s.saveAdams("C:\\Users\\py033\\Desktop\\ur5.cmd", result);



	auto plan = [](double s, double *pee, double *vee, double *aee)->void
	{
		double pee_[6]{ 0.5 + 0.2*s, 0, 0.35, 0, 0, PI };
		double vee_[6]{ 0.2,0,0,0,0,0 };
		double aee_[6]{ 0,0,0,0,0,0 };

		s_vc(6, pee_, pee);
		s_vc(6, vee_, vee);
		s_vc(6, aee_, aee);
	};

	/*
	auto &planner = rbt.planPool().add<aris::dynamic::OptimalTrajectory>();
	planner.setBeginNode({ 0,0,0,0 });
	planner.setEndNode({ 0,1,0,0 });
	planner.setFunction(plan);
	planner.setSolver(&inverse_kinematic);
	planner.setMotionLimit(std::vector<OptimalTrajectory::MotionLimit>(6, OptimalTrajectory::MotionLimit{ 1,-1,1,-1,1,-1,1,-1 }));

	planner.run();
	*/

	std::cout << "demo_ur finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

