#include <iostream>
#include <array>
#include <aris.hpp>

using namespace aris::dynamic;
using namespace aris::robot;

const double PI = 3.14159265358979;

Model rbt;

void build_model() 
{
	
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

	
	UrParam param;
	param.H1 = 0.147;
	param.L1 = 0.427;
	param.L2 = 0.357;
	param.H2 = 0.116;
	param.W1 = 0.141;
	param.W2 = 0.105;


	auto r = aris::dynamic::createModelUr(param);

	//for (auto &p : r->partPool())
	//{
	//	dsp(4, 4, *dynamic_cast<FileGeometry&>(p.geometryPool().front()).prtPm());
	//}
	

	//-0.99500416527803   0.00000000000000   0.09983341664683   0.83228026169050
	//	0.09983341664683   0.00000000000000   0.99500416527803   0.10890468768786
	//	0.00000000000000   1.00000000000000 - 0.00000000000000 - 0.00549100000000
	//	0.00000000000000   0.00000000000000   0.00000000000000   1.00000000000000

	// add solvers //
	auto &forward_kinematic = r->solverPool()[1];
	auto &inverse_kinematic = r->solverPool()[0];

	auto &sim = r->simulatorPool().add<aris::dynamic::Simulator>("sim");
	auto &result = r->simResultPool().add<aris::dynamic::SimResult>("result");

	r->init();

	auto &ee = dynamic_cast<aris::dynamic::GeneralMotion&>(r->generalMotionPool().at(0));

	result.allocateMemory();
	inverse_kinematic.allocateMemory();
	forward_kinematic.allocateMemory();

	ee.setMpe(std::array<double, 6>{0.7, 0.1, 0.2, 0, PI + 0.2 , -PI / 2.0 + 0.1}.data(), "321");
	if(inverse_kinematic.kinPos())
		std::cout <<"failed"<<std::endl;

	std::cout << std::setprecision(16);
	dsp(4, 4, *ee.mpm());

	for (int i = 0; i < 100; ++i) {
		ee.setMpe(std::array<double, 6>{0.7, 0.1 + std::sin(i*0.1) * 0.1, 0.2, 0, PI + 0.2, -PI / 2.0 + 0.1}.data(), "321");
		if (inverse_kinematic.kinPos())
			std::cout << "failed" << std::endl;
		
		double mp[6];
		r->getInputPos(mp);
		dsp(1, 6, mp);

	}






	for (auto &m : r->motionPool()) 
	{
		m.updP();
		m.setMp(m.mp() + 0.01);
		std::cout << m.mp() << std::endl;
	}

	if (forward_kinematic.kinPos())
		std::cout << "failed" << std::endl;

	ee.updP();
	dsp(4, 4, *ee.mpm());

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
	ee.updP();
	dsp(4, 4, *ee.mpm());


	/*
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
	}, nullptr, result);*/


	auto &s = r->simulatorPool().add<aris::dynamic::AdamsSimulator>();
	s.saveAdams("C:\\Users\\py0330\\Desktop\\ur5.cmd", result);



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

