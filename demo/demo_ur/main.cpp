#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

const double PI = 3.14159265358979;

// ur5 的反解求解器，重载了kinPos()函数, 可以用它来求反解 //
class Ur5InverseSolver :public aris::dynamic::InverseKinematicSolver
{
public:
	static const std::string& Type() { static const std::string type("Ur5InverseSolver"); return type; }
	auto virtual type() const->const std::string& override{ return Type(); }

	auto virtual kinPos()->bool override
	{
		double q[6];

		double j6_pnt_loc[3]{ 0.0, 0.0, -0.0823 };
		double joint_pnt[3];
		s_pp2pp(*model().generalMotionPool().at(0).mpm(), j6_pnt_loc, joint_pnt);
		q[0] = std::atan2(joint_pnt[1], joint_pnt[0]) - std::asin(0.10915 / std::sqrt(joint_pnt[0] * joint_pnt[0] + joint_pnt[1] * joint_pnt[1]));

		double pe[6]{ 0,0,0,q[0],0,0 };
		double pm1[16], pm2[16], pm3[16], pm4[16];
		s_pe2pm(pe, pm1, "321");

		s_inv_pm_dot_pm(pm1, *model().generalMotionPool().at(0).mpm(), pm2);
		s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093 + 0.0823, 0.089159 - 0.09465, PI, 0, PI / 2}.data(), pm3, "321");
		s_pm_dot_inv_pm(pm2, pm3, pm4);
		s_pm2pe(pm4, pe, "232");

		q[4] = pe[4];
		q[5] = pe[5];

		double pe2[6]{ 0,0,0,q[0],pe[3],0.0 };
		s_pe2pm(pe2, pm2, "323");
		s_va(3, 0.09465, pm2 + 2, 4, joint_pnt, 1);
		s_va(3, -(0.13585 - 0.1197 + 0.093), pm2 + 1, 4, joint_pnt, 1);

		double pp[3];
		s_inv_pp2pp(pm1, joint_pnt, pp);
		pp[2] -= 0.089159;

		q[2] = PI - std::acos(-(pp[0] * pp[0] + pp[2] * pp[2] - 0.425 * 0.425 - 0.39225 * 0.39225) / (2 * 0.425*0.39225));
		q[1] = -PI + std::acos((-pp[0] * pp[0] - pp[2] * pp[2] - 0.425 * 0.425 + 0.39225 * 0.39225) / (2 * std::sqrt(pp[0] * pp[0] + pp[2] * pp[2]) *0.425)) - std::atan2(pp[2], pp[0]);
		q[3] = pe[3] - q[1] - q[2];

		double pe3[6]{0.0,0.0,0.0,0.0,0.0,0.0}, pm[16];
		for (aris::Size i = 0; i < 6; ++i)
		{
			pe3[5] = q[i];
			s_pm_dot_pm(*model().jointPool().at(i).makJ().pm(), s_pe2pm(pe3, pm, "123"), pm1);
			s_pm_dot_inv_pm(pm1, *model().jointPool().at(i).makI().prtPm(), const_cast<double*>(*model().jointPool().at(i).makI().fatherPart().pm()));
			model().motionPool().at(i).updMp();
		}

		return true;
	};

	virtual ~Ur5InverseSolver() = default;
	explicit Ur5InverseSolver(const std::string &name = "ur5_inverse_solver", aris::Size max_iter_count = 100, double max_error = 1e-10) :InverseKinematicSolver(name, max_iter_count, max_error) {}
};

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
	auto &inverse_kinematic = rbt.solverPool().add<Ur5InverseSolver>();
	auto &inverse_dynamic = rbt.solverPool().add<InverseDynamicSolver>();

	forward_kinematic.allocateMemory();
	inverse_kinematic.allocateMemory();
	inverse_dynamic.allocateMemory();
}


int main()
{
	std::cout << std::endl << "-----------------test model ur---------------------" << std::endl;

	try
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


	std::cout << "demo_ur finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

