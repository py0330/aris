#include "aris.h"

using namespace std;
using namespace aris::dynamic;

int main()
{
	// 本示例展示3轴SCARA机器人的建模过程，aris可以求解任何机构（串联、并联、混联、过约束、欠约束等）的正逆运动学、正逆动力学等问题
	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	// inertia_vector的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	const double link1_position_and_euler321[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
	const double link1_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
	const double link2_position_and_euler321[6]{ 1.0 , 0.0 , 0.0 , PI/2 , 0.0 , 0.0 };
	const double link2_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
	const double link3_position_and_euler321[6]{ 1.0 , 1.0 , 0.0 , PI , 0.0 , 0.0 };
	const double link3_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };

	// 定义关节的位置，以及轴线，SCARA包含3个转动副，转动轴线是Z轴
	const double joint1_position[3]{ 0.0 , 0.0 , 0.0 };
	const double joint1_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint2_position[3]{ 1.0 , 0.0 , 0.0 };
	const double joint2_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint3_position[3]{ 1.0 , 1.0 , 0.0 };
	const double joint3_axis[3]{ 0.0 , 0.0 , 1.0 };

	// 定义末端位置与321欧拉角，这个位置为机构起始时的位置
	const double end_effector_position_and_euler321[6]{ 0.0 , 1.0 , 0.0 , PI , 0.0 , 0.0 };


	////////////////////////////////////////////////// 开始建模 ///////////////////////////////////////////////
	Model m;

	// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
	auto &link1 = m.addPartByPe(link1_position_and_euler321, "321", link1_inertia_vector);
	auto &link2 = m.addPartByPe(link2_position_and_euler321, "321", link2_inertia_vector);
	auto &link3 = m.addPartByPe(link3_position_and_euler321, "321", link3_inertia_vector);
	
	// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
	auto &joint1 = m.addRevoluteJoint(link1, m.ground(), joint1_position, joint1_axis);
	auto &joint2 = m.addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
	auto &joint3 = m.addRevoluteJoint(link3, link2, joint3_position, joint3_axis);
	
	// 添加驱动，驱动位于关节上
	auto &motion1 = m.addMotion(joint1);
	auto &motion2 = m.addMotion(joint2);
	auto &motion3 = m.addMotion(joint3);

	// 添加末端，第一个参数表明末端位于link3上，第二个参数表明末端的位姿是相对于地面的，后两个参数定义了末端的起始位姿
	auto &end_effector = m.addGeneralMotionByPe(link3, m.ground(), end_effector_position_and_euler321, "321");
	////////////////////////////////////////////////// 建模完毕 ///////////////////////////////////////////////
	
	
	////////////////////////////////////////////////// 添加求解器并分配内存 ///////////////////////////////////////////////
	// 添加两个求解器，并为求解器分配内存。注意，求解器一但分配内存后，请不要再添加或删除杆件、关节、驱动、末端等所有元素
	auto &forward_kinematic_solver = m.solverPool().add<ForwardKinematicSolver>();
	auto &inverse_kinematic_solver = m.solverPool().add<InverseKinematicSolver>();
	auto &forward_dynamic_solver = m.solverPool().add<ForwardDynamicSolver>();
	auto &inverse_dynamic_solver = m.solverPool().add<InverseDynamicSolver>();
	forward_kinematic_solver.allocateMemory();
	inverse_kinematic_solver.allocateMemory();
	forward_dynamic_solver.allocateMemory();
	inverse_dynamic_solver.allocateMemory();

	// 现在求位置反解，首先设置末端的位置与321欧拉角,之后求解
	double end_effector_pe[6]{ 0.0 , 0.5 , 0.0 , 0.3 , 0.0 , 0.0 };
	end_effector.setMpe(end_effector_pe, "321");
	inverse_kinematic_solver.kinPos();

	// 现在求速度反解，首先设置末端的速度,之后求解
	double end_effector_point_and_angular_velocity[6]{ 0.3 , -0.2 , 0.0 , 0.0 , 0.0 , 0.3 };
	end_effector.setMva(end_effector_point_and_angular_velocity);
	inverse_kinematic_solver.kinVel();

	// 现在设置加速度，来求动力学反解
	double motion_acceleration[3]{ 0.1 , -0.2 , 0.3 };
	motion1.setMa(motion_acceleration[0]);
	motion2.setMa(motion_acceleration[1]);
	motion3.setMa(motion_acceleration[2]);
	inverse_dynamic_solver.dynAccAndFce();


	////////////////////////////////////////////////// 打印结果 ///////////////////////////////////////////////
	// 这就求解好了，现在可以查看驱动的位置、速度、驱动力。除此之外aris还会求出每个杆件的位姿/速度/加速度
	std::cout << "motion 1 position:" << motion1.mp() << "  velocity:" << motion1.mv() << "  force:" << motion1.mf() << std::endl;
	std::cout << "motion 2 position:" << motion2.mp() << "  velocity:" << motion2.mv() << "  force:" << motion2.mf() << std::endl;
	std::cout << "motion 3 position:" << motion3.mp() << "  velocity:" << motion3.mv() << "  force:" << motion3.mf() << std::endl;

	std::cout << "link 3 pose:" << std::endl
		<< link3.pm()[0][0] << "  " << link3.pm()[0][1] << "  " << link3.pm()[0][2] << "  " << link3.pm()[0][3] << std::endl
		<< link3.pm()[1][0] << "  " << link3.pm()[1][1] << "  " << link3.pm()[1][2] << "  " << link3.pm()[1][3] << std::endl
		<< link3.pm()[2][0] << "  " << link3.pm()[2][1] << "  " << link3.pm()[2][2] << "  " << link3.pm()[2][3] << std::endl
		<< link3.pm()[3][0] << "  " << link3.pm()[3][1] << "  " << link3.pm()[3][2] << "  " << link3.pm()[3][3] << std::endl;

	double link3_velocity[6];
	link3.getVa(link3_velocity);
	std::cout << "link 3 velocity: " << link3_velocity[0] << "  " << link3_velocity[1] << "  " << link3_velocity[2] << "  " 
		<< link3_velocity[3] << "  " << link3_velocity[4] << "  " << link3_velocity[5] << std::endl;

	double link3_acceleration[6];
	link3.getAa(link3_acceleration);
	std::cout << "link 3 acceleration: " << link3_acceleration[0] << "  " << link3_acceleration[1] << "  " << link3_acceleration[2] << "  "
		<< link3_acceleration[3] << "  " << link3_acceleration[4] << "  " << link3_acceleration[5] << std::endl;

	link1.geometryPool().add<ParasolidGeometry>("parasolid_geometry", "C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t");
	link2.geometryPool().add<ParasolidGeometry>("parasolid_geometry", "C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t");
	link3.geometryPool().add<ParasolidGeometry>("parasolid_geometry", "C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t");

	//aris::dynamic::PlanFunction plan = [&](const PlanParam &param) ->int
	//{
	//	double pe[6]{ 0.0 , 1.0 , 0.0 , PI , 0.0 , 0.0 };

	//	pe[0] = -0.5 + 0.5*std::cos(PI * 2 * param.count_ / 1000);
	//	pe[1] = 1.0 + 0.5*std::sin(PI * 2 * param.count_ / 1000);

	//	param.model_->setTime(param.model_->time() + 1 / 1000.0);

	//	param.model_->generalMotionPool().front().setMpe(pe, "321");
	//	inverse_kinematic_solver.kinPos();

	//	return 1000 - param.count_;
	//};

	//

	//auto &adams = m.simulatorPool().add<AdamsSimulator>();
	//

	//auto &result = m.simResultPool().add<SimResult>();
	//adams.simulate(plan, nullptr, result);

	//result.restore(0);

	//adams.saveAdams("C:\\Users\\py033\\Desktop\\3r.cmd", result);


	std::cout << "demo_3R finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

