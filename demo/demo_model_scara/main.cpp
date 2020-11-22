/// \example demo_model_scara/main.cpp
/// 本例子展示基于Scara机器人的建模:
///

#include "aris.hpp"

using namespace std;

/// \addtogroup dynamic_model_group
/// @{
/// 
///
/// @}


class MyPlan:public aris::core::CloneObject<MyPlan, aris::plan::Plan>
{
public:
	auto virtual executeRT()->int override
	{
		const double PI = 3.141592653589793;
		
		
		// 设置时间，如果伺服周期为1000Hz，那么每个周期时间递加0.001s //
		model()->setTime(model()->time() + 0.001);

		// 计算末端的位置 //
		double pe[6]{ 0 , 0 , 0 , 0 , 0 , 0 };
		pe[0] = 1 + 0.3*std::cos(PI * 2 * count() / 1000);
		pe[1] = 1 + 0.5*std::sin(PI * 2 * count() / 1000);
		pe[2] = -0.3 + 0.2*std::sin(PI * 2 * count() / 1000);
		pe[3] = 0.3 + 4 * PI*count() / 1000;

		// 设置末端位置 //
		dynamic_cast<aris::dynamic::GeneralMotion&>(model()->generalMotionPool().front()).setMpe(pe, "321");

		// 求反解 //
		model()->solverPool()[0].kinPos();

		// 轨迹的长度和返回值有关，返回0时轨迹结束，这里仿真1000个周期 //
		return 1000 - count();
	}
};


int main()
{
	// 本示例展示4轴SCARA机器人的建模过程，aris可以求解任何机构（串联、并联、混联、过约束、欠约束等）的正逆运动学、正逆动力学等问题

	//-------------------------------------------- 机器人参数定义 --------------------------------------------//
	/// [Parameter]
	// 定义PI
	const double PI = 3.141592653589793;

	// 定义关节的位置，以及轴线，SCARA为RRPR机构，包含3个转动副和1个移动副，轴线都是Z轴
	const double joint1_position[3]{ 0 , 0 , 0 };
	const double joint1_axis[3]{ 0 , 0 , 1 };
	const double joint2_position[3]{ 1 , 0 , 0 };
	const double joint2_axis[3]{ 0 , 0 , 1 };
	const double joint3_position[3]{ 1 , 1 , 0 };
	const double joint3_axis[3]{ 0 , 0 , 1 };
	const double joint4_position[3]{ 1 , 1 , 0 };
	const double joint4_axis[3]{ 0 , 0 , 1 };

	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	// inertia_vector的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	const double link1_position_and_euler321[6]{ 0 , 0 , 0 , 0 , 0 , 0 };
	const double link1_inertia_vector[10]{ 2 , 0 , 0 , 0 , 1 , 1, 10 , 0, 0, 0 };
	const double link2_position_and_euler321[6]{ 1 , 0 , 0 , PI/2 , 0 , 0 };
	const double link2_inertia_vector[10]{ 2 , 0 , 0 , 0 , 1 , 1, 10 , 0, 0, 0 };
	const double link3_position_and_euler321[6]{ 1 , 1 , 0 , PI/2 , 0 , 0 };
	const double link3_inertia_vector[10]{ 2 , 0 , 0 , 0 , 1 , 1, 10 , 0, 0, 0 };
	const double link4_position_and_euler321[6]{ 1 , 1 , 0 , PI/2 , 0 , 0 };
	const double link4_inertia_vector[10]{ 2 , 0 , 0 , 0 , 1 , 1, 10 , 0, 0, 0 };

	// 定义末端位置与321欧拉角，这个位置为机构起始时的位置
	const double end_effector_position_and_euler321[6]{ 1 , 1 , 0 , PI/2 , 0 , 0 };
	/// [Parameter]

	//-------------------------------------------- 机器人建模 --------------------------------------------//
	/// [Modeling]
	// 定义模型变量
	aris::dynamic::Model m;

	// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
	auto &link1 = m.addPartByPe(link1_position_and_euler321, "321", link1_inertia_vector);
	auto &link2 = m.addPartByPe(link2_position_and_euler321, "321", link2_inertia_vector);
	auto &link3 = m.addPartByPe(link3_position_and_euler321, "321", link3_inertia_vector);
	auto &link4 = m.addPartByPe(link4_position_and_euler321, "321", link4_inertia_vector);
	
	// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
	auto &joint1 = m.addRevoluteJoint(link1, m.ground(), joint1_position, joint1_axis);
	auto &joint2 = m.addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
	auto &joint3 = m.addPrismaticJoint(link3, link2, joint3_position, joint3_axis);
	auto &joint4 = m.addRevoluteJoint(link4, link3, joint4_position, joint4_axis);
	
	// 添加驱动，驱动位于关节上
	auto &motion1 = m.addMotion(joint1);
	auto &motion2 = m.addMotion(joint2);
	auto &motion3 = m.addMotion(joint3);
	auto &motion4 = m.addMotion(joint4);

	// 添加末端，第一个参数表明末端位于link4上，第二个参数表明末端的位姿是相对于地面的，后两个参数定义了末端的起始位姿
	auto &end_effector = m.addGeneralMotionByPe(link4, m.ground(), end_effector_position_and_euler321, "321");
	/// [Modeling]

	//-------------------------------------------- 添加求解器 --------------------------------------------//
	/// [Solver]
	// 添加两个求解器，并为求解器分配内存。注意，求解器一但分配内存后，请不要再添加或删除杆件、关节、驱动、末端等所有元素
	auto &inverse_kinematic_solver = m.solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto &inverse_dynamic_solver = m.solverPool().add<aris::dynamic::InverseDynamicSolver>();
	
	auto &adams = m.simulatorPool().add<aris::dynamic::AdamsSimulator>();
	m.init();
	/// [Solver]

	//-------------------------------------------- 位置反解 --------------------------------------------//
	/// [Inverse_Position]
	// 现在求位置反解，首先设置末端的位置与321欧拉角
	double end_effector_pos_and_eul[6]{ 1.3 , 1 , -0.3 , 0.3 , 0 , 0 };
	end_effector.setMpe(end_effector_pos_and_eul, "321");
	
	// 求解，位置求解需要迭代，有可能会失败,因此这里做一个判断
	if (inverse_kinematic_solver.kinPos()) throw std::runtime_error("kinematic position failed");
	
	// 结果储存在电机的mp()函数里，将结果打印出来
	std::cout << "input position : " << motion1.mp() << "  " << motion2.mp() << "  " << motion3.mp() << "  " << motion4.mp() << std::endl;
	/// [Inverse_Position]

	//-------------------------------------------- 速度反解 --------------------------------------------//
	/// [Inverse_Velocity]
	// 现在求速度反解，首先设置末端的线速度和角速度
	double end_effector_point_and_angular_velocity[6]{ 0.3 , -0.2 , 0.2 , 0 , 0 , 0.3 };
	end_effector.setMva(end_effector_point_and_angular_velocity);
	
	// 求解
	inverse_kinematic_solver.kinVel();
	
	// 结果储存在电机的mv()函数里，将结果打印出来
	std::cout << "input velocity : " << motion1.mv() << "  " << motion2.mv() << "  " << motion3.mv() << "  " << motion4.mv() << std::endl;
	/// [Inverse_Velocity]

	//-------------------------------------------- 动力学反解 --------------------------------------------//
	/// [Inverse_Dynamic]
	// 现在设置电机的加速度，来求动力学反解
	double motion_acceleration[4]{ 9 , 8 , 7 , 6 };
	motion1.setMa(motion_acceleration[0]);
	motion2.setMa(motion_acceleration[1]);
	motion3.setMa(motion_acceleration[2]);
	motion4.setMa(motion_acceleration[3]);

	// 求解
	inverse_dynamic_solver.dynAccAndFce();
	
	// 查看电机的输入力
	std::cout << "input force    : " << motion1.mf() << "  " << motion2.mf() << "  " << motion3.mf() << "  " << motion4.mf() << std::endl;
	
	// 查看末端线加速度与角加速度
	double ee_result[6];
	end_effector.getMaa(ee_result);
	std::cout << "ee acceleration: " << ee_result[0] << "  " << ee_result[1] << "  " << ee_result[2] << "  " << ee_result[3] << "  " << ee_result[4] << "  " << ee_result[5] << std::endl;
	/// [Inverse_Dynamic]

	//-------------------------------------------- 用Adams验证 --------------------------------------------//
	/// [Verify_By_Adams]
	// 使用adams验证计算结果, 为了能够在Adams里显示动画，首先给各个杆件添加 Parasolid 格式的模型，这些文件存在aris的安装目录下
	m.ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:/aris/aris-1.5.0/resource/demo_model_scara/ground.xmt_txt");
	link1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:/aris/aris-1.5.0/resource/demo_model_scara/part1.xmt_txt");
	link2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:/aris/aris-1.5.0/resource/demo_model_scara/part2.xmt_txt");
	link3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:/aris/aris-1.5.0/resource/demo_model_scara/part3.xmt_txt");
	link4.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:/aris/aris-1.5.0/resource/demo_model_scara/part4.xmt_txt");
	
	// 添加一个 Adams 仿真器插件, 之后保存到aris的安装目录下，用户此时就可以在 Adams 查看结果了
	
	adams.saveAdams("C:\\aris\\aris_scara.cmd");
	/// [Verify_By_Adams]


	//-------------------------------------------- 仿真整段轨迹 --------------------------------------------//
	/// [Simulate]
	// 添加一个轨迹 //
	MyPlan my_plan;

	// 添加一个仿真结果 //
	auto &result = m.simResultPool().add<aris::dynamic::SimResult>();
	result.resetModel(&m);
	result.allocateMemory();

	// 仿真，结果保存到上述变量中 //
	adams.simulate(my_plan, result);

	std::cout << result.partResultPool().size() << std::endl;

	// 要想查看结果，可以使用restore函数，这里将model复位到第500个周期处
	result.restore(500);
	std::cout << "result at 500 count: " << std::endl;
	std::cout << "input position : " << motion1.mp() << "  " << motion2.mp() << "  " << motion3.mp() << "  " << motion4.mp() << std::endl;

	// 储存成adams文件，注意这里需要把result参数加上，这个模型文件为整个轨迹的仿真 //
	adams.saveAdams("C:\\aris\\aris_scara_simulate.cmd", result);
	/// [Simulate]

	std::cout << "demo_3R finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

