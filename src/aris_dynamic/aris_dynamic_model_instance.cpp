#include "aris_dynamic_model_instance.h"

namespace aris::dynamic
{
	auto createModelRokaeXB4(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", aris::core::Matrix(PI));

		// add part //
		const double iv1[]{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000 };
		const double iv2[]{ 0.00000000000000, - 0.00045590853958,   0.00000000000000,   0.04335268022295,   0.03857102950268,   0.00000000000000,   0.00000000000000,  -0.00007158819223, - 0.00124024659236, - 0.00022039973889 };
		const double iv3[]{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00357267308650,   0.00000000000000,   0.00720755118394,   0.00000000000000,   0.00010128228006,   0.00002579963338, - 0.00002145700459 };
		const double iv4[]{ 0.00000000000000,   0.01286190182328, - 0.00000039900916, - 0.00019540879236,   0.00000000000000, - 0.00031192998207, - 0.00017961570036, - 0.00000433263974, - 0.00812446836477,   0.00001212996707 };
		const double iv5[]{ 0.00000000000000,   0.00051787887547,   0.00000000000000, - 0.00001456116317, - 0.00000574963837, - 0.00005507173531,   0.00000000000000,   0.00000213768085, - 0.00033657494698,   0.00000019918874 };
		const double iv6[]{ 0.00000000000000,   0.00000000000000,   0.00001191885755, - 0.00001188912259, - 0.00000537482363,   0.00040540332820,   0.00042042901098, - 0.00000202513111,   0.00000076290236, - 0.00000752219357 };

		auto &p1 = model->partPool().add<Part>("L1", iv1);
		auto &p2 = model->partPool().add<Part>("L2", iv2);
		auto &p3 = model->partPool().add<Part>("L3", iv3);
		auto &p4 = model->partPool().add<Part>("L4", iv4);
		auto &p5 = model->partPool().add<Part>("L5", iv5);
		auto &p6 = model->partPool().add<Part>("L6", iv6);

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, 0.176 };
		const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
		const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
		const double j4_pos[3]{ -0.1233, 0.0, 0.6295, };
		const double j5_pos[3]{ 0.32, -0.03235, 0.6295, };
		const double j6_pos[3]{ 0.383, 0.0, 0.6295, };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		// add ee general motion //
		double pq_ee_i[]{ 0.398, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// change robot pose //
		if (robot_pm)
		{
			p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
			p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
			p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
			p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
			p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
			p6.setPm(s_pm_dot_pm(robot_pm, *p6.pm()));
			j1.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ().prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();
		inverse_dynamic.allocateMemory();
		forward_dynamic.allocateMemory();

		inverse_kinematic.setWhichRoot(8);

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		return model;
	}
	auto createModelStewart()->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", aris::core::Matrix(PI));

		// add part //
		auto &p1a = model->partPool().add<Part>("p1a");
		auto &p1b = model->partPool().add<Part>("p1b");
		auto &p2a = model->partPool().add<Part>("p2a");
		auto &p2b = model->partPool().add<Part>("p2b");
		auto &p3a = model->partPool().add<Part>("p3a");
		auto &p3b = model->partPool().add<Part>("p3b");
		auto &p4a = model->partPool().add<Part>("p4a");
		auto &p4b = model->partPool().add<Part>("p4b");
		auto &p5a = model->partPool().add<Part>("p5a");
		auto &p5b = model->partPool().add<Part>("p5b");
		auto &p6a = model->partPool().add<Part>("p6a");
		auto &p6b = model->partPool().add<Part>("p6b");
		auto &up = model->partPool().add<Part>("up");

		// add joint //
		const double up_pos[6][3]
		{
			{ 0,0,-0.289 },
			{ 0.25,0,0.144 },
			{ 0.25,0,0.144 },
			{ -0.25,0,0.144 },
			{ -0.25,0,0.144 },
			{ 0,0,-0.289 },
		};
		const double down_pos[6][3]
		{
			{ 1,0,0 },
			{ 1,0,0 },
			{ 0,0,1.732 },
			{ 0,0,1.732 },
			{ -1,0,0 },
			{ -1,0,0 },
		};

		// 上下平台重合时移动副的方向 //
		double prismatic_direction[6][3];
		double first_axis[6][3];
		double second_axis[6][3];
		for (Size i = 0; i < 6; ++i)
		{
			s_vc(3, up_pos[i], prismatic_direction[i]);
			s_vi(3, down_pos[i], prismatic_direction[i]);
			s_nv(3, 1.0 / s_norm(3, prismatic_direction[i]), prismatic_direction[i]);

			double axis[3]{ 0,1,0 };
			s_vc(3, axis, second_axis[i]);

			s_c3(prismatic_direction[i], second_axis[i], first_axis[i]);
		}

		auto &u1 = model->addUniversalJoint(p1a, model->ground(), down_pos[0], first_axis[0], second_axis[0]);
		auto &p1 = model->addPrismaticJoint(p1b, p1a, down_pos[0], prismatic_direction[0]);
		auto &s1 = model->addSphericalJoint(p1b, up, up_pos[0]);
		auto &u2 = model->addUniversalJoint(p2a, model->ground(), down_pos[1], first_axis[1], second_axis[1]);
		auto &p2 = model->addPrismaticJoint(p2b, p2a, down_pos[1], prismatic_direction[1]);
		auto &s2 = model->addSphericalJoint(p2b, up, up_pos[1]);
		auto &u3 = model->addUniversalJoint(p3a, model->ground(), down_pos[2], first_axis[2], second_axis[2]);
		auto &p3 = model->addPrismaticJoint(p3b, p3a, down_pos[2], prismatic_direction[2]);
		auto &s3 = model->addSphericalJoint(p3b, up, up_pos[2]);
		auto &u4 = model->addUniversalJoint(p4a, model->ground(), down_pos[3], first_axis[3], second_axis[3]);
		auto &p4 = model->addPrismaticJoint(p4b, p4a, down_pos[3], prismatic_direction[3]);
		auto &s4 = model->addSphericalJoint(p4b, up, up_pos[3]);
		auto &u5 = model->addUniversalJoint(p5a, model->ground(), down_pos[4], first_axis[4], second_axis[4]);
		auto &p5 = model->addPrismaticJoint(p5b, p5a, down_pos[4], prismatic_direction[4]);
		auto &s5 = model->addSphericalJoint(p5b, up, up_pos[4]);
		auto &u6 = model->addUniversalJoint(p6a, model->ground(), down_pos[5], first_axis[5], second_axis[5]);
		auto &p6 = model->addPrismaticJoint(p6b, p6a, down_pos[5], prismatic_direction[5]);
		auto &s6 = model->addSphericalJoint(p6b, up, up_pos[5]);

		// add actuation //
		auto &m1 = model->addMotion(p1);
		auto &m2 = model->addMotion(p2);
		auto &m3 = model->addMotion(p3);
		auto &m4 = model->addMotion(p4);
		auto &m5 = model->addMotion(p5);
		auto &m6 = model->addMotion(p6);

		// add ee general motion //
		double pm_ee_i[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = up.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();
		inverse_dynamic.allocateMemory();
		forward_dynamic.allocateMemory();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		return model;
	}
}
