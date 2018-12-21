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
		const double iv1[]{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 };
		const double iv2[]{ 0.00000000000000, - 0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, - 0.00855352993039, - 0.09946674483372, - 0.00712210734359 };
		const double iv3[]{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, - 0.00578410451516,   0.00570606128540 };
		const double iv4[]{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, - 0.66644096559686,   0.00228253380852 };
		const double iv5[]{ 0.00000000000000,   0.05362286897910,   0.00528925153464, - 0.00842588023014,   0.00128498153337, - 0.00389810210572,   0.00000000000000, - 0.00223677867576, - 0.03365036368035, - 0.00415647085627 };
		const double iv6[]{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, - 0.00085124094833,   0.04209529937135,   0.04102481443654, - 0.00067596644891,   0.00017482449876, - 0.00041025776053 };

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


		m1.setFrcCoe(std::array<double, 3>{9.34994758321915,   7.80825641041495,   0.00000000000000}.data());
		m2.setFrcCoe(std::array<double, 3>{11.64080253106441,   13.26518528472506,   3.55567932576820}.data());
		m3.setFrcCoe(std::array<double, 3>{4.77014054273075,   7.85644357492508,   0.34445460269183}.data());
		m4.setFrcCoe(std::array<double, 3>{3.63141668516122,   3.35461524886318,   0.14824771620542}.data());
		m5.setFrcCoe(std::array<double, 3>{2.58310846982020,   1.41963212641879,   0.04855267273770}.data());
		m6.setFrcCoe(std::array<double, 3>{1.78373986219597,   0.31920640440152,   0.03381545544099}.data());


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

		// 尺寸变量 //
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

		// 移动方向，U副第一根轴、第二根轴
		double prismatic_direction[6][3];
		double first_axis[6][3];
		const double second_axis[6][3]
		{
		{ 0,1,0 },
		{ 0,1,0 },
		{ 0,1,0 },
		{ 0,1,0 },
		{ 0,1,0 },
		{ 0,1,0 },
		};
		for (Size i = 0; i < 6; ++i)
		{
			s_vc(3, up_pos[i], prismatic_direction[i]);
			s_vs(3, down_pos[i], prismatic_direction[i]);
			s_nv(3, 1.0 / s_norm(3, prismatic_direction[i]), prismatic_direction[i]);

			s_c3(prismatic_direction[i], second_axis[i], first_axis[i]);
		}

		double p1a_pm[16], p1b_pm[16], p2a_pm[16], p2b_pm[16], p3a_pm[16], p3b_pm[16], p4a_pm[16], p4b_pm[16], p5a_pm[16], p5b_pm[16], p6a_pm[16], p6b_pm[16];
		s_sov_axes2pm(down_pos[0], first_axis[0], second_axis[0], p1a_pm, "xz");
		s_sov_axes2pm(up_pos[0], first_axis[0], second_axis[0], p1b_pm, "xz");
		s_sov_axes2pm(down_pos[1], first_axis[1], second_axis[1], p2a_pm, "xz");
		s_sov_axes2pm(up_pos[1], first_axis[1], second_axis[1], p2b_pm, "xz");
		s_sov_axes2pm(down_pos[2], first_axis[2], second_axis[2], p3a_pm, "xz");
		s_sov_axes2pm(up_pos[2], first_axis[2], second_axis[2], p3b_pm, "xz");
		s_sov_axes2pm(down_pos[3], first_axis[3], second_axis[3], p4a_pm, "xz");
		s_sov_axes2pm(up_pos[3], first_axis[3], second_axis[3], p4b_pm, "xz");
		s_sov_axes2pm(down_pos[4], first_axis[4], second_axis[4], p5a_pm, "xz");
		s_sov_axes2pm(up_pos[4], first_axis[4], second_axis[4], p5b_pm, "xz");
		s_sov_axes2pm(down_pos[5], first_axis[5], second_axis[5], p6a_pm, "xz");
		s_sov_axes2pm(up_pos[5], first_axis[5], second_axis[5], p6b_pm, "xz");


		// add part //
		auto &p1a = model->partPool().add<Part>("p1a", nullptr, p1a_pm);
		auto &p1b = model->partPool().add<Part>("p1b", nullptr, p1b_pm);
		auto &p2a = model->partPool().add<Part>("p2a", nullptr, p2a_pm);
		auto &p2b = model->partPool().add<Part>("p2b", nullptr, p2b_pm);
		auto &p3a = model->partPool().add<Part>("p3a", nullptr, p3a_pm);
		auto &p3b = model->partPool().add<Part>("p3b", nullptr, p3b_pm);
		auto &p4a = model->partPool().add<Part>("p4a", nullptr, p4a_pm);
		auto &p4b = model->partPool().add<Part>("p4b", nullptr, p4b_pm);
		auto &p5a = model->partPool().add<Part>("p5a", nullptr, p5a_pm);
		auto &p5b = model->partPool().add<Part>("p5b", nullptr, p5b_pm);
		auto &p6a = model->partPool().add<Part>("p6a", nullptr, p6a_pm);
		auto &p6b = model->partPool().add<Part>("p6b", nullptr, p6b_pm);
		auto &up = model->partPool().add<Part>("up");

		p1a.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p1b.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p2a.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p2b.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p3a.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p3b.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p4a.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p4b.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p5a.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p5b.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p6a.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p6b.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		up.geometryPool().add<aris::dynamic::ParasolidGeometry>("test", "C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\up.xmt_txt");

		// add joint //
		auto &u1 = model->addUniversalJoint(p1a, model->ground(), down_pos[0], second_axis[0], first_axis[0]);
		auto &p1 = model->addPrismaticJoint(p1b, p1a, down_pos[0], prismatic_direction[0]);
		auto &s1 = model->addSphericalJoint(p1b, up, up_pos[0]);
		auto &u2 = model->addUniversalJoint(p2a, model->ground(), down_pos[1], second_axis[1], first_axis[1]);
		auto &p2 = model->addPrismaticJoint(p2b, p2a, down_pos[1], prismatic_direction[1]);
		auto &s2 = model->addSphericalJoint(p2b, up, up_pos[1]);
		auto &u3 = model->addUniversalJoint(p3a, model->ground(), down_pos[2], second_axis[2], first_axis[2]);
		auto &p3 = model->addPrismaticJoint(p3b, p3a, down_pos[2], prismatic_direction[2]);
		auto &s3 = model->addSphericalJoint(p3b, up, up_pos[2]);
		auto &u4 = model->addUniversalJoint(p4a, model->ground(), down_pos[3], second_axis[3], first_axis[3]);
		auto &p4 = model->addPrismaticJoint(p4b, p4a, down_pos[3], prismatic_direction[3]);
		auto &s4 = model->addSphericalJoint(p4b, up, up_pos[3]);
		auto &u5 = model->addUniversalJoint(p5a, model->ground(), down_pos[4], second_axis[4], first_axis[4]);
		auto &p5 = model->addPrismaticJoint(p5b, p5a, down_pos[4], prismatic_direction[4]);
		auto &s5 = model->addSphericalJoint(p5b, up, up_pos[4]);
		auto &u6 = model->addUniversalJoint(p6a, model->ground(), down_pos[5], second_axis[5], first_axis[5]);
		auto &p6 = model->addPrismaticJoint(p6b, p6a, down_pos[5], prismatic_direction[5]);
		auto &s6 = model->addSphericalJoint(p6b, up, up_pos[5]);

		// add actuation //
		auto &m1 = model->addMotion(p1);
		auto &m2 = model->addMotion(p2);
		auto &m3 = model->addMotion(p3);
		auto &m4 = model->addMotion(p4);
		auto &m5 = model->addMotion(p5);
		auto &m6 = model->addMotion(p6);

		for (int i = 0; i < 6; ++i)
		{

		}

		// add ee general motion //
		double pm_ee_i[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = up.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::StewartInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
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
