#include <algorithm>

#include"aris/robot/servo_press.hpp"

using namespace aris::dynamic;

namespace aris::robot
{
	auto createModelServoPress(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", "Matrix", aris::core::Matrix(aris::PI));

		// add part //
		auto &p1 = model->partPool().add<Part>("L1");

		// add joint //
		const double j1_pos[3]{ 2.800000e-001, 8.750000e-001, -2.500000e-001 };
		const double j1_axis[6]{ 0.0, 1.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);

		// add ee general motion //
		double pq_ee_i[]{ 2.800000e-001, 8.750000e-001, -2.500000e-001, 0, 0, 0, 1 };
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p1.addMarker("ee_makI", pm_ee_i);
		auto &makJ = model->ground().addMarker("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// change robot pose //
		if (robot_pm)
		{
			p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
			j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<InverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();

		return model;
	}
	auto createControllerServoPress()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::Controller);

		for (aris::Size i = 0; i < 1; ++i)
		{
			std::string xml_str =
				"<m" + std::to_string(i) + " type=\"EthercatMotor\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00030924\""
				" vendor_id=\"0x0000009a\" revision_num=\"0x000103F6\" dc_assign_activate=\"0x0300\""
				" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" max_acc=\"10.0\" max_pos_following_error=\"100.0\" max_vel_following_error=\"200.0\""
				" home_pos=\"0\" pos_factor=\"62914560\">"
				"	<pdo_group_pool type=\"PdoPoolObject\">"
				"		<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
				"			<control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
				"			<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
				"			<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
				"			<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
				"			<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
				"		</index_1600>"
				"		<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
				"			<status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
				"			<mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
				"			<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
				"			<vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
				"			<cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
				"		</index_1a00>"
				"	</pdo_group_pool>"
				"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
				"		<home_mode index=\"0x6098\" subindex=\"0\" size=\"1\" config=\"35\"/>"
				"		<home_acc index=\"0x609A\" subindex=\"0\" size=\"4\" config=\"200000\"/>"
				"		<home_high_speed index=\"0x6099\" subindex=\"1\" size=\"4\" config=\"200000\"/>"
				"		<home_low_speed index=\"0x6099\" subindex=\"2\" size=\"4\" config=\"100000\"/>"
				"		<home_offset index=\"0x607C\" subindex=\"0\" size=\"4\" config=\"0\"/>"
				"	</sdo_pool>"
				"</m" + std::to_string(i) + ">";

			controller->motorPool().push_back(new aris::control::EthercatMotor());
			aris::core::fromXmlString(controller->motorPool().back(), xml_str);
		}

		return controller;
	}
	auto createPlanRootServoPress()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		auto &rc = plan_root->planPool().add<aris::plan::Reset>();
		aris::core::fromXmlString(rc.command().findParam("pq"), "<pq default=\"{0.28,0.875,-0.25,0,0,0,1}\"/>");
		aris::core::fromXmlString(rc.command().findParam("pm"), "<pm default=\"{1,0,0,0.28,0,1,0,0.875,0,0,1,-0.25,0,0,0,1}\"/>");
		aris::core::fromXmlString(rc.command().findParam("pe"), "<pe default=\"{0.28,0.875,-0.25,0,0,0}\"/>");
		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();

		return plan_root;
	}
}
