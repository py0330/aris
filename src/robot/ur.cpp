#include <algorithm>
#include <array>

#include"aris/robot/ur.hpp"

using namespace aris::dynamic;

namespace aris::robot
{
	// 具体参数参考MR里面147-158页 //
	auto createModelUr5(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 相对位移 //
		const double pe_j1[6]{ 0.0, 0.0, 0.089159, 0, 0, 0 };
		const double pe_j2[6]{ 0.0, 0.13585, 0.0, 0, PI / 2, 0 };
		const double pe_j3[6]{ 0.0, -0.1197, 0.425, 0, 0, 0 };
		const double pe_j4[6]{ 0.0, 0.0, 0.39225, 0, PI / 2, 0 };
		const double pe_j5[6]{ 0.0, 0.10915 + 0.1197 - 0.13585, 0.0, 0, 0, 0 };
		const double pe_j6[6]{ 0.0, 0.0, 0.09465, 0, 0, 0 };
		const double pe_ee[6]{ 0.0, 0.0823, 0.0, 0, 0, -PI / 2 };

		double pm1[16], pm2[16], pm3[16], pm4[16], pm5[16], pm6[16], pm_ee[16];

		s_pe2pm(pe_j1, pm1, "321");
		s_pm_dot_pm(pm1, s_pe2pm(pe_j2, nullptr, "321"), pm2);
		s_pm_dot_pm(pm2, s_pe2pm(pe_j3, nullptr, "321"), pm3);
		s_pm_dot_pm(pm3, s_pe2pm(pe_j4, nullptr, "321"), pm4);
		s_pm_dot_pm(pm4, s_pe2pm(pe_j5, nullptr, "321"), pm5);
		s_pm_dot_pm(pm5, s_pe2pm(pe_j6, nullptr, "321"), pm6);
		s_pm_dot_pm(pm6, s_pe2pm(pe_ee, nullptr, "321"), pm_ee);

		double iv[10], pm[16];

		const double p1_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		const double p1_iv[10]{ 3.7, 0, 0, 0, 0.010267495893, 0.010267495893, 0.00666, 0.0, 0.0, 0.0 };
		s_pe2pm(p1_pe, pm, "321");
		s_iv2iv(s_pm_dot_pm(pm1, pm), p1_iv, iv);
		auto &p1 = model->partPool().add<Part>("L1", iv);

		const double p2_pe[6]{ 0.0, 0.0, 0.28, 0.0, 0.0, 0.0 };
		const double p2_iv[10]{ 8.393, 0, 0, 0, 0.22689067591, 0.22689067591, 0.0151074, 0.0, 0.0, 0.0 };
		s_pe2pm(p2_pe, pm, "321");
		s_iv2iv(s_pm_dot_pm(pm2, pm), p2_iv, iv);
		auto &p2 = model->partPool().add<Part>("L2", iv);

		const double p3_pe[6]{ 0.0, 0.0, 0.25, 0.0, 0.0, 0.0 };
		const double p3_iv[10]{ 2.275, 0, 0, 0, 0.049443313556, 0.049443313556, 0.004095, 0.0, 0.0, 0.0 };
		s_pe2pm(p3_pe, pm, "321");
		s_iv2iv(s_pm_dot_pm(pm3, pm), p3_iv, iv);
		auto &p3 = model->partPool().add<Part>("L3", iv);

		const double p4_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		const double p4_iv[10]{ 1.219, 0, 0, 0, 0.111172755531, 0.111172755531, 0.21942, 0.0, 0.0, 0.0 };
		s_pe2pm(p4_pe, pm, "321");
		s_iv2iv(s_pm_dot_pm(pm4, pm), p4_iv, iv);
		auto &p4 = model->partPool().add<Part>("L4", iv);

		const double p5_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.00 };
		const double p5_iv[10]{ 1.219, 0, 0, 0, 0.111172755531, 0.111172755531, 0.21942, 0.0, 0.0, 0.0 };
		s_pe2pm(p5_pe, pm, "321");
		s_iv2iv(s_pm_dot_pm(pm5, pm), p5_iv, iv);
		auto &p5 = model->partPool().add<Part>("L5", iv);

		const double p6_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		const double p6_iv[10]{ 0.1879, 0, 0, 0, 0.0171364731454, 0.0171364731454, 0.033822, 0.0, 0.0, 0.0 };
		s_pe2pm(p6_pe, pm, "321");
		s_iv2iv(s_pm_dot_pm(pm6, pm), p6_iv, iv);
		auto &p6 = model->partPool().add<Part>("L6", iv);

		// add solid //
		model->ground().geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/ground.DWG")
			, s_pe2pm(std::array<double, 6>{0, 0, 0.003, 2.3561944875, -PI / 2, 0.0 }.data(), nullptr, "313"));
		p1.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L1.DWG")
			, s_pe2pm(std::array<double, 6>{0.0, 0.0, 0.089159, 0, -PI / 2, 0}.data(), nullptr, "313"));
		p2.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L2.DWG")
			, s_pe2pm(std::array<double, 6>{ 0.0, 0.13585, 0.089159, 0, -PI / 2, PI / 2}.data(), nullptr, "313"));
		p3.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L3.DWG")
			, s_pe2pm(std::array<double, 6>{0.425, 0.13585 - 0.1197, 0.089159, 0, -PI / 2, PI / 2}.data(), nullptr, "313"));
		p4.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L4.DWG")
			, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159, 0, -PI / 2, -PI / 2}.data(), nullptr, "321"));
		p5.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L5.DWG")
			, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, -PI / 2, PI}.data(), nullptr, "313"));
		p6.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L6.DWG")
			, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, -PI / 2, 0}.data(), nullptr, "313"));

		const double L1 = 0.425;
		const double L2 = 0.39225;
		const double W1 = 0.13585 - 0.1197 + 0.093;
		const double W2 = 0.0823;
		const double H1 = 0.089159;
		const double H2 = 0.09465;

		// add joints //
		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 0.0, 1.0, 0.0 };
		const double j5_axis[6]{ 0.0, 0.0, -1.0 };
		const double j6_axis[6]{ 0.0, 1.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), s_pm2pp(pm1), j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, s_pm2pp(pm2), j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, s_pm2pp(pm3), j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, s_pm2pp(pm4), j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, s_pm2pp(pm5), j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, s_pm2pp(pm6), j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		// add ee general motion //
		auto &makI = p6.addMarker("ee_makI", pm_ee);
		auto &makJ = model->ground().addMarker("ee_makJ");
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
			j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::Ur5InverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		return model;
	};
	auto createControllerUr5()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
			std::string xml_str =
				"<m" + std::to_string(i) + " type=\"EthercatMotor\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00030924\""
				" vendor_id=\"0x0000009a\" revision_num=\"0x000103F6\" dc_assign_activate=\"0x0300\""
				" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" max_acc=\"10.0\" max_pos_following_error=\"100.0\" max_vel_following_error=\"200.0\""
				" home_pos=\"0\" pos_factor=\"62914560\">"
				"	<sm_pool type=\"SyncManagerPoolObject\">"
				"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
				"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
				"		<sm type=\"SyncManager\" is_tx=\"false\">"
				"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
				"				<control_word index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"			</index_1600>"
				"		</sm>"
				"		<sm type=\"SyncManager\" is_tx=\"true\">"
				"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
				"				<status_word index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</index_1a00>"
				"		</sm>"
				"	</sm_pool>"
				"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
				"		<home_mode index=\"0x6098\" subindex=\"0\" size=\"1\" config=\"35\"/>"
				"		<home_acc index=\"0x609A\" subindex=\"0\" size=\"4\" config=\"200000\"/>"
				"		<home_high_speed index=\"0x6099\" subindex=\"1\" size=\"4\" config=\"200000\"/>"
				"		<home_low_speed index=\"0x6099\" subindex=\"2\" size=\"4\" config=\"100000\"/>"
				"		<home_offset index=\"0x607C\" subindex=\"0\" size=\"4\" config=\"0\"/>"
				"	</sdo_pool>"
				"</m" + std::to_string(i) + ">";

			controller->slavePool().push_back(new aris::control::EthercatMotor());
			aris::core::fromXmlString(controller->slavePool().back(), xml_str);
		}

		return controller;
	}
	auto createPlanRootUr5()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();

		return plan_root;
	}
}
