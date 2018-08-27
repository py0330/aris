#include <algorithm>

#include"aris_robot_rokae.h"

using namespace aris::dynamic;

namespace aris::robot
{
	auto createRokaeModel()->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		auto &p1 = model->partPool().add<Part>("L1");
		auto &p2 = model->partPool().add<Part>("L2");
		auto &p3 = model->partPool().add<Part>("L3");
		auto &p4 = model->partPool().add<Part>("L4");
		auto &p5 = model->partPool().add<Part>("L5");
		auto &p6 = model->partPool().add<Part>("L6");

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, 0.176 };
		const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
		const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
		const double j4_pos[3]{ -0.1233, 0.0, 0.6295, };
		const double j5_pos[3]{ 0.32, -0.03235, 0.6295, };
		const double j6_pos[3]{ 0.383, 0.0, 0.6295, };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, -1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ -1.0, 0.0, 0.0 };

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
		double pq_ee_i[]{ 3.970000e-001, 0.000000e+000, 6.295000e-001, 0.000000e+000, 0.000000e+000, 0.000000e+000, 1.000000e+000 };
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();

		return model;
	}
	auto createRokaeController()->std::unique_ptr<aris::control::Controller> 
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
			std::string xml_str =
				"<m" + std::to_string(i) + " type=\"EthercatMotion\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00030924\""
				" vendor_id=\"0x0000009a\" revision_num=\"0x000103F6\" dc_assign_activate=\"0x0300\""
				" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" max_acc=\"10.0\" max_pos_following_error=\"100.0\" max_vel_following_error=\"200.0\""
				" home_pos=\"0\" pos_factor=\"62914560\">"
				"	<pdo_group_pool type=\"PdoGroupPoolObject\">"
				"		<index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
				"			<control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
				"			<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
				"			<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
				"			<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
				"			<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
				"		</index_1600>"
				"		<index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
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

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		}

		return controller;
	};

}
