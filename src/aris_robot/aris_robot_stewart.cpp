#include <algorithm>

#include"aris_robot_ur.h"

using namespace aris::dynamic;

namespace aris::robot
{
	auto createModelStewart(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model> { return aris::dynamic::createModelStewart(); }
	auto createControllerStewart()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController("controller"));

		for (aris::Size i = 0; i < 6; ++i)
		{
			double pos_offset[6]
			{
				-0.3, -0.54, -0.54, -0.54, -0.54, -0.54
			};
			double pos_factor[6]
			{
				1280000*1.8/0.01, 1280000 * 1.8 / 0.01, 1280000 * 1.8 / 0.01, 1280000 * 1.8 / 0.01, 1280000 * 1.8 / 0.01, 1280000 * 1.8 / 0.01
			};
			double max_pos[6]
			{
				0.765, 0.765, 0.765, 0.765, 0.765, 0.765
			};
			double min_pos[6]
			{
				0.535, 0.535, 0.535, 0.535, 0.535, 0.535
			};
			double max_vel[6]
			{
				3000/60/1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01
			};
			double max_acc[6]
			{
				3000 / 60 / 1.8*0.01*100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100
			};


			/* 这是台达的 
			std::string xml_str =
				"<m" + std::to_string(i) + " type=\"EthercatMotion\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x10305070\""
				" vendor_id=\"0x000001DD\" revision_num=\"0x02040608\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
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
				"	</sdo_pool>"
				"</m" + std::to_string(i) + ">";
				*/
			/*这是松下的*/
			std::string xml_str =
				"<m" + std::to_string(i) + " type=\"EthercatMotion\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380007\""
				" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"-335544320\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
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
				"	</sdo_pool>"
				"</m" + std::to_string(i) + ">";

			/*std::string xml_str =
					"<m_servo_press type=\"EthercatMotion\" phy_id=\"0\" product_code=\"0x60380007\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"0.01\" max_pos=\"0.26\" max_vel=\"0.125\" min_vel=\"-0.125\""
					" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
					" home_pos=\"0\" pos_factor=\"-3355443200\" pos_offset=\"0.0\">"
					"	<sm_pool type=\"SyncManagerPoolObject\">"
					"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
					"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
					"		<sm type=\"SyncManager\" is_tx=\"false\">"
					"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
					"				<control_word index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
					"				<offset_vel index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"				<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<offset_tor index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
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
					"	</sdo_pool>"
					"</m_servo_press>";
				*/

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		}

		return controller;
	}
	auto createPlanRootStewart()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Home>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<aris::plan::Sleep>();
		plan_root->planPool().add<aris::plan::Recover>();
		auto &rs = plan_root->planPool().add<aris::plan::Reset>();
		rs.command().findParam("pos")->setDefaultValue("{0.5525761606482785,0.5037132398222938,0.5037132398222938,0.5525761606482785,0.4816914860567862,0.4816914860567862}");

		plan_root->planPool().add<aris::plan::MoveAbsJ>();
		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();

		auto &am = plan_root->planPool().add<aris::plan::AutoMove>();
		am.command().findParam("max_pe")->setDefaultValue("{0.04,0.04-0.012,0.04+0.515,0.085,0.085,0.085}");
		am.command().findParam("min_pe")->setDefaultValue("{-0.04,-0.04-0.012,-0.04+0.515,-0.085,-0.085,-0.085}");
		am.command().findParam("init_pe")->setDefaultValue("{0.0,-0.012,0.515,0.0,0.0,0.0}");
		am.command().findParam("init_ve")->setDefaultValue("{0.1,0.1,0.1,0.1,0.1,0.1}");
		am.command().findParam("init_ae")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("init_de")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("pe")->setDefaultValue("{0.0,-0.012,0.515,0.0,0.0,0.0}");
		am.command().findParam("ve")->setDefaultValue("{0.1,0.1,0.1,0.1,0.1,0.1}");
		am.command().findParam("ae")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("de")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("eul_type")->setDefaultValue("123");

		auto &mm = plan_root->planPool().add<aris::plan::ManualMove>();
		mm.command().findParam("ve")->setDefaultValue("{0.05,0.05,0.05,0.05,0.05,0.05}");
		mm.command().findParam("ae")->setDefaultValue("{10,10,10,10,10,10}");
		mm.command().findParam("de")->setDefaultValue("{10,10,10,10,10,10}");
		mm.command().findParam("eul_type")->setDefaultValue("123");
		mm.command().findParam("increase_count")->setDefaultValue("100");

		return plan_root;
	}
}
