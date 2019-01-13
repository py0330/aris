#include <algorithm>

#include"aris_robot_ur.h"

using namespace aris::dynamic;

namespace aris::robot
{
	auto createModelStewart(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model> { return aris::dynamic::createModelStewart(); }
	auto createControllerStewart()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				-1.0, -1.0, -1.0, -1.0, -1.0, -1.0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				-1.0, -1.0, -1.0, -1.0, -1.0, -1.0
			};
#endif
			double pos_factor[6]
			{
				1048576*1.8/0.01, 1.0, 1.0, 1.0, 1.0, 1.0
			};
			double max_pos[6]
			{
				2.0, 2.0, 2.0, 2.0, 2.0, 2.0
			};
			double min_pos[6]
			{
				0.5, 0.5, 0.5, 0.5, 0.5, 0.5
			};
			double max_vel[6]
			{
				3000/60/1.8*0.01, 5.0, 5.0, 5.0, 5.0, 5.0
			};
			double max_acc[6]
			{
				3000 / 60 / 1.8*0.01*100, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0
			};

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

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		}

		return controller;
	}
	auto createPlanRootStewart()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::EnablePlan>();
		plan_root->planPool().add<aris::plan::DisablePlan>();
		plan_root->planPool().add<aris::plan::HomePlan>();
		plan_root->planPool().add<aris::plan::ModePlan>();
		plan_root->planPool().add<aris::plan::RecoverPlan>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<aris::plan::SleepPlan>();
		auto &rs = plan_root->planPool().add<aris::plan::ResetPlan>();
		rs.command().findByName("group")->findByName("pos")->loadXmlStr("<pos default=\"{0.5,0.5,0.5,0.5,0.5,0.5}\" abbreviation=\"p\"/>");

		plan_root->planPool().add<aris::plan::MovePlan>();
		plan_root->planPool().add<aris::plan::MoveJ>();


		plan_root->planPool().add<aris::plan::AutoMove>();

		return plan_root;
	}
}
