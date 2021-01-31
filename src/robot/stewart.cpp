#include <algorithm>

#include"aris/robot/stewart.hpp"

using namespace aris::dynamic;

namespace aris::robot
{
	auto createModelStewart(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model> { return aris::dynamic::createModelStewart(); }
	auto createControllerStewart()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::Controller("controller"));

		for (aris::Size i = 0; i < 6; ++i)
		{
			//double pos_offset[6]
			//{
			//	-0.54, -0.54, -0.54, -0.54, -0.54, -0.54
			//};
			//松下
			//double pos_factor[6]
			//{
			//	8388608 *1.8/0.01, 8388608 * 1.8 / 0.01, 8388608 * 1.8 / 0.01, 8388608 * 1.8 / 0.01, 8388608 * 1.8 / 0.01, 8388608 * 1.8 / 0.01
			//};

			// 台达需要找home
			double pos_offset[6]
			{
				-0.535, -0.535, -0.535, -0.535, -0.535, -0.535
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
				3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01, 3000 / 60 / 1.8*0.01
			};
			double max_acc[6]
			{
				3000 / 60 / 1.8*0.01*100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100, 3000 / 60 / 1.8*0.01 * 100
			};


			//这是台达的 
			std::string xml_str =
				"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x10305070\""
				" vendor_id=\"0x000001DD\" revision_num=\"0x02040608\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]*1.2) + "\" min_vel=\"" + std::to_string(-max_vel[i]*1.2) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.5\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject name=\"sm_pool\">"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotor>";

			// 这是松下的 //
			/*
			std::string xml_str =
				"<EthercatMotor phy_id=\"" + std::to_string(6-i) + "\" product_code=\"0x60380000\""
				" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]*1.2) + "\" min_vel=\"" + std::to_string(-max_vel[i]*1.2) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.5\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject name=\"sm_pool\">"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotor>";
			*/
				
			controller->motorPool().push_back(new aris::control::EthercatMotor());
			aris::core::fromXmlString(controller->motorPool().back(), xml_str);
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
		rs.command().findParam("pos")->setDefaultValue("{0.5411686785981304,0.4991128510566586,0.4991128510566586,0.5411686785981304,0.4768461834488792,0.4768461834488792}");
		
		auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
		mvaj.command().findParam("vel")->setDefaultValue("0.1");

		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();

		auto &am = plan_root->planPool().add<aris::plan::AutoMove>();
		am.command().findParam("max_pe")->setDefaultValue("{0.035,0.035-0.0103,0.037+0.513,0.08,0.08,0.04}");
		am.command().findParam("min_pe")->setDefaultValue("{-0.035,-0.035-0.0103,-0.037+0.513,-0.08,-0.08,-0.04}");
		am.command().findParam("init_pe")->setDefaultValue("{0.0,-0.0103,0.513,0.0,0.0,0.0}");
		am.command().findParam("init_ve")->setDefaultValue("{0.1,0.1,0.11,0.12,0.12,0.1}");
		am.command().findParam("init_ae")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("init_de")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("pe")->setDefaultValue("{0.0,-0.0103,0.513,0.0,0.0,0.0}");
		am.command().findParam("ve")->setDefaultValue("{0.1,0.1,0.11,0.12,0.12,0.1}");
		am.command().findParam("ae")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("de")->setDefaultValue("{10,10,10,10,10,10}");
		am.command().findParam("eul_type")->setDefaultValue("123");

		auto &mm = plan_root->planPool().add<aris::plan::ManualMove>();
		mm.command().findParam("ve")->setDefaultValue("{0.05,0.05,0.05,0.05,0.05,0.05}");
		mm.command().findParam("ae")->setDefaultValue("{10,10,10,10,10,10}");
		mm.command().findParam("de")->setDefaultValue("{10,10,10,10,10,10}");
		mm.command().findParam("eul_type")->setDefaultValue("123");
		mm.command().findParam("increase_count")->setDefaultValue("100");

		plan_root->planPool().add<aris::plan::GetXml>();
		plan_root->planPool().add<aris::plan::SetXml>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();

		return plan_root;
	}
}
