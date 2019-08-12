#include <algorithm>

#include"aris/robot/rokae.hpp"

using namespace aris::dynamic;

namespace aris::robot
{
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0833285350678632,   0.406351037260097,   -0.0643040958674858,   0.655755890051854,   -1.49540282814852,   -3.24773261925571
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 72.857 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 50 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 130.0 / 360 * 2 * PI,	50.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, - 84.0 / 360 * 2 * PI, - 188.0 / 360 * 2 * PI, - 170.0 / 360 * 2 * PI, - 117.0 / 360 * 2 * PI, - 360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1750.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 2500.0 / 360 * 2 * PI,
			};
			
			std::string xml_str =
				"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00\""
				" vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotion>";

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);

#ifndef WIN32
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif
		}

		return controller;
	};
	auto createModelRokaeXB4(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model> 
	{ 
		aris::dynamic::PumaParam param;
		param.d1 = 0.3295;
		param.a1 = 0.04;
		param.a2 = 0.275;
		param.d3 = 0.0;
		param.a3 = 0.025;
		param.d4 = 0.28;

		param.tool0_pe[2] = 0.078;

		param.iv_vec =
		{
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
			{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
			{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
			{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
			{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
		};

		param.mot_frc_vec =
		{
			{ 9.34994758321915, 7.80825641041495, 0.00000000000000 },
			{ 11.64080253106441, 13.26518528472506, 3.55567932576820 },
			{ 4.77014054273075, 7.85644357492508, 0.34445460269183 },
			{ 3.63141668516122, 3.35461524886318, 0.14824771620542 },
			{ 2.58310846982020, 1.41963212641879, 0.04855267273770 },
			{ 1.78373986219597, 0.31920640440152, 0.03381545544099 },
		};

		return aris::dynamic::createModelPuma(param);
	}
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
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
		rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

		auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
		mvaj.command().findParam("vel")->setDefaultValue("0.1");

		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();

		plan_root->planPool().add<aris::plan::GetXml>();
		plan_root->planPool().add<aris::plan::SetXml>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();

		return plan_root;
	}
	auto createRokaeXB4Interface()->std::string
	{
		return 
			"<InterfaceRoot >"
			"    <UiIdPoolObject >"
			"        <Id name=\"id_1\"  value=\"id_1\"/>"
			"        <Id name=\"id_2\"  value=\"id_2\"/>"
			"        <Id name=\"id_3\"  value=\"id_3\"/>"
			"        <Id name=\"id_4\"  value=\"id_4\"/>"
			"        <Id name=\"id_5\"  value=\"id_5\"/>"
			"        <Id name=\"id_6\"  value=\"id_6\"/>"
			"        <Id name=\"id_7\"  value=\"id_7\"/>"
			"        <Id name=\"id_8\"  value=\"id_8\"/>"
			"        <Id name=\"id_9\"  value=\"id_9\"/>"
			"        <Id name=\"id_10\"  value=\"id_10\"/>"
			"        <Id name=\"id_11\"  value=\"id_11\"/>"
			"        <Id name=\"id_12\"  value=\"id_12\"/>"
			"        <Id name=\"id_13\"  value=\"id_13\"/>"
			"        <Id name=\"id_14\"  value=\"id_14\"/>"
			"        <Id name=\"id_15\"  value=\"id_15\"/>"
			"        <Id name=\"id_16\"  value=\"id_16\"/>"
			"        <Id name=\"id_17\"  value=\"id_17\"/>"
			"        <Id name=\"id_18\"  value=\"id_18\"/>"
			"        <Id name=\"id_19\"  value=\"id_19\"/>"
			"        <Id name=\"id_20\"  value=\"id_20\"/>"
			"        <Id name=\"id_21\"  value=\"id_21\"/>"
			"        <Id name=\"id_22\"  value=\"id_22\"/>"
			"        <Id name=\"id_23\"  value=\"id_23\"/>"
			"        <Id name=\"id_24\"  value=\"id_24\"/>"
			"        <Id name=\"id_25\"  value=\"id_25\"/>"
			"        <Id name=\"id_26\"  value=\"id_26\"/>"
			"        <Id name=\"id_27\"  value=\"id_27\"/>"
			"        <Id name=\"id_28\"  value=\"id_28\"/>"
			"        <Id name=\"id_29\"  value=\"id_29\"/>"
			"    </UiIdPoolObject>"
			"    <CmdInterfacePoolObject>"
			"        <Panel text=\"Simulator\" id=\"id_20\">"
			"            <Button text=\"Enable\" id=\"id_21\" cmd=\"mm --stop;am --stop;ds;md;en;rc\" return_attached=\"\"/>"
			"            <Button text=\"Disable\" id=\"id_22\" cmd=\"mm --stop; am --stop; ds;\" return_attached=\"\"/>"
			"            <Button text=\"Reset\" id=\"id_23\" cmd=\"mm --stop; am --stop; rs; rc;\" return_attached=\"\"/>"
			"            <Button text=\"Home\" id=\"id_21\" cmd=\"mm --stop;am --stop;ds;md;en;hm -m=5 --method=17 --high_speed=200;md;en;rc\" return_attached=\"\"/>"
			"        </Panel>"
			"        <Panel text=\"Manual\" id=\"id_21\">"
			"            <Button text=\"Start\" id=\"id_18\" cmd=\"mm --stop; am --stop;mm --start\" return_attached=\"\"/>"
			"            <Button text=\"Stop\" id=\"id_19\" cmd=\"mm --stop; am --stop;mm --stop\" return_attached=\"\"/>"
			"            <Panel text=\"Translate\" id=\"id_8\">"
			"                <Button text=\"Heave+\" id=\"id_2\" repeat=\"0\" rate=\"50\" cmd=\"mm --z=1\" return_attached=\"\" width=\"80\"/>"
			"                <Button text=\"Heave-\" id=\"id_3\" repeat=\"0\" rate=\"50\"  cmd=\"mm --z=-1\" return_attached=\"\"/>"
			"                <Button text=\"Sway+\" id=\"id_4\" repeat=\"0\" rate=\"50\"  cmd=\"mm --x=1\" return_attached=\"\"/>"
			"                <Button text=\"Sway-\" id=\"id_5\" repeat=\"0\"  rate=\"50\" cmd=\"mm --x=-1\" return_attached=\"\"/>"
			"                <Button text=\"Surge+\" id=\"id_6\" repeat=\"0\"  rate=\"50\" cmd=\"mm --y=1\" return_attached=\"\"/>"
			"                <Button text=\"Surge-\" id=\"id_7\" repeat=\"0\"  rate=\"50\" cmd=\"mm --y=-1\" return_attached=\"\"/>"
			"            </Panel>"
			"            <Panel text=\"Rotate\" id=\"id_9\">"
			"                <Button text=\"Yaw+\" id=\"id_10\" repeat=\"0\"  rate=\"50\" cmd=\"mm --c=1\" return_attached=\"\"/>"
			"                <Button text=\"Yaw-\" id=\"id_11\" repeat=\"0\"  rate=\"50\" cmd=\"mm --c=-1\" return_attached=\"\"/>"
			"                <Button text=\"Roll+\" id=\"id_12\" repeat=\"0\"  rate=\"50\" cmd=\"mm --a=1\" return_attached=\"\"/>"
			"                <Button text=\"Roll-\" id=\"id_13\" repeat=\"0\"  rate=\"50\" cmd=\"mm --a=-1\" return_attached=\"\"/>"
			"                <Button text=\"Pitch+\" id=\"id_14\" repeat=\"0\"  rate=\"50\" cmd=\"mm --b=1\" return_attached=\"\"/>"
			"                <Button text=\"Pitch-\" id=\"id_15\" repeat=\"0\"  rate=\"50\" cmd=\"mm --b=-1\" return_attached=\"\"/>"
			"            </Panel>"
			"        </Panel>"
			"        <Panel text=\"Auto\" id=\"id_16\">"
			"            <Button text=\"Start\" id=\"id_16\" cmd=\"mm --stop; am --stop;am --start\" return_attached=\"\"/>"
			"            <Button text=\"Stop\" id=\"id_17\" cmd=\"mm --stop; am --stop;am --stop\" return_attached=\"\"/>"
			"        </Panel>"
			"    </CmdInterfacePoolObject>"
			"</InterfaceRoot>";
	}
}
