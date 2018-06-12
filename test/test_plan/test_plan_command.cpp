#include <iostream>
#include <aris_core.h>
#include <aris_plan.h>

#include "test_plan_command.h"

using namespace aris::core;

void test_command_xml()
{
	try
	{
		const char xml_data[] =
			"<command_parser type=\"CommandParser\">"
			"	<command_pool type=\"CommandPoolObject\" default_child_type=\"Command\">"
			"		<tt default_child_type=\"Param\" default=\"ap0\">"
			"			<ap0 abbreviation=\"a\"/>"
			"			<bu0 type=\"UniqueParam\" default_child_type=\"Param\" default=\"ap1\">"
			"				<ap1 default=\"0\"/>"
			"				<bg1 type=\"GroupParam\" default_child_type=\"Param\">"
			"					<ap2 default=\"1\"/>"
			"					<bp2 default=\"2\" abbreviation=\"b\"/>"
			"				</bg1>"
			"			</bu0>"
			"			<cg0 type=\"GroupParam\" default_child_type=\"Param\" default=\"bg1\">"
			"				<cp1 default=\"0\"/>"
			"				<dg1 type=\"GroupParam\" default_child_type=\"Param\">"
			"					<cp2 default=\"3\" abbreviation=\"c\"/>"
			"					<dp2 default=\"4\" abbreviation=\"d\"/>"
			"				</dg1>"
			"				<eu1 type=\"UniqueParam\" default_child_type=\"Param\">"
			"					<ep2 default=\"5\" abbreviation=\"e\"/>"
			"					<fp2 default=\"6\"/>"
			"				</eu1>"
			"			</cg0>"
			"			<du0 type=\"UniqueParam\" default_child_type=\"Param\" default=\"fp1\">"
			"				<fp1 default=\"0\" abbreviation=\"f\"/>"
			"				<gg1 type=\"GroupParam\" default_child_type=\"Param\">"
			"					<gp2 default=\"1\"/>"
			"					<hp2 default=\"2\"/>"
			"				</gg1>"
			"			</du0>"
			"		</tt>"
			"		<en default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<first abbreviation=\"f\"/>"
			"			<second abbreviation=\"s\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<leg abbreviation=\"l\" default=\"0\"/>"
			"		</en>"
			"		<rc default=\"rc_param\">"
			"			<rc_param type=\"GroupParam\" default_child_type=\"Param\">"
			"				<leg_param type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"					<all abbreviation=\"a\"/>"
			"					<first abbreviation=\"f\"/>"
			"					<second abbreviation=\"s\"/>"
			"					<leg abbreviation=\"l\" default=\"0\"/>"
			"				</leg_param>"
			"				<t1 abbreviation=\"t\" default=\"3000\"/>"
			"				<t2 default=\"3000\"/>"
			"				<margin_offset abbreviation=\"m\" default=\"0.01\"/>"
			"			</rc_param>"
			"			<rc_param2 type=\"GroupParam\" default_child_type=\"Param\">"
			"				<leg_param2 type=\"UniqueParam\" default_child_type=\"Param\" default=\"group\">"
			"					<group type=\"GroupParam\">"
			"						<group2 type=\"GroupParam\"/>"
			"					</group>"
			"					<first2/>"
			"					<second2/>"
			"					<leg2/>"
			"				</leg_param2>"
			"				<t3 default=\"3000\"/>"
			"				<t4 default=\"3000\"/>"
			"				<margin_offset2 default=\"0.01\"/>"
			"			</rc_param2>"
			"		</rc>"
			"	</command_pool>"
			"</command_parser>";

		aris::core::CommandParser parser;
		parser.loadXmlStr(xml_data);

		std::string cmd_string, cmd, cmd_result;
		std::map<std::string, std::string> param, param_result;
		using ParamPair = std::pair<std::string, std::string>;
		
		cmd_string = "en --all";
		cmd = "en";
		param = { ParamPair("all", "") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en --motion_id=1";
		cmd = "en";
		param = { ParamPair("motion_id", "1") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en -m=3";
		cmd = "en";
		param = { ParamPair("motion_id", "3") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en -m";
		cmd = "en";
		param = { ParamPair("motion_id", "0") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en ";
		cmd = "en";
		param = { ParamPair("all", "") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "rc ";
		cmd = "rc";
		param = { ParamPair("all", ""),ParamPair("t1", "3000"),ParamPair("t2", "3000"),ParamPair("margin_offset", "0.01") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;
		

		// following are correct cmd string examples //
		cmd_string = "tt ";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt -a";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2";
		cmd = "tt";
		param = { ParamPair("ap2", "1"), ParamPair("bp2", "2") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2=";
		cmd = "tt";
		param = { ParamPair("ap2", ""), ParamPair("bp2", "2") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt cde";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt ce";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;


		// following are wrong cmd string examples //
		cmd_string = "tt -a -b";
		cmd = "tt";
		try { parser.parse(cmd_string, cmd_result, param_result); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) { };

		cmd_string = "tt --ap1 --ap2";
		cmd = "tt";
		try { parser.parse(cmd_string, cmd_result, param_result); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) { };

		cmd_string = "tt --ap1 --ap1";
		cmd = "tt";
		try { parser.parse(cmd_string, cmd_result, param_result); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) {};
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_command_code()
{
	try
	{
		aris::core::CommandParser parser("parser");
		auto &tt = parser.commandPool().add<aris::core::Command>("tt", "ap0", "");
		auto &ap0 = tt.add<aris::core::Param>("ap0", "", "", 'a');
		auto &bu0 = tt.add<aris::core::UniqueParam>("bu0", "ap1", "");
		auto &ap1 = bu0.add<aris::core::Param>("ap1", "0", "");
		auto &bg1 = bu0.add<aris::core::GroupParam>("bg1", "");
		auto &ap2 = bg1.add<aris::core::Param>("ap2", "1", "");
		auto &bp2 = bg1.add<aris::core::Param>("bp2", "2", "", 'b');
		auto &cg0 = tt.add<aris::core::GroupParam>("cg0", "");
		auto &cp1 = cg0.add<aris::core::Param>("cp1", "0", "");
		auto &dg1 = cg0.add<aris::core::GroupParam>("dg1", "");
		auto &cp2 = dg1.add<aris::core::Param>("cp2", "3", "", 'c');
		auto &dp2 = dg1.add<aris::core::Param>("dp2", "4", "", 'd');
		auto &eu1 = cg0.add<aris::core::UniqueParam>("eu1", "", "");
		auto &ep2 = eu1.add<aris::core::Param>("ep2", "5", "", 'e');
		auto &fp2 = eu1.add<aris::core::Param>("fp2", "6", "");
		auto &du0 = tt.add<aris::core::UniqueParam>("du0", "fp1", "");
		auto &fp1 = du0.add<aris::core::Param>("fp1", "0", "", 'f');
		auto &gg1 = du0.add<aris::core::GroupParam>("gg1", "");
		auto &gp2 = gg1.add<aris::core::Param>("gp2", "1", "");
		auto &hp2 = gg1.add<aris::core::Param>("hp2", "2", "");

		std::string cmd_string, cmd, cmd_result;
		std::map<std::string, std::string> param, param_result;
		using ParamPair = std::pair<std::string, std::string>;

		// following are correct cmd string examples //
		cmd_string = "tt ";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt -a";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2";
		cmd = "tt";
		param = { ParamPair("ap2", "1"), ParamPair("bp2", "2") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2=";
		cmd = "tt";
		param = { ParamPair("ap2", ""), ParamPair("bp2", "2") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt cde";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt ce";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { parser.parse(cmd_string, cmd_result, param_result); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;


		// following are wrong cmd string examples //
		cmd_string = "tt -a -b";
		cmd = "tt";
		try { parser.parse(cmd_string, cmd_result, param_result); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) {};

		cmd_string = "tt --ap1 --ap2";
		cmd = "tt";
		try { parser.parse(cmd_string, cmd_result, param_result); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) {};

		cmd_string = "tt --ap1 --ap1";
		cmd = "tt";
		try { parser.parse(cmd_string, cmd_result, param_result); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) {};
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}


void test_command()
{
	std::cout << std::endl << "-----------------test command---------------------" << std::endl;
	test_command_xml();
	test_command_code();
	std::cout << "-----------------test command finished------------" << std::endl << std::endl;
}

