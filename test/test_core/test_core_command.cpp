#include <iostream>
#include <aris/core/core.hpp>
#include "test_core_command.h"

using namespace aris::core;

void test_command_xml()
{
	try
	{
		const char xml_data[] =
			"<CommandParser>"
			"	<CommandPoolObject>"
			"		<Command name=\"tt\" default=\"ap0\">"
			"			<Param name=\"ap0\" abbreviation=\"a\"/>"
			"			<UniqueParam default=\"ap1\">"
			"				<Param name=\"ap1\" default=\"0\"/>"
			"				<GroupParam>"
			"					<Param name=\"ap2\" default=\"1\"/>"
			"					<Param name=\"bp2\" default=\"2\" abbreviation=\"b\"/>"
			"				</GroupParam>"
			"			</UniqueParam>"
			"			<GroupParam>"
			"				<Param name=\"cp1\" default=\"0\"/>"
			"				<GroupParam>"
			"					<Param name=\"cp2\" default=\"3\" abbreviation=\"c\"/>"
			"					<Param name=\"dp2\" default=\"4\" abbreviation=\"d\"/>"
			"				</GroupParam>"
			"				<UniqueParam>"
			"					<Param name=\"ep2\" default=\"5\" abbreviation=\"e\"/>"
			"					<Param name=\"fp2\" default=\"6\"/>"
			"				</UniqueParam>"
			"			</GroupParam>"
			"			<UniqueParam default=\"fp1\">"
			"				<Param name=\"fp1\" default=\"0\" abbreviation=\"f\"/>"
			"				<GroupParam>"
			"					<Param name=\"gp2\" default=\"1\"/>"
			"					<Param name=\"hp2\" default=\"2\"/>"
			"				</GroupParam>"
			"			</UniqueParam>"
			"		</Command>"
			"		<Command name=\"en\" default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"first\" abbreviation=\"f\"/>"
			"			<Param name=\"second\" abbreviation=\"s\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"leg\" abbreviation=\"l\" default=\"0\"/>"
			"		</Command>"
			"		<Command name=\"rc\" default=\"rc_param\">"
			"			<GroupParam name=\"rc_param\">"
			"				<UniqueParam default=\"all\">"
			"					<Param name=\"all\" abbreviation=\"a\"/>"
			"					<Param name=\"first\" abbreviation=\"f\"/>"
			"					<Param name=\"second\" abbreviation=\"s\"/>"
			"					<Param name=\"leg\" abbreviation=\"l\" default=\"0\"/>"
			"				</UniqueParam>"
			"				<Param name=\"t1\" abbreviation=\"t\" default=\"3000\"/>"
			"				<Param name=\"t2\" default=\"3000\"/>"
			"				<Param name=\"margin_offset\" abbreviation=\"m\" default=\"0.01\"/>"
			"			</GroupParam>"
			"			<GroupParam>"
			"				<UniqueParam default=\"group\">"
			"					<GroupParam name=\"group\">"
			"						<Param name=\"group2\"/>"
			"					</GroupParam>"
			"					<Param name=\"first2\"/>"
			"					<Param name=\"second2\"/>"
			"					<Param name=\"leg2\"/>"
			"				</UniqueParam>"
			"				<Param name=\"t3\" default=\"3000\"/>"
			"				<Param name=\"t4\" default=\"3000\"/>"
			"				<Param name=\"margin_offset2\" default=\"0.01\"/>"
			"			</GroupParam>"
			"		</Command>"
			"	</CommandPoolObject>"
			"</CommandParser>";

		aris::core::CommandParser parser;
		aris::core::fromXmlString(parser, xml_data);
		parser.init();

		std::string cmd_string;
		std::string_view cmd, cmd_result;
		std::map<std::string_view, std::string_view> param, param_result;
		using ParamPair = std::pair<std::string_view, std::string_view>;
		
		cmd_string = "en --all={123, 0}";
		cmd = "en";
		param = { ParamPair("all", "{123, 0}") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en --all={123, 0";
		cmd = "en";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;}
		catch (std::exception &e) { };

		cmd_string = "en --all";
		cmd = "en";
		param = { ParamPair("all", "") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en --motion_id=1";
		cmd = "en";
		param = { ParamPair("motion_id", "1") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en -m=3";
		cmd = "en";
		param = { ParamPair("motion_id", "3") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en -m";
		cmd = "en";
		param = { ParamPair("motion_id", "0") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "en ";
		cmd = "en";
		param = { ParamPair("all", "") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "rc ";
		cmd = "rc";
		param = { ParamPair("all", ""),ParamPair("t1", "3000"),ParamPair("t2", "3000"),ParamPair("margin_offset", "0.01") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;
		

		// following are correct cmd string examples //
		cmd_string = "tt ";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt -a";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2";
		cmd = "tt";
		param = { ParamPair("ap2", "1"), ParamPair("bp2", "2") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2=";
		cmd = "tt";
		param = { ParamPair("ap2", ""), ParamPair("bp2", "2") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt cde";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt ce";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;


		// following are wrong cmd string examples //
		cmd_string = "tt -a -b";
		cmd = "tt";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) { };

		cmd_string = "tt --ap1 --ap2";
		cmd = "tt";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) { };

		cmd_string = "tt --ap1 --ap1";
		cmd = "tt";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
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
		auto &tt = parser.commandPool().emplace_back(aris::core::Command("tt", "ap0"));
		auto &ap0 = tt.add<aris::core::Param>("ap0", "", 'a');
		auto &bu0 = tt.add<aris::core::UniqueParam>("bu0", "ap1");
		auto &ap1 = bu0.add<aris::core::Param>("ap1", "0");
		auto &bg1 = bu0.add<aris::core::GroupParam>("bg1");
		auto &ap2 = bg1.add<aris::core::Param>("ap2", "1");
		auto &bp2 = bg1.add<aris::core::Param>("bp2", "2", 'b');
		auto &cg0 = tt.add<aris::core::GroupParam>("cg0");
		auto &cp1 = cg0.add<aris::core::Param>("cp1", "0");
		auto &dg1 = cg0.add<aris::core::GroupParam>("dg1");
		auto &cp2 = dg1.add<aris::core::Param>("cp2", "3", 'c');
		auto &dp2 = dg1.add<aris::core::Param>("dp2", "4", 'd');
		auto &eu1 = cg0.add<aris::core::UniqueParam>("eu1", "");
		auto &ep2 = eu1.add<aris::core::Param>("ep2", "5", 'e');
		auto &fp2 = eu1.add<aris::core::Param>("fp2", "6");
		auto &du0 = tt.add<aris::core::UniqueParam>("du0", "fp1");
		auto &fp1 = du0.add<aris::core::Param>("fp1", "0", 'f');
		auto &gg1 = du0.add<aris::core::GroupParam>("gg1");
		auto &gp2 = gg1.add<aris::core::Param>("gp2", "1");
		auto &hp2 = gg1.add<aris::core::Param>("hp2", "2");

		parser.init();

		std::string cmd_string;
		std::string_view cmd, cmd_result;
		std::map<std::string_view, std::string_view> param, param_result;
		using ParamPair = std::pair<std::string_view, std::string_view>;

		// following are correct cmd string examples //
		cmd_string = "tt ";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt -a";
		cmd = "tt";
		param = { ParamPair("ap0", "") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2";
		cmd = "tt";
		param = { ParamPair("ap2", "1"), ParamPair("bp2", "2") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt --ap2=";
		cmd = "tt";
		param = { ParamPair("ap2", ""), ParamPair("bp2", "2") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt cde";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;

		cmd_string = "tt ce";
		cmd = "tt";
		param = { ParamPair("cp1", "0"), ParamPair("cp2", "3"), ParamPair("dp2", "4"), ParamPair("ep2", "5") };
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); }
		catch (std::exception &e) { std::cout << "cmd parse failed:\"" << e.what() << std::endl; };
		if (!(cmd_result == cmd && param_result.size() == param.size() && std::equal(param.begin(), param.end(), param_result.begin())))std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl;


		// following are wrong cmd string examples //
		cmd_string = "tt -a -b";
		cmd = "tt";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) {};

		cmd_string = "tt --ap1 --ap2";
		cmd = "tt";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
		catch (std::exception &) {};

		cmd_string = "tt --ap1 --ap1";
		cmd = "tt";
		try { std::tie(cmd_result, param_result) = parser.parse(cmd_string); std::cout << "cmd parse failed \"" + cmd_string + "\"" << std::endl; }
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

