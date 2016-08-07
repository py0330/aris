#include <iostream>
#include <aris.h>

#include "test_core_command.h"

using namespace aris::core;
const char xml_data[] =
"<root>"
"    <command_parser type=\"CommandParser\">"
"        <command_pool type=\"CommandPoolObject\" default_child_type=\"Command\">"
"            <start/>"
"            <stop/>"
"            <exit/>"
"            <en default_child_type=\"Param\" default=\"all\">"
"                <all abbreviation=\"a\"/>"
"                <first abbreviation=\"f\"/>"
"                <second abbreviation=\"s\"/>"
"                <motion_id abbreviation=\"m\" default=\"0\"/>"
"                <physical_id abbreviation=\"p\" default=\"0\"/>"
"                <leg abbreviation=\"l\" default=\"0\"/>"
"            </en>"
"            <ds default_child_type=\"Param\" default=\"all\">"
"                <all abbreviation=\"a\"/>"
"                <first abbreviation=\"f\"/>"
"                <second abbreviation=\"s\"/>"
"                <motion_id abbreviation=\"m\" default=\"0\"/>"
"                <physical_id abbreviation=\"p\" default=\"0\"/>"
"                <leg abbreviation=\"l\" default=\"0\"/>"
"            </ds>"
"            <hm default_child_type=\"Param\" default=\"all\">"
"                <all abbreviation=\"a\"/>"
"                <first abbreviation=\"f\"/>"
"                <second abbreviation=\"s\"/>"
"                <motion_id abbreviation=\"m\" default=\"0\"/>"
"                <physical_id abbreviation=\"p\" default=\"0\"/>"
"                <leg abbreviation=\"l\" default=\"0\"/>"
"            </hm>"
"            <test default_child_type=\"Param\" default=\"all\">"
"                <all abbreviation=\"a\"/>"
"                <motion_id abbreviation=\"m\" default=\"0\"/>"
"                <physical_id abbreviation=\"p\" default=\"0\"/>"
"            </test>"
"            <rc default=\"rc_param\">"
"                <rc_param type=\"GroupParam\" default_child_type=\"Param\">"
"                    <leg_param type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
"                        <all abbreviation=\"a\"/>"
"                        <first abbreviation=\"f\"/>"
"                        <second abbreviation=\"s\"/>"
"                        <leg abbreviation=\"l\" default=\"0\"/>"
"                    </leg_param>"
"                    <t1 abbreviation=\"t\" default=\"3000\"/>"
"                    <t2 default=\"3000\"/>"
"                    <margin_offset abbreviation=\"m\" default=\"0.01\"/>"
"                </rc_param>"
"            </rc>"
"            <wk default_child_type=\"Param\" default=\"wk_param\">"
"                <wk_param type=\"GroupParam\" default_child_type=\"Param\">"
"                    <totalCount abbreviation=\"t\" default=\"3000\"/>"
"                    <n abbreviation=\"n\" default=\"1\"/>"
"                    <distance abbreviation=\"d\" default=\"0.5\"/>"
"                    <height abbreviation=\"h\" default=\"0.05\"/>"
"                    <alpha abbreviation=\"a\" default=\"0\"/>"
"                    <beta abbreviation=\"b\" default=\"0\"/>"
"                </wk_param>"
"            </wk>"
"        </command_pool>"
"    </command_parser>"
"</root>";


void test_core_command()
{
	try
	{
		aris::core::XmlDocument xml_doc;
        xml_doc.Parse(xml_data);

		aris::core::Root root;
		root.registerChildType<aris::core::Param>();
		root.registerChildType<aris::core::UniqueParam>();
		root.registerChildType<aris::core::GroupParam>();
		root.registerChildType<aris::core::Command>();
		root.registerChildType<aris::core::ObjectPool<aris::core::Command> >();
		root.registerChildType<aris::core::CommandParser>();

		root.loadXml(xml_doc);

		auto& parser = static_cast<aris::core::CommandParser&>(*root.findByName("command_parser"));

		
		//get all command of the system  
        std::cout << parser.help() << std::endl;
		//display all command help information in detail
		for (auto &command : parser.commandPool())
		{
            std::cout << command.help()<<std::endl;
		}

		//test the command param
		std::vector<std::string> cmd_string_vec{ "en --all", "en -m=0 --all", "en -motor=0", "en --moto=0", "rc -t=3000","ds","start" };

		for (auto &cmd_string : cmd_string_vec)
		{
			try 
			{
				std::string cmd;
				std::map<std::string, std::string> params;
				parser.parse(cmd_string, cmd, params);

				std::cout << cmd << std::endl;
				int paramPrintLength;
				if (params.empty())
				{
					paramPrintLength = 2;
				}
				else
				{
					paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
					{
						return a.first.length() < b.first.length();
					})->first.length() + 2;
				}
				for (auto &i : params)
				{
					std::cout << std::string(paramPrintLength - i.first.length(), ' ') << i.first << " : " << i.second << std::endl;
				}

				std::cout << std::endl;
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl << std::endl;
			}
		}
		
	
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
}

