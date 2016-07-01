#include <iostream>
#include <aris.h>

#include "test_core_command.h"


using namespace aris::core;
const char xml_data[] =
"<HexapodIII>"
"	<Server ip = \"127.0.0.1\" port = \"5866\">"
"		<Commands type=\"command_pool_object\" default_child_type=\"command\">"
"			<start/>"
"			<stop/>"
"			<exit/>"
"			<en default_child_type=\"param\" default = \"all\">"
"				<all abbreviation = \"a\" />"
"				<first abbreviation = \"f\" />"
"				<second abbreviation = \"s\" />"
"				<motor abbreviation = \"m\" default = \"0\" />"
"				<physical_motor abbreviation = \"p\" default = \"0\" />"
"				<leg abbreviation = \"l\" default = \"0\" />"
"			</en>"
"			<ds default_child_type=\"param\" default = \"all\">"
"				<all abbreviation = \"a\" />"
"				<first abbreviation = \"f\" />"
"				<second abbreviation = \"s\" />"
"				<motor abbreviation = \"m\" default = \"0\" />"
"				<physical_motor abbreviation = \"p\" default = \"0\" />"
"				<leg abbreviation = \"l\" default = \"0\" />"
"			</ds>"
"			<hm default_child_type=\"param\" default = \"all\" >"
"				<all abbreviation = \"a\" />"
"				<first abbreviation = \"f\" />"
"				<second abbreviation = \"s\" />"
"				<motor abbreviation = \"m\" default = \"0\" />"
"				<physical_motor abbreviation = \"p\" default = \"0\" />"
"				<leg abbreviation = \"l\" default = \"0\" />"
"			</hm>"
"			<rc default = \"rc_param\">"
"				<rc_param type = \"group\" default_child_type=\"param\">"
"					<leg_param type = \"unique\" default_child_type=\"param\" default = \"all\">"
"						<all abbreviation = \"a\" />"
"						<first abbreviation = \"f\" />"
"						<second abbreviation = \"s\" />"
"						<leg abbreviation = \"l\" default = \"0\" />"
"					</leg_param>"
"					<t1 abbreviation = \"t\" default = \"3000\" />"
"					<t2 default = \"3000\" />"
"					<margin_offset abbreviation = \"m\" default = \"0.01\" />"
"				</rc_param>"
"			</rc>"
"			<wk default_child_type=\"param\" default = \"wk_param\">"
"				<wk_param type = \"group\" default_child_type=\"param\">"
"					<totalCount abbreviation = \"t\" default = \"3000\" />"
"					<n abbreviation = \"n\" default = \"1\" />"
"					<distance abbreviation = \"d\" default = \"0.5\" />"
"					<height abbreviation = \"h\" default = \"0.05\" />"
"					<alpha abbreviation = \"a\" default = \"0\" />"
"					<beta abbreviation = \"b\" default = \"0\" />"
"				</wk_param>"
"			</wk>"
"			<mv default_child_type=\"param\" default = \"mv_param\">"
"				<mv_param type = \"group\" default_child_type=\"param\" >"
"					<totalCount abbreviation = \"t\" default = \"10000\" />"
"					<mode_param type = \"unique\" default_child_type=\"param\" default = \"pin_param\">"
"						<pee_param type = \"group\" default_child_type=\"param\">"
"							<x abbreviation = \"x\" default = \"0\" />"
"							<y abbreviation = \"y\" default = \"0\" />"
"							<z abbreviation = \"z\" default = \"0\" />"
"							<A abbreviation = \"A\" default = \"0\" />"
"							<B abbreviation = \"B\" default = \"0\" />"
"							<C abbreviation = \"C\" default = \"0\" />"
"						</pee_param>"
"						<pin_param type = \"group\" default_child_type=\"param\">"
"							<motion_param type = \"unique\" default_child_type=\"param\" default = \"all\">"
"								<all abbreviation = \"a\" />"
"								<motion_id abbreviation = \"m\" default = \"0\" />"
"								<physical_id abbreviation = \"p\" default = \"0\" />"
"								<slave_id abbreviation = \"s\" default = \"0\" />"
"							</motion_param>"
"							<velocity abbreviation = \"v\" default = \"0\" />"
"							<force abbreviation = \"f\" default = \"no\" />"
"						</pin_param>"
"					</mode_param>"
"				</mv_param>"
"			</mv>"
"		</Commands>"
"	</Server>"
"</HexapodIII>";

void test_command()
{
	try
	{
		XmlDocument doc;
        doc.Parse(xml_data);
        //doc.LoadFile("/usr/aris/robot/resource/robot_motion.xml");

		CommandParser parser;
		parser.loadXml(doc);

        /*
        getrobothelp(parser);
        std::cout << std::endl;
        for (auto &command : parser.commandPool())
        {
            getcommandhelp(parser, command.name());
            //std::cout << getHelpStream(parser, command.name()).str();
            std::cout << "\n" << std::endl;
        }
        */

        std::vector<std::string> cmd_string_vec{"en --all", "en -m=0 --all", "en -motor=0", "en --moto=0", "rc -t=3000","ds","start" };

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

void getcommandhelp(aris::core::CommandParser &parser,std::string commandname)
{

    /*
    //无换行处理
    auto command = parser.commandPool().findByName(commandname);
    std::map<std::string, std::string> default_params;
    command->addDefaultParam(default_params);
    auto param_map = parser.commandPool().findByName(commandname)->param_map();

    std::cout << commandname << " : " << command->help() << std::endl;
    if (param_map.size() != 0)
    {
        int paramPrintLength = std::max_element(param_map.begin(), param_map.end(), [](decltype(*param_map.begin()) a, decltype(*param_map.begin()) b)
        {
            return a.second->name().length() < b.second->name().length();
        })->second->name().length() + 6;
        std::cout << '\n' << "  usage: [command] [param1]=[value1] [param2]=[value1]...." << std::endl;
        std::cout << '\n' << "  default param:" << std::endl;
        for (auto &param : default_params)
        {
            std::cout << std::string(paramPrintLength - param.first.length(), ' ') << param.first << " : " << param.second << std::endl;
        }
        std::cout << '\n' << "  params note: [param] : [abbr]  [note]" << std::endl;
        for (auto &param : param_map)
        {
            std::cout << std::string(paramPrintLength - param.second->name().length(), ' ') << param.second->name() << " : " << param.second->abbreviation() << "  " << param.second->help() << std::endl;
        }
    }
    */
    /*
    int maxPrintLength = 55;
    auto command = parser.commandPool().findByName(commandname);
    std::map<std::string, std::string> default_params;
    command->addDefaultParam(default_params);
    auto param_map = parser.commandPool().findByName(commandname)->param_map();

    int commandLength = command->help().length();
    int count = 0;
    std::cout << commandname << " : ";
    while (commandLength>maxPrintLength)
    {
        std::cout << command->help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
        std::cout << "\n" << std::string(commandname.length() + 2, ' ');
        count += 1;
        commandLength -= maxPrintLength;
    }
    std::cout << command->help().substr(maxPrintLength*count, std::string::npos) << "\n";


    if (param_map.size() != 0)
    {
        std::cout << "\n  usage: [command] [param1]=[value1] [param2]=[value2]....\n";
        int paramPrintLength = std::max_element(param_map.begin(), param_map.end(), [](decltype(*param_map.begin()) a, decltype(*param_map.begin()) b)
        {
            return a.second->name().length() < b.second->name().length();
        })->second->name().length() + 6;
        std::cout << "\n  default param:\n";
        for (auto &param : default_params)
        {
            std::cout << std::string(paramPrintLength - param.first.length(), ' ') << param.first << " : " << param.second << "\n";
        }
        std::cout << "\n  params help: [param] : [abbr]  [help]\n";
        for (auto &param : param_map)
        {
            int helpStringLength = param.second->help().length();
            int count = 0;
            std::cout << std::string(paramPrintLength - param.second->name().length(), ' ') << param.second->name() << " : " << param.second->abbreviation() << "  ";
            while (helpStringLength>maxPrintLength)
            {
                std::cout << param.second->help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
                std::cout << "\n" << std::string(paramPrintLength + 6, ' ');
                count += 1;
                helpStringLength -= maxPrintLength;
            }
            std::cout << param.second->help().substr(maxPrintLength*count, std::string::npos) << "\n";
        }
    }
*/
}

void getrobothelp(aris::core::CommandParser &parser)
{
    /*
    //无换行处理
    std::cout << "all command: " << std::endl;
    int commandPrintLength = std::max_element(parser.commandPool().begin(), parser.commandPool().end(), [](decltype(*parser.commandPool().begin()) a, decltype(*parser.commandPool().begin()) b)
    {
        return a.name().length() < b.name().length();
    })->name().length() + 2;
    for (auto &command : parser.commandPool())
    {
        std::cout << std::string(commandPrintLength - command.name().length(), ' ') << command.name() << " : " << command.help() << std::endl;
    }
    std::cout << "\nAttention:" << std::endl;
    std::cout << "the param '--help(-h)' can get more help about the specify command. etc: en -h " << std::endl;
    */

    /*
    int maxPrintLength = 55;
    std::cout << "all command: " << std::endl;
    int commandPrintLength = std::max_element(parser.commandPool().begin(), parser.commandPool().end(), [](decltype(*parser.commandPool().begin()) a, decltype(*parser.commandPool().begin()) b)
    {
        return a.name().length() < b.name().length();
    })->name().length() + 2;
    for (auto &command : parser.commandPool())
    {
        int helpStringLength = command.help().length();
        int count = 0;
        std::cout << std::string(commandPrintLength - command.name().length(), ' ') << command.name() << " : ";
        while (helpStringLength>maxPrintLength)
        {
            std::cout << command.help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
            std::cout << "\n" << std::string(commandPrintLength + 2, ' ');
            count += 1;
            helpStringLength -= maxPrintLength;
        }
        std::cout << command.help().substr(maxPrintLength*count, std::string::npos) << std::endl;
    }
    std::cout << "\nAttention:" << std::endl;
    std::cout << "the param '--help(-h)' can get more help about the specify command. etc: en -h " << std::endl;
    */
}

std::stringstream getHelpStream(aris::core::CommandParser &parser, std::string commandname)
{
    /*
    int maxPrintLength = 55;
    std::stringstream helpstream{};
    auto command = parser.commandPool().findByName(commandname);
    std::map<std::string, std::string> default_params;
    command->addDefaultParam(default_params);
    auto param_map = parser.commandPool().findByName(commandname)->param_map();

    int commandLength = command->help().length();
    int count = 0;
    helpstream << commandname << " : ";
    while (commandLength>maxPrintLength)
    {
        helpstream << command->help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
        helpstream << "\n" << std::string(commandname.length() + 2, ' ');
        count += 1;
        commandLength -= maxPrintLength;
    }
    helpstream << command->help().substr(maxPrintLength*count, std::string::npos) << "\n";


    if (param_map.size() != 0)
    {
        helpstream << "\n  usage: [command] [param1]=[value1] [param2]=[value2]....\n";
        int paramPrintLength = std::max_element(param_map.begin(), param_map.end(), [](decltype(*param_map.begin()) a, decltype(*param_map.begin()) b)
        {
            return a.second->name().length() < b.second->name().length();
        })->second->name().length() + 6;
        helpstream << "\n  default param:\n";
        for (auto &param : default_params)
        {
            helpstream << std::string(paramPrintLength - param.first.length(), ' ') << param.first << " : " << param.second << "\n";
        }
        helpstream << "\n  params help: [param] : [abbr]  [help]\n";
        for (auto &param : param_map)
        {
            int helpStringLength = param.second->help().length();
            int count = 0;
            helpstream << std::string(paramPrintLength - param.second->name().length(), ' ') << param.second->name() << " : " << param.second->abbreviation() << "  ";
            while (helpStringLength>maxPrintLength)
            {
                helpstream << param.second->help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
                helpstream << "\n" << std::string(paramPrintLength + 6, ' ');
                count += 1;
                helpStringLength -= maxPrintLength;
            }
            helpstream << param.second->help().substr(maxPrintLength*count, std::string::npos) << "\n";
        }
    }

    return helpstream;
    */
}
