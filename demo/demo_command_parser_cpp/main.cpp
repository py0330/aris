/// \example demo_command_parser_cpp/main.cpp
/// 本例子展示基于cpp解析命令字符串的过程:
///

#include <iostream>
#include "aris.h"

int main()
{
	// 添加CommandParser //
	aris::core::Root root;
	auto &parser = root.add<aris::core::CommandParser>("parser");
	
	// 添加enable命令 //
	auto &enable = parser.commandPool().add<aris::core::Command>("enable", "", "");

	// 添加命令各参数节点 //
	auto &group = enable.add<aris::core::GroupParam>("group", "");
	auto &unique1 = group.add<aris::core::UniqueParam>("unique1", "", "");
	auto &unique2 = group.add<aris::core::UniqueParam>("unique2", "position", "");
	auto &all = unique1.add<aris::core::Param>("all", "", "", 'a');
	auto &motion = unique1.add<aris::core::Param>("motion", "0", "", 'm');
	auto &position = unique2.add<aris::core::Param>("position", "", "", 'p');
	auto &velocity = unique2.add<aris::core::Param>("velocity", "", "", 'v');
	auto &current = unique2.add<aris::core::Param>("current", "", "", 0);

	// 结果：命令与参数集 //
	std::string cmd;
	std::map<std::string, std::string> params;
	
	// 构造输入的命令字符串 //
	std::vector<std::string> cmd_strs
	{
		"enable --all --velocity",
		"enable -m=1 -p",
		"enable ap",
		"enable -a",
		"enable -a -m=1 --position",
		"enable -p"
	};

	// parse以上命令，前4个成功，后2个失败 //
	for (auto &cmd_str : cmd_strs)
	{
		try
		{
			// parse //
			parser.parse(cmd_str, cmd, params);

			// 打印命令和参数 //
			std::cout << "-----------" << "parsing" << " -----------" << std::endl;
			std::cout << "string : " << cmd_str << std::endl;
			std::cout << "cmd    : " << cmd << std::endl << "params : " << std::endl;
			for (auto &p : params)
			{
				std::cout << std::setfill(' ') << std::setw(10) << p.first << " : " << p.second << std::endl;
			}
			std::cout << "-----------" << "finished" << "-----------" << std::endl;
		}
		catch (std::exception &e)
		{
			// 打印错误信息 //
			std::cout << "-----------" << "parsing failed" << "  -----------" << std::endl;
			std::cout << e.what() << std::endl;
			std::cout << "-----------" << "parsing finished" << "-----------" << std::endl;
		}
	}
	
	std::cout << "demo_command_parser_cpp finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

