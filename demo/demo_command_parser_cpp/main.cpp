/// \example demo_command_parser_cpp/main.cpp
/// 本例子展示基于cpp解析命令字符串的过程:
///

#include <iostream>
#include "aris.hpp"

int main()
{
	// 添加CommandParser //
	aris::core::CommandParser parser;
	
	// 添加enable命令 //
	auto &enable = parser.commandPool().add<aris::core::Command>("enable", "");

	// 添加命令各参数节点 //
	auto &group = enable.add<aris::core::GroupParam>("group");
	auto &unique1 = group.add<aris::core::UniqueParam>("unique1", "");
	auto &unique2 = group.add<aris::core::UniqueParam>("unique2", "position");
	auto &all = unique1.add<aris::core::Param>("all", "", 'a');
	auto &motion = unique1.add<aris::core::Param>("motion", "0", 'm');
	auto &position = unique2.add<aris::core::Param>("position", "", 'p');
	auto &velocity = unique2.add<aris::core::Param>("velocity", "", 'v');
	auto &current = unique2.add<aris::core::Param>("current", "", 0);

	// 和用户进行交互 //
	for (;;)
	{
		std::cout << "please input command, you can input \"exit\" to leave program:" << std::endl;
		
		// 获取命令字符串 //
		std::string cmd_string;
		std::getline(std::cin, cmd_string);

		// 如果是exit，那么退出 //
		if (cmd_string == "exit")break;

		// 以下变量用来保存分析的结果，包括命令与参数集 //
		std::string cmd;
		std::map<std::string, std::string> params;

		// parse //
		try
		{
			parser.parse(cmd_string, cmd, params);

			// 打印命令和参数 //
			std::cout << "------------------------------------------" << std::endl;
			std::cout << "cmd    : " << cmd << std::endl << "params : " << std::endl;
			for (auto &p : params)
			{
				std::cout << std::setfill(' ') << std::setw(10) << p.first << " : " << p.second << std::endl;
			}
			std::cout << "------------------------------------------" << std::endl << std::endl;
		}
		catch (std::exception &e)
		{
			// 打印错误信息 //
			std::cout << "------------------------------------------" << std::endl;
			std::cout << e.what() << std::endl;
			std::cout << "------------------------------------------" << std::endl << std::endl;
		}
	}



	




	// 构造输入的命令字符串 //
	//std::vector<std::string> cmd_strs
	//{
	//	"enable --all --velocity",
	//	"enable -m=1 -p",
	//	"enable ap",
	//	"enable -a",
	//	"enable -a -m=1 --position",
	//	"enable -p"
	//};

	std::cout << "demo_command_parser_cpp finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

