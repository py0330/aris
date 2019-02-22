/// \example demo_command_parser_xml/main.cpp
/// 本例子展示基于xml解析命令字符串的过程，xml文件位于：安装目录/resource/demo_command_parser_xml/command.xml，内容如下：
/// \include demo_command_parser_xml/resource/command.xml
/// 以下为C++源码
///

#include <iostream>
#include "aris.hpp"

int main()
{
	// 读取xml文档 //
	aris::core::CommandParser parser;
	parser.loadXmlFile(ARIS_INSTALL_PATH + std::string("/resource/demo_command_parser_xml/command.xml"));

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
	
	std::cout << "demo_command_parser_xml finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

