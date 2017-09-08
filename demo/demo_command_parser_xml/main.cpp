/// \example demo_command_parser_xml/main.cpp
/// 本例子展示基于xml解析命令字符串的过程，xml文件位于：安装目录/resource/demo_command_parser_xml/command.xml，内容如下：
/// \include demo_command_parser_xml/resource/command.xml
/// 以下为C++源码
///

#include <iostream>
#include "aris.h"

int main()
{
	// 注册命令解析所需要的类 //
	aris::core::Root root;
	root.registerChildType<aris::core::Param>();
	root.registerChildType<aris::core::UniqueParam>();
	root.registerChildType<aris::core::GroupParam>();
	root.registerChildType<aris::core::Command>();
	root.registerChildType<aris::core::ObjectPool<aris::core::Command> >();
	root.registerChildType<aris::core::CommandParser>();
	
	// 读取xml文档 //
	aris::core::XmlDocument xml_doc;
	xml_doc.LoadFile((ARIS_INSTALL_PATH + std::string("/resource/demo_command_parser_xml/command.xml")).c_str());

	// 从xml里得到parser // 
	root.loadXml(xml_doc);
	auto& parser = static_cast<aris::core::CommandParser&>(*root.findByName("command_parser"));

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
	
	std::cout << "demo_command_parser_xml finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

