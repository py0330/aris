///// \example demo_command_parser_xml/main.cpp
///// 本例子展示基于xml解析命令字符串的过程，xml文件位于：安装目录/resource/demo_command_parser_xml/command.xml，内容如下：
///// \include demo_command_parser_xml/resource/command.xml
///// 以下为C++源码
/////

#include <iostream>
#include <chrono>
#include "aris.h"

int main()
{
	// 这里xml是发送的内容：
	// + 根节点的名称代表调用的函数
	// + 一级节点代表函数的参数，一级节点的名称可以任意设置（比如以下的list节点可以用其他名称），内容是参数内容
	// + 一级节点的顺序代表参数顺序
	// 以下xml字符串代表调用函数
	char xml[] =
		"<sendDataToMotion>"
		"    <list>"
		"#1P1500T100\r\n"
		"#1P1510T100\r\n"
		"#1P1520T100\r\n"
		"#1P1530T100\r\n"
		"#1P1540T100\r\n"
		"#1P1550T100\r\n"
		"#1P1560T100\r\n"
		"#1P1570T100\r\n"
		"#1P1580T100\r\n"
		"#1P1590T100\r\n"
		"#1P1600T100\r\n"
		"#1P1610T100\r\n"
		"#1P1620T100\r\n"
		"#1P1630T100\r\n"
		"#1P1640T100\r\n"
		"#1P1650T100\r\n"
		"#1P1660T100\r\n"
		"#1P1670T100\r\n"
		"#1P1680T100\r\n"
		"#1P1690T100\r\n"
		"#1P1700T100\r\n"
		"#1P1710T100\r\n"
		"#1P1720T100\r\n"
		"#1P1730T100\r\n"
		"#1P1740T100\r\n"
		"#1P1750T100\r\n"
		"#1P1760T100\r\n"
		"#1P1770T100\r\n"
		"#1P1780T100\r\n"
		"#1P1790T100\r\n"
		"#1P1800T100\r\n"
		"#1P1810T100\r\n"
		"#1P1820T100\r\n"
		"#1P1830T100\r\n"
		"#1P1840T100\r\n"
		"#1P1850T100\r\n"
		"#1P1860T100\r\n"
		"#1P1870T100\r\n"
		"#1P1880T100\r\n"
		"#1P1890T100\r\n"
		"#1P1900T100\r\n"
		"#1P1910T100\r\n"
		"#1P1920T100\r\n"
		"#1P1930T100\r\n"
		"#1P1940T100\r\n"
		"#1P1950T100\r\n"
		"#1P1960T100\r\n"
		"#1P1970T100\r\n"
		"#1P1980T100\r\n"
		"#1P1990T100\r\n"
		"</list>"
		"    <pause>100</pause>"
		"</sendDataToMotion>";
	
	// 整个aris预计只用一个接口，就是以下函数
	// auto sendStringToAris(const char *data_to_aris, int data_to_size, char *data_from_aris, int *data_from_size)->void;
	// data_to_aris：发送的函数xml
	// data_to_size：xml字符串长度
	// data_from_aris：aris返回的内容
	// data_from_size：aris返回的字符串长度
	//
	// 目前只支持调用sendDataToMotion
	// 后续会添加正反解模型等
	int ret;
	aris::server::sendStringToAris(xml, std::strlen(xml), nullptr, &ret);

	std::cout << "demo_control_wifi finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

