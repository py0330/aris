#include <iostream>
#include <aris.h>

#include "test_core_msg.h"

using namespace aris::core;

void test_core_msg()
{
	std::cout << aris::core::logExeName() << std::endl;
	std::cout << aris::core::logFileName() << std::endl;
	std::cout << aris::core::logDirPath() << std::endl;
	std::cout << aris::core::logFileTimeFormat(std::chrono::system_clock::now()) << std::endl;
	
	
	struct A
	{
		char str[12];
		double d;
		int i;
	};

	A from[3]{ {"a is good", 3.14, -1},{ "b is bad", -0.027, -108 },{ "c is normal", 130.25, 125 } };
	A to[3];

	Msg msg;

	msg.copyStruct(from[0], from[1], from[2]);
	msg.pasteStruct(to[0], to[1], to[2]);

	for (auto i = 0; i < 3; ++i)
	{
		if (std::strcmp(from[i].str, to[i].str) != 0 || from[i].d != to[i].d || from[i].i != to[i].i)
			std::cout<<"msg copy struct failed";
	}
	


}