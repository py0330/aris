#include <iostream>
#include <aris/core/core.hpp>
#include "test_core_log.h"

using namespace aris::core;

void test_log()
{
	if (aris::core::logExeName() != "test_core")std::cout << "aris::core::logExeName() failed" << std::endl;
}
void test_msg_copy()
{
	struct A
	{
		char str[12];
		double d;
		int i;
	};

	A from[3]{ { "a is good", 3.14, -1 },{ "b is bad", -0.027, -108 },{ "c is normal", 130.25, 125 } };
	A to[3];

	Msg msg;

	msg.copyStruct(from[0], from[1], from[2]);
	msg.pasteStruct(to[0], to[1], to[2]);

	for (auto i = 0; i < 3; ++i)
	{
		if (std::strcmp(from[i].str, to[i].str) != 0 || from[i].d != to[i].d || from[i].i != to[i].i)
			std::cout << "aris::core::Msg::copyStruct or pasteStruct failed";
	}
}
void test_msg_stream()
{
	Msg msg;
	aris::core::MsgStream stream(msg);
	stream << "1234 abc\n aaa" << '\0' <<std::flush;
	if (std::string(msg.data()) != "1234 abc\n aaa") std::cout << "aris::core::MsgStream input failed";

	MsgFix<8192> msg_fix;
	aris::core::MsgStream stream_fix(msg_fix);
	stream_fix << "1234 abc\n aaa" << '\0' << std::flush;
	if (std::string(msg_fix.data()) != "1234 abc\n aaa") std::cout << "aris::core::MsgStream input failed";
}


void test_msg()
{
	std::cout << std::endl << "-----------------test msg---------------------" << std::endl;

	test_log();
	test_msg_copy();
	test_msg_stream();








	std::cout << "-----------------test msg finished------------" << std::endl << std::endl;
}