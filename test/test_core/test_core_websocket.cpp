#include <iostream>
#include <future>
#include <aris_core.h>
#include "test_core_websocket.h"
#include "sha1.h"

#ifdef WIN32
#include <Winsock.h>
#endif

using namespace aris::core;
auto pack_data(const aris::core::MsgBase &msg)->std::string
{
	int size = msg.size() + sizeof(MsgHeader);

	std::string s;
	if (size < 126)
	{
		s.resize(size + 2);
		s[0] = 0x81;
		s[1] = size;
		std::copy_n(msg.data() - sizeof(MsgHeader), size, &s[2]);
	}
	else if (size < 0xFFFF)
	{
		s.resize(size + 4);
		s[0] = 0x81;
		s[1] = 126;
		s[2] = size >> 8;
		s[3] = size & 0xFF;
		std::copy_n(msg.data() - sizeof(MsgHeader), size, &s[4]);
	}
	else
	{
		s.resize(size + 10);
		s[0] = 0x81;
		s[1] = 127;
		s[2] = 0;
		s[3] = 0;
		s[4] = 0;
		s[5] = 0;
		s[6] = size >> 24;
		s[7] = size >> 16;
		s[8] = size >> 8;
		s[9] = size & 0xFF;
		std::copy_n(msg.data() - sizeof(MsgHeader), size, &s[10]);
	}


	return s;
};

void test_core_websocket()
{
	std::cout << std::endl << "-----------------test websocket---------------------" << std::endl;
	
	WebSocket s;

	s.startServer("5866");




	char a;
	std::cin >> a;


	std::cout << "-----------------test websocket finished------------" << std::endl << std::endl;
}

