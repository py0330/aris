#include <iostream>
#include <future>
#include <aris_core.h>
#include "test_core_websocket.h"
#include "sha1.h"

#ifdef WIN32
#include <Winsock.h>
#endif

using namespace aris::core;

void test_core_websocket()
{
	std::cout << std::endl << "-----------------test websocket---------------------" << std::endl;
	
	WebSocket s;

	s.startServer("5866");




	char a;
	std::cin >> a;


	std::cout << "-----------------test websocket finished------------" << std::endl << std::endl;
}

