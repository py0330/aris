#include <iostream>
#include <aris.h>

#include "test_core_xml.h"
#include "test_core_socket.h"
#include "test_core_command.h"
#include "test_core_msg.h"
#include "test_core_pipe.h"


int main(int argc, char *argv[])
{
	//test_core_xml();
	test_core_msg();
	//test_core_socket();
	//test_core_pipe();
	//test_core_command();

	std::cout << "test_core finished, press any key to continue" << std::endl;
	char s;
	std::cin >> s;

	return 0;
}