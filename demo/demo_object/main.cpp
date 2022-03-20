/// \example demo_object/main.cpp
/// 本例子展示构造数据结构的过程:
///

#include "aris.hpp"

int main()
{
	aris::core::Socket server, client;
	server.setConnectType(aris::core::Socket::Type::TCP_RAW);
	client.setConnectType(aris::core::Socket::Type::TCP_RAW);


	server.setOnReceivedRawData([](aris::core::Socket *s, const char *data, int size)->int {
		std::cout << "server received:" << std::string(data, size) << std::endl;
		return 0;
	});

	client.setOnReceivedRawData([](aris::core::Socket *s, const char *data, int size)->int {
		std::cout << "client received:" << std::string(data, size) << std::endl;
		return 0;
	});

	server.startServer("5866");
	client.connect("127.0.0.1", "5866");

	client.sendRawData("abcd\n", 5);
	client.sendRawData("abcde\n", 6);
	client.sendRawData("abcdef\n", 7);

	server.sendRawData("ABCDEFG\n", 8);
	server.sendRawData("ABCDEF\n", 7);
	server.sendRawData("ABCDE\n", 6);

	std::this_thread::sleep_for(std::chrono::seconds(10));

	return 0;
}

