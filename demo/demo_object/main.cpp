/// \example demo_object/main.cpp
/// 本例子展示构造数据结构的过程:
///

#include "aris.hpp"

class A { 
public:
	std::string name; 
};
class B :public A {};
class C {
public:
	auto resetPool(aris::core::PointerArray<A>* pool) { a_pool_.reset(pool); }
	auto pool()->aris::core::PointerArray<A>& { return *a_pool_; }

private:
	std::unique_ptr<aris::core::PointerArray<A>> a_pool_{new aris::core::PointerArray<A> };
};

ARIS_REGISTRATION{
	aris::core::class_<A>("A")
		.prop("name", &A::name)
		;

	aris::core::class_<B>("B")
		;

	aris::core::class_<aris::core::PointerArray<A>>("A_POOL")
		.asRefArray()
		;

	aris::core::class_<C>("C")
		.prop("a_pool", &C::resetPool, &C::pool)
		;
}


int main(){
	{
		C c;

		std::cout << aris::core::toXmlString(c) << std::endl;
	
	}

	
	auto &cs = aris::server::ControlServer::instance();
	
	try {
		aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\kaanh.xml");
	}
	catch (std::runtime_error& e)
	{
		std::cout << e.what() << std::endl;
	}
	cs.model().init();

	std::cout << dynamic_cast<aris::dynamic::MultiModel&>(cs.model()).tools().size() << std::endl;


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

