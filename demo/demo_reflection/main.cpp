#include <iostream>
#include <array>
#include <aris/core/reflection.hpp>
#include <aris.hpp>

struct Base { int c; virtual ~Base() = default; };
struct A:public Base 
{ 
	Base base;
	
	int a;
	int *b;

	int getI() { return -1000; };
	void setI(int a) { std::cout << "set value:" << a << std::endl; };
};

ARIS_REGISTRATION
{
	aris::core::class_<Base>("Base")
	.constructor<const Base&>()
	.property("ddd", &Base::c);
	
	aris::core::class_<A>("aaa")
		.constructor<const A&>()
		.property("base", &A::base)
		.property("a", &A::a)
		.property("b", &A::b)
		.property("c", &A::setI, &A::getI);

	
}



auto to_xml(aris::core::Instance i)
{
	aris::core::XmlDocument doc;



	i.type();




}



int main()
{
	int a = 50;

	auto t = aris::core::getType("aaa");

	A aaa;

	int &bbb = a;

	Base &b = aaa;
	std::cout << typeid(b).name() << std::endl;



	try
	{
		aris::core::Instance k = b;
		k.set("a", 1000);
		std::cout << "get : " << k.get("a").to<int>() << std::endl;

		k.set("a", bbb);
		std::cout << "get : " << k.get("a").to<int>() << std::endl;

		//t.properties().at("b").set(aaa, &bbb);
		//std::cout << t.properties().at("b").get<int*>(aaa) << std::endl;

		//t.properties().at("c").set(aaa, bbb);
		//std::cout << t.properties().at("c").get<int>(aaa) << std::endl;


		auto b = k.get("base");

		b.set("ddd", 1532);
		auto i = b.get("ddd").to<int>();
		std::cout << i << std::endl;

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	





	std::cout << "demo_reflection finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

