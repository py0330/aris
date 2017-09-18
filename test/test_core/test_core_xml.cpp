#include <iostream>
#include <aris.h>

#include "test_core_xml.h"


using namespace aris::core;

class Child :public Object
{
public:
	static auto Type()->const std::string &{ static const std::string type("Child"); return std::ref(type); }
	auto virtual type() const->const std::string& override{ return Type(); }

	int a = 0;

	Child(const std::string &name = "child") :Object(name) {};
};

void test_core_xml()
{
	Root root;
	root.registerChildType<Child>();

	auto &obj1 = root.add<Object>("obj1");
	auto &obj2 = root.add<Object>("obj2");
	auto &obj3 = root.add<Object>("obj3");

	auto &obj1_1 = obj1.add<Object>("obj1-1");
	auto &obj1_2 = obj1.add<Object>("obj1-2");
	auto &obj1_3 = obj1.add<Object>("obj1-3");

	auto &obj2_1 = obj2.add<Child>("obj2-1");
	auto &obj2_2 = obj2.add<Object>("obj2-2");
	auto &obj2_3 = obj2.add<Child>("obj2-3");
	auto &obj2_4 = obj2.add<Child>("obj2-4");

	

	for (auto &child : obj2.children())
	{
		std::cout << &child << " " << child.type() << " " << child.name() << " " << static_cast<Child&>(child).a << std::endl;
	}
	std::cout << std::endl;
	obj2.save("aaa");
	obj2_1.a = 100;
	for (auto &child : obj2.children())
	{
		std::cout << &child << " " << child.type() << " " << child.name() << " " << static_cast<Child&>(child).a << std::endl;
	}
	std::cout << std::endl;
	obj2.load("aaa");
	for (auto &child : obj2.children())
	{
		std::cout << &child << " " << child.type() << " " << child.name() << " " << static_cast<Child&>(child).a << std::endl;
	}
	std::cout << std::endl;




	/*for (auto &child : obj1.children())
	{
		std::cout << &child << " " << child.type() << " " << child.name() << std::endl;
	}
	obj1 = std::move(obj2);
	for (auto &child : obj1.children())
	{
		std::cout << &child << " " << child.type() << " " << child.name() << std::endl;
	}*/

}