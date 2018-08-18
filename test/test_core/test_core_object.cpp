#include <iostream>
#include <aris_core.h>
#include "test_core_object.h"


using namespace aris::core;

class Man :public Object
{
public:
	static auto Type()->const std::string &{ static const std::string type{ "Man" }; return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
	auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override
	{
		Object::saveXml(xml_ele);
		xml_ele.SetAttribute("age", age_);
		xml_ele.SetAttribute("job", job_.c_str());
	}
	auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
	{
		Object::loadXml(xml_ele);
		age_ = attributeInt32(xml_ele, "age");
		job_ = attributeString(xml_ele, "job");
	}

	Man(const std::string &name = "man", int age = 0, const std::string job = "teacher") :Object(name), age_(age), job_(job){}
private:
	int age_;
	std::string job_;
};
class Man1 :public Object
{
public:
	static auto Type()->const std::string &{ static const std::string type{ "Man" }; return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override
	{
		Object::saveXml(xml_ele);
		xml_ele.SetAttribute("age", age_);
		xml_ele.SetAttribute("job", job_.c_str());
	}
	auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
	{
		Object::loadXml(xml_ele);
		age_ = attributeInt32(xml_ele, "age");
		job_ = attributeString(xml_ele, "job");
	}

	Man1(const std::string &name = "man", int age = 0, const std::string job = "teacher") :Object(name), age_(age), job_(job) {}
private:
	int age_;
	std::string job_;
};
class Man2 :public Object
{
public:
	static auto Type()->const std::string &{ static const std::string type{ "Man" }; return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override
	{
		Object::saveXml(xml_ele);
		xml_ele.SetAttribute("age", age_);
		xml_ele.SetAttribute("job", job_.c_str());
	}
	auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
	{
		Object::loadXml(xml_ele);
		age_ = attributeInt32(xml_ele, "age");
		job_ = attributeString(xml_ele, "job");
	}

	Man2(const std::string &name = "man", int age = 0, const std::string job = "teacher") :Object(name), age_(age), job_(job) {}
private:
	int age_;
	std::string job_;
};

void test_register()
{
	Object root;
	root.registerType<Man>();
	Object::registerTypeGlobal<Man1>();
}
void test_big_five()
{
	// test default ctor //
	Object root("root");
	root.registerType<Man>();
	root.add<Man>("root_man", 11, "worker");
	root.add<Object>("root_obj");

	if (root.xmlString() != "<root type=\"Object\">\n    <root_man type=\"Man\" age=\"11\" job=\"worker\"/>\n    <root_obj type=\"Object\"/>\n</root>\n") std::cout<< "aris::core::Object default ctor failed" <<std::endl;

	// test copy ctor //
	auto left(root);
	if(left.xmlString() != root.xmlString())std::cout << "aris::core::Object copy ctor failed" << std::endl;


	// test move ctor //
	auto &child1 = left.children().front();
	auto &child2 = left.children().back();

	auto left2(std::move(left));
	if (left2.xmlString() != root.xmlString() || &child1 != &left2.children().front() || &child2 != &left2.children().back())std::cout << "aris::core::Object move ctor failed" << std::endl;

	// test copy assign //
	Object root2("root2");
	auto &aaa2 = root2.add<Object>("aaa");
	auto &bbb2 = root2.add<Object>("bbb");
	auto &ccc2 = root2.add<Object>("ccc");
	auto &ccc2_1 = ccc2.add<Man>("ccc_1");
	auto &ccc2_2 = ccc2.add<Object>("ccc_2");
	auto &ccc2_3 = ccc2.add<Object>("ccc_3");
	auto &ddd2 = root2.add<Object>("ddd");
	auto &ddd2_1 = ddd2.add<Object>("ddd_1");
	auto &ddd2_2 = ddd2.add<Object>("ddd_2");
	auto &ddd2_3 = ddd2.add<Object>("ddd_3");
	auto &eee2 = root2.add<Object>("eee");
	auto &eee2_1 = eee2.add<Man>("eee_1");

	aaa2 = root;
	if ((aaa2.xmlString() != root.xmlString())) std::cout << "aris::core::Object copy assign failed" << std::endl;

	bbb2 = root;
	if ((bbb2.xmlString() != root.xmlString()) || (bbb2.id() != 1)) std::cout << "aris::core::Object copy assign failed" << std::endl;

	ccc2 = root;
	if ((ccc2.xmlString() != root.xmlString()) || (ccc2.id() != 2) || (&ccc2_1 != &ccc2.children().at(0)) || (&ccc2_2 != &ccc2.children().at(1))) std::cout << "aris::core::Object copy assign failed" << std::endl;

	ddd2 = root;
	if ((ddd2.xmlString() != root.xmlString()) || (ddd2.id() != 3) || (&ddd2_2 != &ddd2.children().at(1))) std::cout << "aris::core::Object copy assign failed" << std::endl;

	eee2 = root;
	if ((eee2.xmlString() != root.xmlString()) || (eee2.id() != 4) || (&eee2_1 != &eee2.children().at(0))) std::cout << "aris::core::Object copy assign failed" << std::endl;

	// test move assign //
	Object root3("root3");
	auto &aaa3 = root3.add<Object>("aaa");
	auto &bbb3 = root3.add<Object>("bbb");
	auto &ccc3 = root3.add<Object>("ccc");
	auto &ccc3_1 = ccc3.add<Man>("ccc_1");
	auto &ccc3_2 = ccc3.add<Object>("ccc_2");
	auto &ccc3_3 = ccc3.add<Object>("ccc_3");
	auto &ddd3 = root3.add<Object>("ddd");
	auto &ddd3_1 = ddd3.add<Object>("ddd_1");
	auto &ddd3_2 = ddd3.add<Object>("ddd_2");
	auto &ddd3_3 = ddd3.add<Object>("ddd_3");
	auto &eee3 = root3.add<Object>("eee");
	auto &eee3_1 = eee3.add<Man>("eee_1");

	aaa3 = std::move(aaa2);
	if ((aaa3.xmlString() != root.xmlString())) std::cout << "aris::core::Object move assign failed" << std::endl;

	bbb3 = std::move(bbb2);
	if ((bbb3.xmlString() != root.xmlString()) || (bbb3.id() != 1)) std::cout << "aris::core::Object move assign failed" << std::endl;

	ccc3 = std::move(ccc2);
	if ((ccc3.xmlString() != root.xmlString()) || (ccc3.id() != 2) || (&ccc3_1 != &ccc3.children().at(0)) || (&ccc3_2 != &ccc3.children().at(1))) std::cout << "aris::core::Object move assign failed" << std::endl;

	ddd3 = std::move(ddd2);
	if ((ddd3.xmlString() != root.xmlString()) || (ddd3.id() != 3) || (&ddd3_2 != &ddd3.children().at(1))) std::cout << "aris::core::Object move assign failed" << std::endl;

	eee3 = std::move(eee2);
	if ((eee3.xmlString() != root.xmlString()) || (eee3.id() != 4) || (&eee3_1 != &eee3.children().at(0))) std::cout << "aris::core::Object move assign failed" << std::endl;

}
void test_xml()
{
}



void test_object()
{
	std::cout << std::endl << "-----------------test object---------------------" << std::endl;
	test_big_five();
	std::cout << "-----------------test object finished------------" << std::endl << std::endl;
}