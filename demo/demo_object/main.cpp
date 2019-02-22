/// \example demo_object/main.cpp
/// 本例子展示构造数据结构的过程:
///

#include "aris.hpp"

using namespace aris::core;

class Child :public Object
{
public:
	static auto Type()->const std::string &{ static const std::string type{ "Child" }; return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
	auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override
	{
		Object::saveXml(xml_ele);
		xml_ele.SetAttribute("age", age_);
	}
	auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
	{
		Object::loadXml(xml_ele);
		age_ = attributeInt32(xml_ele, "age");
	}

	Child(const std::string &name = "child", int age = 0) :Object(name), age_(age) {};

private:
	int age_;
};
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

	Man(const std::string &name = "man", int age = 0, const std::string job = "teacher") :Object(name), age_(age), job_(job)
	{
		registerType<Child>();
	};
private:
	int age_;
	std::string job_;
};
class Family :public Object
{
public:
	static auto Type()->const std::string &{ static const std::string type{ "Family" }; return type; }
	auto virtual type() const->const std::string& override{ return Type(); }

	Family(const std::string &name = "family") :Object(name) 
	{
		registerType<Man>();
	};
};

class Boy :public Child
{
public:
	static auto Type()->const std::string &{ static const std::string type{ "Boy" }; return type; }
	auto virtual type() const->const std::string& override{ return Type(); }

	Boy(const std::string &name = "boy", int age = 0) :Child(name, age){};
};

int main()
{
	// 手动构造family1 object //
	Object family1("family1");
	auto &father1 = family1.add<Object>("father");
	family1.add<Object>("uncle");
	father1.add<Object>("tom");
	father1.add<Object>("bob");

	// 将family1的xml字符串打印出来 //
	std::cout << family1.xmlString() << std::endl;


	// 使用xml字符串构造family2 //
	Object family2;
	family2.loadXmlStr(
		"<family2 type=\"Object\">"
		"	<father type=\"Object\">"
		"		<tom type=\"Object\"/>"
		"		<bob type=\"Object\"/>"
		"	</father>"
		"	<uncle type=\"Object\"/>"
		"</family2>");


	// 将family2的xml字符串打印出来 //
	std::cout << family2.xmlString() << std::endl;


	// 使用自己定义的Family，Man，Child类型构造family3 //
	Family family3("family3");
	auto &father3 = family3.add<Man>("father", 35, "teacher");
	family3.add<Man>("uncle", 33, "policeman");
	father3.add<Child>("tom", 8);
	father3.add<Child>("bob", 6);

	std::cout << family3.xmlString() << std::endl;

	// 以上代码也可以使用xml字符串构造 //
	Family family4;
	family4.loadXmlStr(
		"<family4 type=\"Family\">"
		"	<father type=\"Man\" age=\"35\" job=\"teacher\">"
		"		<tom type=\"Child\" age=\"8\"/>"
		"		<bob type=\"Child\" age=\"6\"/>"
		"	</father>"
		"	<uncle type=\"Man\" age=\"33\" job=\"policeman\"/>"
		"</family4>");

	std::cout << family4.xmlString() << std::endl;

	// 注册新类型 //
	Family family5;
	aris::core::Object::registerTypeGlobal<Boy>();
	family5.loadXmlStr(
		"<family5 type=\"Family\">"
		"	<father type=\"Man\" age=\"35\" job=\"teacher\">"
		"		<tom type=\"Child\" age=\"8\"/>"
		"		<bob type=\"Child\" age=\"6\"/>"
		"		<bill type=\"Boy\" age=\"3\"/>"
		"	</father>"
		"	<uncle type=\"Man\" age=\"33\" job=\"policeman\"/>"
		"</family5>");
	std::cout << family5.xmlString() << std::endl;

	Family family6;
	family6.registerType<Boy>();
	family6.loadXmlStr(
		"<family6 type=\"Family\">"
		"	<father type=\"Man\" age=\"35\" job=\"teacher\">"
		"		<tom type=\"Child\" age=\"8\"/>"
		"		<bob type=\"Child\" age=\"6\"/>"
		"		<bill type=\"Boy\" age=\"3\"/>"
		"	</father>"
		"	<uncle type=\"Man\" age=\"33\" job=\"policeman\"/>"
		"</family6>");
	std::cout << family6.xmlString() << std::endl;



	return 0;
}

