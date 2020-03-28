#include <iostream>
#include <array>
#include <aris/core/reflection.hpp>
#include <aris/core/serialization.hpp>
#include <aris.hpp>

struct Base { int c; virtual ~Base() = default; };
struct A:public Base 
{ 
	Base base;
	
	int a;
	int *b;

	const int& getI() { return a; };
	void setI(int a) { std::cout << "set value:" << a << std::endl; this->a = a; };

	int& getJ() { return a; };
	void setJ(const int &a) { std::cout << "set value:" << a << std::endl; };
};

ARIS_REGISTRATION
{
	aris::core::class_<Base>("Base")
		.property("ddd", &Base::c);
	
	aris::core::class_<A>("aaa")
		.property("base", &A::base)
		.property("a", &A::a)
		.property("c", &A::setI, &A::getI)
		.property("ds", &A::getJ)
		.property("d", &A::setJ, &A::getJ);

	aris::core::class_<std::vector<int>>("vector")
		.asArray();
}






int main()
{
	// 创建模型 //
	aris::dynamic::Serial3Param param;
	param.a1 = 0.7;
	param.a2 = 0.5;
	param.a3 = 0.6;
	auto m = aris::dynamic::createModelSerial3Axis(param);

	// 获得求解器 //
	auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(m->solverPool()[0]);
	
	// 设置末端位置 //
	double ee[3]{ 1.68070023071933, 0.35446729674924, -0.22165182186613};
	solver.setPosEE(ee);
	
	// 设置解，一共4个，设为4时会选最优解 //
	solver.setWhichRoot(4);
	
	// 求解 //
	solver.kinPos();
	
	// 打印电机位置 //
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;


	m->motionPool()[0].setMp(0.2);
	m->motionPool()[1].setMp(0.3);
	m->motionPool()[2].setMp(0.5);

	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());

	aris::core::Object obj("123");
	std::cout << aris::core::toXmlString(obj) << std::endl;
	std::cout << obj.xmlString() << std::endl;
	
	auto &types = aris::core::reflect_types();
	auto &names = aris::core::reflect_names();

	solver.setWhichRoot(0);
	m->solverPool()[0].kinPos();
	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;


	solver.setWhichRoot(1);
	m->solverPool()[0].kinPos();
	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;

	solver.setWhichRoot(4);
	m->solverPool()[0].kinPos();
	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;

	
	std::cout << aris::core::is_container<std::vector<int>>::value << std::endl;
	std::cout << aris::core::is_container<int>::value << std::endl;

	int a = 50;

	std::cout << "before" << std::endl;
	auto t = aris::core::getType(std::string("aaa"));
	std::cout << "end" << std::endl;

	
	aris::core::Instance vec_ins = std::vector<int>{ 1,2,3 };
	auto &value_ele = vec_ins.at(0).to<int>();
	

	
	value_ele = 1000;
	std::cout << vec_ins.at(0).to<int>() << std::endl;
	

	vec_ins.push_back(150);


	A aaa;

	int &bbb = a;

	Base &b = aaa;
	//std::cout << typeid(b).name() << std::endl;

	aris::core::Instance i1 = 5.9546;
	std::cout << i1.to<double>() << std::endl;



	try
	{
		aris::core::Instance k = b;
		k.set("a", 1000);
		std::cout << "get : " << k.get("a").to<int>() << std::endl;

		k.set("a", bbb);
		std::cout << "get : " << k.get("a").to<int>() << std::endl;

		k.set("ds", 1350);
		std::cout << "get : " << k.get("ds").to<int>() << std::endl;

		k.set("ds", 1500);
		std::cout << "get : " << k.get("ds").to<int>() << std::endl;

		k.set("d", 1258);
		std::cout << "get : " << k.get("d").to<int>() << std::endl;

		k.set("c", 1366);
		std::cout << "get : " << k.get("c").to<int>() << std::endl;

		std::cout << "reference::" << k.get("d").isReference() << std::endl;
		std::cout << "reference::" << k.get("c").isReference() << std::endl;
		std::cout << "reference::" << k.get("ds").isReference() << std::endl;

		//t.properties().at("b").set(aaa, &bbb);
		//std::cout << t.properties().at("b").get<int*>(aaa) << std::endl;

		//t.properties().at("c").set(aaa, bbb);
		//std::cout << t.properties().at("c").get<int>(aaa) << std::endl;


		auto b = k.get("base");

		b.set("ddd", 1532);
		auto i = b.get("ddd").to<int>();
		std::cout << i << std::endl;



		std::cout << aris::core::toXmlString(k) << std::endl;

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	


	std::cout << "demo_reflection finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

