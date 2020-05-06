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

	int& getE() { return a; };
	void setE(int *a) { std::cout << "set e:" << a << std::endl; };
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
		.property("d", &A::setJ, &A::getJ)
		.property("e", &A::setE, &A::getE);

	aris::core::class_<std::vector<int>>("vector")
		.asArray();

	aris::core::class_<std::vector<A>>("vector_A")
		.asArray();
}


void Interptask(void *arg)
{
	aris::control::aris_rt_task_set_periodic(1000000);
	static int i = 0;
	for (;;)// while(1)//(int i=0;i<100;i++)
	{
		// i++;
		i++;
		aris::control::aris_rt_task_wait_period();
		std::cout << "count11233:=" << i << std::endl;
	}
}



int main()
{
	/*
		std::cout << "start testing IO board" << std::endl;

		//创建EtherCAT主站对象
		aris::control::EthercatMaster mst;

		//自动扫描、连接从站
		mst.scan();
		mst.init();
		//打印主站扫描的从站个数
		std::cout << "slave num:" << mst.slavePool().size() << std::endl;

		//1、主站对象mst的成员函数setControlStrategy()创建一个实时线程。其形参即是被调用的实时函数，在实时核中每1ms被调用1次
		//2、被调用函数可以实现主站与从站之间的数据交互，打印服务，log服务。(本例以读、写IO信号，并打印为例)
		mst.setControlStrategy([&]()
		{
			static int count{ 0 };	//count用于计数本函数setControlStrategy()执行的次数
			static std::uint8_t value{ 0x01 };

			if (++count % 1000 == 0)	//实时核执行一次周期是1ms，每执行1000次，执行本if条件内的语句
			{
				//控制EtherCAT IO板卡的DO口实现“走马灯”逻辑
				value = value << 2;
				if (value == 0) value = 0x01;

				//成员函数mout()是实时核打印函数接口，成员函数lout()是实时核log函数接口
				mst.mout() << "count:" << std::dec << count << std::endl;
				mst.lout() << "count:" << std::dec << count << std::endl;

				//1、成员函数ecSlavePool()创建从站vector，在实时核中要使用ecSlavePool()，在非实时核使用SlavePool()，at(1)表示第2个从站，即EtherCAT IO板卡在物理连接层面属于第二个从站
				//2、writePdo是写函数，第一个形参是index，第二个形参是subindex，第三个形参写DO的数值，第四个形参表示写操作的bit数
				//mst.slavePool().at(1).writePdo(0x7001, 0x01, &value, 8);

				int a = 5000000;
				int b = 0;
				//mst.slavePool().at(0).writePdo(0x607A, 0x00, &a, 32);
				//mst.slavePool().at(0).readPdo(0x6064, 0x00, &a, 32);
				// mst.slavePool().at(1).readPdo(0x6064,0x00,&b,32);
				// mst.mout() << "count1:" << a << std::endl;
				//mst.mout() << "count2:" << b << std::endl;

			}
		});

		//启动实时线程
		mst.start();

		//非实时线程睡眠100秒钟
		//std::this_thread::sleep_for(std::chrono::seconds(100));

		auto handle = aris::control::aris_rt_task_create();
		aris::control::aris_rt_task_start(handle, &Interptask, nullptr);
		aris::control::aris_rt_task_join(handle);

		while (1)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		//关闭实时线程
		mst.stop();

		return 0;
	
	*/
	
	/*
	auto hhhhhh = aris::control::aris_rt_task_create();
	
	aris::control::aris_rt_task_start(hhhhhh, [](void*)->void
	{
		aris::control::aris_rt_task_set_periodic(100000000);

		for (int i = 0;i<100;++i)
		{
			aris::control::aris_rt_task_wait_period();

			std::cout << "count:"<<i << std::endl;
		}
		
	}, nullptr);
	
	aris::control::aris_rt_task_join(hhhhhh);
	
	*/

	std::cout << typeid(unsigned char).hash_code() << std::endl;
	std::cout << typeid(signed char).hash_code() << std::endl;
	std::cout << typeid(char).hash_code() << std::endl;
	std::cout << typeid(std::int8_t).hash_code() << std::endl;
	std::cout << typeid(std::uint8_t).hash_code() << std::endl;

	auto controller = aris::robot::createControllerRokaeXB4();

	auto &s = controller->slavePool()[0];
	auto &ec_s = dynamic_cast<aris::control::EthercatMotor&>(s);

	auto &slave_pool = controller->slavePool();

	auto &objjjj = ec_s.smPool();
	auto &ec_controller = dynamic_cast<aris::control::EthercatController&>(*controller);

	//std::cout << controller->xmlString() << std::endl;

	//std::cout << aris::core::toXmlString(*controller) << std::endl;
	//auto strsssssss = controller->xmlString();

	//aris::control::EthercatController ec;
	//aris::core::fromXmlString(ec, strsssssss);
	//std::cout << aris::core::toXmlString(ec) << std::endl;


	//aris::control::EthercatMotor slave2;
	//dynamic_cast<aris::control::EthercatMotor&>(s).smPool2() = dynamic_cast<aris::control::EthercatMotor&>(s).smPool();
	//
	//std::cout << aris::core::toXmlString(s) << std::endl;

	////std::cout << s.xmlString() << std::endl;
	////std::cout << aris::core::toXmlString(s) << std::endl;
	//aris::core::fromXmlString(slave2, aris::core::toXmlString(s));
	////std::cout << slave2.xmlString() << std::endl;
	//std::cout << aris::core::toXmlString(slave2) << std::endl;

	/*
	auto model = aris::robot::createModelRokaeXB4();
	std::cout << aris::core::toXmlString(*model) << std::endl;

	auto xml_str = aris::core::toXmlString(*model);

	aris::dynamic::Model model3 = *model;



	aris::dynamic::Model model2;
	aris::core::fromXmlString(model2, xml_str);



	std::cout << "---------------------------------------------------" << std::endl;

	//std::cout << model->xmlString() << std::endl;

	std::cout << aris::core::toXmlString(model2) << std::endl;

	char tessssssssssssssss;
	std::cin >> tessssssssssssssss;
	*/
	
	/*
	// 创建模型 //
	aris::dynamic::Serial3Param param;
	param.external_axes.push_back({ aris::dynamic::ExternalAxis::RotationalAxis });
	param.external_axes.push_back({ aris::dynamic::ExternalAxis::RotationalAxis });

	param.a1 = 0.7;
	param.a2 = 0.5;
	param.a3 = 0.6;
	auto m = aris::dynamic::createModelSerial3Axis(param);

	// 获得求解器 //
	auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(m->solverPool()[0]);
	
	// 设置末端位置 和 外部轴位置 //
	double ee[]{ 1,0,0,1.68070023071933,0,1,0, 0.35446729674924,0,0,1, -0.22165182186613,0,0,0,1};
	double ext[]{ 0.1,0.2 };
	//solver.setPmEE(ee, ext);

	double eula[3]{ 0.0, 1.57, 3.14 };
	solver.setEulaAngle(eula, "321");

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

	//solver.setWhichRoot(0);
	m->solverPool()[0].kinPos();
	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;

	//solver.setWhichRoot(1);
	m->solverPool()[0].kinPos();
	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;

	//solver.setWhichRoot(4);
	m->solverPool()[0].kinPos();
	m->solverPool()[1].kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	for (auto &m : m->motionPool())std::cout << m.mp() << "  ";
	std::cout << std::endl;
	
	auto &solver2 = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool()[1]);
	m->motionPool()[0].setMp(0.5);
	solver2.kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	
	m->motionPool()[0].setMp(0.8);
	solver2.kinPos();
	aris::dynamic::dsp(4, 4, *m->partPool()[3].markerPool().findByName("tool0")->pm());
	//*/
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	int a = 50;

	std::cout << "before" << std::endl;
	auto t = aris::core::getType(std::string("aaa"));
	std::cout << "end" << std::endl;

	std::vector<A> vec_A_ele;
	aris::core::Instance vec_A_ins = vec_A_ele;

	A aaaaaaa;
	aaaaaaa.a = 1256;
	aaaaaaa.base.c = 2589;
	vec_A_ins.push_back(aaaaaaa);
	vec_A_ins.push_back(aaaaaaa);

	std::cout << aris::core::toXmlString(vec_A_ele) << std::endl;

	auto str = aris::core::toXmlString(vec_A_ele);

	std::cout << str << std::endl;

	std::vector<A> vec_A_ele2;
	aris::core::fromXmlString(vec_A_ele2, str);

	vec_A_ele2[1].a = 586;

	std::cout << aris::core::toXmlString(vec_A_ele2) << std::endl;

	aris::core::Instance vec_ins = std::vector<int>{ 1,2,3 };
	auto &value_ele = *vec_ins.at(0).castTo<int>();
	

	
	value_ele = 1000;
	std::cout << *vec_ins.at(0).castTo<int>() << std::endl;
	

	vec_ins.push_back(150);


	A aaa;

	int &bbb = a;

	Base &b = aaa;
	//std::cout << typeid(b).name() << std::endl;

	aris::core::Instance i1 = 5.9546;
	std::cout << *i1.castTo<double>() << std::endl;

	try
	{
		aris::core::Instance k = b;
		k.set("a", 1000);
		std::cout << "get : " << *k.get("a").castTo<int>() << std::endl;

		k.set("a", bbb);
		std::cout << "get : " << *k.get("a").castTo<int>() << std::endl;

		k.set("ds", 1350);
		std::cout << "get : " << *k.get("ds").castTo<int>() << std::endl;

		k.set("ds", 1500);
		std::cout << "get : " << *k.get("ds").castTo<int>() << std::endl;

		k.set("d", 1258);
		std::cout << "get : " << *k.get("d").castTo<int>() << std::endl;

		k.set("c", 1366);
		std::cout << "get : " << *k.get("c").castTo<int>() << std::endl;

		std::cout << "reference::" << k.get("d").isReference() << std::endl;
		std::cout << "reference::" << k.get("c").isReference() << std::endl;
		std::cout << "reference::" << k.get("ds").isReference() << std::endl;

		//t.properties().at("b").set(aaa, &bbb);
		//std::cout << t.properties().at("b").get<int*>(aaa) << std::endl;

		//t.properties().at("c").set(aaa, bbb);
		//std::cout << t.properties().at("c").get<int>(aaa) << std::endl;


		auto b = k.get("base");

		b.set("ddd", 1532);
		auto i = *b.get("ddd").castTo<int>();
		std::cout << i << std::endl;



		std::cout << aris::core::toXmlString(k) << std::endl;

		k.get("base").set("ddd", 100);

		auto xml_str = aris::core::toXmlString(k);

		aris::core::fromXmlString(k, xml_str);
		std::cout << aris::core::toXmlString(k) << std::endl;

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	
	aris::core::Object obj("aaaa");
	std::cout << aris::core::toXmlString(obj) << std::endl;

	aris::core::Object obj2;
	aris::core::fromXmlString(obj2, aris::core::toXmlString(obj));
	std::cout << aris::core::toXmlString(obj2) << std::endl;

	aris::core::Param p1;
	std::cout << aris::core::toXmlString(p1) << std::endl;

	aris::core::Param p2;
	aris::core::fromXmlString(p2, aris::core::toXmlString(p1));
	std::cout << aris::core::toXmlString(p2) << std::endl;

	aris::core::Command cmd;
	cmd.push_back(new aris::core::Param("aaa"));
	std::cout << aris::core::toXmlString(cmd) << std::endl;


	// 添加CommandParser //
	aris::core::CommandParser parser;

	// 添加enable命令 //
	auto &enable = parser.commandPool().add<aris::core::Command>("enable", "");

	// 添加命令各参数节点 //
	auto &group = enable.add<aris::core::GroupParam>("group");
	auto &unique1 = group.add<aris::core::UniqueParam>("unique1", "");
	auto &unique2 = group.add<aris::core::UniqueParam>("unique2", "position");
	auto &all = unique1.add<aris::core::Param>("all", "", 'a');
	auto &motion = unique1.add<aris::core::Param>("motion", "0", 'm');
	auto &position = unique2.add<aris::core::Param>("position", "", 'p');
	auto &velocity = unique2.add<aris::core::Param>("velocity", "", 'v');
	auto &current = unique2.add<aris::core::Param>("current", "", 0);


	std::cout << parser.xmlString() << std::endl;


	std::cout << aris::core::toXmlString(parser) << std::endl;


	aris::core::CommandParser parser2;
	aris::core::fromXmlString(parser2, parser.xmlString());

	std::cout << parser2.xmlString() << std::endl;

	//for (;;)
	//{
	//	aris::core::Command cmd2;
	//	aris::core::fromXmlString(cmd2, aris::core::toXmlString(cmd));
	//	std::cout << aris::core::toXmlString(cmd2) << std::endl;

	//	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	//}


	std::cout << "demo_reflection finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

