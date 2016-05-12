#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"


using namespace aris::control;

class MyMaster :public aris::control::Master
{
protected:
	virtual auto controlStrategy()->void override
	{
		std::int32_t pos;
		slavePool().at(0).readPdoIndex(0x6064, 0x0000, pos);

		static int count{ 0 };
#ifdef UNIX
		if (count++ % 100 == 0)rt_printf("%d:%d\n", pos);
#endif
	};
};

void test_control_ethercat()
{
	MyMaster master;

	for (auto &slave : master.slavePool())
	{

	}


#ifdef WIN32
	master.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	master.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

	master.start();

	char a;
	std::cin >> a;

	master.stop();

	std::cout << "finished test aris::control::master" << std::endl;
}